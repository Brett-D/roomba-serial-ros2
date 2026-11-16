//! Thread-safe serial connection to an iRobot Roomba 500-series robot (port of
//! roomba_driver/roomba.py): the termios port, the sensor stream reader thread with its
//! primary/secondary layout rotation, stall restarts and reconnects, mode changes and polling.
//! Error strings match the Python RoombaError / pyserial messages (they reach logs and services).

use std::ffi::{CStr, CString};
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Condvar, Mutex, MutexGuard};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use crate::driver::{Level, Link};
use crate::oi::{self, Field, SensorValues, StreamParser};

/// time.monotonic(): CLOCK_MONOTONIC in seconds.
pub fn mono() -> f64 {
    let mut ts = libc::timespec { tv_sec: 0, tv_nsec: 0 };
    // SAFETY: clock_gettime only writes the timespec passed to it.
    unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts) };
    ts.tv_sec as f64 + ts.tv_nsec as f64 * 1e-9
}

/// Locks, ignoring poisoning (a panicking callback must not take the driver down with it).
pub fn lock<T>(m: &Mutex<T>) -> MutexGuard<'_, T> {
    m.lock().unwrap_or_else(|e| e.into_inner())
}

fn sleep(s: f64) {
    std::thread::sleep(Duration::from_secs_f64(s));
}

// ------------------------------------------------------------------ serial port
/// The serial device, shared by the reader thread (reads) and writers.
pub trait Port: Send + Sync {
    /// Wait up to `timeout` for data; Ok(0) on timeout.
    fn read(&self, buf: &mut [u8], timeout: Duration) -> Result<usize, String>;
    /// Write everything and wait until it has been sent (pyserial write + flush).
    fn write(&self, data: &[u8]) -> Result<(), String>;
    fn set_dtr(&self, on: bool) -> Result<(), String>;
    fn reset_input_buffer(&self) -> Result<(), String>;
}

pub type Opener = Box<dyn Fn(&str, u32) -> Result<Arc<dyn Port>, String> + Send + Sync>;

fn errno() -> i32 {
    std::io::Error::last_os_error().raw_os_error().unwrap_or(0)
}

/// str(OSError(errno)) in Python: "[Errno 5] Input/output error".
pub fn os_error(e: i32) -> String {
    // SAFETY: strerror returns a pointer to a static (or thread-local) NUL-terminated string.
    let msg = unsafe { CStr::from_ptr(libc::strerror(e)) }.to_string_lossy().into_owned();
    format!("[Errno {e}] {msg}")
}

fn retryable(e: i32) -> bool {
    matches!(e, libc::EAGAIN | libc::EINTR | libc::EALREADY | libc::EINPROGRESS) || e == libc::EWOULDBLOCK
}

/// A tty opened and configured like pyserial does (raw 8N1, no flow control, DTR/RTS low).
pub struct TtyPort {
    fd: libc::c_int,
    write_timeout: Duration,
}

impl Drop for TtyPort {
    fn drop(&mut self) {
        // SAFETY: the fd is owned by this struct and closed once.
        unsafe { libc::close(self.fd) };
    }
}

fn baud_constant(baud: u32) -> Option<libc::speed_t> {
    Some(match baud {
        50 => libc::B50,
        75 => libc::B75,
        110 => libc::B110,
        134 => libc::B134,
        150 => libc::B150,
        200 => libc::B200,
        300 => libc::B300,
        600 => libc::B600,
        1200 => libc::B1200,
        1800 => libc::B1800,
        2400 => libc::B2400,
        4800 => libc::B4800,
        9600 => libc::B9600,
        19200 => libc::B19200,
        38400 => libc::B38400,
        57600 => libc::B57600,
        115200 => libc::B115200,
        230400 => libc::B230400,
        460800 => libc::B460800,
        500000 => libc::B500000,
        576000 => libc::B576000,
        921600 => libc::B921600,
        1000000 => libc::B1000000,
        _ => return None,
    })
}

impl TtyPort {
    pub fn open(path: &str, baud: u32) -> Result<TtyPort, String> {
        let speed = baud_constant(baud).ok_or_else(|| format!("Invalid baud rate: {baud}"))?;
        let c = CString::new(path).map_err(|_| format!("could not open port {path}: embedded NUL"))?;
        // SAFETY: plain open(2) of a NUL-terminated path.
        let fd = unsafe { libc::open(c.as_ptr(), libc::O_RDWR | libc::O_NOCTTY | libc::O_NONBLOCK) };
        if fd < 0 {
            let e = errno();
            return Err(format!("[Errno {e}] could not open port {path}: {}: '{path}'", os_error(e)));
        }
        let port = TtyPort { fd, write_timeout: Duration::from_millis(500) };
        port.configure(speed)?;
        port.modem_line(libc::TIOCM_DTR, false).or_else(ignore_unsupported)?;
        port.modem_line(libc::TIOCM_RTS, false).or_else(ignore_unsupported)?;
        port.reset_input_buffer()?;
        Ok(port)
    }

    fn configure(&self, speed: libc::speed_t) -> Result<(), String> {
        use libc::*;
        let fail = || {
            let e = errno();
            // SAFETY: strerror returns a static string.
            let msg = unsafe { CStr::from_ptr(strerror(e)) }.to_string_lossy().into_owned();
            format!("Could not configure port: ({e}, '{msg}')")
        };
        // SAFETY: termios is plain data; tcgetattr/tcsetattr only access the struct passed in.
        unsafe {
            let mut t: termios = std::mem::zeroed();
            if tcgetattr(self.fd, &mut t) != 0 {
                return Err(fail());
            }
            t.c_cflag |= CLOCAL | CREAD;
            t.c_lflag &= !(ICANON | ECHO | ECHOE | ECHOK | ECHONL | ISIG | IEXTEN | ECHOCTL | ECHOKE);
            t.c_oflag &= !(OPOST | ONLCR | OCRNL);
            t.c_iflag &= !(INLCR | IGNCR | ICRNL | IGNBRK | IUCLC | PARMRK);
            cfsetispeed(&mut t, speed);
            cfsetospeed(&mut t, speed);
            t.c_cflag &= !CSIZE;
            t.c_cflag |= CS8;
            t.c_cflag &= !CSTOPB;
            t.c_iflag &= !(INPCK | ISTRIP);
            t.c_cflag &= !(PARENB | PARODD | CMSPAR);
            t.c_iflag &= !(IXON | IXOFF | IXANY);
            t.c_cflag &= !CRTSCTS;
            t.c_cc[VMIN] = 0;
            t.c_cc[VTIME] = 0;
            if tcsetattr(self.fd, TCSANOW, &t) != 0 {
                return Err(fail());
            }
        }
        Ok(())
    }

    fn modem_line(&self, line: libc::c_int, on: bool) -> Result<(), i32> {
        let request = if on { libc::TIOCMBIS } else { libc::TIOCMBIC };
        // SAFETY: TIOCMBIS/TIOCMBIC read one int from the pointer.
        if unsafe { libc::ioctl(self.fd, request, &line) } != 0 {
            return Err(errno());
        }
        Ok(())
    }
}

/// pyserial ignores EINVAL / ENOTTY when setting DTR and RTS at open (e.g. on a pty).
fn ignore_unsupported(e: i32) -> Result<(), String> {
    if e == libc::EINVAL || e == libc::ENOTTY {
        Ok(())
    } else {
        Err(os_error(e))
    }
}

impl Port for TtyPort {
    fn read(&self, buf: &mut [u8], timeout: Duration) -> Result<usize, String> {
        // pyserial's read(in_waiting or 1) asks for the input queue length first; on a hung-up
        // port (adapter unplugged) that ioctl is what fails, with EIO.
        let mut waiting: libc::c_int = 0;
        // SAFETY: FIONREAD writes one int.
        if unsafe { libc::ioctl(self.fd, libc::FIONREAD, &mut waiting) } != 0 {
            return Err(os_error(errno()));
        }
        let mut pfd = libc::pollfd { fd: self.fd, events: libc::POLLIN, revents: 0 };
        // SAFETY: one valid pollfd.
        let r = unsafe { libc::poll(&mut pfd, 1, timeout.as_millis() as libc::c_int) };
        if r < 0 {
            let e = errno();
            return if retryable(e) { Ok(0) } else { Err(format!("read failed: {}", os_error(e))) };
        }
        if r == 0 {
            return Ok(0);
        }
        // SAFETY: reads into the caller's buffer, at most its length.
        let n = unsafe { libc::read(self.fd, buf.as_mut_ptr().cast(), buf.len()) };
        if n < 0 {
            let e = errno();
            return if retryable(e) { Ok(0) } else { Err(format!("read failed: {}", os_error(e))) };
        }
        if n == 0 {
            return Err("device reports readiness to read but returned no data \
                        (device disconnected or multiple access on port?)"
                .into());
        }
        Ok(n as usize)
    }

    fn write(&self, data: &[u8]) -> Result<(), String> {
        let deadline = Instant::now() + self.write_timeout;
        let mut rest = data;
        while !rest.is_empty() {
            // SAFETY: writes from the caller's slice, at most its length.
            let n = unsafe { libc::write(self.fd, rest.as_ptr().cast(), rest.len()) };
            if n < 0 {
                let e = errno();
                if !retryable(e) {
                    return Err(format!("write failed: {}", os_error(e)));
                }
            } else {
                rest = &rest[n as usize..];
                if rest.is_empty() {
                    break;
                }
            }
            let left = deadline.saturating_duration_since(Instant::now());
            if left.is_zero() {
                return Err("Write timeout".into());
            }
            let mut pfd = libc::pollfd { fd: self.fd, events: libc::POLLOUT, revents: 0 };
            // SAFETY: one valid pollfd.
            if unsafe { libc::poll(&mut pfd, 1, left.as_millis().max(1) as libc::c_int) } == 0 {
                return Err("Write timeout".into());
            }
        }
        // SAFETY: tcdrain on our own fd.
        if unsafe { libc::tcdrain(self.fd) } != 0 {
            return Err(format!("write failed: {}", os_error(errno())));
        }
        Ok(())
    }

    fn set_dtr(&self, on: bool) -> Result<(), String> {
        self.modem_line(libc::TIOCM_DTR, on).map_err(os_error)
    }

    fn reset_input_buffer(&self) -> Result<(), String> {
        // SAFETY: tcflush on our own fd.
        if unsafe { libc::tcflush(self.fd, libc::TCIFLUSH) } != 0 {
            return Err(os_error(errno()));
        }
        Ok(())
    }
}

pub fn tty_opener() -> Opener {
    Box::new(|path, baud| TtyPort::open(path, baud).map(|p| Arc::new(p) as Arc<dyn Port>))
}

// ------------------------------------------------------------------ stream supervision (pure)
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Layout {
    Primary,
    Secondary,
}

/// Which stream layout is requested and when to swap the secondary one in (_rotate_stream).
#[derive(Clone, Debug, Default)]
pub struct Rotation {
    pub stream_ids: Vec<u8>,
    pub secondary_ids: Option<Vec<u8>>,
    pub secondary_n: Option<usize>,
    pub secondary_period: f64,
    pub secondary_sent: f64,
    pub awaiting_secondary: bool,
    pub stream_wanted: bool,
}

impl Rotation {
    /// Swap the secondary layout in for one frame every secondary_period seconds: the layout to
    /// request now, if any.
    pub fn rotate(&mut self, now: f64, got_secondary: bool, running: bool) -> Option<Layout> {
        if self.secondary_ids.is_none() || !self.stream_wanted || !running {
            return None;
        }
        if self.awaiting_secondary {
            if got_secondary || now - self.secondary_sent > 0.1 {
                self.awaiting_secondary = false;
                return Some(Layout::Primary);
            }
        } else if now - self.secondary_sent > self.secondary_period {
            self.secondary_sent = now;
            self.awaiting_secondary = true;
            return Some(Layout::Secondary);
        }
        None
    }
}

/// What to do when the stream has gone quiet.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Restart {
    /// Log the (30 s throttled) stall warning.
    pub warn: bool,
    /// Pulse BRC through DTR first.
    pub wake: bool,
    /// Send Start (128) before the stream request: the OI was restarted (rebooted / woke up).
    pub start: bool,
}

/// Stream stall detection of the reader loop.
#[derive(Clone, Debug)]
pub struct StallMonitor {
    pub last_frame: f64,
    last_restart: f64,
    last_wake: f64,
    last_stall_warning: f64,
    /// Stream restarts since the last good frame.
    pub restarts: u32,
}

impl StallMonitor {
    pub fn new(now: f64) -> Self {
        Self { last_frame: now, last_restart: 0.0, last_wake: 0.0, last_stall_warning: 0.0, restarts: 0 }
    }

    pub fn frame(&mut self, now: f64) {
        self.last_frame = now;
        self.restarts = 0;
    }

    pub fn check(&mut self, now: f64, stream_wanted: bool, timeout: f64, wake_with_dtr: bool) -> Option<Restart> {
        if !(stream_wanted && now - self.last_frame > timeout && now - self.last_restart > 1.0) {
            return None;
        }
        self.last_restart = now;
        self.restarts += 1;
        let warn = self.restarts > 1 && now - self.last_stall_warning > 30.0;
        if warn {
            self.last_stall_warning = now;
        }
        let wake = wake_with_dtr && self.restarts > 1 && now - self.last_wake > 5.0;
        if wake {
            self.last_wake = now;
        }
        // Mode changes pause the stream for ~0.3-0.5 s, so first just ask for it again: Start
        // (128) would also drop Safe/Full to Passive. Only if that fails is the OI restarted.
        Some(Restart { warn, wake, start: self.restarts > 1 })
    }
}

// ------------------------------------------------------------------ the robot
pub type LogFn = Arc<dyn Fn(Level, &str) + Send + Sync>;
/// callback(values of one frame, monotonic time, robot), called from the reader thread.
pub type FrameCallback = Arc<dyn Fn(&SensorValues, f64, &Roomba) + Send + Sync>;

struct Io {
    ser: Option<Arc<dyn Port>>,
    rot: Rotation,
}

impl Io {
    fn write(&self, data: &[u8]) -> Result<(), String> {
        let ser = self.ser.as_ref().ok_or("serial port is not open")?;
        ser.write(data).map_err(|e| format!("serial write failed: {e}"))
    }

    fn request_stream(&mut self) -> Result<(), String> {
        self.rot.awaiting_secondary = false;
        let cmd = oi::stream(&self.rot.stream_ids)?;
        self.write(&cmd)
    }
}

struct Frames {
    latest: SensorValues,
    count: u64,
    last_time: f64,
}

pub struct Roomba {
    pub port: String,
    pub baud: u32,
    pub stream_timeout: f64,
    wake_with_dtr: AtomicBool,
    log: LogFn,
    opener: Opener,
    io: Mutex<Io>,
    frames: Mutex<Frames>,
    frame_cv: Condvar,
    callbacks: Mutex<Vec<FrameCallback>>,
    parser: Mutex<StreamParser>,
    running: AtomicBool,
    reader: Mutex<Option<JoinHandle<()>>>,
    reader_done: Arc<(Mutex<bool>, Condvar)>,
}

impl Roomba {
    pub fn new(port: &str, baud: u32, log: LogFn, opener: Opener) -> Arc<Self> {
        Arc::new(Self {
            port: port.into(),
            baud,
            stream_timeout: 0.5,
            wake_with_dtr: AtomicBool::new(false),
            log,
            opener,
            io: Mutex::new(Io { ser: None, rot: Rotation { secondary_period: 0.2, ..Default::default() } }),
            frames: Mutex::new(Frames { latest: SensorValues::default(), count: 0, last_time: 0.0 }),
            frame_cv: Condvar::new(),
            callbacks: Mutex::new(Vec::new()),
            parser: Mutex::new(StreamParser::default()),
            running: AtomicBool::new(false),
            reader: Mutex::new(None),
            reader_done: Arc::new((Mutex::new(true), Condvar::new())),
        })
    }

    fn warn(&self, msg: &str) {
        (self.log)(Level::Warn, msg)
    }

    pub fn set_wake_with_dtr(&self, on: bool) {
        self.wake_with_dtr.store(on, Ordering::Relaxed)
    }

    // ------------------------------------------------------------ connection
    pub fn open(&self) -> Result<(), String> {
        let mut io = lock(&self.io);
        if io.ser.is_none() {
            io.ser = Some((self.opener)(&self.port, self.baud)?);
        }
        Ok(())
    }

    pub fn close(&self) {
        self.stop_stream();
        lock(&self.io).ser = None;
    }

    pub fn write(&self, data: &[u8]) -> Result<(), String> {
        lock(&self.io).write(data)
    }

    /// Pulse BRC (Mini-DIN pin 5) low through the adapter's DTR line (needs DTR wired to pin 5).
    /// Pulses of 50-500 ms in quick succession change the robot's baud rate, so keep this ~1 s.
    pub fn wake_for(&self, pulse_s: f64) -> Result<(), String> {
        {
            let io = lock(&self.io);
            let ser = io.ser.as_ref().ok_or("serial port is not open")?;
            ser.set_dtr(true).map_err(|e| format!("wake pulse failed: {e}"))?;
        }
        sleep(pulse_s);
        let io = lock(&self.io);
        if let Some(ser) = &io.ser {
            ser.set_dtr(false).map_err(|e| format!("wake pulse failed: {e}"))?;
        }
        Ok(())
    }

    // ------------------------------------------------------------ modes
    /// Start the OI / return to Passive mode.
    pub fn start(&self) -> Result<(), String> {
        self.write(&[oi::START])
    }

    /// Opcode 173: stop the OI (mode Off). Not supported by every 500-series firmware.
    pub fn stop_oi(&self) -> Result<(), String> {
        let mut io = lock(&self.io);
        io.rot.stream_wanted = false;
        io.write(&[oi::STOP])
    }

    /// Last OI mode reported by the sensor stream (None if not streaming yet).
    pub fn mode(&self) -> Option<i32> {
        lock(&self.frames).latest.get(Field::OiMode)
    }

    /// Send the opcode(s) for a mode change without waiting for confirmation.
    pub fn request_mode(&self, mode: u8) -> Result<(), String> {
        if mode == oi::MODE_OFF {
            return self.stop_oi();
        }
        if !(oi::MODE_PASSIVE..=oi::MODE_FULL).contains(&mode) {
            return Err(format!("unknown OI mode {mode}"));
        }
        let resume_stream = self.streaming() && !lock(&self.io).rot.stream_wanted;
        if mode == oi::MODE_PASSIVE || matches!(self.mode(), None | Some(0)) || resume_stream {
            self.start()?;
            sleep(0.02);
        }
        if mode == oi::MODE_SAFE {
            self.write(&[oi::SAFE])?;
        } else if mode == oi::MODE_FULL {
            self.write(&[oi::FULL])?;
        }
        if resume_stream {
            let mut io = lock(&self.io);
            io.rot.stream_wanted = true;
            io.request_stream()?;
        }
        Ok(())
    }

    /// Change mode and return the mode the robot reports afterwards.
    pub fn set_mode(&self, mode: u8, timeout: f64) -> Result<Option<i32>, String> {
        self.request_mode(mode)?;
        if mode == oi::MODE_OFF {
            return Ok(Some(0));
        }
        if self.streaming() {
            self.wait_for(|v| v.get(Field::OiMode) == Some(mode as i32), timeout);
            return Ok(self.mode());
        }
        sleep(0.05);
        Ok(self.sensors(35, 0.5)?.get(Field::OiMode))
    }

    // ------------------------------------------------------------ polled sensors (stream stopped)
    /// Opcode 142: read one packet or group and return its decoded values.
    pub fn sensors(&self, packet_id: u8, timeout: f64) -> Result<SensorValues, String> {
        if self.streaming() {
            return Err("stop the sensor stream before polling".into());
        }
        let request = oi::sensors(packet_id)?;
        let size = oi::packet_size(packet_id)?;
        let io = lock(&self.io);
        let ser = io.ser.clone().ok_or("serial port is not open")?;
        ser.reset_input_buffer().map_err(|e| format!("serial read failed: {e}"))?;
        io.write(&request)?;
        let mut data = vec![0u8; size];
        let mut got = 0;
        let deadline = Instant::now() + Duration::from_secs_f64(timeout);
        while got < size && Instant::now() < deadline {
            got += ser.read(&mut data[got..], Duration::from_millis(50)).map_err(|e| format!("serial read failed: {e}"))?;
        }
        if got != size {
            return Err(format!("expected {size} bytes, got {got} (robot asleep or not connected?)"));
        }
        oi::decode(packet_id, &data)
    }

    // ------------------------------------------------------------ streamed sensors
    pub fn streaming(&self) -> bool {
        self.running.load(Ordering::SeqCst)
    }

    pub fn add_frame_callback(&self, callback: FrameCallback) {
        lock(&self.callbacks).push(callback);
    }

    /// The driver's primary + secondary layouts, which together cover every sensor.
    pub fn start_stream(self: &Arc<Self>) -> Result<(), String> {
        self.start_stream_with(&oi::STREAM_PRIMARY, Some(&oi::STREAM_SECONDARY), 0.2)
    }

    /// Opcode 148: have the robot send the given packets every 15 ms. With secondary ids, the
    /// stream briefly switches to that layout every secondary_period seconds (one frame) and then
    /// back, so more sensors can be covered with small frames.
    pub fn start_stream_with(self: &Arc<Self>, ids: &[u8], secondary: Option<&[u8]>, secondary_period: f64) -> Result<(), String> {
        let mut layouts = vec![ids];
        layouts.extend(secondary);
        for layout in &layouts {
            for &pid in layout.iter() {
                oi::packet_range(pid)?;
            }
            if oi::stream_payload_length(layout)? + 3 > oi::MAX_STREAM_FRAME {
                return Err("too much data to stream every 15 ms at 115200 baud".into());
            }
        }
        if layouts.len() > 1 && oi::stream_payload_length(layouts[0])? == oi::stream_payload_length(layouts[1])? {
            return Err("primary and secondary stream layouts must differ in size".into());
        }
        {
            let mut io = lock(&self.io);
            io.rot.stream_ids = ids.to_vec();
            io.rot.secondary_ids = secondary.map(<[u8]>::to_vec);
            io.rot.secondary_n = match secondary {
                Some(s) => Some(oi::stream_payload_length(s)?),
                None => None,
            };
            io.rot.secondary_period = secondary_period;
            *lock(&self.parser) = StreamParser::new(&layouts)?;
            io.rot.stream_wanted = true;
            io.request_stream()?;
        }
        if !self.running.swap(true, Ordering::SeqCst) {
            *lock(&self.reader_done.0) = false;
            let me = Arc::clone(self);
            let handle = std::thread::Builder::new()
                .name("roomba-stream".into())
                .spawn(move || {
                    me.reader_loop();
                    let (done, cv) = &*me.reader_done;
                    *lock(done) = true;
                    cv.notify_all();
                })
                .map_err(|e| e.to_string())?;
            *lock(&self.reader) = Some(handle);
        }
        Ok(())
    }

    pub fn pause_stream(&self) -> Result<(), String> {
        let mut io = lock(&self.io);
        io.rot.stream_wanted = false;
        io.write(&oi::pause_resume_stream(false))
    }

    pub fn resume_stream(&self) -> Result<(), String> {
        let mut io = lock(&self.io);
        io.rot.stream_wanted = true;
        io.write(&oi::pause_resume_stream(true))
    }

    pub fn stop_stream(&self) {
        if !self.streaming() {
            return;
        }
        {
            let mut io = lock(&self.io);
            io.rot.stream_wanted = false;
            self.running.store(false, Ordering::SeqCst);
            let _ = io.write(&oi::pause_resume_stream(false));
        }
        let handle = lock(&self.reader).take();
        if let Some(handle) = handle {
            if handle.thread().id() != std::thread::current().id() {
                // join(timeout=1.0)
                let (done, cv) = &*self.reader_done;
                let guard = lock(done);
                let (guard, _) = cv.wait_timeout_while(guard, Duration::from_secs(1), |d| !*d).unwrap_or_else(|e| e.into_inner());
                if *guard {
                    drop(guard);
                    let _ = handle.join();
                }
            }
        }
        let io = lock(&self.io);
        if let Some(ser) = &io.ser {
            let _ = ser.reset_input_buffer();
        }
    }

    /// (values, monotonic_time) of the most recent stream frame, accumulated over all layouts.
    pub fn latest(&self) -> (SensorValues, f64) {
        let f = lock(&self.frames);
        (f.latest, f.last_time)
    }

    /// Wait for a new stream frame whose accumulated values satisfy predicate.
    pub fn wait_for(&self, predicate: impl Fn(&SensorValues) -> bool, timeout: f64) -> bool {
        let deadline = Instant::now() + Duration::from_secs_f64(timeout.max(0.0));
        let mut f = lock(&self.frames);
        let seen = f.count;
        loop {
            if f.count > seen && predicate(&f.latest) {
                return true;
            }
            let now = Instant::now();
            if now >= deadline {
                return false;
            }
            f = self.frame_cv.wait_timeout(f, deadline - now).unwrap_or_else(|e| e.into_inner()).0;
        }
    }

    pub fn bad_frames(&self) -> u64 {
        lock(&self.parser).bad_frames
    }

    fn rotate_stream(&self, now: f64, got_secondary: bool) -> Result<(), String> {
        // Under the io lock, so a concurrent pause/stop can't be undone by a stream request.
        let mut io = lock(&self.io);
        match io.rot.rotate(now, got_secondary, self.streaming()) {
            Some(Layout::Primary) => io.request_stream(),
            Some(Layout::Secondary) => {
                let cmd = oi::stream(io.rot.secondary_ids.as_deref().unwrap_or_default())?;
                io.write(&cmd)
            }
            None => Ok(()),
        }
    }

    fn deliver(&self, values: &SensorValues, stamp: f64) {
        {
            let mut f = lock(&self.frames);
            f.latest.update(values);
            f.count += 1;
            f.last_time = stamp;
        }
        self.frame_cv.notify_all();
        let callbacks = lock(&self.callbacks).clone();
        for callback in callbacks {
            // Keep the reader alive whatever a callback does.
            if let Err(e) = catch_unwind(AssertUnwindSafe(|| callback(values, stamp, self))) {
                let what = e
                    .downcast_ref::<String>()
                    .cloned()
                    .or_else(|| e.downcast_ref::<&str>().map(|s| s.to_string()))
                    .unwrap_or_default();
                (self.log)(Level::Error, &format!("sensor callback failed: {what}"));
            }
        }
    }

    fn reader_loop(&self) {
        let mut stall = StallMonitor::new(mono());
        let mut buf = vec![0u8; 4096];
        while self.streaming() {
            let ser = lock(&self.io).ser.clone();
            let Some(ser) = ser else {
                self.reconnect();
                stall.last_frame = mono();
                continue;
            };
            let n = match ser.read(&mut buf, Duration::from_millis(50)) {
                Ok(n) => n,
                Err(e) => {
                    (self.log)(Level::Error, &format!("serial read failed ({e}); reconnecting"));
                    self.drop_port();
                    continue;
                }
            };
            drop(ser);
            let now = mono();
            let mut got_secondary = false;
            if n > 0 {
                let frames = lock(&self.parser).feed_frames(&buf[..n]);
                let secondary_n = lock(&self.io).rot.secondary_n;
                for (frame_n, values) in frames {
                    stall.frame(now);
                    got_secondary |= Some(frame_n) == secondary_n;
                    self.deliver(&values, now);
                }
            }
            if let Err(e) = self.rotate_stream(now, got_secondary) {
                self.warn(&e);
            }
            let wanted = lock(&self.io).rot.stream_wanted;
            let Some(restart) = stall.check(now, wanted, self.stream_timeout, self.wake_with_dtr.load(Ordering::Relaxed))
            else {
                continue;
            };
            if restart.warn {
                self.warn("no sensor data from the Roomba (asleep?); restarting the stream");
            }
            let result = (|| {
                if restart.wake {
                    self.wake_for(1.0)?;
                }
                let mut io = lock(&self.io);
                if io.rot.stream_wanted && self.streaming() {
                    if restart.start {
                        io.write(&[oi::START])?;
                        sleep(0.02);
                    }
                    io.request_stream()?;
                }
                Ok::<(), String>(())
            })();
            if let Err(e) = result {
                self.warn(&e);
            }
        }
    }

    fn drop_port(&self) {
        lock(&self.io).ser = None;
    }

    fn reconnect(&self) {
        sleep(1.0);
        if !self.streaming() {
            return;
        }
        {
            let mut io = lock(&self.io);
            match (self.opener)(&self.port, self.baud) {
                Ok(port) => io.ser = Some(port),
                Err(_) => return,
            }
        }
        (self.log)(Level::Info, &format!("reopened {}", self.port));
        let wanted = {
            let io = lock(&self.io);
            let mut layouts: Vec<&[u8]> = vec![&io.rot.stream_ids];
            layouts.extend(io.rot.secondary_ids.as_deref());
            if let Ok(p) = StreamParser::new(&layouts) {
                *lock(&self.parser) = p;
            }
            io.rot.stream_wanted
        };
        if wanted {
            let result = self.start().and_then(|_| {
                sleep(0.02);
                lock(&self.io).request_stream()
            });
            if let Err(e) = result {
                self.warn(&e);
            }
        }
    }
}

impl Link for Roomba {
    fn write(&self, data: &[u8]) -> Result<(), String> {
        Roomba::write(self, data)
    }
    fn is_open(&self) -> bool {
        lock(&self.io).ser.is_some()
    }
    fn streaming(&self) -> bool {
        Roomba::streaming(self)
    }
    fn wake_with_dtr(&self) -> bool {
        self.wake_with_dtr.load(Ordering::Relaxed)
    }
    fn wake(&self) -> Result<(), String> {
        self.wake_for(1.0)
    }
    fn request_mode(&self, mode: u8) -> Result<(), String> {
        Roomba::request_mode(self, mode)
    }
    fn set_mode(&self, mode: u8, timeout: f64) -> Result<Option<i32>, String> {
        Roomba::set_mode(self, mode, timeout)
    }
    fn wait_for_frame(&self, timeout: f64) -> bool {
        self.wait_for(|_| true, timeout)
    }
    fn latest(&self) -> SensorValues {
        Roomba::latest(self).0
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::VecDeque;

    #[test]
    fn rotation_swaps_in_the_secondary_layout_once_per_period() {
        let mut r = Rotation { secondary_ids: Some(vec![7]), secondary_period: 0.2, stream_wanted: true, ..Default::default() };
        assert_eq!(r.rotate(100.0, false, true), Some(Layout::Secondary));
        assert_eq!(r.rotate(100.05, false, true), None);
        assert_eq!(r.rotate(100.06, true, true), Some(Layout::Primary)); // the secondary frame arrived
        assert_eq!(r.rotate(100.15, false, true), None);
        assert_eq!(r.rotate(100.21, false, true), Some(Layout::Secondary));
        assert_eq!(r.rotate(100.32, false, true), Some(Layout::Primary)); // no secondary frame within 0.1 s
        r.stream_wanted = false;
        assert_eq!(r.rotate(101.0, false, true), None);
        r.stream_wanted = true;
        assert_eq!(r.rotate(101.0, false, false), None);
        r.secondary_ids = None;
        assert_eq!(r.rotate(101.0, false, true), None);
    }

    #[test]
    fn stall_monitor_requests_then_restarts_the_oi() {
        let mut s = StallMonitor::new(1000.0);
        assert_eq!(s.check(1000.4, true, 0.5, false), None);
        assert_eq!(s.check(1000.6, false, 0.5, false), None); // stream not wanted
        assert_eq!(s.check(1000.6, true, 0.5, false), Some(Restart { warn: false, wake: false, start: false }));
        assert_eq!(s.check(1001.0, true, 0.5, false), None); // at most once a second
        assert_eq!(s.check(1001.7, true, 0.5, true), Some(Restart { warn: true, wake: true, start: true }));
        assert_eq!(s.check(1002.8, true, 0.5, true), Some(Restart { warn: false, wake: false, start: true }));
        assert_eq!(s.check(1006.9, true, 0.5, true), Some(Restart { warn: false, wake: true, start: true }));
        s.frame(1007.0);
        assert_eq!(s.restarts, 0);
        assert_eq!(s.check(1007.4, true, 0.5, true), None);
        assert_eq!(s.check(1035.0, true, 0.5, false).unwrap().start, false);
        assert_eq!(s.check(1036.1, true, 0.5, false), Some(Restart { warn: true, wake: false, start: true }));
    }

    /// A pretend robot: hands out queued reads and records writes.
    #[derive(Default)]
    struct MockPort {
        reads: Mutex<VecDeque<Result<Vec<u8>, String>>>,
        writes: Mutex<Vec<Vec<u8>>>,
    }

    impl Port for MockPort {
        fn read(&self, buf: &mut [u8], timeout: Duration) -> Result<usize, String> {
            match lock(&self.reads).pop_front() {
                Some(Ok(data)) => {
                    buf[..data.len()].copy_from_slice(&data);
                    Ok(data.len())
                }
                Some(Err(e)) => Err(e),
                None => {
                    std::thread::sleep(timeout);
                    Ok(0)
                }
            }
        }
        fn write(&self, data: &[u8]) -> Result<(), String> {
            lock(&self.writes).push(data.to_vec());
            Ok(())
        }
        fn set_dtr(&self, _on: bool) -> Result<(), String> {
            Err(os_error(libc::ENOTTY))
        }
        fn reset_input_buffer(&self) -> Result<(), String> {
            Ok(())
        }
    }

    fn frame(payload: &[u8]) -> Vec<u8> {
        let mut body = vec![oi::STREAM_HEADER, payload.len() as u8];
        body.extend_from_slice(payload);
        let sum = body.iter().fold(0u8, |a, &b| a.wrapping_add(b));
        body.push(sum.wrapping_neg());
        body
    }

    fn robot(port: Arc<MockPort>) -> (Arc<Roomba>, Arc<Mutex<Vec<String>>>) {
        let logs = Arc::new(Mutex::new(Vec::new()));
        let l = Arc::clone(&logs);
        let opened = Mutex::new(Some(port));
        let opener: Opener = Box::new(move |_, _| match lock(&opened).take() {
            Some(p) => Ok(p as Arc<dyn Port>),
            None => Err("[Errno 2] could not open port x".into()),
        });
        (Roomba::new("/dev/fake", 115200, Arc::new(move |_, m| lock(&l).push(m.to_string())), opener), logs)
    }

    #[test]
    fn stream_frames_reach_callbacks_and_waiters() {
        let port = Arc::new(MockPort::default());
        let primary = frame(&[35, 2, 43, 0, 10, 44, 0, 20]);
        lock(&port.reads).extend([Ok(primary[..4].to_vec()), Ok(primary[4..].to_vec())]);
        let (r, _) = robot(Arc::clone(&port));
        let seen = Arc::new(Mutex::new(Vec::new()));
        let s = Arc::clone(&seen);
        r.add_frame_callback(Arc::new(move |v: &SensorValues, _t, _r: &Roomba| lock(&s).push(v.int(Field::LeftEncoderCounts))));
        r.open().unwrap();
        r.start_stream_with(&[35, 43, 44], Some(&[35]), 0.2).unwrap();
        assert!(Link::wait_for_frame(&*r, 1.0) || r.latest().0.get(Field::OiMode) == Some(2));
        assert_eq!(r.mode(), Some(2));
        assert_eq!(*lock(&seen), [10]);
        r.close();
        let writes = lock(&port.writes).clone();
        assert_eq!(writes[0], [148, 3, 35, 43, 44]);
        assert_eq!(writes[1], [148, 1, 35]); // the secondary layout is swapped in right away
        assert_eq!(writes.last().unwrap(), &vec![150, 0]);
        assert!(!r.streaming() && !Link::is_open(&*r));
    }

    #[test]
    fn read_error_drops_the_port_and_reconnects() {
        let port = Arc::new(MockPort::default());
        lock(&port.reads).push_back(Err("read failed: [Errno 5] Input/output error".into()));
        let logs = Arc::new(Mutex::new(Vec::<String>::new()));
        let l = Arc::clone(&logs);
        let p = Arc::clone(&port);
        let r = Roomba::new(
            "/dev/fake",
            115200,
            Arc::new(move |_, m| lock(&l).push(m.to_string())),
            Box::new(move |_, _| Ok(Arc::clone(&p) as Arc<dyn Port>)),
        );
        r.open().unwrap();
        r.start_stream_with(&[35], None, 0.2).unwrap();
        std::thread::sleep(Duration::from_millis(1300));
        r.close();
        let logs = lock(&logs).clone();
        assert_eq!(logs[0], "serial read failed (read failed: [Errno 5] Input/output error); reconnecting");
        assert_eq!(logs[1], "reopened /dev/fake");
        let writes = lock(&port.writes).clone();
        assert_eq!(&writes[..3], [vec![148, 1, 35], vec![128], vec![148, 1, 35]]);
    }

    #[test]
    fn errors_match_python() {
        let (r, _) = robot(Arc::new(MockPort::default()));
        assert_eq!(r.write(&[128]).unwrap_err(), "serial port is not open");
        assert_eq!(r.wake_for(0.0).unwrap_err(), "serial port is not open");
        r.open().unwrap();
        assert_eq!(r.wake_for(0.0).unwrap_err(), "wake pulse failed: [Errno 25] Inappropriate ioctl for device");
        assert_eq!(r.request_mode(7).unwrap_err(), "unknown OI mode 7");
        assert_eq!(os_error(libc::EIO), "[Errno 5] Input/output error");
        let e = TtyPort::open("/nonexistent/ttyX", 115200).err().unwrap();
        assert_eq!(e, "[Errno 2] could not open port /nonexistent/ttyX: [Errno 2] No such file or directory: '/nonexistent/ttyX'");
    }

    #[test]
    fn polling_reads_one_packet() {
        let port = Arc::new(MockPort::default());
        lock(&port.reads).push_back(Ok(vec![2]));
        let (r, _) = robot(Arc::clone(&port));
        r.open().unwrap();
        assert_eq!(r.sensors(35, 0.5).unwrap().get(Field::OiMode), Some(2));
        assert_eq!(lock(&port.writes)[0], [142, 35]);
        assert_eq!(r.sensors(35, 0.1).unwrap_err(), "expected 1 bytes, got 0 (robot asleep or not connected?)");
    }
}
