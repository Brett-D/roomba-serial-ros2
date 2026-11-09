"""Thread-safe serial connection to an iRobot Roomba 500-series robot (Open Interface).

Works without ROS:

    from roomba_driver.roomba import Roomba
    with Roomba('/dev/ttyUSB0') as robot:
        robot.start()
        print(robot.sensors(3))          # battery packet
        robot.safe()
        robot.display_text('HI')
"""
import logging
import threading
import time

import serial

from roomba_driver import oi


class RoombaError(RuntimeError):
    pass


class Roomba:

    def __init__(self, port, baud=115200, logger=None, stream_timeout=0.5):
        self.port = port
        self.baud = baud
        self.stream_timeout = stream_timeout
        self.wake_with_dtr = False
        self.log = logger or logging.getLogger('roomba')
        self._ser = None
        self._io_lock = threading.RLock()
        self._frame_cv = threading.Condition()
        self._latest = {}
        self._frame_count = 0
        self._last_frame_time = 0.0
        self._callbacks = []
        self._stream_ids = None
        self._secondary_ids = None
        self._secondary_n = None
        self._secondary_sent = 0.0
        self._awaiting_secondary = False
        self.secondary_period = 0.2
        self._stream_wanted = False
        self._parser = None
        self._reader = None
        self._running = False

    # ------------------------------------------------------------ connection
    def open(self):
        with self._io_lock:
            if self._ser is None:
                self._ser = self._open_port()

    def _open_port(self):
        ser = serial.Serial()
        ser.port = self.port
        ser.baudrate = self.baud
        ser.timeout = 0.05
        ser.write_timeout = 0.5
        # De-assert DTR/RTS (TTL high) so an optional DTR->BRC wire stays idle.
        ser.dtr = False
        ser.rts = False
        ser.open()
        return ser

    @property
    def is_open(self):
        ser = self._ser
        return ser is not None and ser.is_open

    def close(self):
        self.stop_stream()
        with self._io_lock:
            if self._ser is not None:
                try:
                    self._ser.close()
                except (serial.SerialException, OSError):
                    pass
                self._ser = None

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *exc):
        self.close()

    def write(self, data):
        with self._io_lock:
            if not self.is_open:
                raise RoombaError('serial port is not open')
            try:
                self._ser.write(data)
                self._ser.flush()
            except (serial.SerialException, OSError) as e:
                raise RoombaError(f'serial write failed: {e}') from e

    def wake(self, pulse_s=1.0):
        """Pulse BRC (Mini-DIN pin 5) low through the adapter's DTR line (needs DTR wired to pin 5).

        Pulses of 50-500 ms in quick succession change the robot's baud rate, so keep this ~1 s.
        """
        try:
            with self._io_lock:
                if not self.is_open:
                    raise RoombaError('serial port is not open')
                self._ser.dtr = True
            time.sleep(pulse_s)
            with self._io_lock:
                if self.is_open:
                    self._ser.dtr = False
        except (serial.SerialException, OSError) as e:
            raise RoombaError(f'wake pulse failed: {e}') from e

    # ------------------------------------------------------------ modes
    def start(self):
        """Start the OI / return to Passive mode."""
        self.write(oi.cmd(oi.START))

    passive = start

    def control(self):
        self.write(oi.cmd(oi.CONTROL))

    def safe(self):
        self.write(oi.cmd(oi.SAFE))

    def full(self):
        self.write(oi.cmd(oi.FULL))

    def stop_oi(self):
        """Opcode 173: stop the OI (mode Off). Not supported by every 500-series firmware."""
        self._stream_wanted = False
        self.write(oi.cmd(oi.STOP))

    def reset(self):
        """Opcode 7: reboot the robot, as if the battery had been removed and reinserted."""
        self.write(oi.cmd(oi.RESET))

    def set_baud(self, rate):
        """Change the robot's baud rate and follow it locally. Lasts until the robot loses power."""
        with self._io_lock:
            self.write(oi.baud(rate))
            time.sleep(0.1)
            try:
                self._ser.baudrate = rate
            except (serial.SerialException, OSError) as e:
                raise RoombaError(f'cannot switch the port to {rate} baud: {e}') from e
            self.baud = rate

    @property
    def mode(self):
        """Last OI mode reported by the sensor stream (None if not streaming yet)."""
        with self._frame_cv:
            return self._latest.get('oi_mode')

    def request_mode(self, mode):
        """Send the opcode(s) for a mode change without waiting for confirmation."""
        if mode == oi.MODE_OFF:
            self.stop_oi()
            return
        if mode not in (oi.MODE_PASSIVE, oi.MODE_SAFE, oi.MODE_FULL):
            raise ValueError(f'unknown OI mode {mode}')
        resume_stream = self._running and not self._stream_wanted
        if mode == oi.MODE_PASSIVE or self.mode in (None, oi.MODE_OFF) or resume_stream:
            self.start()
            time.sleep(0.02)
        if mode == oi.MODE_SAFE:
            self.safe()
        elif mode == oi.MODE_FULL:
            self.full()
        if resume_stream:
            self._stream_wanted = True
            self._request_stream()

    def set_mode(self, mode, timeout=1.0):
        """Change mode and return the mode the robot reports afterwards."""
        self.request_mode(mode)
        if mode == oi.MODE_OFF:
            return oi.MODE_OFF
        if self._running:
            self.wait_for(lambda v: v.get('oi_mode') == mode, timeout)
            return self.mode
        time.sleep(0.05)
        return self.sensors(35)['oi_mode']

    # ------------------------------------------------------------ cleaning behaviours (-> Passive)
    def power_off(self):
        self.write(oi.cmd(oi.POWER))

    def spot(self):
        self.write(oi.cmd(oi.SPOT))

    def clean(self):
        self.write(oi.cmd(oi.CLEAN))

    def max_clean(self):
        self.write(oi.cmd(oi.MAX))

    def seek_dock(self):
        self.write(oi.cmd(oi.SEEK_DOCK))

    # ------------------------------------------------------------ actuators (Safe / Full)
    def drive(self, velocity_mm_s, radius_mm):
        self.write(oi.drive(velocity_mm_s, radius_mm))

    def drive_straight(self, velocity_mm_s):
        self.drive(velocity_mm_s, oi.RADIUS_STRAIGHT)

    def turn_in_place(self, velocity_mm_s, clockwise=False):
        self.drive(velocity_mm_s, oi.RADIUS_TURN_CW if clockwise else oi.RADIUS_TURN_CCW)

    def drive_direct(self, left_mm_s, right_mm_s):
        self.write(oi.drive_direct(left_mm_s, right_mm_s))

    def drive_pwm(self, left_pwm, right_pwm):
        self.write(oi.drive_pwm(left_pwm, right_pwm))

    def stop_drive(self):
        self.drive_direct(0, 0)

    def motors(self, side_brush=False, vacuum=False, main_brush=False,
               side_brush_clockwise=False, main_brush_outward=False):
        self.write(oi.motors(side_brush, vacuum, main_brush, side_brush_clockwise, main_brush_outward))

    def pwm_motors(self, main_brush=0, side_brush=0, vacuum=0):
        self.write(oi.pwm_motors(main_brush, side_brush, vacuum))

    def leds(self, debris=False, spot=False, dock=False, check_robot=False,
             power_color=0, power_intensity=0):
        self.write(oi.leds(debris, spot, dock, check_robot, power_color, power_intensity))

    def scheduling_leds(self, weekdays=0, schedule=False, clock=False, am=False, pm=False, colon=False):
        self.write(oi.scheduling_leds(weekdays, schedule, clock, am, pm, colon))

    def digit_leds_raw(self, segments):
        self.write(oi.digit_leds_raw(segments))

    def display_text(self, text):
        self.write(oi.digit_leds_ascii(text))

    def song(self, number, notes):
        self.write(oi.song(number, notes))

    def play(self, number):
        self.write(oi.play(number))

    # ------------------------------------------------------------ buttons, schedule, clock
    def buttons(self, mask):
        self.write(oi.buttons(mask))

    def schedule(self, days, times):
        self.write(oi.schedule(days, times))

    def set_day_time(self, day, hour, minute):
        self.write(oi.set_day_time(day, hour, minute))

    # ------------------------------------------------------------ polled sensors (stream stopped)
    def sensors(self, packet_id, timeout=0.5):
        """Opcode 142: read one packet or group and return its decoded values."""
        return self._poll(oi.sensors(packet_id), [packet_id], timeout)

    def query_list(self, ids, timeout=0.5):
        """Opcode 149: read several packets in one request."""
        return self._poll(oi.query_list(ids), list(ids), timeout)

    def _poll(self, request, ids, timeout):
        if self._running:
            raise RoombaError('stop the sensor stream before polling')
        size = sum(oi.packet_size(pid) for pid in ids)
        with self._io_lock:
            if not self.is_open:
                raise RoombaError('serial port is not open')
            try:
                self._ser.reset_input_buffer()
                self.write(request)
                data = b''
                deadline = time.monotonic() + timeout
                while len(data) < size and time.monotonic() < deadline:
                    data += self._ser.read(size - len(data))
            except (serial.SerialException, OSError) as e:
                raise RoombaError(f'serial read failed: {e}') from e
        if len(data) != size:
            raise RoombaError(f'expected {size} bytes, got {len(data)} (robot asleep or not connected?)')
        values = {}
        offset = 0
        for pid in ids:
            n = oi.packet_size(pid)
            values.update(oi.decode(pid, data[offset:offset + n]))
            offset += n
        return values

    # ------------------------------------------------------------ streamed sensors
    @property
    def streaming(self):
        return self._running

    def add_frame_callback(self, callback):
        """callback(values: dict, monotonic_time: float), called from the reader thread."""
        self._callbacks.append(callback)

    def start_stream(self, ids=None, callback=None, secondary_ids=None, secondary_period=0.2):
        """Opcode 148: have the robot send the given packets every 15 ms.

        With secondary_ids, the stream briefly switches to that layout every secondary_period
        seconds (one frame) and then back, so more sensors can be covered with small frames.
        Defaults to the driver's primary/secondary layouts, which together cover every sensor.
        """
        if ids is None:
            ids = oi.STREAM_PRIMARY
            secondary_ids = oi.STREAM_SECONDARY if secondary_ids is None else secondary_ids
        ids = list(ids)
        layouts = [ids] + ([list(secondary_ids)] if secondary_ids else [])
        for layout in layouts:
            for pid in layout:
                oi.packet_ids(pid)
            if oi.stream_payload_length(layout) + 3 > oi.MAX_STREAM_FRAME:
                raise ValueError('too much data to stream every 15 ms at 115200 baud')
        if len(layouts) > 1 and oi.stream_payload_length(layouts[0]) == oi.stream_payload_length(layouts[1]):
            raise ValueError('primary and secondary stream layouts must differ in size')
        if callback is not None:
            self.add_frame_callback(callback)
        self._stream_ids = ids
        self._secondary_ids = layouts[1] if len(layouts) > 1 else None
        self._secondary_n = oi.stream_payload_length(self._secondary_ids) if self._secondary_ids else None
        self.secondary_period = secondary_period
        self._parser = oi.StreamParser(*layouts)
        self._stream_wanted = True
        self._request_stream()
        if not self._running:
            self._running = True
            self._reader = threading.Thread(target=self._reader_loop, name='roomba-stream', daemon=True)
            self._reader.start()

    def pause_stream(self):
        with self._io_lock:
            self._stream_wanted = False
            self.write(oi.pause_resume_stream(False))

    def resume_stream(self):
        with self._io_lock:
            self._stream_wanted = True
            self.write(oi.pause_resume_stream(True))

    def stop_stream(self):
        if not self._running:
            return
        with self._io_lock:
            self._stream_wanted = False
            self._running = False
            try:
                self.write(oi.pause_resume_stream(False))
            except RoombaError:
                pass
        if self._reader is not None and self._reader is not threading.current_thread():
            self._reader.join(timeout=1.0)
        self._reader = None
        with self._io_lock:
            if self.is_open:
                try:
                    self._ser.reset_input_buffer()
                except (serial.SerialException, OSError):
                    pass

    def latest(self):
        """(values, monotonic_time) of the most recent stream frame."""
        with self._frame_cv:
            return dict(self._latest), self._last_frame_time

    def wait_for(self, predicate, timeout):
        """Wait for a new stream frame whose accumulated values satisfy predicate."""
        deadline = time.monotonic() + timeout
        with self._frame_cv:
            seen = self._frame_count
            while True:
                if self._frame_count > seen and predicate(self._latest):
                    return True
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return False
                self._frame_cv.wait(remaining)

    def wait_for_frame(self, timeout=1.0):
        return self.wait_for(lambda _values: True, timeout)

    @property
    def bad_frames(self):
        return self._parser.bad_frames if self._parser else 0

    def _request_stream(self):
        self._awaiting_secondary = False
        self.write(oi.stream(self._stream_ids))

    def _rotate_stream(self, now, got_secondary):
        """Swap the secondary layout in for one frame every secondary_period seconds."""
        with self._io_lock:   # so a concurrent pause/stop can't be undone by a stream request
            if not self._secondary_ids or not self._stream_wanted or not self._running:
                return
            if self._awaiting_secondary:
                if got_secondary or now - self._secondary_sent > 0.1:
                    self._request_stream()
            elif now - self._secondary_sent > self.secondary_period:
                self._secondary_sent = now
                self._awaiting_secondary = True
                self.write(oi.stream(self._secondary_ids))

    def _deliver(self, values, stamp):
        with self._frame_cv:
            self._latest.update(values)
            self._frame_count += 1
            self._last_frame_time = stamp
            self._frame_cv.notify_all()
        for callback in list(self._callbacks):
            try:
                callback(values, stamp)
            except Exception as e:  # keep the reader alive whatever a callback does
                self.log.error(f'sensor callback failed: {e!r}')

    def _reader_loop(self):
        last_frame = time.monotonic()
        last_restart = 0.0
        last_wake = 0.0
        last_stall_warning = 0.0
        restarts = 0                      # stream restarts since the last good frame
        while self._running:
            ser = self._ser
            if ser is None or not ser.is_open:
                self._reconnect()
                last_frame = time.monotonic()
                continue
            try:
                data = ser.read(ser.in_waiting or 1)
            except (serial.SerialException, OSError, TypeError) as e:
                self.log.error(f'serial read failed ({e}); reconnecting')
                self._drop_port()
                continue
            now = time.monotonic()
            got_secondary = False
            for frame_n, values in self._parser.feed_frames(data) if data else ():
                last_frame = now
                restarts = 0
                got_secondary |= frame_n == self._secondary_n
                self._deliver(values, now)
            try:
                self._rotate_stream(now, got_secondary)
            except RoombaError as e:
                self.log.warning(str(e))
            if self._stream_wanted and now - last_frame > self.stream_timeout and now - last_restart > 1.0:
                last_restart = now
                restarts += 1
                if restarts > 1 and now - last_stall_warning > 30.0:
                    last_stall_warning = now
                    self.log.warning('no sensor data from the Roomba (asleep?); restarting the stream')
                try:
                    if self.wake_with_dtr and restarts > 1 and now - last_wake > 5.0:
                        last_wake = now
                        self.wake()
                    with self._io_lock:
                        if self._stream_wanted and self._running:
                            # Mode changes pause the stream for ~0.3-0.5 s, so first just ask
                            # for it again: Start (128) would also drop Safe/Full to Passive.
                            # Only if that fails is the OI restarted (robot rebooted / woke up).
                            if restarts > 1:
                                self.start()
                                time.sleep(0.02)
                            self._request_stream()
                except RoombaError as e:
                    self.log.warning(str(e))

    def _drop_port(self):
        with self._io_lock:
            if self._ser is not None:
                try:
                    self._ser.close()
                except (serial.SerialException, OSError):
                    pass
                self._ser = None

    def _reconnect(self):
        time.sleep(1.0)
        if not self._running:
            return
        try:
            with self._io_lock:
                self._ser = self._open_port()
        except (serial.SerialException, OSError):
            return
        self.log.info(f'reopened {self.port}')
        self._parser = oi.StreamParser(*([self._stream_ids] + ([self._secondary_ids] if self._secondary_ids else [])))
        if self._stream_wanted:
            try:
                self.start()
                time.sleep(0.02)
                self._request_stream()
            except RoombaError as e:
                self.log.warning(str(e))
