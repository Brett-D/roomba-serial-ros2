#!/usr/bin/env python3
"""Generate the wiring / pinout diagrams in this folder (SVG).

    python3 make_diagrams.py

PNG copies were rendered from the SVGs with a headless browser, e.g.
    msedge --headless --screenshot=wiring-usb-ttl.png --window-size=1120,700 wiring-usb-ttl.svg
"""
import base64
import pathlib

HERE = pathlib.Path(__file__).resolve().parent

BLUE, GREEN, BLACK, RED, PURPLE, ORANGE, GREY = (
    '#1e63d6', '#2e9e44', '#222222', '#d32f2f', '#8e44ad', '#ef8f00', '#9e9e9e')
FONT = 'Segoe UI, Helvetica, Arial, sans-serif'


class Svg:
    def __init__(self, width, height):
        self.w, self.h = width, height
        self.parts = [f'<rect width="{width}" height="{height}" fill="#ffffff"/>']

    def add(self, s):
        self.parts.append(s)

    def text(self, x, y, t, size=14, anchor='start', fill='#222', weight='normal'):
        self.add(f'<text x="{x}" y="{y}" font-size="{size}" text-anchor="{anchor}" fill="{fill}" '
                 f'font-weight="{weight}">{t}</text>')

    def box(self, x, y, w, h, fill, stroke, title, sub=None):
        self.add(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="10" fill="{fill}" '
                 f'stroke="{stroke}" stroke-width="2"/>')
        self.text(x + w / 2, y + 28, title, 17, 'middle', weight='bold')
        if sub:
            self.text(x + w / 2, y + 47, sub, 12, 'middle', '#444')

    def wire(self, d, color, width=4, dash=None):
        da = f' stroke-dasharray="{dash}"' if dash else ''
        self.add(f'<path d="{d}" fill="none" stroke="{color}" stroke-width="{width}" '
                 f'stroke-linecap="round" stroke-linejoin="round"{da}/>')

    def dot(self, x, y, stroke='#333'):
        self.add(f'<circle cx="{x}" cy="{y}" r="6" fill="#fff" stroke="{stroke}" stroke-width="2"/>')

    def section(self, y, title, sub=''):
        self.add(f'<rect x="30" y="{y}" width="{self.w - 60}" height="34" rx="6" fill="#f5f5f5"/>')
        self.text(45, y + 23, title, 17, weight='bold')
        if sub:
            self.text(self.w - 45, y + 22, sub, 13, 'end', '#555')

    def save(self, name):
        body = '\n'.join(self.parts)
        (HERE / name).write_text(
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{self.w}" height="{self.h}" '
            f'viewBox="0 0 {self.w} {self.h}" font-family="{FONT}">\n{body}\n</svg>\n', encoding='utf-8')
        print('wrote', name)


def warning_box(s, x, y, w):
    s.add(f'<rect x="{x}" y="{y}" width="{w}" height="70" rx="6" fill="#ffebee" stroke="{RED}" stroke-width="1.5"/>')
    s.text(x + w / 2, y + 24, '⚠ Pins 1 &amp; 2 = Vpwr', 14, 'middle', '#c62828', 'bold')
    s.text(x + w / 2, y + 42, 'raw battery, ~14–17 V', 12, 'middle', '#c62828')
    s.text(x + w / 2, y + 60, 'DO NOT CONNECT', 13, 'middle', '#c62828', 'bold')


# --------------------------------------------------------------------------- USB-TTL (recommended)
def usb_ttl():
    s = Svg(1120, 640)
    s.text(560, 40, 'Roomba 500-series ⇄ USB-TTL serial adapter (recommended)', 23, 'middle', weight='bold')
    s.text(560, 64, 'FT232RL-style adapter with its voltage jumper on 5V. Works with any computer: '
           'Jetson, Raspberry Pi, laptop.', 13, 'middle', '#444')

    s.box(40, 100, 200, 200, '#e8f5e9', '#2e7d32', 'Computer', 'Jetson / Pi / PC')
    s.add(f'<rect x="200" y="190" width="40" height="24" fill="#90a4ae" stroke="#455a64" stroke-width="2"/>')
    s.text(140, 207, 'USB port', 13, 'middle', '#455a64', 'bold')
    s.text(140, 250, '/dev/ttyUSB0', 13, 'middle', '#333')
    s.text(140, 268, '115200 8N1', 13, 'middle', '#333')

    s.box(400, 100, 250, 380, '#e3f0fc', '#1565c0', 'USB-TTL adapter', 'FT232RL, jumper on 5V')
    s.add(f'<rect x="385" y="190" width="30" height="24" fill="#90a4ae" stroke="#455a64" stroke-width="2"/>')
    s.wire('M240 202 H385', '#607d8b', 8)
    s.text(312, 192, 'mini-USB cable', 12, 'middle', '#455a64')
    # jumper drawing
    s.add('<rect x="430" y="300" width="90" height="70" rx="6" fill="#fff" stroke="#1565c0"/>')
    for i, lab in enumerate(('3.3V', '', '5V')):
        s.add(f'<circle cx="{450 + i * 25}" cy="325" r="7" fill="#ffd54f" stroke="#795548"/>')
        if lab:
            s.text(450 + i * 25, 352, lab, 11, 'middle', '#333', 'bold')
    s.add('<rect x="468" y="315" width="40" height="20" rx="3" fill="#222" opacity="0.85"/>')
    s.text(475, 390, 'jumper: middle + 5V', 12, 'middle', '#1565c0', 'bold')

    pins = [('GND', 160, BLACK, 'Pin 6 · GND', None),
            ('CTS', 200, '#bbb', None, 'not connected'),
            ('VCC', 240, '#bbb', None, 'not connected'),
            ('TXD', 280, BLUE, 'Pin 3 · RXD', None),
            ('RXD', 320, GREEN, 'Pin 4 · TXD', None),
            ('DTR', 360, PURPLE, 'Pin 5 · BRC', None)]
    s.box(860, 100, 220, 300, '#f3f3f3', '#424242', 'Roomba 500', '7-pin Mini-DIN port')
    labels = {BLACK: 'GND', BLUE: 'adapter TX ⟶ Roomba RX', GREEN: 'adapter RX ⟵ Roomba TX',
              PURPLE: 'optional: wake / keep awake'}
    for name, y, color, rpin, note in pins:
        s.text(635, y + 5, name, 14, 'end', '#999' if note else '#222', 'bold' if not note else 'normal')
        s.dot(650, y, '#bbb' if note else '#333')
        if note:
            s.wire(f'M650 {y} H690', '#bbb', 3)
            s.text(698, y + 5, '✕ ' + note, 12, fill='#c62828')
            continue
        s.wire(f'M650 {y} H860', color, 4, '10 7' if color == PURPLE else None)
        s.text(755, y - 9, labels[color], 12, 'middle', color, 'bold')
        s.dot(860, y)
        s.text(875, y + 5, rpin, 14)
    warning_box(s, 872, 410, 196)

    notes = ['TX goes to RX: adapter TXD → Roomba pin 3, adapter RXD ← Roomba pin 4. A common ground is required.',
             'Leave VCC unconnected: the adapter is powered by USB. Never wire anything to Roomba pins 1 or 2.',
             'Never use a DB9 / RS-232 "USB serial" adapter: it swings ±12 V and can damage the Roomba (0–5 V TTL).',
             'DTR → pin 5 is optional. It lets software pulse BRC low to wake the robot and keep it awake in passive mode.',
             'Pin positions as seen looking into the robot\'s socket: see mini-din-pinout.svg. Check with a multimeter first.']
    for i, n in enumerate(notes):
        s.text(45, 520 + i * 22, '• ' + n, 13, fill='#333')
    s.save('wiring-usb-ttl.svg')


# --------------------------------------------------------------------------- Jetson UART (alternative)
def jetson_uart():
    s = Svg(1120, 600)
    s.text(560, 40, 'Alternative: Jetson 40-pin header UART direct (works, but marginal)', 23, 'middle',
           weight='bold')
    s.text(560, 64, 'Jetson Orin Nano: pins 8/10 = /dev/ttyTHS1, 3.3 V logic. '
           'Prefer the USB-TTL adapter.', 13, 'middle', '#c62828')
    s.box(40, 100, 220, 300, '#e8f5e9', '#2e7d32', 'Jetson', '40-pin header (3.3 V)')
    s.box(860, 100, 220, 300, '#f3f3f3', '#424242', 'Roomba 500', '7-pin Mini-DIN port')
    yt, yr, yg = 180, 260, 360
    for y, t in ((yt, 'Pin 8 · TXD'), (yr, 'Pin 10 · RXD'), (yg, 'Pin 6 · GND')):
        s.text(245, y + 5, t, 14, 'end')
        s.dot(260, y)
    for y, t in ((yt, 'Pin 3 · RXD'), (yr, 'Pin 4 · TXD'), (yg, 'Pin 6 · GND')):
        s.text(875, y + 5, t, 14)
        s.dot(860, y)
    s.wire(f'M260 {yt} H860', BLUE)
    s.text(560, yt - 10, 'Jetson TX ⟶ Roomba RX (3.3 V into a 5 V input: marginal)', 12, 'middle', BLUE, 'bold')
    s.wire(f'M260 {yr} H860', GREEN)
    s.text(680, yr - 10, 'Roomba TX (5 V, ~1 kΩ internal) ⟶ Jetson RX', 12, 'middle', GREEN, 'bold')
    nx = 470
    s.wire(f'M{nx} {yr} V{yr + 22}', GREEN, 3)
    s.wire(f'M{nx} {yr + 72} V{yg}', BLACK, 3)
    s.add(f'<rect x="{nx - 13}" y="{yr + 22}" width="26" height="50" rx="3" fill="#fff8e1" stroke="#333" stroke-width="2"/>')
    s.text(nx + 20, yr + 45, 'R 2 kΩ', 13, weight='bold')
    s.text(nx + 20, yr + 61, 'pin 10 → GND', 11, fill='#555')
    s.wire(f'M260 {yg} H860', BLACK)
    s.text(680, yg - 10, 'common ground', 12, 'middle', BLACK, 'bold')
    for y in (yr, yg):
        s.add(f'<circle cx="{nx}" cy="{y}" r="6" fill="{BLACK}"/>')
    warning_box(s, 872, 410, 196)

    notes = ['The Roomba\'s TX has roughly 1 kΩ of output resistance, so 2 kΩ to ground divides 5 V down to ~3.3 V '
             '(measured 5.0 V → 3.7 V at 3 kΩ load).',
             'Jetson TX at 3.3 V is only just above the Roomba\'s input threshold. It needed bytes spaced ~10 ms apart '
             'and 2 stop bits, and still occasionally garbled.',
             'A BSS138 4-channel level shifter did NOT work between the Jetson header and the Roomba: lines '
             'sat at 1.5–2 V. The same shifter works fine with an ESP32.',
             'Avoid baud rates ≤ 1200 on the Tegra UART: they wedged /dev/ttyTHS1 until a reboot.',
             'Garbled bytes can turn into real commands; a corrupted byte once started a cleaning cycle. '
             'Lift the robot when testing this setup.']
    for i, n in enumerate(notes):
        s.text(45, 500 + i * 20, '• ' + n, 12.5, fill='#333')
    s.save('wiring-jetson-uart.svg')


# --------------------------------------------------------------------------- Mini-DIN pinout
def pinout():
    s = Svg(1120, 520)
    s.text(560, 40, 'Roomba 500-series serial port: 7-pin Mini-DIN pinout', 23, 'middle', weight='bold')
    s.text(560, 64, 'Looking straight into the socket on the robot, with the slot in the cover on the right '
           '(same orientation as the photo).', 13, 'middle', '#444')
    photo = HERE / 'roomba-port.jpg'
    cx, cy = 610, 270
    if photo.exists():
        b64 = base64.b64encode(photo.read_bytes()).decode()
        s.add(f'<image href="data:image/jpeg;base64,{b64}" x="50" y="{cy - 105}" width="300" height="211"/>')
        s.add(f'<rect x="50" y="{cy - 105}" width="300" height="211" fill="none" stroke="#333" stroke-width="2"/>')
        s.text(200, cy + 128, 'Roomba 550 serial port', 12, 'middle', '#555')
        s.text(400, cy + 8, '=', 26, 'middle', '#999')
    s.add(f'<circle cx="{cx}" cy="{cy}" r="150" fill="#eee" stroke="{GREY}" stroke-width="10"/>')
    s.add(f'<rect x="752" y="{cy - 16}" width="16" height="32" fill="#fff"/>')
    s.add(f'<circle cx="{cx}" cy="{cy}" r="134" fill="#d6d6d6" stroke="#bdbdbd" stroke-width="2"/>')
    s.add(f'<rect x="{cx - 16}" y="{cy - 16}" width="32" height="32" fill="#444"/>')
    s.add(f'<rect x="772" y="{cy - 18}" width="58" height="36" rx="3" fill="#333"/>')
    s.text(801, cy + 38, 'slot in cover', 11, 'middle', '#555')
    s.text(760, cy - 30, 'notch', 11, 'middle', '#555')
    pins = {4: (0, -99, GREEN, '#fff'), 7: (79, -80, '#666', '#fff'), 2: (-83, -39, '#fff', RED),
            6: (83, 2, BLACK, '#fff'), 1: (-83, 40, '#fff', RED), 3: (0, 100, BLUE, '#fff'),
            5: (79, 81, PURPLE, '#fff')}
    for n, (dx, dy, fill, tc) in pins.items():
        x, y = cx + dx, cy + dy
        if n in (1, 2):
            s.add(f'<circle cx="{x}" cy="{y}" r="17" fill="#fff" stroke="{RED}" stroke-width="3"/>')
            s.add(f'<line x1="{x - 17}" y1="{y - 17}" x2="{x + 17}" y2="{y + 17}" stroke="{RED}" '
                  f'stroke-width="2" opacity="0.5"/>')
        else:
            dash = ' stroke-dasharray="4 3"' if n == 5 else ''
            s.add(f'<circle cx="{x}" cy="{y}" r="17" fill="{fill}" stroke="#333" stroke-width="2"{dash}/>')
        s.text(x, y + 6, str(n), 17, 'middle', tc, 'bold')

    lx, ly = 840, cy - 140
    s.text(lx, ly, 'Pin · signal → connect to', 14, weight='bold')
    legend = [('3', BLUE, '#fff', 'RXD (into Roomba)', 'adapter TXD', False),
              ('4', GREEN, '#fff', 'TXD (out of Roomba)', 'adapter RXD', False),
              ('6', BLACK, '#fff', 'GND', 'adapter GND', False),
              ('7', '#666', '#fff', 'GND', 'spare ground', False),
              ('5', PURPLE, '#fff', 'BRC (wake / baud)', 'optional: adapter DTR', False),
              ('1', '#fff', RED, 'Vpwr ~14–17 V', 'DO NOT CONNECT', True),
              ('2', '#fff', RED, 'Vpwr ~14–17 V', 'DO NOT CONNECT', True)]
    y = ly + 28
    for n, fill, tc, name, to, red in legend:
        s.add(f'<circle cx="{lx + 13}" cy="{y}" r="13" fill="{fill}" stroke="{RED if red else "#333"}" stroke-width="2"/>')
        s.text(lx + 13, y + 5, n, 13, 'middle', tc, 'bold')
        s.text(lx + 34, y - 2, name, 12.5, fill='#c62828' if red else '#222', weight='bold')
        s.text(lx + 34, y + 13, to, 11, fill='#c62828' if red else '#555')
        y += 40
    notes = ['Verify before wiring: pins 1 &amp; 2 read battery voltage against pin 6/7; pins 6 &amp; 7 have continuity to battery (−).',
             'iRobot\'s spec drawing is rotated 90° from this view. Use the notch beside pin 6 as the reference.',
             'Making a plug? Seen from its solder side, a male Mini-DIN matches this picture. Its pin face is the mirror image.']
    for i, n in enumerate(notes):
        s.text(45, 450 + i * 20, '• ' + n, 12.5, fill='#333')
    s.save('mini-din-pinout.svg')


if __name__ == '__main__':
    usb_ttl()
    jetson_uart()
    pinout()
