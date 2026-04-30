#!/usr/bin/env python3
"""
Render mockups of the SSD1306 128x64 OLED screens drawn by the firmware.

Mirrors the layout in src/fw/state_views.c using the same font_8x5 bitmap
from external/pico-ssd1306/include/font.h, so the proportions match what
the device actually shows.

Examples:
    # Idle view at 12:00, on USB power, all sensors connected
    tools/render_display.py idle --time 12:00 --power \
        --fork --shock --imu-frame --imu-fork -o idle.png

    # Idle on battery at 87%
    tools/render_display.py idle --time 09:42 --battery 87 --fork --shock

    # GPS waiting for fix
    tools/render_display.py gps-wait --sat 5 --epe 2.3

    # GPS fix acquired
    tools/render_display.py gps-wait --fix
"""

import argparse
import sys

try:
    from PIL import Image
except ImportError:
    sys.stderr.write("error: Pillow is required (pip install pillow)\n")
    sys.exit(1)


# --- Font ----------------------------------------------------------------
# font_8x5 from external/pico-ssd1306/include/font.h.
# Format: 8 rows tall, 5 columns wide, 1px additional spacing per char,
# ASCII 32..126. Each glyph is 5 column bytes, LSB = top pixel.
FONT_8X5 = [
    0x00, 0x00, 0x00, 0x00, 0x00,  # ' '
    0x00, 0x00, 0x5F, 0x00, 0x00,  # '!'
    0x00, 0x07, 0x00, 0x07, 0x00,  # '"'
    0x14, 0x7F, 0x14, 0x7F, 0x14,  # '#'
    0x24, 0x2A, 0x7F, 0x2A, 0x12,  # '$'
    0x23, 0x13, 0x08, 0x64, 0x62,  # '%'
    0x36, 0x49, 0x56, 0x20, 0x50,  # '&'
    0x00, 0x08, 0x07, 0x03, 0x00,  # '\''
    0x00, 0x1C, 0x22, 0x41, 0x00,  # '('
    0x00, 0x41, 0x22, 0x1C, 0x00,  # ')'
    0x2A, 0x1C, 0x7F, 0x1C, 0x2A,  # '*'
    0x08, 0x08, 0x3E, 0x08, 0x08,  # '+'
    0x00, 0x80, 0x70, 0x30, 0x00,  # ','
    0x08, 0x08, 0x08, 0x08, 0x08,  # '-'
    0x00, 0x00, 0x60, 0x60, 0x00,  # '.'
    0x20, 0x10, 0x08, 0x04, 0x02,  # '/'
    0x3E, 0x51, 0x49, 0x45, 0x3E,  # '0'
    0x00, 0x42, 0x7F, 0x40, 0x00,  # '1'
    0x72, 0x49, 0x49, 0x49, 0x46,  # '2'
    0x21, 0x41, 0x49, 0x4D, 0x33,  # '3'
    0x18, 0x14, 0x12, 0x7F, 0x10,  # '4'
    0x27, 0x45, 0x45, 0x45, 0x39,  # '5'
    0x3C, 0x4A, 0x49, 0x49, 0x31,  # '6'
    0x41, 0x21, 0x11, 0x09, 0x07,  # '7'
    0x36, 0x49, 0x49, 0x49, 0x36,  # '8'
    0x46, 0x49, 0x49, 0x29, 0x1E,  # '9'
    0x00, 0x00, 0x14, 0x00, 0x00,  # ':'
    0x00, 0x40, 0x34, 0x00, 0x00,  # ';'
    0x00, 0x08, 0x14, 0x22, 0x41,  # '<'
    0x14, 0x14, 0x14, 0x14, 0x14,  # '='
    0x00, 0x41, 0x22, 0x14, 0x08,  # '>'
    0x02, 0x01, 0x59, 0x09, 0x06,  # '?'
    0x3E, 0x41, 0x5D, 0x59, 0x4E,  # '@'
    0x7C, 0x12, 0x11, 0x12, 0x7C,  # 'A'
    0x7F, 0x49, 0x49, 0x49, 0x36,  # 'B'
    0x3E, 0x41, 0x41, 0x41, 0x22,  # 'C'
    0x7F, 0x41, 0x41, 0x41, 0x3E,  # 'D'
    0x7F, 0x49, 0x49, 0x49, 0x41,  # 'E'
    0x7F, 0x09, 0x09, 0x09, 0x01,  # 'F'
    0x3E, 0x41, 0x41, 0x51, 0x73,  # 'G'
    0x7F, 0x08, 0x08, 0x08, 0x7F,  # 'H'
    0x00, 0x41, 0x7F, 0x41, 0x00,  # 'I'
    0x20, 0x40, 0x41, 0x3F, 0x01,  # 'J'
    0x7F, 0x08, 0x14, 0x22, 0x41,  # 'K'
    0x7F, 0x40, 0x40, 0x40, 0x40,  # 'L'
    0x7F, 0x02, 0x1C, 0x02, 0x7F,  # 'M'
    0x7F, 0x04, 0x08, 0x10, 0x7F,  # 'N'
    0x3E, 0x41, 0x41, 0x41, 0x3E,  # 'O'
    0x7F, 0x09, 0x09, 0x09, 0x06,  # 'P'
    0x3E, 0x41, 0x51, 0x21, 0x5E,  # 'Q'
    0x7F, 0x09, 0x19, 0x29, 0x46,  # 'R'
    0x26, 0x49, 0x49, 0x49, 0x32,  # 'S'
    0x03, 0x01, 0x7F, 0x01, 0x03,  # 'T'
    0x3F, 0x40, 0x40, 0x40, 0x3F,  # 'U'
    0x1F, 0x20, 0x40, 0x20, 0x1F,  # 'V'
    0x3F, 0x40, 0x38, 0x40, 0x3F,  # 'W'
    0x63, 0x14, 0x08, 0x14, 0x63,  # 'X'
    0x03, 0x04, 0x78, 0x04, 0x03,  # 'Y'
    0x61, 0x59, 0x49, 0x4D, 0x43,  # 'Z'
    0x00, 0x7F, 0x41, 0x41, 0x41,  # '['
    0x02, 0x04, 0x08, 0x10, 0x20,  # '\\'
    0x00, 0x41, 0x41, 0x41, 0x7F,  # ']'
    0x04, 0x02, 0x01, 0x02, 0x04,  # '^'
    0x40, 0x40, 0x40, 0x40, 0x40,  # '_'
    0x00, 0x03, 0x07, 0x08, 0x00,  # '`'
    0x20, 0x54, 0x54, 0x78, 0x40,  # 'a'
    0x7F, 0x28, 0x44, 0x44, 0x38,  # 'b'
    0x38, 0x44, 0x44, 0x44, 0x28,  # 'c'
    0x38, 0x44, 0x44, 0x28, 0x7F,  # 'd'
    0x38, 0x54, 0x54, 0x54, 0x18,  # 'e'
    0x00, 0x08, 0x7E, 0x09, 0x02,  # 'f'
    0x18, 0xA4, 0xA4, 0x9C, 0x78,  # 'g'
    0x7F, 0x08, 0x04, 0x04, 0x78,  # 'h'
    0x00, 0x44, 0x7D, 0x40, 0x00,  # 'i'
    0x20, 0x40, 0x40, 0x3D, 0x00,  # 'j'
    0x7F, 0x10, 0x28, 0x44, 0x00,  # 'k'
    0x00, 0x41, 0x7F, 0x40, 0x00,  # 'l'
    0x7C, 0x04, 0x78, 0x04, 0x78,  # 'm'
    0x7C, 0x08, 0x04, 0x04, 0x78,  # 'n'
    0x38, 0x44, 0x44, 0x44, 0x38,  # 'o'
    0xFC, 0x18, 0x24, 0x24, 0x18,  # 'p'
    0x18, 0x24, 0x24, 0x18, 0xFC,  # 'q'
    0x7C, 0x08, 0x04, 0x04, 0x08,  # 'r'
    0x48, 0x54, 0x54, 0x54, 0x24,  # 's'
    0x04, 0x04, 0x3F, 0x44, 0x24,  # 't'
    0x3C, 0x40, 0x40, 0x20, 0x7C,  # 'u'
    0x1C, 0x20, 0x40, 0x20, 0x1C,  # 'v'
    0x3C, 0x40, 0x30, 0x40, 0x3C,  # 'w'
    0x44, 0x28, 0x10, 0x28, 0x44,  # 'x'
    0x4C, 0x90, 0x90, 0x90, 0x7C,  # 'y'
    0x44, 0x64, 0x54, 0x4C, 0x44,  # 'z'
    0x00, 0x08, 0x36, 0x41, 0x00,  # '{'
    0x00, 0x00, 0x77, 0x00, 0x00,  # '|'
    0x00, 0x41, 0x36, 0x08, 0x00,  # '}'
    0x02, 0x01, 0x02, 0x04, 0x02,  # '~'
]

FONT_HEIGHT = 8
FONT_WIDTH = 5
FONT_GAP = 1
FONT_FIRST = 32
FONT_LAST = 126

DISPLAY_WIDTH = 128
DISPLAY_HEIGHT = 64


class Canvas:
    """Mirrors the subset of the ssd1306 API that state_views.c uses."""

    def __init__(self, width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT):
        self.width = width
        self.height = height
        self.pixels = [[0] * width for _ in range(height)]

    def clear(self):
        for row in self.pixels:
            for x in range(self.width):
                row[x] = 0

    def _set(self, x, y):
        if 0 <= x < self.width and 0 <= y < self.height:
            self.pixels[y][x] = 1

    def draw_char(self, x, y, scale, ch):
        code = ord(ch)
        if code < FONT_FIRST or code > FONT_LAST:
            return
        base = (code - FONT_FIRST) * FONT_WIDTH
        for col in range(FONT_WIDTH):
            column_byte = FONT_8X5[base + col]
            for row in range(FONT_HEIGHT):
                if column_byte & (1 << row):
                    for sy in range(scale):
                        for sx in range(scale):
                            self._set(x + col * scale + sx, y + row * scale + sy)

    def draw_string(self, x, y, scale, s):
        advance = (FONT_WIDTH + FONT_GAP) * scale
        for i, ch in enumerate(s):
            self.draw_char(x + i * advance, y, scale, ch)

    def to_image(self, scale_up=8, bezel=24, panel_color=(255, 255, 255),
                 background=(0, 0, 0), bezel_color=(30, 30, 32)):
        img = Image.new("RGB", (self.width, self.height), background)
        px = img.load()
        for y in range(self.height):
            for x in range(self.width):
                if self.pixels[y][x]:
                    px[x, y] = panel_color
        big = img.resize(
            (self.width * scale_up, self.height * scale_up), Image.NEAREST
        )
        if bezel <= 0:
            return big
        framed = Image.new(
            "RGB",
            (big.width + bezel * 2, big.height + bezel * 2),
            bezel_color,
        )
        framed.paste(big, (bezel, bezel))
        return framed


# --- Screens (mirror src/fw/state_views.c) -------------------------------


def render_idle(canvas, *, hour, minute, battery_power, voltage_percentage,
                fork, shock, imu_frame, imu_fork):
    canvas.clear()

    if battery_power:
        if voltage_percentage > 99:
            battery_str = "FULL"
        else:
            battery_str = f"{voltage_percentage:3d}%"
    else:
        battery_str = " PWR"

    time_str = f"{hour:02d}:{minute:02d}"

    canvas.draw_string(96, 0, 1, battery_str)
    canvas.draw_string(0, 0, 2, time_str)

    if fork:
        canvas.draw_string(0, 24, 1, "fork")
    if shock:
        canvas.draw_string(30, 24, 1, "shock")
    if imu_frame:
        canvas.draw_string(63, 24, 1, "iFra")
    if imu_fork:
        canvas.draw_string(90, 24, 1, "iFor")


def render_gps_wait(canvas, *, fix_ready, satellites, epe):
    canvas.clear()
    if fix_ready:
        canvas.draw_string(0, 0, 2, "GPS OK")
        canvas.draw_string(0, 24, 1, "press to start")
    else:
        canvas.draw_string(0, 0, 2, "GPS...")
        canvas.draw_string(0, 24, 1, f"SAT:{satellites} EPE:{epe:.1f}")


def render_message(canvas, *, text):
    """Mirrors src/fw/display.c display_message: scale 2 at (0, 10)."""
    canvas.clear()
    canvas.draw_string(0, 10, 2, text)


def render_message_subtitle(canvas, *, text, subtitle):
    """Mirrors src/fw/display.c display_message_with_subtitle."""
    canvas.clear()
    canvas.draw_string(0, 0, 2, text)
    canvas.draw_string(0, 24, 1, subtitle)


def render_cal_step(canvas, *, text, battery_power, voltage_percentage,
                    fork, shock):
    """Mirrors src/fw/calibration_flow.c idle_update."""
    canvas.clear()
    if battery_power:
        if voltage_percentage > 99:
            battery_str = "FULL"
        else:
            battery_str = f"{voltage_percentage:3d}%"
    else:
        battery_str = " PWR"
    canvas.draw_string(96, 0, 1, battery_str)
    canvas.draw_string(0, 0, 2, text)
    if fork:
        canvas.draw_string(0, 24, 1, "fork")
    if shock:
        canvas.draw_string(40, 24, 1, "shock")


# --- CLI -----------------------------------------------------------------


def parse_time(s):
    try:
        h, m = s.split(":")
        return int(h), int(m)
    except Exception as exc:
        raise argparse.ArgumentTypeError(
            f"expected HH:MM, got {s!r}"
        ) from exc


def cmd_idle(args):
    canvas = Canvas()
    hour, minute = parse_time(args.time)
    on_battery = args.battery is not None
    if on_battery and args.power:
        sys.exit("error: --battery and --power are mutually exclusive")
    render_idle(
        canvas,
        hour=hour,
        minute=minute,
        battery_power=on_battery,
        voltage_percentage=args.battery if on_battery else 0,
        fork=args.fork,
        shock=args.shock,
        imu_frame=args.imu_frame,
        imu_fork=args.imu_fork,
    )
    save(canvas, args)


def cmd_gps_wait(args):
    canvas = Canvas()
    render_gps_wait(
        canvas,
        fix_ready=args.fix,
        satellites=args.sat,
        epe=args.epe,
    )
    save(canvas, args)


def cmd_message(args):
    canvas = Canvas()
    render_message(canvas, text=args.text)
    save(canvas, args)


def cmd_message_sub(args):
    canvas = Canvas()
    render_message_subtitle(canvas, text=args.text, subtitle=args.subtitle)
    save(canvas, args)


def cmd_cal_step(args):
    canvas = Canvas()
    on_batt = args.battery is not None
    if on_batt and args.power:
        sys.exit("error: --battery and --power are mutually exclusive")
    render_cal_step(
        canvas,
        text=args.text,
        battery_power=on_batt,
        voltage_percentage=args.battery if on_batt else 0,
        fork=args.fork,
        shock=args.shock,
    )
    save(canvas, args)


def save(canvas, args):
    img = canvas.to_image(scale_up=args.scale, bezel=args.bezel)
    img.save(args.output)
    print(args.output)


def main():
    parser = argparse.ArgumentParser(
        description="Render SSD1306 screens drawn by the Sufni DAQ firmware."
    )
    sub = parser.add_subparsers(dest="cmd", required=True)

    common = argparse.ArgumentParser(add_help=False)
    common.add_argument(
        "-o", "--output", default="display.png",
        help="output PNG path (default: display.png)",
    )
    common.add_argument(
        "--scale", type=int, default=8,
        help="pixel scale-up factor for the rendered image (default: 8)",
    )
    common.add_argument(
        "--bezel", type=int, default=24,
        help="bezel padding in scaled pixels; 0 disables (default: 24)",
    )

    p_idle = sub.add_parser(
        "idle", parents=[common], help="render the IDLE state view"
    )
    p_idle.add_argument(
        "--time", default="12:00", help="HH:MM displayed time (default: 12:00)"
    )
    power = p_idle.add_mutually_exclusive_group()
    power.add_argument(
        "--power", action="store_true",
        help="device on USB power (shows ' PWR')",
    )
    power.add_argument(
        "--battery", type=int, metavar="PCT",
        help="device on battery, percentage 0-100 (>99 shows 'FULL')",
    )
    p_idle.add_argument("--fork", action="store_true", help="fork sensor present")
    p_idle.add_argument("--shock", action="store_true", help="shock sensor present")
    p_idle.add_argument(
        "--imu-frame", action="store_true", help="frame IMU present (iFra)"
    )
    p_idle.add_argument(
        "--imu-fork", action="store_true", help="fork IMU present (iFor)"
    )
    p_idle.set_defaults(func=cmd_idle)

    p_gps = sub.add_parser(
        "gps-wait", parents=[common], help="render the GPS_WAIT state view"
    )
    p_gps.add_argument(
        "--fix", action="store_true", help="show 'GPS OK' / press to start",
    )
    p_gps.add_argument("--sat", type=int, default=0, help="satellites in view")
    p_gps.add_argument(
        "--epe", type=float, default=0.0, help="estimated position error",
    )
    p_gps.set_defaults(func=cmd_gps_wait)

    p_msg = sub.add_parser(
        "message", parents=[common],
        help="render a single large message (display_message)",
    )
    p_msg.add_argument("text", help="text drawn at scale 2")
    p_msg.set_defaults(func=cmd_message)

    p_msgsub = sub.add_parser(
        "message-sub", parents=[common],
        help="render a message with subtitle (display_message_with_subtitle)",
    )
    p_msgsub.add_argument("text", help="title at scale 2")
    p_msgsub.add_argument("subtitle", help="subtitle at scale 1")
    p_msgsub.set_defaults(func=cmd_message_sub)

    p_cal = sub.add_parser(
        "cal-step", parents=[common],
        help="render a calibration step (calibration_flow.c idle_update)",
    )
    p_cal.add_argument("text", help="step label at scale 2 (e.g. 'CAL EXP')")
    cal_power = p_cal.add_mutually_exclusive_group()
    cal_power.add_argument("--power", action="store_true",
                           help="device on USB power (' PWR')")
    cal_power.add_argument("--battery", type=int, metavar="PCT",
                           help="device on battery, percentage 0-100")
    p_cal.add_argument("--fork", action="store_true", help="fork sensor present")
    p_cal.add_argument("--shock", action="store_true", help="shock sensor present")
    p_cal.set_defaults(func=cmd_cal_step)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
