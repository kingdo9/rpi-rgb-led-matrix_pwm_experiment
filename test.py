#!/usr/bin/env python3
"""
Simple script to display a solid red color on a 64x32 RGB LED matrix using the
hzeller rpi-rgb-led-matrix library's Python bindings.

The script runs until interrupted (Ctrl+C) and supports command-line flags for 
matrix configuration (e.g. --led-gpio-mapping, --led-brightness, etc.).

Usage (example):
    sudo python3 script_name.py --led-gpio-mapping=adafruit-hat-pwm

Note: Running with root privileges (sudo) is recommended for accessing hardware.
"""
import time
import argparse
import sys
from rgbmatrix import RGBMatrix, RGBMatrixOptions

# Parse command-line arguments for LED matrix configuration
parser = argparse.ArgumentParser(description="Display a solid red color on a 64x32 RGB LED matrix.")
parser.add_argument("-m", "--led-gpio-mapping", help="Hardware mapping (e.g. 'regular', 'adafruit-hat', 'adafruit-hat-pwm').")
parser.add_argument("--led-rows", type=int, default=32, help="Display rows. Default: 32")
parser.add_argument("--led-cols", type=int, default=64, help="Display columns. Default: 64")
parser.add_argument("--led-chain", type=int, default=1, help="Daisy-chained panels. Default: 1")
parser.add_argument("--led-parallel", type=int, default=1, help="Parallel chains. Default: 1")
parser.add_argument("--led-brightness", type=int, default=100, help="Brightness (1-100). Default: 100")
parser.add_argument("--led-pwm-bits", type=int, default=11, help="PWM bits. Default: 11")
parser.add_argument("--led-pwm-dither-bits", type=int, default=0, help="PWM dither bits. Default: 0")
parser.add_argument("--led-pwm-lsb-nanoseconds", type=int, default=130, help="PWM LSB nanoseconds. Default: 130")
parser.add_argument("--led-scan-mode", type=int, choices=[0, 1], default=1, help="Scan mode: 0=progressive, 1=interlaced. Default: 1")
parser.add_argument("--led-slowdown-gpio", type=int, default=1, help="GPIO slowdown (0-4). Default: 1")
parser.add_argument("--led-no-hardware-pulse", action="store_true", help="Disable hardware pin pulse generation")
parser.add_argument("--led-show-refresh", action="store_true", help="Show refresh rate (blinking LED)")
parser.add_argument("--led-row-addr-type", type=int, choices=[0, 1, 2, 3, 4, 5, 6], default=0, help="Row address type. Default: 0")
parser.add_argument("--led-multiplexing", type=int, choices=range(0, 22), default=0, help="Multiplexing type. Default: 0")
parser.add_argument("--led-panel-type", type=str, default="", help="Panel type (e.g. 'FM6126A') if required. Default: none")
args = parser.parse_args()

# Configure matrix options based on provided arguments
options = RGBMatrixOptions()
if args.led_gpio_mapping:
    options.hardware_mapping = args.led_gpio_mapping  # e.g. "adafruit-hat-pwm"
options.rows = args.led_rows
options.cols = args.led_cols
options.pwm_dither_bits = args.led_pwm_dither_bits
options.chain_length = args.led_chain
options.parallel = args.led_parallel
options.brightness = args.led_brightness
options.pwm_bits = args.led_pwm_bits
options.pwm_lsb_nanoseconds = args.led_pwm_lsb_nanoseconds
options.scan_mode = args.led_scan_mode
options.gpio_slowdown = args.led_slowdown_gpio
if args.led_no_hardware_pulse:
    options.disable_hardware_pulsing = True
if args.led_show_refresh:
    options.show_refresh_rate = True
options.row_address_type = args.led_row_addr_type
options.multiplexing = args.led_multiplexing
if args.led_panel_type:
    options.panel_type = args.led_panel_type

# Initialize the RGB matrix with the given options
matrix = RGBMatrix(options=options)

try:
    # Fill the display with solid red (full red, no green or blue)
    matrix.Fill(255, 0, 0)
    print("Displaying solid red color. Press Ctrl+C to stop.")
    # Loop indefinitely until interrupted
    while True:
        time.sleep(1)  # Sleep to reduce CPU usage; the matrix continues displaying
except Exception as e:
    print(str(e))

except KeyboardInterrupt:
    # On Ctrl+C, clear the display and exit gracefully
    print("Stopping and clearing the display...")
    matrix.Clear()  # Turn off all LEDs
    sys.exit(0)
