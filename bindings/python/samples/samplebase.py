import argparse
import time
import sys
import os

sys.path.append(os.path.abspath(os.path.dirname(__file__) + '/..'))
from rgbmatrix import RGBMatrix, RGBMatrixOptions


class SampleBase(object):
    def __init__(self, *args, **kwargs):
        self.parser = argparse.ArgumentParser()

        self.parser.add_argument("-r", "--led-rows", action="store", help="Display rows. 16 for 16x32, 32 for 32x32. Default: 32", default=32, type=int)
        self.parser.add_argument("--led-cols", action="store", help="Panel columns. Typically 32 or 64. (Default: 32)", default=32, type=int)
        self.parser.add_argument("-c", "--led-chain", action="store", help="Daisy-chained boards. Default: 1.", default=1, type=int)
        self.parser.add_argument("-P", "--led-parallel", action="store", help="For Plus-models or RPi2: parallel chains. 1..3. Default: 1", default=1, type=int)
        self.parser.add_argument("-p", "--led-pwm-bits", action="store", help="Bits used for PWM. Something between 1..11. Default: 11", default=11, type=int)
        self.parser.add_argument("-b", "--led-brightness", action="store", help="Sets brightness level. Default: 100. Range: 1..100", default=100, type=int)
        self.parser.add_argument("-m", "--led-gpio-mapping", help="Hardware Mapping: regular, adafruit-hat, adafruit-hat-pwm" , choices=['regular', 'adafruit-hat', 'adafruit-hat-pwm'], type=str)
        self.parser.add_argument("--led-limit-refresh", action="store", help="Limit refresh rate to this frequency in Hz. Useful to keep a constant refresh rate on loaded system. 0=no limit. Default: 0", default=0, type=int)
        self.parser.add_argument("--led-no-busy-waiting", action="store_true", help="Don't use busy waiting when limiting refresh rate.")
        self.parser.add_argument("--led-scan-mode", action="store", help="Progressive or interlaced scan. 0 Progressive, 1 Interlaced (default)", default=1, choices=range(2), type=int)
        self.parser.add_argument("--led-pwm-dither-bits", action="store", help="Time dithering of lower bits.  Default: 0", default=0, type=int)
        self.parser.add_argument("--led-pwm-lsb-nanoseconds", action="store", help="Base time-unit for the on-time in the lowest significant bit in nanoseconds. Default: 130", default=130, type=int)
        self.parser.add_argument("--led-show-refresh", action="store_true", help="Shows the current refresh rate of the LED panel")
        self.parser.add_argument("--led-slowdown-gpio", action="store", help="Slow down writing to GPIO. Range: 0..30. Default: 1", default=1, type=int)
        self.parser.add_argument("--led-rp1-pio", action="store", help="On Raspberry Pi 5-family boards, force the RP1 PIO backend. 0=default RP1 RIO, 1=PIO. Default: native library default.", default=None, choices=[0, 1], type=int)
        self.parser.add_argument("--led-no-hardware-pulse", action="store", help="Don't use hardware pin-pulse generation")
        self.parser.add_argument("--led-rgb-sequence", action="store", help="Switch if your matrix has led colors swapped. Default: RGB", default="RGB", type=str)
        self.parser.add_argument("--led-pixel-mapper", action="store", help="Apply pixel mappers. e.g \"Rotate:90\"", default="", type=str)
        self.parser.add_argument("--led-row-addr-type", action="store", help="0 = default; 1 = AB-addressed panels; 2 = direct row select; 3 = ABC-addressed panels; 4 = ABC Shift + DE direct; 5 = shift-register row select", default=0, type=int, choices=[0,1,2,3,4,5])
        self.parser.add_argument("--led-spwm-row-addr-type", action="store", help="SPWM-only row select. 0 = direct A-E row flow; 1 = shift-register blank-clock A/C row-select; 2 = shift-register blank-clock A+B with wrap-C row-select", default=0, type=int, choices=[0,1,2])
        self.parser.add_argument("--led-spwm-scan", action="store", help="SPWM scan-row count for row-select types 1/2. 0 = use rows/2", default=0, type=int)
        self.parser.add_argument("--led-spwm-data-layout", action="store", help="SPWM data layout. 0 = panel default; 1 = full-height left on RGB2/right on RGB1; 2 = reversed; cannot be combined with --led-multiplexing", default=0, type=int, choices=[0,1,2])
        self.parser.add_argument("--led-spwm-register-config", action="store", help="SPWM register profile. 0 = main; N = runtime catalog regtypeN", default=-1, type=int)
        self.parser.add_argument("--led-spwm-force-register", action="store", help="Override the profile's rotating RGB register slot with a shared list or R:<words>;G:<words>;B:<words>", default=None, type=str)
        self.parser.add_argument("--led-spwm-force-register1", action="store", help="Override SPWM profile register slot 1; fixed slots accept one shared word or distinct R/G/B words, rotating slots accept shared or RGB-labelled lists", default=None, type=str)
        self.parser.add_argument("--led-spwm-force-register2", action="store", help="Override SPWM profile register slot 2; fixed slots accept one shared word or distinct R/G/B words, rotating slots accept shared or RGB-labelled lists", default=None, type=str)
        self.parser.add_argument("--led-spwm-force-register3", action="store", help="Override SPWM profile register slot 3; fixed slots accept one shared word or distinct R/G/B words, rotating slots accept shared or RGB-labelled lists", default=None, type=str)
        self.parser.add_argument("--led-spwm-force-register4", action="store", help="Override SPWM profile register slot 4; fixed slots accept one shared word or distinct R/G/B words, rotating slots accept shared or RGB-labelled lists", default=None, type=str)
        self.parser.add_argument("--led-spwm-force-register5", action="store", help="Override SPWM profile register slot 5; fixed slots accept one shared word or distinct R/G/B words, rotating slots accept shared or RGB-labelled lists", default=None, type=str)
        self.parser.add_argument("--led-spwm-force-register6", action="store", help="Override SPWM profile register slot 6; fixed slots accept one shared word or distinct R/G/B words, rotating slots accept shared or RGB-labelled lists", default=None, type=str)
        self.parser.add_argument("--led-multiplexing", action="store", help="Multiplexing type: 0=direct; 1=strip; 2=checker; 3=spiral; 4=ZStripe; 5=ZnMirrorZStripe; 6=coreman; 7=Kaler2Scan; 8=ZStripeUneven... (Default: 0)", default=0, type=int)
        self.parser.add_argument("--led-panel-type", action="store", help="Needed to initialize special panels. Supported: 'FM6126A', 'FM6127', 'FM6373', 'ICND1065L', 'SM16380SH', 'FM6363', 'FM6353'", default="", type=str)
        self.parser.add_argument("--led-no-drop-privs", dest="drop_privileges", help="Don't drop privileges from 'root' after initializing the hardware.", action='store_false')
        self.parser.set_defaults(drop_privileges=True)

    def usleep(self, value):
        time.sleep(value / 1000000.0)

    def run(self):
        print("Running")

    def process(self):
        self.args = self.parser.parse_args()

        options = RGBMatrixOptions()

        if self.args.led_gpio_mapping != None:
          options.hardware_mapping = self.args.led_gpio_mapping
        options.rows = self.args.led_rows
        options.cols = self.args.led_cols
        options.chain_length = self.args.led_chain
        options.parallel = self.args.led_parallel
        options.scan_mode = self.args.led_scan_mode
        options.row_address_type = self.args.led_row_addr_type
        options.spwm_row_address_type = self.args.led_spwm_row_addr_type
        options.spwm_scan_rows = self.args.led_spwm_scan
        options.spwm_data_layout = self.args.led_spwm_data_layout
        options.spwm_register_config = self.args.led_spwm_register_config
        if self.args.led_spwm_force_register is not None:
          options.spwm_force_register = self.args.led_spwm_force_register
        if self.args.led_spwm_force_register1 is not None:
          options.spwm_force_register1 = self.args.led_spwm_force_register1
        if self.args.led_spwm_force_register2 is not None:
          options.spwm_force_register2 = self.args.led_spwm_force_register2
        if self.args.led_spwm_force_register3 is not None:
          options.spwm_force_register3 = self.args.led_spwm_force_register3
        if self.args.led_spwm_force_register4 is not None:
          options.spwm_force_register4 = self.args.led_spwm_force_register4
        if self.args.led_spwm_force_register5 is not None:
          options.spwm_force_register5 = self.args.led_spwm_force_register5
        if self.args.led_spwm_force_register6 is not None:
          options.spwm_force_register6 = self.args.led_spwm_force_register6
        options.multiplexing = self.args.led_multiplexing
        options.pwm_bits = self.args.led_pwm_bits
        options.pwm_dither_bits = self.args.led_pwm_dither_bits
        options.brightness = self.args.led_brightness
        options.pwm_lsb_nanoseconds = self.args.led_pwm_lsb_nanoseconds
        options.limit_refresh_rate_hz = self.args.led_limit_refresh
        options.led_rgb_sequence = self.args.led_rgb_sequence
        options.pixel_mapper_config = self.args.led_pixel_mapper
        options.panel_type = self.args.led_panel_type

        if self.args.led_show_refresh:
          options.show_refresh_rate = 1
        if self.args.led_no_busy_waiting:
          options.disable_busy_waiting = True

        if self.args.led_slowdown_gpio != None:
            options.gpio_slowdown = self.args.led_slowdown_gpio
        if self.args.led_rp1_pio != None:
            options.rp1_pio = self.args.led_rp1_pio
        if self.args.led_no_hardware_pulse:
          options.disable_hardware_pulsing = True
        if not self.args.drop_privileges:
          options.drop_privileges=False

        self.matrix = RGBMatrix(options = options)

        try:
            # Start loop
            print("Press CTRL-C to stop sample")
            self.run()
        except KeyboardInterrupt:
            print("Exiting\n")
            sys.exit(0)

        return True
