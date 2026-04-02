#!/usr/bin/env python
from samplebase import SampleBase
import math
import time


def scale_col(val, lo, hi):
    if val < lo:
        return 0
    if val > hi:
        return 255
    return 255 * (val - lo) / (hi - lo)


def rotate(x, y, sin, cos):
    return x * cos - y * sin, x * sin + y * cos


class RotatingBlockGenerator(SampleBase):
    def __init__(self, *args, **kwargs):
        super(RotatingBlockGenerator, self).__init__(*args, **kwargs)

    def run(self):
        cent_x = self.matrix.width / 2
        cent_y = self.matrix.height / 2

        display_square = min(self.matrix.width, self.matrix.height) * 0.7
        min_display_x = int(cent_x - display_square / 2)
        max_display_x = int(cent_x + display_square / 2)
        min_display_y = int(cent_y - display_square / 2)
        max_display_y = int(cent_y + display_square / 2)
        rotate_square = display_square * 1.41
        min_draw_x = int(cent_x - rotate_square / 2)
        max_draw_x = int(cent_x + rotate_square / 2)
        min_draw_y = int(cent_y - rotate_square / 2)
        max_draw_y = int(cent_y + rotate_square / 2)

        deg_to_rad = 2 * 3.14159265 / 360
        rotation_speed_dps = 60.0
        start_time = time.monotonic()

        # Pre-calculate color ramps once outside the frame loop.
        x_col_table = []
        for x in range(min_display_x, max_display_x):
            x_col_table.append(int(scale_col(x, min_display_x, max_display_x)))

        y_col_table = []
        for y in range(min_display_y, max_display_y):
            y_col_table.append(int(scale_col(y, min_display_y, max_display_y)))

        offset_canvas = self.matrix.CreateFrameCanvas()

        while True:
            # Keep the cube speed tied to elapsed time instead of completed
            # frames so larger layouts don't visibly slow the animation.
            rotation = ((time.monotonic() - start_time) *
                        rotation_speed_dps) % 360.0

            # calculate sin and cos once for each frame
            angle = rotation * deg_to_rad
            sin = math.sin(angle)
            cos = math.cos(angle)

            # Clearing in native code is much cheaper than sending thousands of
            # black SetPixel() calls through Python for the background.
            offset_canvas.Clear()
            set_pixel = offset_canvas.SetPixel

            for x in range(min_draw_x, max_draw_x):
                for y in range(min_draw_y, max_draw_y):
                    # Inverse-map each destination pixel back into the source
                    # square so every visible pixel is covered exactly once.
                    src_x, src_y = rotate(x - cent_x, y - cent_y, -sin, cos)
                    src_x = int(src_x + cent_x)
                    src_y = int(src_y + cent_y)

                    if (src_x < min_display_x or src_x >= max_display_x or
                            src_y < min_display_y or src_y >= max_display_y):
                        continue

                    x_col = x_col_table[src_x - min_display_x]
                    y_col = y_col_table[src_y - min_display_y]
                    set_pixel(x, y, x_col, 255 - y_col, y_col)

            offset_canvas = self.matrix.SwapOnVSync(offset_canvas)


# Main function
if __name__ == "__main__":
    rotating_block_generator = RotatingBlockGenerator()
    if (not rotating_block_generator.process()):
        rotating_block_generator.print_help()
