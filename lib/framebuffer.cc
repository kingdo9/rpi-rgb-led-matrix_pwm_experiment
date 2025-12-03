// -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
// Copyright (C) 2013 Henner Zeller <h.zeller@acm.org>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation version 2.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://gnu.org/licenses/gpl-2.0.txt>

// The framebuffer is the workhorse: it represents the frame in some internal
// format that is friendly to be dumped to the matrix quickly. Provides methods
// to manipulate the content.

#include "framebuffer-internal.h"

#include <assert.h>
#include <ctype.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>  // For std::string
#include <vector>  // For std::vector
#include <cctype>  // For tolower

#include <algorithm>

#include <stdarg.h>

#include "gpio.h"
#include "../include/graphics.h"

#include <chrono>
#include <stdarg.h>
#include <unistd.h>

namespace rgb_matrix {
namespace internal {
// Forward declarations for local helpers.
static int custom_strcasecmp(const char *s1, const char *s2);
static int custom_strncasecmp(const char *s1, const char *s2, size_t n);

// We need one global instance of a timing correct pulser. There are different
// implementations depending on the context.
static PinPulser *sOutputEnablePulser = NULL;
// Dedicated OE pulser for the FM6373 path so we can match the MRV412-style
// fixed OE gate without software jitter. We size this in nanoseconds to equal
// 4 DCLKs by default and reuse the hardware PWM if available.
static PinPulser *sFM6373OEPulser = NULL;
static bool sFM6373_Enabled = false;  // Global variable, default is false until --led-panel-type=fm6373
static bool sFM6373_DebugOnce = true;  // Enables logging for the current frame
static bool sFM6373_DebugAlways = false;  // Persist debug logging every frame
static int sFM6373_DebugFramesPending = 0;  // Number of upcoming frames to log

static inline bool FM6373_DebugEnabled() {
  return sFM6373_DebugAlways || sFM6373_DebugOnce;
}

static void FM6373_Debug(const char *fmt, ...) {
  if (!FM6373_DebugEnabled()) return;  // only log when enabled

  using clock = std::chrono::steady_clock;
  static const clock::time_point t0 = clock::now();

  clock::time_point now = clock::now();
  double t_seconds =
      std::chrono::duration_cast<std::chrono::duration<double>>(now - t0).count();

  // Print prefix with timestamp
  fprintf(stderr, "[FM6373 t=%9.6f] ", t_seconds);

  // Print the actual message
  va_list ap;
  va_start(ap, fmt);
  vfprintf(stderr, fmt, ap);
  va_end(ap);
}

static void FM6373_ConfigureDebugFromEnv() {
  const char *env = getenv("FM6373_DEBUG");
  if (env && *env) {
    if (custom_strcasecmp(env, "always") == 0 ||
        custom_strcasecmp(env, "all") == 0 ||
        custom_strcasecmp(env, "on") == 0) {
      sFM6373_DebugAlways = true;
    } else if (custom_strcasecmp(env, "once") == 0) {
      sFM6373_DebugFramesPending = 1;
    } else {
      const int frames = atoi(env);
      if (frames > 0) sFM6373_DebugFramesPending = frames;
    }
  }

  // Backwards-compatible: FM6373_DEBUG_ONCE forces a single debug frame.
  const char *env_once = getenv("FM6373_DEBUG_ONCE");
  if (env_once && *env_once) {
    const int frames = atoi(env_once);
    sFM6373_DebugFramesPending = (frames > 0) ? frames : 1;
  }
}

// Refresh per-frame debug toggle. If FM6373_DebugAlways is set, we keep the
// flag high every frame; otherwise we enable it for the requested number of
// frames and count down here.
static void FM6373_EnableDebugForFrameIfRequested() {
  if (sFM6373_DebugAlways) {
    sFM6373_DebugOnce = true;
    return;
  }

  if (sFM6373_DebugFramesPending > 0) {
    sFM6373_DebugOnce = true;
    --sFM6373_DebugFramesPending;
  } else {
    sFM6373_DebugOnce = false;
  }
}

// --- FM6373 hardware OE helper ------------------------------------------------
// Default OE gate width sized to 4 DCLKs. Override with env FM6373_OE_NS if you
// have a measured DCLK period.
static int FM6373_GetOEPulseNanoseconds() {
  static int cached_ns = -1;
  if (cached_ns >= 0) return cached_ns;
  const char *env_ns = getenv("FM6373_OE_NS");
  if (env_ns && *env_ns) {
    const int val = atoi(env_ns);
    if (val > 0) {
      cached_ns = val;
      return cached_ns;
    }
  }
  // Default assumption: ~300ns per DCLK -> 4 clocks ~1200ns.
  const int kDefaultDclkNs = 300;
  cached_ns = 4 * kDefaultDclkNs;
  return cached_ns;
}

static void FM6373_EnsureOEPulser(GPIO *io, const HardwareMapping &h) {
  if (sFM6373OEPulser != NULL) return;
  std::vector<int> specs;
  specs.push_back(FM6373_GetOEPulseNanoseconds());
  // Prefer hardware pulser (PWM) to minimize jitter; fallback to timer pulser.
  sFM6373OEPulser = PinPulser::Create(io, h.output_enable, true, specs);
  if (sFM6373OEPulser == NULL) {
    sFM6373OEPulser = sOutputEnablePulser;
  }
}



#ifdef ONLY_SINGLE_SUB_PANEL
#  define SUB_PANELS_ 1
#else
#  define SUB_PANELS_ 2
#endif

// Custom case-insensitive string comparison functions to fix compilation issues
static int custom_strcasecmp(const char *s1, const char *s2) {
    while (*s1 && *s2) {
        if (std::tolower(static_cast<unsigned char>(*s1)) != std::tolower(static_cast<unsigned char>(*s2))) {
            return std::tolower(static_cast<unsigned char>(*s1)) - std::tolower(static_cast<unsigned char>(*s2));
        }
        ++s1;
        ++s2;
    }
    return std::tolower(static_cast<unsigned char>(*s1)) - std::tolower(static_cast<unsigned char>(*s2));
}

static int custom_strncasecmp(const char *s1, const char *s2, size_t n) {
    for (size_t i = 0; i < n && s1[i] && s2[i]; ++i) {
        int c1 = std::tolower(static_cast<unsigned char>(s1[i]));
        int c2 = std::tolower(static_cast<unsigned char>(s2[i]));
        if (c1 != c2) {
            return c1 - c2;
        }
    }
    return 0;
}

PixelDesignator *PixelDesignatorMap::get(int x, int y) {
  if (x < 0 || y < 0 || x >= width_ || y >= height_)
    return NULL;
  return buffer_ + (y*width_) + x;
}

PixelDesignatorMap::PixelDesignatorMap(int width, int height,
                                       const PixelDesignator &fill_bits)
  : width_(width), height_(height), fill_bits_(fill_bits),
    buffer_(new PixelDesignator[width * height]) {
}

PixelDesignatorMap::~PixelDesignatorMap() {
  delete [] buffer_;
}

// Different panel types use different techniques to set the row address.
// We abstract that away with different implementations of RowAddressSetter
class RowAddressSetter {
public:
  virtual ~RowAddressSetter() {}
  virtual gpio_bits_t need_bits() const = 0;
  virtual void SetRowAddress(GPIO *io, int row) = 0;
};

namespace {

// The default DirectRowAddressSetter just sets the address in parallel
// output lines ABCDE with A the LSB and E the MSB.
class DirectRowAddressSetter : public RowAddressSetter {
public:
  DirectRowAddressSetter(int double_rows, const HardwareMapping &h)
    : row_mask_(0), last_row_(-1) {
    assert(double_rows <= 32);  // need to resize row_lookup_
    if (double_rows > 16) row_mask_ |= h.e;
    if (double_rows > 8)  row_mask_ |= h.d;
    if (double_rows > 4)  row_mask_ |= h.c;
    if (double_rows > 2)  row_mask_ |= h.b;
    row_mask_ |= h.a;
    for (int i = 0; i < double_rows; ++i) {
      // To avoid the bit-fiddle in the critical path, utilize
      // a lookup-table for all possible rows.
      gpio_bits_t row_address = (i & 0x01) ? h.a : 0;
      row_address |= (i & 0x02) ? h.b : 0;
      row_address |= (i & 0x04) ? h.c : 0;
      row_address |= (i & 0x08) ? h.d : 0;
      row_address |= (i & 0x10) ? h.e : 0;
      row_lookup_[i] = row_address;
    }
  }

  virtual gpio_bits_t need_bits() const { return row_mask_; }

  virtual void SetRowAddress(GPIO *io, int row) {
    if (row == last_row_) return;
    io->WriteMaskedBits(row_lookup_[row], row_mask_);
    last_row_ = row;
  }

private:
  gpio_bits_t row_mask_;
  gpio_bits_t row_lookup_[32];
  int last_row_;
};

// The SM5266RowAddressSetter (ABC Shifter + DE direct) sets bits ABC using
// a 8 bit shifter and DE directly. The panel this works with has 8 SM5266
// shifters (4 for the top 32 rows and 4 for the bottom 32 rows).
// DE is used to select the active shifter
// (rows 1-8/33-40, 9-16/41-48, 17-24/49-56, 25-32/57-64).
// Rows are enabled by shifting in 8 bits (high bit first) with a high bit
// enabling that row. This allows up to 8 rows per group to be active at the
// same time (if they have the same content), but that isn't implemented here.
// BK, DIN and DCK are the designations on the SM5266P datasheet.
// BK = Enable Input, DIN = Serial In, DCK = Clock
class SM5266RowAddressSetter : public RowAddressSetter {
public:
  SM5266RowAddressSetter(int double_rows, const HardwareMapping &h)
    : row_mask_(h.a | h.b | h.c),
      last_row_(-1),
      bk_(h.c),
      din_(h.b),
      dck_(h.a) {
    assert(double_rows <= 32); // designed for up to 1/32 panel
    if (double_rows > 8)  row_mask_ |= h.d;
    if (double_rows > 16) row_mask_ |= h.e;
    for (int i = 0; i < double_rows; ++i) {
      gpio_bits_t row_address = 0;
      row_address |= (i & 0x08) ? h.d : 0;
      row_address |= (i & 0x10) ? h.e : 0;
      row_lookup_[i] = row_address;
    }
  }

  virtual gpio_bits_t need_bits() const { return row_mask_; }

  virtual void SetRowAddress(GPIO *io, int row) {
    if (row == last_row_) return;
    io->SetBits(bk_);  // Enable serial input for the shifter
    for (int r = 7; r >= 0; r--) {
      if (row % 8 == r) {
        io->SetBits(din_);
      } else {
        io->ClearBits(din_);
      }
      io->SetBits(dck_);
      io->SetBits(dck_);  // Longer clock time; tested with Pi3
      io->ClearBits(dck_);
    }
    io->ClearBits(bk_);  // Disable serial input to keep unwanted bits out of the shifters
    last_row_ = row;
    // Set bits D and E to enable the proper shifter to display the selected
    // row.
    io->WriteMaskedBits(row_lookup_[row], row_mask_);
  }

private:
  gpio_bits_t row_mask_;
  int last_row_;
  const gpio_bits_t bk_;
  const gpio_bits_t din_;
  const gpio_bits_t dck_;
  gpio_bits_t row_lookup_[32];
};

class B707ShiftRegisterRowAddressSetter : public RowAddressSetter {
public:
  B707ShiftRegisterRowAddressSetter(int double_rows, const HardwareMapping &h)
    : row_mask_(h.a | h.b | h.c),
      last_row_(-1),
      bk_(h.b),
      din_(h.c),
      dck_(h.a) {
    assert(double_rows <= 32); // designed for up to 1/32 panel
  }

  virtual gpio_bits_t need_bits() const { return row_mask_; }

  virtual void SetRowAddress(GPIO *io, int row) {
    if (row == last_row_) return;
    io->SetBits(bk_);  // Enable serial input for the shifter
    if (row == 0) {
        io->SetBits(din_);
      } else {
        io->ClearBits(din_);
      }
    io->SetBits(dck_);
    io->SetBits(dck_);  // Longer clock time; tested with Pi3
    io->ClearBits(dck_);
    io->ClearBits(bk_);  // Disable serial input to keep unwanted bits out of the shifters
    last_row_ = row;
  }

private:
  gpio_bits_t row_mask_;
  int last_row_;
  const gpio_bits_t bk_;
  const gpio_bits_t din_;
  const gpio_bits_t dck_;
};


class ShiftRegisterRowAddressSetter : public RowAddressSetter {
public:
  ShiftRegisterRowAddressSetter(int double_rows, const HardwareMapping &h)
    : double_rows_(double_rows),
      row_mask_(h.a | h.b), clock_(h.a), data_(h.b),
      last_row_(-1) {
  }
  virtual gpio_bits_t need_bits() const { return row_mask_; }

  virtual void SetRowAddress(GPIO *io, int row) {
    if (row == last_row_) return;
    for (int activate = 0; activate < double_rows_; ++activate) {
      io->ClearBits(clock_);
      if (activate == double_rows_ - 1 - row) {
        io->ClearBits(data_);
      } else {
        io->SetBits(data_);
      }
      io->SetBits(clock_);
    }
    io->ClearBits(clock_);
    io->SetBits(clock_);
    last_row_ = row;
  }

private:
  const int double_rows_;
  const gpio_bits_t row_mask_;
  const gpio_bits_t clock_;
  const gpio_bits_t data_;
  int last_row_;
};

// Issue #823
// An shift register row address setter that does not use B but C for the
// data. Clock is inverted.
class ABCShiftRegisterRowAddressSetter : public RowAddressSetter {
public:
  ABCShiftRegisterRowAddressSetter(int double_rows, const HardwareMapping &h)
    : double_rows_(double_rows),
      row_mask_(h.a | h.c),
      clock_(h.a),
      data_(h.c),
      last_row_(-1) {
  }
  virtual gpio_bits_t need_bits() const { return row_mask_; }

  virtual void SetRowAddress(GPIO *io, int row) {
    for (int activate = 0; activate < double_rows_; ++activate) {
      io->ClearBits(clock_);
      if (activate == double_rows_ - 1 - row) {
        io->SetBits(data_);
      } else {
        io->ClearBits(data_);
      }
      io->SetBits(clock_);
    }
    io->SetBits(clock_);
    io->ClearBits(clock_);
    last_row_ = row;
  }

private:
  const int double_rows_;
  const gpio_bits_t row_mask_;
  const gpio_bits_t clock_;
  const gpio_bits_t data_;
  int last_row_;
};

// The DirectABCDRowAddressSetter sets the address by one of
// row pin ABCD for 32х16 matrix 1:4 multiplexing. The matrix has
// 4 addressable rows. Row is selected by a low level on the
// corresponding row address pin. Other row address pins must be in high level.
//
// Row addr| 0 | 1 | 2 | 3
// --------+---+---+---+---
// Line A  | 0 | 1 | 1 | 1
// Line B  | 1 | 0 | 1 | 1
// Line C  | 1 | 1 | 0 | 1
// Line D  | 1 | 1 | 1 | 0
class DirectABCDLineRowAddressSetter : public RowAddressSetter {
public:
  DirectABCDLineRowAddressSetter(int double_rows, const HardwareMapping &h)
    : last_row_(-1) {
	row_mask_ = h.a | h.b | h.c | h.d;

	row_lines_[0] = /*h.a |*/ h.b | h.c | h.d;
	row_lines_[1] = h.a /*| h.b*/ | h.c | h.d;
	row_lines_[2] = h.a | h.b /*| h.c */| h.d;
	row_lines_[3] = h.a | h.b | h.c /*| h.d*/;
  }

  virtual gpio_bits_t need_bits() const { return row_mask_; }

  virtual void SetRowAddress(GPIO *io, int row) {
    if (row == last_row_) return;

    gpio_bits_t row_address = row_lines_[row % 4];

    io->WriteMaskedBits(row_address, row_mask_);
    last_row_ = row;
  }

private:
  gpio_bits_t row_lines_[4];
  gpio_bits_t row_mask_;
  int last_row_;
};

// FM6373 RowAddressSetter - for row addressing
// Based on C# code, it uses shift registers for scan lines.
// Fixed: Added const HardwareMapping &h to constructor and row_lookup_
class FM6373_DP32019B_TEST : public RowAddressSetter {
public:
  FM6373_DP32019B_TEST(int double_rows, const HardwareMapping &h)
    : double_rows_(double_rows),
      row_mask_(h.a | h.b | h.c | h.d | h.e),  // Assuming 5 address lines for 32 rows
      clock_(h.clock),
      latch_(h.strobe),
      last_row_(-1) {
    // Precompute row lookup table
    for (int i = 0; i < double_rows_; ++i) {
      gpio_bits_t row_address = 0;
      row_address |= (i & 0x01) ? h.a : 0;
      row_address |= (i & 0x02) ? h.b : 0;
      row_address |= (i & 0x04) ? h.c : 0;
      row_address |= (i & 0x08) ? h.d : 0;
      row_address |= (i & 0x10) ? h.e : 0;
      row_lookup_[i] = row_address;
    }
  }

  virtual gpio_bits_t need_bits() const { return row_mask_ | clock_ | latch_; }

  virtual void SetRowAddress(GPIO *io, int row) {
    if (row == last_row_) return;
    // For FM6373, assuming parallel addressing for simplicity
    const gpio_bits_t bits = row_lookup_[row];
    io->WriteMaskedBits(bits, row_mask_);
    FM6373_Debug("[FM6373] SetRowAddress row=%d bits=0x%08x\n", row, bits);
    last_row_ = row;
  }

private:
  const int double_rows_;
  const gpio_bits_t row_mask_;
  const gpio_bits_t clock_;
  const gpio_bits_t latch_;
  gpio_bits_t row_lookup_[32];
  int last_row_;
};

}

const struct HardwareMapping *Framebuffer::hardware_mapping_ = NULL;
RowAddressSetter *Framebuffer::row_setter_ = NULL;

Framebuffer::Framebuffer(int rows, int columns, int parallel,
                         int scan_mode,
                         const char *led_sequence, bool inverse_color,
                         PixelDesignatorMap **mapper)
  : rows_(rows),
    parallel_(parallel),
    height_(rows * parallel),
    columns_(columns),
    scan_mode_(scan_mode),
    inverse_color_(inverse_color),
    pwm_bits_(kBitPlanes), do_luminance_correct_(true), brightness_(100),
    double_rows_(rows / SUB_PANELS_),
    buffer_size_(double_rows_ * columns_ * kBitPlanes * sizeof(gpio_bits_t)),
    shared_mapper_(mapper) {
  assert(hardware_mapping_ != NULL);   // Called InitHardwareMapping() ?
  assert(shared_mapper_ != NULL);  // Storage should be provided by RGBMatrix.
  assert(rows_ >=4 && rows_ <= 64 && rows_ % 2 == 0);
  if (parallel > hardware_mapping_->max_parallel_chains) {
    fprintf(stderr, "The %s GPIO mapping only supports %d parallel chain%s, "
            "but %d was requested.\n", hardware_mapping_->name,
            hardware_mapping_->max_parallel_chains,
            hardware_mapping_->max_parallel_chains > 1 ? "s" : "", parallel);
    abort();
  }
  assert(parallel >= 1 && parallel <= 6);

  bitplane_buffer_ = new gpio_bits_t[double_rows_ * columns_ * kBitPlanes];

  // If we're the first Framebuffer created, the shared PixelMapper is
  // still NULL, so create one.
  // The first PixelMapper represents the physical layout of a standard matrix
  // with the specific knowledge of the framebuffer, setting up PixelDesignators
  // in a way that they are useful for this Framebuffer.
  //
  // Newly created PixelMappers then can just re-arrange PixelDesignators
  // from the parent PixelMapper opaquely without having to know the details.
  if (*shared_mapper_ == NULL) {
    // Gather all the bits for given color for fast Fill()s and use the right
    // bits according to the led sequence
    const struct HardwareMapping &h = *hardware_mapping_;
    gpio_bits_t r = h.p0_r1 | h.p0_r2 | h.p1_r1 | h.p1_r2 | h.p2_r1 | h.p2_r2 | h.p3_r1 | h.p3_r2 | h.p4_r1 | h.p4_r2 | h.p5_r1 | h.p5_r2;
    gpio_bits_t g = h.p0_g1 | h.p0_g2 | h.p1_g1 | h.p1_g2 | h.p2_g1 | h.p2_g2 | h.p3_g1 | h.p3_g2 | h.p4_g1 | h.p4_g2 | h.p5_g1 | h.p5_g2;
    gpio_bits_t b = h.p0_b1 | h.p0_b2 | h.p1_b1 | h.p1_b2 | h.p2_b1 | h.p2_b2 | h.p3_b1 | h.p3_b2 | h.p4_b1 | h.p4_b2 | h.p5_b1 | h.p5_b2;
    PixelDesignator fill_bits;
    fill_bits.r_bit = GetGpioFromLedSequence('R', led_sequence, r, g, b);
    fill_bits.g_bit = GetGpioFromLedSequence('G', led_sequence, r, g, b);
    fill_bits.b_bit = GetGpioFromLedSequence('B', led_sequence, r, g, b);

    *shared_mapper_ = new PixelDesignatorMap(columns_, height_, fill_bits);
    for (int y = 0; y < height_; ++y) {
      for (int x = 0; x < columns_; ++x) {
        InitDefaultDesignator(x, y, led_sequence, (*shared_mapper_)->get(x, y));
      }
    }
  }

  Clear();
}

Framebuffer::~Framebuffer() {
  delete [] bitplane_buffer_;
}

// TODO: this should also be parsed from some special formatted string, e.g.
// {addr={22,23,24,25,15},oe=18,clk=17,strobe=4, p0={11,27,7,8,9,10},...}
/* static */ void Framebuffer::InitHardwareMapping(const char *named_hardware) {
  if (named_hardware == NULL || *named_hardware == '\0') {
    named_hardware = "regular";
  }

  struct HardwareMapping *mapping = NULL;
  for (HardwareMapping *it = matrix_hardware_mappings; it->name; ++it) {
    if (custom_strcasecmp(it->name, named_hardware) == 0) {
      mapping = it;
      break;
    }
  }

  if (!mapping) {
    fprintf(stderr, "There is no hardware mapping named '%s'.\nAvailable: ",
            named_hardware);
    for (HardwareMapping *it = matrix_hardware_mappings; it->name; ++it) {
      if (it != matrix_hardware_mappings) fprintf(stderr, ", ");
      fprintf(stderr, "'%s'", it->name);
    }
    fprintf(stderr, "\n");
    abort();
  }

  if (mapping->max_parallel_chains == 0) {
    // Auto determine.
    struct HardwareMapping *h = mapping;
    if ((h->p0_r1 | h->p0_g1 | h->p0_g1 | h->p0_r2 | h->p0_g2 | h->p0_g2) > 0)
      ++mapping->max_parallel_chains;
    if ((h->p1_r1 | h->p1_g1 | h->p1_g1 | h->p1_r2 | h->p1_g2 | h->p1_g2) > 0)
      ++mapping->max_parallel_chains;
    if ((h->p2_r1 | h->p2_g1 | h->p2_g1 | h->p2_r2 | h->p2_g2 | h->p2_g2) > 0)
      ++mapping->max_parallel_chains;
    if ((h->p3_r1 | h->p3_g1 | h->p3_g1 | h->p3_r2 | h->p3_g2 | h->p3_g2) > 0)
      ++mapping->max_parallel_chains;
    if ((h->p4_r1 | h->p4_g1 | h->p4_g1 | h->p4_r2 | h->p4_g2 | h->p4_g2) > 0)
      ++mapping->max_parallel_chains;
    if ((h->p5_r1 | h->p5_g1 | h->p5_g1 | h->p5_r2 | h->p5_g2 | h->p5_g2) > 0)
      ++mapping->max_parallel_chains;
  }
  hardware_mapping_ = mapping;
}

/* static */ void Framebuffer::InitGPIO(GPIO *io, int rows, int parallel,
                                        bool allow_hardware_pulsing,
                                        int pwm_lsb_nanoseconds,
                                        int dither_bits,
                                        int row_address_type) {
  if (sOutputEnablePulser != NULL)
    return;  // already initialized.

  const struct HardwareMapping &h = *hardware_mapping_;
  // Tell GPIO about all bits we intend to use.
  gpio_bits_t all_used_bits = 0;

  all_used_bits |= h.output_enable | h.clock | h.strobe;

  all_used_bits |= h.p0_r1 | h.p0_g1 | h.p0_b1 | h.p0_r2 | h.p0_g2 | h.p0_b2;
  if (parallel >= 2) {
    all_used_bits |= h.p1_r1 | h.p1_g1 | h.p1_b1 | h.p1_r2 | h.p1_g2 | h.p1_b2;
  }
  if (parallel >= 3) {
    all_used_bits |= h.p2_r1 | h.p2_g1 | h.p2_b1 | h.p2_r2 | h.p2_g2 | h.p2_b2;
  }
  if (parallel >= 4) {
    all_used_bits |= h.p3_r1 | h.p3_g1 | h.p3_b1 | h.p3_r2 | h.p3_g2 | h.p3_b2;
  }
  if (parallel >= 5) {
    all_used_bits |= h.p4_r1 | h.p4_g1 | h.p4_b1 | h.p4_r2 | h.p4_g2 | h.p4_b2;
  }
  if (parallel >= 6) {
    all_used_bits |= h.p5_r1 | h.p5_g1 | h.p5_b1 | h.p5_r2 | h.p5_g2 | h.p5_b2;
  }

  const int double_rows = rows / SUB_PANELS_;
  switch (row_address_type) {
  case 0:
    row_setter_ = new DirectRowAddressSetter(double_rows, h);
    break;
  case 1:
    row_setter_ = new ShiftRegisterRowAddressSetter(double_rows, h);
    break;
  case 2:
    row_setter_ = new DirectABCDLineRowAddressSetter(double_rows, h);
    break;
  case 3:
    row_setter_ = new ABCShiftRegisterRowAddressSetter(double_rows, h);
    break;
  case 4:
    row_setter_ = new SM5266RowAddressSetter(double_rows, h);
    break;
  case 5:
    row_setter_ = new B707ShiftRegisterRowAddressSetter(double_rows, h);
    break;
  case 6:  // For FM6373
    row_setter_ = new FM6373_DP32019B_TEST(double_rows, h);
    break;
  default:
    assert(0);  // unexpected type.
  }

  all_used_bits |= row_setter_->need_bits();

  // Adafruit HAT identified by the same prefix.
  const bool is_some_adafruit_hat = (0 == custom_strncasecmp(h.name, "adafruit-hat",
                                                  strlen("adafruit-hat")));
  // Initialize outputs, make sure that all of these are supported bits.
  const gpio_bits_t result = io->InitOutputs(all_used_bits,
                                             is_some_adafruit_hat);
  assert(result == all_used_bits);  // Impl: all bits declared in gpio.cc ?

  std::vector<int> bitplane_timings;
  uint32_t timing_ns = pwm_lsb_nanoseconds;
  for (int b = 0; b < kBitPlanes; ++b) {
    bitplane_timings.push_back(timing_ns);
    if (b >= dither_bits) timing_ns *= 2;
  }
  sOutputEnablePulser = PinPulser::Create(io, h.output_enable,
                                          allow_hardware_pulsing,
                                          bitplane_timings);
}

// NOTE: first version for panel initialization sequence, need to refine
// until it is more clear how different panel types are initialized to be
// able to abstract this more.

static void InitFM6126(GPIO *io, const struct HardwareMapping &h, int columns) {
  const gpio_bits_t bits_on
    = h.p0_r1 | h.p0_g1 | h.p0_b1 | h.p0_r2 | h.p0_g2 | h.p0_b2
    | h.p1_r1 | h.p1_g1 | h.p1_b1 | h.p1_r2 | h.p1_g2 | h.p1_b2
    | h.p2_r1 | h.p2_g1 | h.p2_b1 | h.p2_r2 | h.p2_g2 | h.p2_b2
    | h.p3_r1 | h.p3_g1 | h.p3_b1 | h.p3_r2 | h.p3_g2 | h.p3_b2
    | h.p4_r1 | h.p4_g1 | h.p4_b1 | h.p4_r2 | h.p4_g2 | h.p4_b2
    | h.p5_r1 | h.p5_g1 | h.p5_b1 | h.p5_r2 | h.p5_g2 | h.p5_b2
    | h.a;  // Address bit 'A' is always on.
  const gpio_bits_t bits_off = h.a;
  const gpio_bits_t mask = bits_on | h.strobe;

  // Init bits. TODO: customize, as we can do things such as brightness here,
  // which would allow more higher quality output.
  static const char* init_b12 = "0111111111111111";  // full bright
  static const char* init_b13 = "0000000001000000";  // panel on.

  io->ClearBits(h.clock | h.strobe);

  for (int i = 0; i < columns; ++i) {
    gpio_bits_t value = init_b12[i % 16] == '0' ? bits_off : bits_on;
    if (i > columns - 12) value |= h.strobe;
    io->WriteMaskedBits(value, mask);
    io->SetBits(h.clock);
    io->ClearBits(h.clock);
  }
  io->ClearBits(h.strobe);

  for (int i = 0; i < columns; ++i) {
    gpio_bits_t value = init_b13[i % 16] == '0' ? bits_off : bits_on;
    if (i > columns - 13) value |= h.strobe;
    io->WriteMaskedBits(value, mask);
    io->SetBits(h.clock);
    io->ClearBits(h.clock);
  }
  io->ClearBits(h.strobe);
}

// The FM6217 is very similar to the FM6216.
// FM6217 adds Register 3 to allow for automatic bad pixel suppression.
static void InitFM6127(GPIO *io, const struct HardwareMapping &h, int columns) {
  const gpio_bits_t bits_r_on= h.p0_r1 | h.p0_r2;
  const gpio_bits_t bits_g_on= h.p0_g1 | h.p0_g2;
  const gpio_bits_t bits_b_on= h.p0_b1 | h.p0_b2;
  const gpio_bits_t bits_on= bits_r_on | bits_g_on | bits_b_on;
  const gpio_bits_t bits_off = 0;

  const gpio_bits_t mask = bits_on | h.strobe;

  static const char* init_b12 = "1111111111001110";  // register 1
  static const char* init_b13 = "1110000001100010";  // register 2.
  static const char* init_b11 = "0101111100000000";  // register 3.
  io->ClearBits(h.clock | h.strobe);
  for (int i = 0; i < columns; ++i) {
    gpio_bits_t value = init_b12[i % 16] == '0' ? bits_off : bits_on;
    if (i > columns - 12) value |= h.strobe;
    io->WriteMaskedBits(value, mask);
    io->SetBits(h.clock);
    io->ClearBits(h.clock);
  }
  io->ClearBits(h.strobe);

  for (int i = 0; i < columns; ++i) {
    gpio_bits_t value = init_b13[i % 16] == '0' ? bits_off : bits_on;
    if (i > columns - 13) value |= h.strobe;
    io->WriteMaskedBits(value, mask);
    io->SetBits(h.clock);
    io->ClearBits(h.clock);
  }
  io->ClearBits(h.strobe);

  for (int i = 0; i < columns; ++i) {
    gpio_bits_t value = init_b11[i % 16] == '0' ? bits_off : bits_on;
    if (i > columns - 11) value |= h.strobe;
    io->WriteMaskedBits(value, mask);
    io->SetBits(h.clock);
    io->ClearBits(h.clock);
  }
  io->ClearBits(h.strobe);
}


// Set A..E pins to a given row index (0..31).
static inline void FM6373_SetRowBits(GPIO *io,
                                     const HardwareMapping &h,
                                     uint8_t row) {
                                      
  // Mirror the logic-capture timing: change address lines while DCLK is high.
  const gpio_bits_t mask = h.a | h.b | h.c | h.d | h.e;
  gpio_bits_t bits = 0;
  if (row & 0x01) bits |= h.a;
  if (row & 0x02) bits |= h.b;
  if (row & 0x04) bits |= h.c;
  if (row & 0x08) bits |= h.d;
  if (row & 0x10) bits |= h.e;

  io->WriteMaskedBits(bits, mask);

  FM6373_Debug("[FM6373] RowBits row=%u A=%d B=%d C=%d D=%d E=%d mask=0x%08x\n",
               row,
               (row & 0x01) != 0, (row & 0x02) != 0, (row & 0x04) != 0,
               (row & 0x08) != 0, (row & 0x10) != 0,
               mask);
}


// Each sequence is one header word followed by the same data word for each
// cascaded column-driver (7 copies -> 8 drivers total). The capture shows the
// eighth word's lower five bits delivered by the subsequent LEConfig5 pulse, so
// we keep only the upper 11 bits of that eighth word here and feed the tail bits
// to PWM_Display_Register_Send_Last_Word.

// Helper to create an 8-word block with the same 16-bit value.
#define FM6373_REPEAT8(value) \
  { (value), (value), (value), (value), (value), (value), (value), (value) }

static const uint16_t register_block_1[] = {
  0x0040,  // header
  0x00AA, 0x00AA, 0x00AA, 0x00AA,
  0x00AA, 0x00AA, 0x00AA, 0x00AA
};

static const uint16_t register_block_2[] = FM6373_REPEAT8(0x01AA);
static const uint16_t register_block_3[] = FM6373_REPEAT8(0xF400);
static const uint16_t register_block_4[] = FM6373_REPEAT8(0x0055);
static const uint16_t register_block_5[] = FM6373_REPEAT8(0x0155);

#undef FM6373_REPEAT8

struct PWM_DISPLAY_REGISTER_DATA {
  // Sequence of 16-bit words to send MSB-first on RGB lines.
  const uint16_t *words;
  size_t          word_count;

  // If true, the last word is split: the upper partial_bit_count bits are
  // shifted out during the preload, and the remaining lower bits are returned
  // to be replayed as a short LAT pattern.
  bool     has_partial_word;
  uint16_t partial_word;
  uint8_t  partial_bit_count;  // Number of MSBs to emit during preload.
};

// Helper to declare a row config with the usual "split last word" pattern.
#define FM6373_ROW_CONFIG(name, block, partial_word_literal) \
  static const PWM_DISPLAY_REGISTER_DATA name = {            \
    block,                                                   \
    sizeof(block) / sizeof((block)[0]),                      \
    true,                                                    \
    (partial_word_literal),                                  \
    11                                                       \
  }

// Last parameter is for the split word which is partially sent on the LAT pulse
FM6373_ROW_CONFIG(fm6373_row1_config, register_block_1, 0x00AA);
FM6373_ROW_CONFIG(fm6373_row2_config, register_block_2, 0x01AA);
FM6373_ROW_CONFIG(fm6373_row3_config, register_block_3, 0xF400);
FM6373_ROW_CONFIG(fm6373_row4_config, register_block_4, 0x0055);
FM6373_ROW_CONFIG(fm6373_row5_config, register_block_5, 0x0155);

#undef FM6373_ROW_CONFIG

// Table of configs indexed by (row - 1) for rows 1..5.
static const PWM_DISPLAY_REGISTER_DATA *const k_fm6373_row_configs[] = {
  &fm6373_row1_config,
  &fm6373_row2_config,
  &fm6373_row3_config,
  &fm6373_row4_config,
  &fm6373_row5_config,
};

// If other rows have different sequences, add them to k_fm6373_row_configs.
// If a row matches row1, you can simply re-use fm6373_row1_config.
static const PWM_DISPLAY_REGISTER_DATA *fm6373_row_preload_config_for_row(
    uint8_t row) {
  const size_t count =
      sizeof(k_fm6373_row_configs) / sizeof(k_fm6373_row_configs[0]);

  if (row >= 1 && row <= count) {
    return k_fm6373_row_configs[row - 1];
  }
  return NULL;
}


// Shift 'bit_count' MSBs from 'value' out on all RGB data lines.
// DCLK is toggled for each bit; LAT remains untouched here.
static inline void ShiftMSBFirst(GPIO *io,
                                 const HardwareMapping &h,
                                 gpio_bits_t rgb_mask,
                                 uint16_t value,
                                 int bit_count) {
  if (bit_count <= 0) return;
  if (bit_count > 16) bit_count = 16;

  FM6373_Debug("[FM6373] ShiftMSBFirst value=0x%04x bits=%d rgb_mask=0x%08x\n",
               value, bit_count, rgb_mask);

  for (int bit = 15; bit >= 16 - bit_count; --bit) {
    const gpio_bits_t data_bits = (value & (1 << bit)) ? rgb_mask : 0;
    io->WriteMaskedBits(data_bits, rgb_mask);
    io->SetBits(h.clock);
    io->ClearBits(h.clock);
  }
}

// Send 'pulses' LE (LAT) pulses with LAT high and data lines low by default.
// Optionally, 'pattern_bits' (up to 8 bits) can be replayed on the data lines
// during the first pattern_bit_count clocks to emit tail bits for a split word.
static void PWM_Display_Send_LAT_Pulses(GPIO *io,
                                        const HardwareMapping &h,
                                        uint8_t row,
                                        int pulses,
                                        uint8_t pattern_bits,
                                        int pattern_bit_count) {
  FM6373_Debug("[FM6373] LEPulses row=%u pulses=%d pattern=0x%02x bits=%d\n",
               row, pulses, pattern_bits, pattern_bit_count);

  const gpio_bits_t data_mask =
      h.p0_r1 | h.p0_g1 | h.p0_b1 |
      h.p0_r2 | h.p0_g2 | h.p0_b2 |
      h.p1_r1 | h.p1_g1 | h.p1_b1 |
      h.p1_r2 | h.p1_g2 | h.p1_b2;

  io->ClearBits(data_mask);
  io->ClearBits(h.strobe);
  io->ClearBits(h.clock);         // back low before LAT pulse train
  io->SetBits(h.strobe);  // LAT high

  if (pattern_bit_count > 0) {
    const int total = (pattern_bit_count > pulses) ? pattern_bit_count : pulses;
    for (int i = 0; i < total; ++i) {
      const int bit_index = pattern_bit_count - 1 - i;
      const bool bit_on =
          (bit_index >= 0) ? ((pattern_bits >> bit_index) & 0x01) : false;
      const gpio_bits_t bits = bit_on ? data_mask : 0;
      io->WriteMaskedBits(bits, data_mask);
      io->SetBits(h.clock);
      io->ClearBits(h.clock);
    }
  } else {
    for (int i = 0; i < pulses; ++i) {
      io->SetBits(h.clock);
      io->ClearBits(h.clock);
    }
  }

  io->ClearBits(h.strobe);
  io->SetBits(h.clock);
  FM6373_SetRowBits(io, h, row);
}


// Send 16-bit words on all RGB data lines with LAT low.
// Returns how many tail bits remain from the final word (LSB-aligned) so they
// can be replayed during a LAT-pulse pattern. The tail pattern (up to 8 bits)
// is written into *tail_bits_out if non-NULL.
static int PWM_Display_Register_Send(GPIO *io,
                                     const HardwareMapping &h,
                                     uint8_t row,
                                     uint8_t *tail_bits_out) {
  const PWM_DISPLAY_REGISTER_DATA *config =
      fm6373_row_preload_config_for_row(row);
  if (config == NULL || config->word_count == 0 || config->words == NULL) {
    FM6373_Debug("[FM6373] RegSend row=%u skipped (no config)\n", row);
    return 0;  // nothing to do
  }

  FM6373_Debug("[FM6373] RegSend row=%u words=%zu partial=%d partial_bits=%u\n",
               row, config->word_count,
               config->has_partial_word, config->partial_bit_count);

  // Mask of all RGB outputs we want to drive in parallel.
  const gpio_bits_t rgb_mask =
      h.p0_r1 | h.p0_g1 | h.p0_b1 |
      h.p0_r2 | h.p0_g2 | h.p0_b2 |
      h.p1_r1 | h.p1_g1 | h.p1_b1 |
      h.p1_r2 | h.p1_g2 | h.p1_b2;

  // Ensure LAT is low during the preload phase (acts as CS active).
  io->ClearBits(h.strobe);
  io->ClearBits(h.clock);
  io->ClearBits(rgb_mask);

  // Send all but the last word.
  const size_t word_count = config->word_count;
  if (word_count == 0) return 0;
  const size_t last_index = word_count - 1;

  size_t clocks_sent = 0;
  for (size_t i = 0; i + 1 < word_count; ++i) {
    ShiftMSBFirst(io, h, rgb_mask, config->words[i], 16);
    clocks_sent += 16;
  }

  int tail_bits = 0;
  uint8_t tail_pattern = 0;

  if (config->has_partial_word && config->partial_bit_count > 0) {
    const uint16_t last_word = config->words[last_index];
    // Send just the upper bits (e.g. first 11) here.
    ShiftMSBFirst(io, h, rgb_mask, last_word, config->partial_bit_count);
    clocks_sent += config->partial_bit_count;

    tail_bits = 16 - config->partial_bit_count;
    if (tail_bits > 0 && tail_bits <= 8) {
      tail_pattern = static_cast<uint8_t>(last_word & ((1 << tail_bits) - 1));
      FM6373_Debug(
          "[FM6373] RegSend tail row=%u word=0x%04x tail_bits=%d "
          "tail_pattern=0x%02x clocks=%zu\n",
          row, last_word, tail_bits, tail_pattern, clocks_sent);
    } else {
      // Either no bits left or more than 8 bits, which we don't support as
      // a simple LAT pattern; in that case we simply ignore the split.
      tail_bits = 0;
    }
  } else {
    // No split needed, send the full word.
    ShiftMSBFirst(io, h, rgb_mask, config->words[last_index], 16);
    clocks_sent += 16;
  }

  // Leave data lines low at the end, LAT still low.
  io->ClearBits(rgb_mask);
  io->ClearBits(h.clock);

  if (tail_bits_out) *tail_bits_out = tail_pattern;
  return tail_bits;
}


// Frame-level preamble for FM6373:
//   row 0, LE=3   (VSYNC-like)
//   row 0, LE=11  (EN_OP-like)
//   row 0, LE=14  (PRE_ACT-like)
// followed by 5 register blocks (each 8×16-bit words).
//
// PWM_Display_Register_Send()
//   Sends the first 7 of 8 words, then only the upper 11 bits of the 8th.
//   The remaining lower 5 bits are emitted via a LAT pattern (5 clocks) to
//   match the observed timing.
static void FM6373_Init_And_Registers(GPIO *io, const HardwareMapping &h) {
  FM6373_Debug("[FM6373] === Frame preamble start ===\n");

  // Row 0 command pulses: VSYNC / EN_OP / PRE_ACT equivalents.
  PWM_Display_Send_LAT_Pulses(io, h, 0, 3, 0, 0);
  PWM_Display_Send_LAT_Pulses(io, h, 0, 11, 0, 0);
  PWM_Display_Send_LAT_Pulses(io, h, 0, 14, 0, 0);

  // Register 1
  uint8_t tail_bits = 0;
  int tail_count = PWM_Display_Register_Send(io, h, 1, &tail_bits);
  PWM_Display_Send_LAT_Pulses(io, h, 0, 5, tail_bits, tail_count);

  // Register 2
  tail_count = PWM_Display_Register_Send(io, h, 2, &tail_bits);
  PWM_Display_Send_LAT_Pulses(io, h, 0, 5, tail_bits, tail_count);

  // Register 3
  tail_count = PWM_Display_Register_Send(io, h, 3, &tail_bits);
  PWM_Display_Send_LAT_Pulses(io, h, 0, 5, tail_bits, tail_count);

  // Register 4
  tail_count = PWM_Display_Register_Send(io, h, 4, &tail_bits);
  PWM_Display_Send_LAT_Pulses(io, h, 0, 5, tail_bits, tail_count);

  // Register 5
  tail_count = PWM_Display_Register_Send(io, h, 5, &tail_bits);
  PWM_Display_Send_LAT_Pulses(io, h, 0, 5, tail_bits, tail_count);

  FM6373_Debug("[FM6373] === Frame preamble end ===\n");
}

bool Framebuffer::SetPWMBits(uint8_t value) {
  if (value < 1 || value > kBitPlanes)
    return false;
  pwm_bits_ = value;
  return true;
}

inline gpio_bits_t *Framebuffer::ValueAt(int double_row, int column, int bit) {
  return &bitplane_buffer_[ double_row * (columns_ * kBitPlanes)
                            + bit * columns_
                            + column ];
}

void Framebuffer::Clear() {
  if (inverse_color_) {
    Fill(0, 0, 0);
  } else  {
    // Cheaper.
    memset(bitplane_buffer_, 0,
           sizeof(*bitplane_buffer_) * double_rows_ * columns_ * kBitPlanes);
  }
}

struct ColorLookup {
  uint16_t color[256];
};

class ColorLookupTable {
  public:
    static const ColorLookup &GetLookup(uint8_t brightness) {
      static ColorLookupTable instance;
      return instance.lookups_[brightness - 1];
    }

    
  private:
    // Do CIE1931 luminance correction and scale to output bitplanes
    static uint16_t luminance_cie1931(uint8_t c, uint8_t brightness) {
      float out_factor = ((1 << internal::Framebuffer::kBitPlanes) - 1);
      float v = (float) c * brightness / 255.0;
      return roundf(out_factor * ((v <= 8) ? v / 902.3 : pow((v + 16) / 116.0, 3)));
    }

    ColorLookupTable() {
      for (int c = 0; c < 256; ++c)
        for (int b = 0; b < 100; ++b)
          lookups_[b].color[c] = luminance_cie1931(c, b + 1);
    }

    ColorLookup lookups_[100]{};
  };
  
  static inline uint16_t CIEMapColor(uint8_t brightness, uint8_t c) {
    return ColorLookupTable::GetLookup(brightness).color[c];
  }

// Non luminance correction. TODO: consider getting rid of this.
static inline uint16_t DirectMapColor(uint8_t brightness, uint8_t c) {
  // simple scale down the color value
  c = c * brightness / 100;

  // shift to be left aligned with top-most bits.
  constexpr int shift = internal::Framebuffer::kBitPlanes - 8;
  return (shift > 0) ? (c << shift) : (c >> -shift);
}

inline void Framebuffer::MapColors(
  uint8_t r, uint8_t g, uint8_t b,
  uint16_t *red, uint16_t *green, uint16_t *blue) {

  if (do_luminance_correct_) {
    *red   = CIEMapColor(brightness_, r);
    *green = CIEMapColor(brightness_, g);
    *blue  = CIEMapColor(brightness_, b);
  } else {
    *red   = DirectMapColor(brightness_, r);
    *green = DirectMapColor(brightness_, g);
    *blue  = DirectMapColor(brightness_, b);
  }

  if (inverse_color_) {
    *red = ~(*red);
    *green = ~(*green);
    *blue = ~(*blue);
  }
}

void Framebuffer::Fill(uint8_t r, uint8_t g, uint8_t b) {
  uint16_t red, green, blue;
  MapColors(r, g, b, &red, &green, &blue);
  const PixelDesignator &fill = (*shared_mapper_)->GetFillColorBits();

  for (int bits = kBitPlanes - pwm_bits_; bits < kBitPlanes; ++bits) {
    uint16_t mask = 1 << bits;
    gpio_bits_t plane_bits = 0;
    plane_bits |= ((red & mask) == mask)   ? fill.r_bit : 0;
    plane_bits |= ((green & mask) == mask) ? fill.g_bit : 0;
    plane_bits |= ((blue & mask) == mask)  ? fill.b_bit : 0;

    for (int row = 0; row < double_rows_; ++row) {
      gpio_bits_t *row_data = ValueAt(row, 0, bits);
      for (int col = 0; col < columns_; ++col) {
        *row_data++ = plane_bits;
      }
    }
  }
}

void Framebuffer::SubFill(int x, int y, int width, int height, uint8_t r, uint8_t g, uint8_t b) {

  uint16_t red, green, blue;
  MapColors(r, g, b, &red, &green, &blue);

  int safe_y = std::max(0, y);
  int safe_y_max = std::min((*shared_mapper_)->height(), y + height);
  int safe_x = std::max(0, x);
  int safe_x_max = std::min((*shared_mapper_)->width(), x + width);

  for (int row = safe_y; row < safe_y_max; row++)
  {
    const PixelDesignator* designator = (*shared_mapper_)->get(safe_x, row);

    for (int col = safe_x; col < safe_x_max; col++)
    {
      if (designator == NULL) continue;
      const long pos = designator->gpio_word;
      if (pos < 0) continue;  // non-used pixel marker.

      gpio_bits_t* bits = bitplane_buffer_ + pos;
      const int min_bit_plane = kBitPlanes - pwm_bits_;
      bits += (columns_ * min_bit_plane);
      const gpio_bits_t r_bits = designator->r_bit;
      const gpio_bits_t g_bits = designator->g_bit;
      const gpio_bits_t b_bits = designator->b_bit;
      const gpio_bits_t designator_mask = designator->mask;
      for (uint16_t mask = 1 << min_bit_plane; mask != 1 << kBitPlanes; mask <<= 1) {
        gpio_bits_t color_bits = 0;
        if (red & mask)   color_bits |= r_bits;
        if (green & mask) color_bits |= g_bits;
        if (blue & mask)  color_bits |= b_bits;
        *bits = (*bits & designator_mask) | color_bits;
        bits += columns_;
      }
      designator++;
    }
  }
}

int Framebuffer::width() const { return (*shared_mapper_)->width(); }
int Framebuffer::height() const { return (*shared_mapper_)->height(); }

void Framebuffer::SetPixel(int x, int y, uint8_t r, uint8_t g, uint8_t b) {
  const PixelDesignator *designator = (*shared_mapper_)->get(x, y);
  if (designator == NULL) return;
  const long pos = designator->gpio_word;
  if (pos < 0) return;  // non-used pixel marker.

  uint16_t red, green, blue;
  MapColors(r, g, b, &red, &green, &blue);

  gpio_bits_t *bits = bitplane_buffer_ + pos;
  const int min_bit_plane = kBitPlanes - pwm_bits_;
  bits += (columns_ * min_bit_plane);
  const gpio_bits_t r_bits = designator->r_bit;
  const gpio_bits_t g_bits = designator->g_bit;
  const gpio_bits_t b_bits = designator->b_bit;
  const gpio_bits_t designator_mask = designator->mask;
  for (uint16_t mask = 1<<min_bit_plane; mask != 1<<kBitPlanes; mask <<=1 ) {
    gpio_bits_t color_bits = 0;
    if (red & mask)   color_bits |= r_bits;
    if (green & mask) color_bits |= g_bits;
    if (blue & mask)  color_bits |= b_bits;
    *bits = (*bits & designator_mask) | color_bits;
    bits += columns_;
  }
}

void Framebuffer::SetPixels(int x, int y, int width, int height, Color *colors) {
  for (int iy = 0; iy < height; ++iy) {
    for (int ix = 0; ix < width; ++ix) {
      SetPixel(x + ix, y + iy, colors->r, colors->g, colors->b);
      ++colors;
    }
  }
}

// Strange LED-mappings such as RBG or so are handled here.
gpio_bits_t Framebuffer::GetGpioFromLedSequence(char col,
                                                const char *led_sequence,
                                                gpio_bits_t default_r,
                                                gpio_bits_t default_g,
                                                gpio_bits_t default_b) {
  const char *pos = strchr(led_sequence, col);
  if (pos == NULL) pos = strchr(led_sequence, tolower(col));
  if (pos == NULL) {
    fprintf(stderr, "LED sequence '%s' does not contain any '%c'.\n",
            led_sequence, col);
    abort();
  }
  switch (pos - led_sequence) {
  case 0: return default_r;
  case 1: return default_g;
  case 2: return default_b;
  }
  return default_r;  // String too long, should've been caught earlier.
}

void Framebuffer::InitDefaultDesignator(int x, int y, const char *seq,
                                        PixelDesignator *d) {
  const struct HardwareMapping &h = *hardware_mapping_;
  gpio_bits_t *bits = ValueAt(y % double_rows_, x, 0);
  d->gpio_word = bits - bitplane_buffer_;
  d->r_bit = d->g_bit = d->b_bit = 0;
  if (y < rows_) {
    if (y < double_rows_) {
      d->r_bit = GetGpioFromLedSequence('R', seq, h.p0_r1, h.p0_g1, h.p0_b1);
      d->g_bit = GetGpioFromLedSequence('G', seq, h.p0_r1, h.p0_g1, h.p0_b1);
      d->b_bit = GetGpioFromLedSequence('B', seq, h.p0_r1, h.p0_g1, h.p0_b1);
    } else {
      d->r_bit = GetGpioFromLedSequence('R', seq, h.p0_r2, h.p0_g2, h.p0_b2);
      d->g_bit = GetGpioFromLedSequence('G', seq, h.p0_r2, h.p0_g2, h.p0_b2);
      d->b_bit = GetGpioFromLedSequence('B', seq, h.p0_r2, h.p0_g2, h.p0_b2);
    }
  }
  else if (y >= rows_ && y < 2 * rows_) {
    if (y - rows_ < double_rows_) {
      d->r_bit = GetGpioFromLedSequence('R', seq, h.p1_r1, h.p1_g1, h.p1_b1);
      d->g_bit = GetGpioFromLedSequence('G', seq, h.p1_r1, h.p1_g1, h.p1_b1);
      d->b_bit = GetGpioFromLedSequence('B', seq, h.p1_r1, h.p1_g1, h.p1_b1);
    } else {
      d->r_bit = GetGpioFromLedSequence('R', seq, h.p1_r2, h.p1_g2, h.p1_b2);
      d->g_bit = GetGpioFromLedSequence('G', seq, h.p1_r2, h.p1_g2, h.p1_b2);
      d->b_bit = GetGpioFromLedSequence('B', seq, h.p1_r2, h.p1_g2, h.p1_b2);
    }
  }
  else if (y >= 2*rows_ && y < 3 * rows_) {
    if (y - 2*rows_ < double_rows_) {
      d->r_bit = GetGpioFromLedSequence('R', seq, h.p2_r1, h.p2_g1, h.p2_b1);
      d->g_bit = GetGpioFromLedSequence('G', seq, h.p2_r1, h.p2_g1, h.p2_b1);
      d->b_bit = GetGpioFromLedSequence('B', seq, h.p2_r1, h.p2_g1, h.p2_b1);
    } else {
      d->r_bit = GetGpioFromLedSequence('R', seq, h.p2_r2, h.p2_g2, h.p2_b2);
      d->g_bit = GetGpioFromLedSequence('G', seq, h.p2_r2, h.p2_g2, h.p2_b2);
      d->b_bit = GetGpioFromLedSequence('B', seq, h.p2_r2, h.p2_g2, h.p2_b2);
    }
  }
  else if (y >= 3*rows_ && y < 4 * rows_) {
    if (y - 3*rows_ < double_rows_) {
      d->r_bit = GetGpioFromLedSequence('R', seq, h.p3_r1, h.p3_g1, h.p3_b1);
      d->g_bit = GetGpioFromLedSequence('G', seq, h.p3_r1, h.p3_g1, h.p3_b1);
      d->b_bit = GetGpioFromLedSequence('B', seq, h.p3_r1, h.p3_g1, h.p3_b1);
    } else {
      d->r_bit = GetGpioFromLedSequence('R', seq, h.p3_r2, h.p3_g2, h.p3_b2);
      d->g_bit = GetGpioFromLedSequence('G', seq, h.p3_r2, h.p3_g2, h.p3_b2);
      d->b_bit = GetGpioFromLedSequence('B', seq, h.p3_r2, h.p3_g2, h.p3_b2);
    }
  }
  else if (y >= 4*rows_ && y < 5 * rows_){
    if (y - 4*rows_ < double_rows_) {
      d->r_bit = GetGpioFromLedSequence('R', seq, h.p4_r1, h.p4_g1, h.p4_b1);
      d->g_bit = GetGpioFromLedSequence('G', seq, h.p4_r1, h.p4_g1, h.p4_b1);
      d->b_bit = GetGpioFromLedSequence('B', seq, h.p4_r1, h.p4_g1, h.p4_b1);
    } else {
      d->r_bit = GetGpioFromLedSequence('R', seq, h.p4_r2, h.p4_g2, h.p4_b2);
      d->g_bit = GetGpioFromLedSequence('G', seq, h.p4_r2, h.p4_g2, h.p4_b2);
      d->b_bit = GetGpioFromLedSequence('B', seq, h.p4_r2, h.p4_g2, h.p4_b2);
    }

  }
  else {
    if (y - 5*rows_ < double_rows_) {
      d->r_bit = GetGpioFromLedSequence('R', seq, h.p5_r1, h.p5_g1, h.p5_b1);
      d->g_bit = GetGpioFromLedSequence('G', seq, h.p5_r1, h.p5_g1, h.p5_b1);
      d->b_bit = GetGpioFromLedSequence('B', seq, h.p5_r1, h.p5_g1, h.p5_b1);
    } else {
      d->r_bit = GetGpioFromLedSequence('R', seq, h.p5_r2, h.p5_g2, h.p5_b2);
      d->g_bit = GetGpioFromLedSequence('G', seq, h.p5_r2, h.p5_g2, h.p5_b2);
      d->b_bit = GetGpioFromLedSequence('B', seq, h.p5_r2, h.p5_g2, h.p5_b2);
    }
  }

  d->mask = ~(d->r_bit | d->g_bit | d->b_bit);
}

void Framebuffer::Serialize(const char **data, size_t *len) const {
  *data = reinterpret_cast<const char*>(bitplane_buffer_);
  *len = buffer_size_;
}

bool Framebuffer::Deserialize(const char *data, size_t len) {
  if (len != buffer_size_) return false;
  memcpy(bitplane_buffer_, data, len);
  return true;
}

void Framebuffer::CopyFrom(const Framebuffer *other) {
  if (other == this) return;
  memcpy(bitplane_buffer_, other->bitplane_buffer_, buffer_size_);
}

void Framebuffer::DumpToMatrix(GPIO *io, int pwm_low_bit) {
  const struct HardwareMapping &h = *hardware_mapping_;
  gpio_bits_t color_clk_mask = 0;  // Mask of bits while clocking in.
  color_clk_mask |= h.p0_r1 | h.p0_g1 | h.p0_b1 | h.p0_r2 | h.p0_g2 | h.p0_b2;
  if (parallel_ >= 2) {
    color_clk_mask |= h.p1_r1 | h.p1_g1 | h.p1_b1 | h.p1_r2 | h.p1_g2 | h.p1_b2;
  }
  if (parallel_ >= 3) {
    color_clk_mask |= h.p2_r1 | h.p2_g1 | h.p2_b1 | h.p2_r2 | h.p2_g2 | h.p2_b2;
  }
  if (parallel_ >= 4) {
    color_clk_mask |= h.p3_r1 | h.p3_g1 | h.p3_b1 | h.p3_r2 | h.p3_g2 | h.p3_b2;
  }
  if (parallel_ >= 5) {
    color_clk_mask |= h.p4_r1 | h.p4_g1 | h.p4_b1 | h.p4_r2 | h.p4_g2 | h.p4_b2;
  }
  if (parallel_ >= 6) {
    color_clk_mask |= h.p5_r1 | h.p5_g1 | h.p5_b1 | h.p5_r2 | h.p5_g2 | h.p5_b2;
  }

  color_clk_mask |= h.clock;

  // For FM6373 panels, inject the VSYNC / EN_OP / PRE_ACT + config preamble
  // right at the start of each frame, matching the logic capture. Add a long
  // inter-frame pause so logic analyzers can easily spot frame boundaries.
  if (sFM6373_Enabled) {
    DumpToMatrixFM6373(io); // Implement this new function
    return;
  }
  

  // Depending if we do dithering, we might not always show the lowest bits.
  const int start_bit = std::max(pwm_low_bit, kBitPlanes - pwm_bits_);

  const uint8_t half_double = double_rows_/2;
  for (uint8_t row_loop = 0; row_loop < double_rows_; ++row_loop) {
    uint8_t d_row;
    switch (scan_mode_) {
    case 0:  // progressive
    default:
      d_row = row_loop;
      break;

    case 1:  // interlaced
      d_row = ((row_loop < half_double)
               ? (row_loop << 1)
               : ((row_loop - half_double) << 1) + 1);
    }

    // Rows can't be switched very quickly without ghosting, so we do the
    // full PWM of one row before switching rows.
    for (int b = start_bit; b < kBitPlanes; ++b) {
      gpio_bits_t *row_data = ValueAt(d_row, 0, b);
      // While the output enable is still on, we can already clock in the next
      // data.
      for (int col = 0; col < columns_; ++col) {
        const gpio_bits_t &out = *row_data++;
        io->WriteMaskedBits(out, color_clk_mask);  // col + reset clock
        io->SetBits(h.clock);               // Rising edge: clock color in.
      }
      io->ClearBits(color_clk_mask);    // clock back to normal.

      // OE of the previous row-data must be finished before strobe.
      sOutputEnablePulser->WaitPulseFinished();

      // Setting address and strobing needs to happen in dark time.
      row_setter_->SetRowAddress(io, d_row);

      io->SetBits(h.strobe);   // Strobe in the previously clocked in row.
      io->ClearBits(h.strobe);

      // Now switch on for the sleep time necessary for that bit-plane.
      sOutputEnablePulser->SendPulse(b);
    }
  }
}

// --- REPLACE DumpToMatrixFM6373 IN lib/framebuffer.cc ---

void Framebuffer::DumpToMatrixFM6373(GPIO *io) {
  const HardwareMapping &h = *hardware_mapping_;

  // Enable debug logging for this frame if requested via env.
  FM6373_EnableDebugForFrameIfRequested();

  // Data lines we actively drive for this FM6373 panel (one HUB75 chain)
  const gpio_bits_t rgb_mask =
      h.p0_r1 | h.p0_g1 | h.p0_b1 |
      h.p0_r2 | h.p0_g2 | h.p0_b2;
  const gpio_bits_t data_mask = rgb_mask | h.clock;

  // 128-wide FM6373/DP32019B cabinet: 16 px per chip -> 8 chips.
  // This gives 8×16-bit words between each LAT, matching the MRV412.
  const int chips = 8;
  const int channels = 16;   // FM6373 outputs per chip

  // Make sure the hardware pulser is ready for a fixed-width OE gate.
  FM6373_EnsureOEPulser(io, h);
  if (sFM6373OEPulser) sFM6373OEPulser->WaitPulseFinished();
  io->ClearBits(h.output_enable);  // idle low (polarity inverted in InitPanels)

  FM6373_Debug("[FM6373] === Frame start rows=%d columns=%d ===\n",
               double_rows_, columns_);

  // Send VSYNC / EN_OP / PRE_ACT + PRAM preamble we already matched to MRV412.
  FM6373_Init_And_Registers(io, h);

  // --- Data phase: channel-major ordering, 8×16-bit words then 1-CLK LAT ---
  for (int row = 0; row < double_rows_; ++row) {
    // A. Select row (A..E)
    io->ClearBits(h.clock | h.strobe | rgb_mask);
    row_setter_->SetRowAddress(io, row);
    FM6373_Debug("[FM6373] Row %d data shift begin\n", row);

    // Start a fresh OE pattern for this row.
    if (sFM6373OEPulser) sFM6373OEPulser->WaitPulseFinished();
    io->ClearBits(h.output_enable);

    // B. 16 FM6373 channels, from 15 down to 0 (matches FM6363/6373 docs)
    for (int ch = channels - 1; ch >= 0; --ch) {

      // 8 cascaded chips across 128 pixels, from last to first
      for (int chip = chips - 1; chip >= 0; --chip) {
        const int x = chip * 16 + ch;  // logical pixel index along the row

        // Build 16-bit grayscale words for this (row, x)
        uint16_t r0 = 0, g0 = 0, b0 = 0;
        uint16_t r1 = 0, g1 = 0, b1 = 0;

        for (int b = 0; b < pwm_bits_; ++b) {
          const gpio_bits_t bits = *ValueAt(row, x, b);
          const int out_shift = b + (16 - pwm_bits_);

          if (bits & h.p0_r1) r0 |= (1 << out_shift);
          if (bits & h.p0_g1) g0 |= (1 << out_shift);
          if (bits & h.p0_b1) b0 |= (1 << out_shift);
          if (bits & h.p0_r2) r1 |= (1 << out_shift);
          if (bits & h.p0_g2) g1 |= (1 << out_shift);
          if (bits & h.p0_b2) b1 |= (1 << out_shift);
        }

        // Shift these 16 bits MSB-first on the six RGB lines.
        for (int bit = 15; bit >= 0; --bit) {
          const uint16_t m = 1u << bit;
          gpio_bits_t out = 0;
          if (r0 & m) out |= h.p0_r1;
          if (g0 & m) out |= h.p0_g1;
          if (b0 & m) out |= h.p0_b1;
          if (r1 & m) out |= h.p0_r2;
          if (g1 & m) out |= h.p0_g2;
          if (b1 & m) out |= h.p0_b2;

          io->WriteMaskedBits(out, data_mask);
          io->SetBits(h.clock);             // DCLK rising edge: data sampled
          io->ClearBits(h.clock);
        }
      }

      // C. DATA_LATCH: 1 CLK with LAT high, all data low.
      //    Force OE low while latching.
      io->ClearBits(h.output_enable);
      io->ClearBits(rgb_mask);
      io->SetBits(h.strobe);   // LE high
      io->SetBits(h.clock);    // LE length = 1 DCLK
      io->ClearBits(h.clock);
      io->ClearBits(h.strobe); // LE low again

      // Hardware-timed OE gate for this channel worth of data.
      if (sFM6373OEPulser) sFM6373OEPulser->SendPulse(0);
    }

  // Optional display burst removed: rely on the fixed OE gate instead.
  }

  // Only disable one-shot debug after a full frame (preamble + data) so we see
  // all register/tail logs on the requested passes.
  if (!sFM6373_DebugAlways) sFM6373_DebugOnce = false;

  // After all RGB data is shifted, keep clocking the address bus so panels
  // with external mux logic (e.g. --led-multiplexing=21) see a full sweep of
  // row addresses without re-sending color data. Only OE, CLK and ABCDE are
  // active in this pass to avoid disturbing the latched frame.
  if (sFM6373OEPulser) sFM6373OEPulser->WaitPulseFinished();
  io->ClearBits(rgb_mask | h.strobe);
  // Sweep the full 5-bit address space if E is wired so we always toggle it.
  const int sweep_rows = (h.e ? 32 : (h.d ? 16 : (h.c ? 8 : (h.b ? 4 : 2))));
  const gpio_bits_t row_mask = h.a | h.b | h.c | h.d | h.e;
  for (int row = 0; row < sweep_rows; ++row) {
    const gpio_bits_t addr_bits =
        (row & 0x01 ? h.a : 0) |
        (row & 0x02 ? h.b : 0) |
        (row & 0x04 ? h.c : 0) |
        (row & 0x08 ? h.d : 0) |
        (row & 0x10 ? h.e : 0);
    io->WriteMaskedBits(addr_bits, row_mask);
    io->ClearBits(h.output_enable);
    io->SetBits(h.clock);
    io->ClearBits(h.clock);
    if (sFM6373OEPulser) {
      sFM6373OEPulser->SendPulse(0);
      sFM6373OEPulser->WaitPulseFinished();
    } else if (sOutputEnablePulser) {
      sOutputEnablePulser->SendPulse(0);
      sOutputEnablePulser->WaitPulseFinished();
    }
  }
  
}



/*static*/ void Framebuffer::InitializePanels(GPIO *io,
                                              const char *panel_type,
                                              int columns) {

  // Default: disable FM6373 mode unless explicitly selected.
  sFM6373_Enabled = false;
  SetHardwarePulsePolarityInverted(false);
  if (!panel_type || panel_type[0] == '\0') return;
  if (custom_strncasecmp(panel_type, "fm6126", 6) == 0) {
    InitFM6126(io, *hardware_mapping_, columns);
  }
  else if (custom_strncasecmp(panel_type, "fm6127", 6) == 0) {
    InitFM6127(io, *hardware_mapping_, columns);
  }
  else if (custom_strncasecmp(panel_type, "fm6373", 6) == 0) {
    sFM6373_Enabled = true;  // enable preamble in DumpToMatrix()
    FM6373_ConfigureDebugFromEnv();

    // FM6373: flip OE polarity for hardware PWM pulses.
    SetHardwarePulsePolarityInverted(true);

  }
  // else if (...) { other panel types }
  else {
    fprintf(stderr, "Unknown panel type '%s'; typo ?\n", panel_type);
  }
}
}  // namespace internal
}  // namespace rgb_matrix
