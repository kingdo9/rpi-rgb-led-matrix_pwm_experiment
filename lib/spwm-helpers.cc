// -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-

#include "spwm-helpers.h"
#include "spwm-panel-config.h"
#include "framebuffer-internal.h"

#include <algorithm>
#include <atomic>
#include <ctype.h>
#include <errno.h>
#include <limits.h>
#include <stdlib.h>
#include <string>
#include <time.h>
#include <unistd.h>

namespace rgb_matrix {
namespace internal {

// Allocate storage for each register slot and seed the latch timing for fixed
// register blocks.
SPWM_Config::SPWM_Config(size_t spwm_register_count,
                         const SPWM_Register_Timing &spwm_default_timing,
                         size_t spwm_register_repeat_count)
    : default_timing_(spwm_default_timing),
      register_repeat_count_(spwm_register_repeat_count) {
  register_words_.resize(spwm_register_count);
  register_data_.resize(spwm_register_count);
  rgb_registers_.resize(spwm_register_count);
  fixed_rgb_registers_.resize(spwm_register_count);

  for (size_t spwm_register_index = 0;
       spwm_register_index < spwm_register_count;
       ++spwm_register_index) {
    register_data_[spwm_register_index].words = nullptr;
    register_data_[spwm_register_index].word_count = 0;
    register_data_[spwm_register_index].timing = default_timing_;
    rgb_registers_[spwm_register_index].timing = default_timing_;
  }
}

// Normalize a fixed register payload to the configured repeat count. Single
// words are duplicated and shorter payloads are padded with their final word.
std::vector<uint16_t> SPWM_Config::spwm_expand_words_for_register(
    const std::vector<uint16_t> &spwm_words) const {
  if (spwm_words.empty()) return std::vector<uint16_t>();
  if (register_repeat_count_ == 0) return spwm_words;
  if (spwm_words.size() == 1) {
    return std::vector<uint16_t>(register_repeat_count_, spwm_words[0]);
  }

  std::vector<uint16_t> spwm_expanded_words = spwm_words;
  spwm_expanded_words.resize(register_repeat_count_, spwm_words.back());
  return spwm_expanded_words;
}

// Store a fixed register payload in a 1-based register slot and expose it
// through the lightweight upload view.
void SPWM_Config::spwm_add_register(size_t spwm_register_index,
                                    const std::vector<uint16_t> &spwm_words,
                                    const SPWM_Register_Timing *spwm_timing) {
  if (spwm_register_index == 0 || spwm_register_index > register_words_.size()) {
    return;
  }

  const size_t spwm_index = spwm_register_index - 1;
  fixed_rgb_registers_[spwm_index].present = false;
  register_words_[spwm_index] = spwm_expand_words_for_register(spwm_words);

  SPWM_Register_Data spwm_register_data = {
    register_words_[spwm_index].empty() ? nullptr
                                        : register_words_[spwm_index].data(),
    register_words_[spwm_index].size(),
    spwm_timing != nullptr ? *spwm_timing : default_timing_,
  };
  register_data_[spwm_index] = spwm_register_data;
}

// Store the rotating RGB register sequence for a 1-based register slot.
void SPWM_Config::spwm_add_rgb_register(
    size_t spwm_register_index,
    const std::array<std::vector<uint16_t>, 3> &spwm_channel_sequences,
    const SPWM_Register_Timing &spwm_timing) {
  if (spwm_register_index == 0 || spwm_register_index > rgb_registers_.size()) {
    return;
  }

  SPWM_RGB_Register &spwm_rgb_register = rgb_registers_[spwm_register_index - 1];
  spwm_rgb_register.present = true;
  spwm_rgb_register.channel_sequences = spwm_channel_sequences;
  spwm_rgb_register.sequence_index = 0;
  spwm_rgb_register.sequence_length = std::min(
      spwm_rgb_register.channel_sequences[0].size(),
      std::min(spwm_rgb_register.channel_sequences[1].size(),
               spwm_rgb_register.channel_sequences[2].size()));
  spwm_rgb_register.timing = spwm_timing;
}

// Replace one existing slot while preserving its profile-selected type and
// timing. This shared-value form applies the same word(s) to every RGB lane.
bool SPWM_Config::spwm_force_register_words(
    size_t spwm_register_index,
    const std::vector<uint16_t> &spwm_words) {
  if (spwm_register_index == 0 || spwm_words.empty() ||
      spwm_register_index > register_data_.size()) {
    return false;
  }

  if (spwm_has_rgb_register(spwm_register_index)) {
    const std::array<std::vector<uint16_t>, 3> spwm_channel_sequences = {
        spwm_words, spwm_words, spwm_words};
    return spwm_force_register_rgb_words(spwm_register_index,
                                         spwm_channel_sequences);
  }

  const SPWM_Register_Data &spwm_register_data =
      register_data_[spwm_register_index - 1];
  if (spwm_words.size() != 1 || spwm_register_data.words == nullptr ||
      spwm_register_data.word_count == 0) {
    return false;
  }

  const SPWM_Register_Timing spwm_timing = spwm_register_data.timing;
  spwm_add_register(spwm_register_index, spwm_words, &spwm_timing);
  return true;
}

// Replace one rotating slot with equal-length physical-channel sequences and
// restart its per-init cursor from the first word.
bool SPWM_Config::spwm_force_register_rgb_words(
    size_t spwm_register_index,
    const std::array<std::vector<uint16_t>, 3> &spwm_channel_sequences) {
  if (!spwm_has_rgb_register(spwm_register_index) ||
      spwm_channel_sequences[0].empty() ||
      spwm_channel_sequences[0].size() != spwm_channel_sequences[1].size() ||
      spwm_channel_sequences[0].size() != spwm_channel_sequences[2].size()) {
    return false;
  }

  SPWM_RGB_Register &spwm_rgb_register =
      rgb_registers_[spwm_register_index - 1];
  spwm_rgb_register.channel_sequences = spwm_channel_sequences;
  spwm_rgb_register.sequence_index = 0;
  spwm_rgb_register.sequence_length = spwm_channel_sequences[0].size();
  return true;
}

// Replace one fixed register with distinct physical values on the R/G/B data
// lanes. The underlying fixed payload remains in place to retain its repeat
// count and per-register latch timing.
bool SPWM_Config::spwm_force_fixed_register_rgb_words(
    size_t spwm_register_index, const uint16_t spwm_channel_words[3]) {
  if (spwm_channel_words == nullptr || spwm_register_index == 0 ||
      spwm_register_index > register_data_.size() ||
      spwm_has_rgb_register(spwm_register_index)) {
    return false;
  }

  const SPWM_Register_Data &spwm_register_data =
      register_data_[spwm_register_index - 1];
  if (spwm_register_data.words == nullptr ||
      spwm_register_data.word_count == 0) {
    return false;
  }

  SPWM_Fixed_RGB_Register &spwm_fixed_register =
      fixed_rgb_registers_[spwm_register_index - 1];
  spwm_fixed_register.present = true;
  spwm_fixed_register.frame = {
      spwm_channel_words[0], spwm_channel_words[1], spwm_channel_words[2]};
  return true;
}

// Read the optional physical-channel sidecar used by diagnostic fixed slots.
bool SPWM_Config::spwm_get_fixed_register_rgb_frame(
    size_t spwm_register_index, SPWM_RGB_Frame *spwm_rgb_frame) const {
  if (spwm_rgb_frame == nullptr || spwm_register_index == 0 ||
      spwm_register_index > fixed_rgb_registers_.size()) {
    return false;
  }

  const SPWM_Fixed_RGB_Register &spwm_fixed_register =
      fixed_rgb_registers_[spwm_register_index - 1];
  if (!spwm_fixed_register.present) return false;

  *spwm_rgb_frame = spwm_fixed_register.frame;
  return true;
}

// Replace all rotating RGB payloads with the same user-supplied sequence for
// each color channel. Fixed register slots and per-slot timing stay unchanged.
bool SPWM_Config::spwm_force_rgb_register_words(
    const std::vector<uint16_t> &spwm_words) {
  if (spwm_words.empty()) return false;

  const std::array<std::vector<uint16_t>, 3> spwm_channel_sequences = {
      spwm_words, spwm_words, spwm_words};
  return spwm_force_rgb_register_words(spwm_channel_sequences);
}

// Apply distinct physical-channel sequences to every rotating slot in the
// active profile; current profiles contain one such slot.
bool SPWM_Config::spwm_force_rgb_register_words(
    const std::array<std::vector<uint16_t>, 3> &spwm_channel_sequences) {
  if (spwm_channel_sequences[0].empty() ||
      spwm_channel_sequences[0].size() != spwm_channel_sequences[1].size() ||
      spwm_channel_sequences[0].size() != spwm_channel_sequences[2].size()) {
    return false;
  }

  bool spwm_forced_register = false;
  for (size_t spwm_register_index = 1;
       spwm_register_index <= rgb_registers_.size();
       ++spwm_register_index) {
    if (!spwm_has_rgb_register(spwm_register_index)) continue;
    if (spwm_force_register_rgb_words(spwm_register_index,
                                      spwm_channel_sequences)) {
      spwm_forced_register = true;
    }
  }
  return spwm_forced_register;
}

// Return the fixed payload for a 1-based register slot, or nullptr if the slot
// is out of range.
const SPWM_Register_Data *SPWM_Config::spwm_get_register_data(
    size_t spwm_register_index) const {
  if (spwm_register_index == 0 || spwm_register_index > register_data_.size()) {
    return nullptr;
  }
  return &register_data_[spwm_register_index - 1];
}

// Return true when the selected register slot uses a rotating RGB sequence.
bool SPWM_Config::spwm_has_rgb_register(size_t spwm_register_index) const {
  if (spwm_register_index == 0 || spwm_register_index > rgb_registers_.size()) {
    return false;
  }
  return rgb_registers_[spwm_register_index - 1].present;
}

// Return the configured register timing for the selected slot, or the panel
// default when the slot is empty or invalid.
const SPWM_Register_Timing &SPWM_Config::spwm_get_register_timing(
    size_t spwm_register_index) const {
  if (spwm_register_index == 0 || spwm_register_index > register_data_.size()) {
    return default_timing_;
  }
  if (spwm_has_rgb_register(spwm_register_index)) {
    return rgb_registers_[spwm_register_index - 1].timing;
  }
  return register_data_[spwm_register_index - 1].timing;
}

// Advance to the next RGB register triple for the slot and wrap around at the
// end of the shortest channel sequence.
SPWM_RGB_Frame SPWM_Config::spwm_next_rgb_frame(size_t spwm_register_index) {
  const SPWM_RGB_Frame SPWM_EMPTY_RGB_FRAME = {0, 0, 0};
  if (!spwm_has_rgb_register(spwm_register_index)) return SPWM_EMPTY_RGB_FRAME;

  SPWM_RGB_Register &spwm_rgb_register = rgb_registers_[spwm_register_index - 1];
  if (spwm_rgb_register.sequence_length == 0) return SPWM_EMPTY_RGB_FRAME;

  const size_t spwm_sequence_index = spwm_rgb_register.sequence_index;
  spwm_rgb_register.sequence_index =
      (spwm_rgb_register.sequence_index + 1) % spwm_rgb_register.sequence_length;

  const SPWM_RGB_Frame spwm_rgb_frame = {
    spwm_rgb_register.channel_sequences[0][spwm_sequence_index],
    spwm_rgb_register.channel_sequences[1][spwm_sequence_index],
    spwm_rgb_register.channel_sequences[2][spwm_sequence_index],
  };
  return spwm_rgb_frame;
}

// Parse the command-line register sequence without changing the destination on
// failure. strtoul base 0 accepts the requested 0x-prefixed words.
bool spwm_parse_forced_register_words(
    const char *spwm_value, std::vector<uint16_t> *spwm_words) {
  if (spwm_value == nullptr || spwm_words == nullptr) return false;

  std::vector<uint16_t> spwm_parsed_words;
  const char *spwm_cursor = spwm_value;
  while (*spwm_cursor != '\0' &&
         isspace(static_cast<unsigned char>(*spwm_cursor))) {
    ++spwm_cursor;
  }
  if (*spwm_cursor == '\0') return false;

  while (*spwm_cursor != '\0') {
    if (*spwm_cursor == '-' || *spwm_cursor == '+') return false;

    errno = 0;
    char *spwm_end = nullptr;
    const unsigned long spwm_word = strtoul(spwm_cursor, &spwm_end, 0);
    if (errno != 0 || spwm_end == spwm_cursor || spwm_word > 0xfffful) {
      return false;
    }
    spwm_parsed_words.push_back(static_cast<uint16_t>(spwm_word));
    spwm_cursor = spwm_end;

    while (*spwm_cursor != '\0' &&
           isspace(static_cast<unsigned char>(*spwm_cursor))) {
      ++spwm_cursor;
    }
    if (*spwm_cursor == '\0') {
      *spwm_words = spwm_parsed_words;
      return true;
    }
    if (*spwm_cursor != ',') return false;

    ++spwm_cursor;
    while (*spwm_cursor != '\0' &&
           isspace(static_cast<unsigned char>(*spwm_cursor))) {
      ++spwm_cursor;
    }
    if (*spwm_cursor == '\0') return false;
  }

  return false;
}

// Parse the shared legacy form or the R:/G:/B: channel-labelled form without
// modifying the destination unless the complete value is valid.
bool spwm_parse_forced_rgb_register_words(
    const char *spwm_value,
    std::array<std::vector<uint16_t>, 3> *spwm_channel_sequences) {
  if (spwm_value == nullptr || spwm_channel_sequences == nullptr) return false;

  // Preserve the original unlabelled-list syntax as the fast path. A single
  // labelled channel is also shared; otherwise all R/G/B labels are required
  // and must contain the same number of rotating words.
  std::vector<uint16_t> spwm_shared_words;
  if (spwm_parse_forced_register_words(spwm_value, &spwm_shared_words)) {
    *spwm_channel_sequences = {
        spwm_shared_words, spwm_shared_words, spwm_shared_words};
    return true;
  }

  std::array<std::vector<uint16_t>, 3> spwm_parsed_channels;
  bool spwm_channel_present[3] = {false, false, false};
  size_t spwm_channel_count = 0;
  const char *spwm_cursor = spwm_value;

  while (*spwm_cursor != '\0') {
    while (isspace(static_cast<unsigned char>(*spwm_cursor))) ++spwm_cursor;
    if (*spwm_cursor == '\0') return false;

    size_t spwm_channel_index = 0;
    switch (tolower(static_cast<unsigned char>(*spwm_cursor))) {
      case 'r':
        spwm_channel_index = 0;
        break;
      case 'g':
        spwm_channel_index = 1;
        break;
      case 'b':
        spwm_channel_index = 2;
        break;
      default:
        return false;
    }
    ++spwm_cursor;
    while (isspace(static_cast<unsigned char>(*spwm_cursor))) ++spwm_cursor;
    if (*spwm_cursor != ':' || spwm_channel_present[spwm_channel_index]) {
      return false;
    }
    ++spwm_cursor;

    const char *spwm_sequence_start = spwm_cursor;
    while (*spwm_cursor != '\0' && *spwm_cursor != ';') ++spwm_cursor;
    const std::string spwm_sequence(
        spwm_sequence_start,
        static_cast<size_t>(spwm_cursor - spwm_sequence_start));
    if (!spwm_parse_forced_register_words(
            spwm_sequence.c_str(),
            &spwm_parsed_channels[spwm_channel_index])) {
      return false;
    }

    spwm_channel_present[spwm_channel_index] = true;
    ++spwm_channel_count;
    if (*spwm_cursor == ';') {
      ++spwm_cursor;
      const char *spwm_next_channel = spwm_cursor;
      while (isspace(static_cast<unsigned char>(*spwm_next_channel))) {
        ++spwm_next_channel;
      }
      if (*spwm_next_channel == '\0') return false;
    }
  }

  if (spwm_channel_count == 1) {
    size_t spwm_source_channel = 0;
    while (!spwm_channel_present[spwm_source_channel]) ++spwm_source_channel;
    for (size_t spwm_channel_index = 0; spwm_channel_index < 3;
         ++spwm_channel_index) {
      spwm_parsed_channels[spwm_channel_index] =
          spwm_parsed_channels[spwm_source_channel];
    }
  } else if (spwm_channel_count != 3 ||
             spwm_parsed_channels[0].size() !=
                 spwm_parsed_channels[1].size() ||
             spwm_parsed_channels[0].size() !=
                 spwm_parsed_channels[2].size()) {
    return false;
  }

  *spwm_channel_sequences = spwm_parsed_channels;
  return true;
}

namespace {

// ------------------------------
// Runtime state and small helpers
// ------------------------------
std::atomic<const SPWM_RGB_Register_Profile_View *>
    spwm_pending_rgb_register_profile(nullptr);
std::atomic<const SPWM_RGB_Register_Profile_View *>
    spwm_last_emitted_rgb_register_profile(nullptr);
std::atomic<const SPWM_RGB_Register_Profile_View *>
    spwm_last_rejected_rgb_register_profile(nullptr);
// These two values are only read or written by the refresh thread.
const SPWM_RGB_Register_Profile_View *
    spwm_rgb_register_profile_being_emitted = nullptr;
size_t spwm_rgb_register_profile_words_remaining = 0;

std::atomic<const SPWM_Fixed_Register_Profile_View *>
    spwm_pending_fixed_register_profile(nullptr);
std::atomic<const SPWM_Fixed_Register_Profile_View *>
    spwm_last_emitted_fixed_register_profile(nullptr);
std::atomic<const SPWM_Fixed_Register_Profile_View *>
    spwm_last_rejected_fixed_register_profile(nullptr);
// These values are only read or written by the refresh thread.
const SPWM_Fixed_Register_Profile_View *
    spwm_fixed_register_profile_being_emitted = nullptr;
uint32_t spwm_fixed_register_profile_remaining_mask = 0;

// These validators check only structural shape; callers provide the
// process-lifetime backing storage required by the public view contract.
// Whether a requested slot exists and has the expected type is checked after
// the refresh thread consumes the request against the active runtime config.
bool spwm_is_valid_rgb_register_profile(
    const SPWM_RGB_Register_Profile_View *spwm_profile) {
  if (spwm_profile == nullptr || spwm_profile->name == nullptr ||
      spwm_profile->register_index == 0) {
    return false;
  }

  const size_t spwm_word_count = spwm_profile->channel_word_counts[0];
  if (spwm_word_count == 0) return false;

  for (size_t spwm_channel = 0; spwm_channel < 3; ++spwm_channel) {
    if (spwm_profile->channel_words[spwm_channel] == nullptr ||
        spwm_profile->channel_word_counts[spwm_channel] != spwm_word_count) {
      return false;
    }
  }
  return true;
}

bool spwm_is_valid_fixed_register_profile(
    const SPWM_Fixed_Register_Profile_View *spwm_profile) {
  if (spwm_profile == nullptr || spwm_profile->name == nullptr ||
      spwm_profile->entries == nullptr || spwm_profile->entry_count == 0 ||
      spwm_profile->entry_count > SPWM_FORCE_REGISTER_COUNT) {
    return false;
  }

  uint32_t spwm_register_mask = 0;
  for (size_t spwm_entry_index = 0;
       spwm_entry_index < spwm_profile->entry_count;
       ++spwm_entry_index) {
    const size_t spwm_register_index =
        spwm_profile->entries[spwm_entry_index].register_index;
    if (spwm_register_index == 0 ||
        spwm_register_index > SPWM_FORCE_REGISTER_COUNT) {
      return false;
    }

    const uint32_t spwm_register_bit =
        static_cast<uint32_t>(1u << (spwm_register_index - 1));
    if ((spwm_register_mask & spwm_register_bit) != 0) return false;
    spwm_register_mask |= spwm_register_bit;
  }
  return true;
}

// Clear the asynchronous diagnostic hand-off at SPWM setup/teardown. The
// refresh thread must be stopped before this resets its non-atomic progress
// fields.
void spwm_reset_register_profile_state() {
  spwm_pending_rgb_register_profile.store(nullptr, std::memory_order_release);
  spwm_last_emitted_rgb_register_profile.store(
      nullptr, std::memory_order_release);
  spwm_last_rejected_rgb_register_profile.store(
      nullptr, std::memory_order_release);
  spwm_rgb_register_profile_being_emitted = nullptr;
  spwm_rgb_register_profile_words_remaining = 0;

  spwm_pending_fixed_register_profile.store(nullptr,
                                             std::memory_order_release);
  spwm_last_emitted_fixed_register_profile.store(
      nullptr, std::memory_order_release);
  spwm_last_rejected_fixed_register_profile.store(
      nullptr, std::memory_order_release);
  spwm_fixed_register_profile_being_emitted = nullptr;
  spwm_fixed_register_profile_remaining_mask = 0;
}

typedef bool (*SPWM_Env_Int_Validator)(int value);

enum SPWM_Auto_Tune_Section {
  SPWM_AUTO_TUNE_SECTION_NONE = 0,
  SPWM_AUTO_TUNE_SECTION_UPLOAD,
  SPWM_AUTO_TUNE_SECTION_FREE,
  SPWM_AUTO_TUNE_SECTION_COUNT,
};

struct SPWM_Auto_Tune_Frame_State {
  bool enabled;
  uint64_t last_oe_end_ns;
  SPWM_Auto_Tune_Section last_oe_section;
};

struct SPWM_OE_Gate_State {
  int remaining;
  bool active;
  SPWM_Auto_Tune_Frame_State *auto_tune;
  SPWM_Auto_Tune_Section section;
  bool pulse_each_clock;
  bool capture_start_time;
};

// Scan timing fixes the shift-register row-select pulse positions for the
// duration of one upload/free-run phase. Cache those positions alongside the
// scan config instead of resolving panel settings again on every blank clock.
struct SPWM_Shiftreg_Waveform_Context {
  int row_address_type;
  int blank_clks;
  int a_pulse_start;
  int a_pulse_end;
  int type2_b_pulse_start;
};

struct SPWM_Scan_Config {
  int row_clks;
  int advance_phase;
  int oe_arm_phase;
  int oe_clks;
  bool skip_first_oe;
  bool row_before_oe;
  SPWM_Shiftreg_Waveform_Context shiftreg_waveform;
};

struct SPWM_Scan_State {
  int row;
  int phase;
  bool oe_primed;
  // True only on the scan step that wrapped from the last logical row back to
  // row 0. Shift-register row-address types 1 and 2 use this to emit Channel C
  // only on the real end-of-scan wrap pulse, not on the initial row-0 pulse at
  // frame start.
  bool wrapped_to_row_zero;
  gpio_bits_t shiftreg_row_bits;
  bool shiftreg_row_bits_valid;
};

typedef bool (*SPWM_Scan_Pre_Clock_Handler)(
    GPIO *io, const HardwareMapping &h,
    RowAddressSetter *spwm_row_setter, int spwm_double_rows,
    const SPWM_Scan_Config &spwm_scan_config,
    SPWM_OE_Gate_State *spwm_oe_gate,
    SPWM_Scan_State *spwm_scan_state);

constexpr int SPWM_WORD_BIT_COUNT = 16;
constexpr int SPWM_RGB_COLOR_COUNT = 3;
constexpr int SPWM_MAX_RGB_SIGNAL_COUNT = 6 * SPWM_RGB_COLOR_COUNT;
constexpr uint64_t SPWM_NANOS_PER_SECOND = 1000000000ull;
constexpr int SPWM_TYPE2_ROW_SELECT_B_LEAD_CLKS = 2;

// Panels such as FM6363 and ICND1065L advance the next row during the blank
// clocks instead of latching a direct A/B/C row address ahead of time. This
// setter only reserves those pins and returns them low when the logical row
// changes; SPWM timing code emits the actual row-select waveform.
class SPWMBlankClockRowSelectSetter : public RowAddressSetter {
 public:
  explicit SPWMBlankClockRowSelectSetter(const HardwareMapping &h)
      : row_mask_(h.a | h.b | h.c), last_row_(-1) {}

  virtual gpio_bits_t need_bits() const { return row_mask_; }

  virtual void SetRowAddress(GPIO *io, int row) {
    if (row == last_row_) return;
    io->ClearBits(row_mask_);
    last_row_ = row;
  }

 private:
  const gpio_bits_t row_mask_;
  int last_row_;
};

// One pre-expanded 16-clock payload for a single grayscale word block. Each
// entry already contains the GPIO bits that should be driven for that clock.
struct SPWM_Pixel_Block_GPIO_Bits {
  gpio_bits_t word_gpio_bits[SPWM_WORD_BIT_COUNT];
};

// Frame-local pin routing shared by every full-height pixel block. Flattening
// chain/color pins once keeps runtime-state reads and HardwareMapping expansion
// out of the per-word routing loop.
struct SPWM_RGB_Routing_Context {
  int signal_count;
  gpio_bits_t rgb1_bits[SPWM_MAX_RGB_SIGNAL_COUNT];
  gpio_bits_t rgb2_bits[SPWM_MAX_RGB_SIGNAL_COUNT];
  gpio_bits_t rgb1_rgb2_bits[SPWM_MAX_RGB_SIGNAL_COUNT];
  bool serial_uses_rgb1;
  bool serial_uses_rgb2;
  bool split_left_uses_rgb2;
};

// Resolve physical driver slots to framebuffer columns once per upload. Split
// layouts use both columns; ordinary and serial layouts leave second_column at
// -1, including sparse/unconnected physical slots.
struct SPWM_Column_Route {
  int first_column;
  int second_column;
};

struct SPWM_Register_Output_Masks {
  gpio_bits_t red_mask;
  gpio_bits_t green_mask;
  gpio_bits_t blue_mask;
  gpio_bits_t all_mask;
};

struct SPWM_Auto_Tune_Control {
  bool loaded;
  bool enabled;
  int window_frames;
  int max_step_clks;
  int frames_collected;
  int current_oe_after_upload_clk_count;
  uint64_t gap_sum_ns[SPWM_AUTO_TUNE_SECTION_COUNT];
  uint64_t gap_count[SPWM_AUTO_TUNE_SECTION_COUNT];
};

// Build the runtime register layout from the default panel profile.
SPWM_Config spwm_create_initial_config() {
  const SPWM_Panel_Profile &spwm_default_profile =
      spwm_get_default_panel_profile();
  return spwm_default_profile.create_config(
      spwm_default_profile.settings,
      spwm_default_profile.settings.default_columns,
      SPWM_ROW_ADDRESS_TYPE_0_DIRECT_AE,
      -1);
}

// Convert one physical shift-clock slot back to a visible framebuffer column.
// Panels such as 172-wide ICND1065L still shift through unconnected positions
// inside a wider physical chain, so those slots must stay blank.
int spwm_map_physical_column_to_visible(
    int spwm_physical_column, const SPWM_Panel_Settings &spwm_settings) {
  int spwm_visible_column = spwm_physical_column;
  const int spwm_gap_count = std::max(
      0, std::min(spwm_settings.missing_column_count,
                  SPWM_Panel_Settings::kMaxMissingColumnSlots));
  for (int spwm_gap = 0; spwm_gap < spwm_gap_count; ++spwm_gap) {
    const int spwm_missing_column =
        spwm_settings.missing_column_positions[spwm_gap];
    if (spwm_physical_column == spwm_missing_column) return -1;
    if (spwm_physical_column > spwm_missing_column) {
      --spwm_visible_column;
    }
  }
  return spwm_visible_column;
}

// Return the init sequence associated with the default panel profile.
SPWM_Init_Sequence spwm_get_initial_init_sequence() {
  return spwm_get_default_panel_profile().init_sequence;
}

struct SPWM_Runtime_State {
  SPWM_Runtime_State()
      : config(spwm_create_initial_config()),
        active_parallel_chains(1),
        row_address_type(SPWM_ROW_ADDRESS_TYPE_0_DIRECT_AE),
        scan_rows(0),
        panel_columns(spwm_get_default_panel_profile().settings.default_columns),
        data_layout(SPWM_DATA_LAYOUT_PROFILE_DEFAULT),
        multiplexing(0),
        enabled(false),
        init_sequence(spwm_get_initial_init_sequence()),
        last_initial_oe_start_ns(0),
        target_initial_oe_start_ns(0),
        panel_settings(spwm_get_default_panel_profile().settings) {}

  SPWM_Config config;
  int active_parallel_chains;
  int row_address_type;
  int scan_rows;
  int panel_columns;
  int data_layout;
  int multiplexing;
  bool enabled;
  SPWM_Init_Sequence init_sequence;
  uint64_t last_initial_oe_start_ns;
  uint64_t target_initial_oe_start_ns;
  SPWM_Panel_Settings panel_settings;
};

// Return the singleton runtime state shared by SPWM setup and refresh helpers.
SPWM_Runtime_State &spwm_get_runtime_state() {
  static SPWM_Runtime_State spwm_runtime_state;
  return spwm_runtime_state;
}

// Confirm that a diagnostic slot will actually be visited by the active init
// script. A configured-but-unemitted slot could otherwise leave Demo 15 waiting
// forever for a completion mask that can never clear.
bool spwm_init_sequence_emits_register(
    const SPWM_Init_Sequence &spwm_init_sequence,
    SPWM_Init_Step_Type spwm_step_type, size_t spwm_register_index) {
  if (spwm_init_sequence.steps == nullptr || spwm_register_index == 0) {
    return false;
  }
  for (size_t spwm_step_index = 0;
       spwm_step_index < spwm_init_sequence.step_count;
       ++spwm_step_index) {
    const SPWM_Init_Step &spwm_step =
        spwm_init_sequence.steps[spwm_step_index];
    if (spwm_step.type == spwm_step_type &&
        spwm_step.value == spwm_register_index) {
      return true;
    }
  }
  return false;
}

// Consume a pending diagnostic profile on the refresh thread so the active
// register vectors are never modified while an init sequence is reading them.
void spwm_apply_pending_rgb_register_profile() {
  const SPWM_RGB_Register_Profile_View *spwm_profile =
      spwm_pending_rgb_register_profile.exchange(nullptr,
                                                 std::memory_order_acq_rel);
  if (spwm_profile == nullptr) return;
  if (!spwm_is_valid_rgb_register_profile(spwm_profile)) {
    spwm_last_rejected_rgb_register_profile.store(
        spwm_profile, std::memory_order_release);
    return;
  }

  SPWM_Runtime_State &spwm_runtime_state = spwm_get_runtime_state();
  SPWM_Config &spwm_config = spwm_runtime_state.config;
  if (!spwm_config.spwm_has_rgb_register(spwm_profile->register_index) ||
      !spwm_init_sequence_emits_register(
          spwm_runtime_state.init_sequence, SPWM_INIT_STEP_RGB_REGISTER,
          spwm_profile->register_index)) {
    spwm_last_rejected_rgb_register_profile.store(
        spwm_profile, std::memory_order_release);
    return;
  }

  std::array<std::vector<uint16_t>, 3> spwm_channel_sequences;
  for (size_t spwm_channel = 0; spwm_channel < 3; ++spwm_channel) {
    const uint16_t *spwm_words = spwm_profile->channel_words[spwm_channel];
    spwm_channel_sequences[spwm_channel].assign(
        spwm_words,
        spwm_words + spwm_profile->channel_word_counts[spwm_channel]);
  }

  const SPWM_Register_Timing spwm_timing =
      spwm_config.spwm_get_register_timing(spwm_profile->register_index);
  spwm_config.spwm_add_rgb_register(spwm_profile->register_index,
                                    spwm_channel_sequences, spwm_timing);
  spwm_rgb_register_profile_being_emitted = spwm_profile;
  spwm_rgb_register_profile_words_remaining =
      spwm_profile->channel_word_counts[0];
}

// Consume a pending fixed-register profile on the refresh thread. Validate all
// selected slots before replacing any of them so a profile is applied as one
// complete unit at the init-sequence boundary.
void spwm_apply_pending_fixed_register_profile() {
  const SPWM_Fixed_Register_Profile_View *spwm_profile =
      spwm_pending_fixed_register_profile.exchange(nullptr,
                                                   std::memory_order_acq_rel);
  if (spwm_profile == nullptr) return;
  if (!spwm_is_valid_fixed_register_profile(spwm_profile)) {
    spwm_last_rejected_fixed_register_profile.store(
        spwm_profile, std::memory_order_release);
    return;
  }

  SPWM_Runtime_State &spwm_runtime_state = spwm_get_runtime_state();
  SPWM_Config &spwm_config = spwm_runtime_state.config;
  for (size_t spwm_entry_index = 0;
       spwm_entry_index < spwm_profile->entry_count;
       ++spwm_entry_index) {
    const size_t spwm_register_index =
        spwm_profile->entries[spwm_entry_index].register_index;
    const SPWM_Register_Data *spwm_register_data =
        spwm_config.spwm_get_register_data(spwm_register_index);
    if (spwm_register_data == nullptr || spwm_register_data->words == nullptr ||
        spwm_register_data->word_count == 0 ||
        spwm_config.spwm_has_rgb_register(spwm_register_index) ||
        !spwm_init_sequence_emits_register(
            spwm_runtime_state.init_sequence, SPWM_INIT_STEP_REGISTER,
            spwm_register_index)) {
      spwm_last_rejected_fixed_register_profile.store(
          spwm_profile, std::memory_order_release);
      return;
    }
  }

  uint32_t spwm_register_mask = 0;
  for (size_t spwm_entry_index = 0;
       spwm_entry_index < spwm_profile->entry_count;
       ++spwm_entry_index) {
    const SPWM_Fixed_Register_Profile_Entry &spwm_entry =
        spwm_profile->entries[spwm_entry_index];
    if (!spwm_config.spwm_force_fixed_register_rgb_words(
            spwm_entry.register_index, spwm_entry.channel_words)) {
      spwm_last_rejected_fixed_register_profile.store(
          spwm_profile, std::memory_order_release);
      return;
    }
    spwm_register_mask |= static_cast<uint32_t>(
        1u << (spwm_entry.register_index - 1));
  }

  spwm_fixed_register_profile_being_emitted = spwm_profile;
  spwm_fixed_register_profile_remaining_mask = spwm_register_mask;
}

// Option 0 follows the selected panel profile. Profiles that do not opt into a
// full-height layout retain the normal paired RGB1/RGB2 upload.
int spwm_get_active_data_layout() {
  const SPWM_Runtime_State &spwm_runtime_state = spwm_get_runtime_state();
  return spwm_runtime_state.data_layout !=
                 SPWM_DATA_LAYOUT_PROFILE_DEFAULT
             ? spwm_runtime_state.data_layout
             : spwm_runtime_state.panel_settings.default_data_layout;
}

// Return whether this layout expands paired framebuffer rows into one upload
// for every physical row.
bool spwm_data_layout_uses_full_height_rows(int spwm_data_layout) {
  return spwm_data_layout == SPWM_DATA_LAYOUT_FULL_HEIGHT_LEFT_RGB2 ||
         spwm_data_layout == SPWM_DATA_LAYOUT_FULL_HEIGHT_LEFT_RGB1 ||
         spwm_data_layout == SPWM_DATA_LAYOUT_FULL_HEIGHT_SERIAL_BOTH ||
         spwm_data_layout == SPWM_DATA_LAYOUT_FULL_HEIGHT_SERIAL_RGB1 ||
         spwm_data_layout == SPWM_DATA_LAYOUT_FULL_HEIGHT_SERIAL_RGB2;
}

// Return whether the full panel width is divided between RGB1 and RGB2.
bool spwm_data_layout_splits_horizontal_lanes(int spwm_data_layout) {
  return spwm_data_layout == SPWM_DATA_LAYOUT_FULL_HEIGHT_LEFT_RGB2 ||
         spwm_data_layout == SPWM_DATA_LAYOUT_FULL_HEIGHT_LEFT_RGB1;
}

// Return whether the layout uses full-width serial column routing.
bool spwm_data_layout_uses_serial_columns(int spwm_data_layout) {
  return spwm_data_layout == SPWM_DATA_LAYOUT_FULL_HEIGHT_SERIAL_BOTH ||
         spwm_data_layout == SPWM_DATA_LAYOUT_FULL_HEIGHT_SERIAL_RGB1 ||
         spwm_data_layout == SPWM_DATA_LAYOUT_FULL_HEIGHT_SERIAL_RGB2;
}

// Return the singleton storage used by the OE gap auto-tuner.
SPWM_Auto_Tune_Control &spwm_get_auto_tune_control_storage() {
  static SPWM_Auto_Tune_Control spwm_auto_tune_control = {
      false, false, 5, 8, 0, 0, {0}, {0}};
  return spwm_auto_tune_control;
}

// Validator for integer environment overrides that must be strictly positive.
bool spwm_is_positive_env_int(int spwm_value) { return spwm_value > 0; }

// Validator for integer environment overrides that may be zero.
bool spwm_is_non_negative_env_int(int spwm_value) { return spwm_value >= 0; }

// Use the runtime value when it is positive; otherwise keep the fallback
// default.
int spwm_resolve_positive_or_fallback(int spwm_value, int spwm_fallback) {
  return spwm_value > 0 ? spwm_value : spwm_fallback;
}

// Positive-only ceiling division used for derived geometry such as chip counts.
int spwm_ceil_div_positive(int spwm_numerator, int spwm_denominator) {
  if (spwm_numerator <= 0 || spwm_denominator <= 0) return 0;
  return (spwm_numerator + spwm_denominator - 1) / spwm_denominator;
}

// Parse an environment variable into a bounded int and reject malformed strings.
bool spwm_parse_env_int(const char *spwm_env_name, int *spwm_value) {
  const char *spwm_env_value = getenv(spwm_env_name);
  if (spwm_env_value == nullptr || *spwm_env_value == '\0') return false;

  char *spwm_end = nullptr;
  errno = 0;
  const long spwm_parsed_value = strtol(spwm_env_value, &spwm_end, 10);
  if (errno != 0 || spwm_end == spwm_env_value || *spwm_end != '\0') {
    return false;
  }
  if (spwm_parsed_value < INT_MIN || spwm_parsed_value > INT_MAX) return false;

  *spwm_value = static_cast<int>(spwm_parsed_value);
  return true;
}

// Apply a validated integer environment override when the variable is present.
void spwm_apply_int_env_override(const char *spwm_env_name,
                                 SPWM_Env_Int_Validator spwm_validator,
                                 int *spwm_value) {
  int spwm_parsed_value = 0;
  if (spwm_parse_env_int(spwm_env_name, &spwm_parsed_value) &&
      spwm_validator(spwm_parsed_value)) {
    *spwm_value = spwm_parsed_value;
  }
}

// Apply a boolean-like environment override where any non-zero value means true.
void spwm_apply_bool_env_override(const char *spwm_env_name,
                                  bool *spwm_value) {
  int spwm_parsed_value = 0;
  if (spwm_parse_env_int(spwm_env_name, &spwm_parsed_value)) {
    *spwm_value = (spwm_parsed_value != 0);
  }
}

// -------------------------
// Environment override layer
// -------------------------
// Layer all supported SPWM_* environment overrides on top of the selected panel
// profile.
void spwm_apply_panel_env_overrides(SPWM_Panel_Settings *spwm_settings) {
  if (spwm_settings == nullptr) return;

  spwm_apply_bool_env_override("SPWM_AUTO_TUNE_OE_GAPS",
                               &spwm_settings->auto_tune_oe_gaps);
  spwm_apply_int_env_override("SPWM_AUTO_TUNE_FRAMES",
                              spwm_is_positive_env_int,
                              &spwm_settings->auto_tune_frames);
  spwm_apply_int_env_override("SPWM_AUTO_TUNE_MAX_STEP_CLKS",
                              spwm_is_positive_env_int,
                              &spwm_settings->auto_tune_max_step_clks);
  spwm_apply_int_env_override("SPWM_FIRST_OE_CLK_LENGTH",
                              spwm_is_non_negative_env_int,
                              &spwm_settings->first_oe_clk_length);
  spwm_apply_int_env_override("SPWM_END_OF_FRAME_EXTRA_ROW_CYCLES",
                              spwm_is_non_negative_env_int,
                              &spwm_settings->end_of_frame_extra_row_cycles);
  spwm_apply_int_env_override("SPWM_FRAME_END_SLEEP_US",
                              spwm_is_non_negative_env_int,
                              &spwm_settings->frame_end_sleep_us);
  spwm_apply_int_env_override("SPWM_RGB_UPLOAD_LAT_SPACER_CLK_COUNT",
                              spwm_is_non_negative_env_int,
                              &spwm_settings->rgb_upload_lat_spacer_clk_count);
  spwm_apply_int_env_override("SPWM_OE_DURING_UPLOAD_CLK_COUNT",
                              spwm_is_positive_env_int,
                              &spwm_settings->oe_during_upload_clk_count);
  spwm_apply_int_env_override("SPWM_OE_AFTER_UPLOAD_CLK_COUNT",
                              spwm_is_positive_env_int,
                              &spwm_settings->oe_after_upload_clk_count);
  spwm_apply_int_env_override("SPWM_OE_CLK_LOOK_BEHIND",
                              spwm_is_non_negative_env_int,
                              &spwm_settings->oe_clk_look_behind);
  spwm_apply_int_env_override("SPWM_OE_CLK_LENGTH",
                              spwm_is_non_negative_env_int,
                              &spwm_settings->oe_clk_length);
  spwm_apply_int_env_override("SPWM_SHIFT_REG_ROW_SELECT_A_PULSE_CLK_COUNT",
                              spwm_is_non_negative_env_int,
                              &spwm_settings->shiftreg_row_select_a_pulse_clk_count);
  spwm_apply_int_env_override("SPWM_SHIFT_REG_ROW_SELECT_A_PULSE_START_CLK",
                              spwm_is_non_negative_env_int,
                              &spwm_settings->shiftreg_row_select_a_pulse_start_clk);
  spwm_apply_bool_env_override("SPWM_SHIFT_REG_ROW_SELECT_A_PULSE_CENTERED",
                               &spwm_settings->shiftreg_row_select_a_pulse_centered);
}

// ---------------------
// OE auto-tuning logic
// ---------------------
// Clear the accumulated OE gap measurements for the current tuning window.
void spwm_reset_auto_tune_window(SPWM_Auto_Tune_Control *spwm_control) {
  if (spwm_control == nullptr) return;

  spwm_control->frames_collected = 0;
  std::fill(spwm_control->gap_sum_ns,
            spwm_control->gap_sum_ns + SPWM_AUTO_TUNE_SECTION_COUNT,
            0);
  std::fill(spwm_control->gap_count,
            spwm_control->gap_count + SPWM_AUTO_TUNE_SECTION_COUNT,
            0);
}

// Refresh the auto-tune state from the currently active panel settings.
void spwm_load_auto_tune_control_defaults(
    SPWM_Auto_Tune_Control *spwm_control) {
  if (spwm_control == nullptr) return;

  const SPWM_Panel_Settings &spwm_settings = spwm_get_panel_settings();
  spwm_control->loaded = true;
  spwm_control->enabled = spwm_settings.auto_tune_oe_gaps;
  spwm_control->window_frames = spwm_settings.auto_tune_frames;
  spwm_control->max_step_clks = spwm_settings.auto_tune_max_step_clks;
  spwm_control->current_oe_after_upload_clk_count =
      spwm_settings.oe_after_upload_clk_count;
  spwm_reset_auto_tune_window(spwm_control);
}

// Lazily initialize and return the active OE auto-tune controller.
SPWM_Auto_Tune_Control &spwm_get_auto_tune_control() {
  SPWM_Auto_Tune_Control &spwm_control = spwm_get_auto_tune_control_storage();
  if (!spwm_control.loaded) {
    spwm_load_auto_tune_control_defaults(&spwm_control);
  }
  return spwm_control;
}

// Read a monotonic timestamp suitable for frame-to-frame phase locking and OE
// gap measurements.
uint64_t spwm_get_monotonic_nanos() {
  struct timespec spwm_timespec;
#ifdef CLOCK_MONOTONIC_RAW
  clock_gettime(CLOCK_MONOTONIC_RAW, &spwm_timespec);
#elif defined(CLOCK_MONOTONIC)
  clock_gettime(CLOCK_MONOTONIC, &spwm_timespec);
#else
  timespec_get(&spwm_timespec, TIME_UTC);
#endif
  return static_cast<uint64_t>(spwm_timespec.tv_sec) * SPWM_NANOS_PER_SECOND +
         static_cast<uint64_t>(spwm_timespec.tv_nsec);
}

// Wait until the monotonic clock reaches the requested deadline, sleeping
// coarsely when there is time to spare.
void spwm_wait_until_monotonic_nanos(uint64_t spwm_deadline_ns) {
  for (;;) {
    const uint64_t spwm_now_ns = spwm_get_monotonic_nanos();
    if (spwm_now_ns >= spwm_deadline_ns) return;

    const uint64_t spwm_remaining_ns = spwm_deadline_ns - spwm_now_ns;
    if (spwm_remaining_ns > 200000) {
      const long spwm_sleep_us =
          static_cast<long>((spwm_remaining_ns - 50000) / 1000);
      if (spwm_sleep_us > 0) {
        SleepMicroseconds(spwm_sleep_us);
        continue;
      }
    }
  }
}

// Integer division rounded to the nearest whole number while preserving the sign
// of the numerator.
int64_t spwm_rounded_div_nearest(int64_t spwm_numerator,
                                 uint64_t spwm_denominator) {
  if (spwm_denominator == 0) return 0;
  if (spwm_numerator >= 0) {
    return (spwm_numerator + static_cast<int64_t>(spwm_denominator / 2)) /
           static_cast<int64_t>(spwm_denominator);
  }

  const int64_t spwm_magnitude = -spwm_numerator;
  return -((spwm_magnitude + static_cast<int64_t>(spwm_denominator / 2)) /
           static_cast<int64_t>(spwm_denominator));
}

// Clamp the row period so it always remains long enough to contain the OE pulse.
int spwm_clamp_row_clks(int spwm_row_clks, int spwm_oe_clks) {
  const int spwm_min_row_clks = spwm_oe_clks + 1;
  return spwm_row_clks < spwm_min_row_clks ? spwm_min_row_clks
                                           : spwm_row_clks;
}

// Adjust the post-upload row period toward the upload-time gap while limiting
// the maximum step per tuning window.
void spwm_apply_auto_tune_step(uint64_t spwm_target_gap_ns,
                               uint64_t spwm_average_gap_ns,
                               int spwm_oe_clks,
                               int spwm_max_step_clks,
                               int *spwm_row_clks) {
  if (spwm_row_clks == nullptr || spwm_average_gap_ns == 0) return;

  const int spwm_effective_gap_clks =
      (*spwm_row_clks > spwm_oe_clks) ? (*spwm_row_clks - spwm_oe_clks) : 1;
  const uint64_t spwm_ns_per_clk =
      spwm_average_gap_ns / static_cast<uint64_t>(spwm_effective_gap_clks);
  if (spwm_ns_per_clk == 0) return;

  const int64_t spwm_delta_ns =
      static_cast<int64_t>(spwm_target_gap_ns) -
      static_cast<int64_t>(spwm_average_gap_ns);
  int spwm_delta_clks = static_cast<int>(
      spwm_rounded_div_nearest(spwm_delta_ns, spwm_ns_per_clk));
  if (spwm_delta_clks > spwm_max_step_clks) {
    spwm_delta_clks = spwm_max_step_clks;
  }
  if (spwm_delta_clks < -spwm_max_step_clks) {
    spwm_delta_clks = -spwm_max_step_clks;
  }
  if (spwm_delta_clks == 0) return;

  *spwm_row_clks = spwm_clamp_row_clks(*spwm_row_clks + spwm_delta_clks,
                                       spwm_oe_clks);
}

// ------------------------
// Register-stream emitters
// ------------------------
// Collect the RGB GPIO masks used when sending register blocks rather than
// framebuffer data. Register payloads need to be broadcast to every active
// parallel chain so they stay in sync.
SPWM_Register_Output_Masks spwm_get_register_output_masks(
    const HardwareMapping &h) {
  const int spwm_parallel_chains = spwm_get_parallel_chains();
  const gpio_bits_t spwm_red_masks[] = {
      h.p0_r1 | h.p0_r2, h.p1_r1 | h.p1_r2, h.p2_r1 | h.p2_r2,
      h.p3_r1 | h.p3_r2, h.p4_r1 | h.p4_r2, h.p5_r1 | h.p5_r2,
  };
  const gpio_bits_t spwm_green_masks[] = {
      h.p0_g1 | h.p0_g2, h.p1_g1 | h.p1_g2, h.p2_g1 | h.p2_g2,
      h.p3_g1 | h.p3_g2, h.p4_g1 | h.p4_g2, h.p5_g1 | h.p5_g2,
  };
  const gpio_bits_t spwm_blue_masks[] = {
      h.p0_b1 | h.p0_b2, h.p1_b1 | h.p1_b2, h.p2_b1 | h.p2_b2,
      h.p3_b1 | h.p3_b2, h.p4_b1 | h.p4_b2, h.p5_b1 | h.p5_b2,
  };

  SPWM_Register_Output_Masks spwm_masks = {0, 0, 0, 0};
  for (int spwm_chain = 0; spwm_chain < spwm_parallel_chains; ++spwm_chain) {
    spwm_masks.red_mask |= spwm_red_masks[spwm_chain];
    spwm_masks.green_mask |= spwm_green_masks[spwm_chain];
    spwm_masks.blue_mask |= spwm_blue_masks[spwm_chain];
  }
  spwm_masks.all_mask =
      spwm_masks.red_mask | spwm_masks.green_mask | spwm_masks.blue_mask;
  return spwm_masks;
}

// Build the 16 per-clock GPIO values for one set of physical R/G/B control
// words, used by both fixed and rotating register profiles.
SPWM_Pixel_Block_GPIO_Bits spwm_make_rgb_register_gpio_bits(
    const SPWM_RGB_Frame &spwm_rgb_frame,
    const SPWM_Register_Output_Masks &spwm_masks) {
  SPWM_Pixel_Block_GPIO_Bits spwm_block_gpio_bits = {{0}};

  for (int spwm_bit = 0; spwm_bit < SPWM_WORD_BIT_COUNT; ++spwm_bit) {
    gpio_bits_t spwm_out_bits = 0;
    const uint16_t spwm_bit_mask = static_cast<uint16_t>(1u << spwm_bit);
    if (spwm_rgb_frame.r & spwm_bit_mask) {
      spwm_out_bits |= spwm_masks.red_mask;
    }
    if (spwm_rgb_frame.g & spwm_bit_mask) {
      spwm_out_bits |= spwm_masks.green_mask;
    }
    if (spwm_rgb_frame.b & spwm_bit_mask) {
      spwm_out_bits |= spwm_masks.blue_mask;
    }

    spwm_block_gpio_bits.word_gpio_bits[spwm_bit] = spwm_out_bits;
  }

  return spwm_block_gpio_bits;
}

// Write the binary row value directly onto the A-E address lines.
void spwm_set_row_bits(GPIO *io, const HardwareMapping &h, uint8_t spwm_row) {
  const gpio_bits_t spwm_row_mask = h.a | h.b | h.c | h.d | h.e;
  gpio_bits_t spwm_row_bits = 0;
  if (spwm_row & 0x01) spwm_row_bits |= h.a;
  if (spwm_row & 0x02) spwm_row_bits |= h.b;
  if (spwm_row & 0x04) spwm_row_bits |= h.c;
  if (spwm_row & 0x08) spwm_row_bits |= h.d;
  if (spwm_row & 0x10) spwm_row_bits |= h.e;
  io->WriteMaskedBits(spwm_row_bits, spwm_row_mask);
}

// Clamp the number of data clocks that overlap the tail LAT window so it never
// exceeds the shifted payload length.
int spwm_resolve_tail_latch_clocks(const SPWM_Register_Timing &spwm_timing,
                                   int spwm_total_data_clocks) {
  if (spwm_timing.lat_clocks == nullptr || spwm_timing.lat_count == 0) {
    return 0;
  }

  int spwm_tail_latch_clocks = spwm_timing.lat_clocks[0];
  if (spwm_tail_latch_clocks > spwm_total_data_clocks) {
    return spwm_total_data_clocks;
  }
  return spwm_tail_latch_clocks;
}

// Reset LAT, CLK, and RGB lines before shifting a register block.
void spwm_begin_register_stream(GPIO *io, const HardwareMapping &h,
                                gpio_bits_t spwm_rgb_mask) {
  io->ClearBits(h.strobe);
  io->ClearBits(h.clock);
  io->ClearBits(spwm_rgb_mask);
}

// Finish a register block by clearing LAT, emitting any requested LAT-low
// spacer clocks, and updating the row bits. Leave CLK low so this does not add
// an unrequested shift edge after the register latch.
void spwm_end_register_stream(GPIO *io, const HardwareMapping &h,
                              gpio_bits_t spwm_rgb_mask, uint8_t spwm_row,
                              int spwm_space_clocks) {
  io->ClearBits(h.strobe);
  if (spwm_space_clocks > 0) {
    io->WriteMaskedBits(0, spwm_rgb_mask);
    for (int spwm_clock_index = 0;
         spwm_clock_index < spwm_space_clocks;
         ++spwm_clock_index) {
      io->SetBits(h.clock);
      io->ClearBits(h.clock);
    }
  }
  io->WriteMaskedBits(0, spwm_rgb_mask);
  spwm_set_row_bits(io, h, spwm_row);
}

// Emit any extra LAT sections that occur after a register payload has already
// been shifted. RGB lines stay low during these postamble clocks.
void spwm_send_register_extra_lat_clocks(
    GPIO *io, const HardwareMapping &h, gpio_bits_t spwm_rgb_mask,
    const SPWM_Register_Timing &spwm_timing) {
  if (spwm_timing.lat_clocks == nullptr || spwm_timing.lat_count <= 1) {
    return;
  }

  io->WriteMaskedBits(0, spwm_rgb_mask);
  for (size_t spwm_lat_index = 1;
       spwm_lat_index < spwm_timing.lat_count;
       ++spwm_lat_index) {
    if (spwm_timing.lat_space_clocks > 0) {
      io->ClearBits(h.strobe);
      for (int spwm_clock_index = 0;
           spwm_clock_index < spwm_timing.lat_space_clocks;
           ++spwm_clock_index) {
        io->SetBits(h.clock);
        io->ClearBits(h.clock);
      }
    }

    const int spwm_lat_clocks = spwm_timing.lat_clocks[spwm_lat_index];
    if (spwm_lat_clocks <= 0) continue;

    io->SetBits(h.strobe);
    for (int spwm_clock_index = 0;
         spwm_clock_index < spwm_lat_clocks;
         ++spwm_clock_index) {
      io->SetBits(h.clock);
      io->ClearBits(h.clock);
    }
  }
}

// Emit a LAT-high clock burst used by panel-specific init sequences, followed
// by any requested LAT-low spacer clocks before leaving the row lines idle.
void spwm_send_lat_pulses(GPIO *io, const HardwareMapping &h,
                          uint8_t spwm_row, int spwm_pulses,
                          int spwm_space_clocks) {
  const SPWM_Register_Output_Masks spwm_masks =
      spwm_get_register_output_masks(h);

  io->ClearBits(spwm_masks.all_mask);
  io->ClearBits(h.strobe);
  io->ClearBits(h.clock);
  io->SetBits(h.strobe);

  for (int spwm_pulse_index = 0;
       spwm_pulse_index < spwm_pulses;
       ++spwm_pulse_index) {
    io->SetBits(h.clock);
    io->ClearBits(h.clock);
  }

  io->ClearBits(h.strobe);
  for (int spwm_clock_index = 0;
       spwm_clock_index < spwm_space_clocks;
       ++spwm_clock_index) {
    io->SetBits(h.clock);
    io->ClearBits(h.clock);
  }
  io->SetBits(h.clock);
  spwm_set_row_bits(io, h, spwm_row);
}

// Shift one fixed register block into the panel, overlap LAT with the tail data
// clocks, then emit any extra post-data LAT sections required by the panel.
// Diagnostic profiles can provide distinct R/G/B words for the same fixed
// slot; normal panel profiles continue broadcasting one word to every lane.
// Purpose: Shift one fixed SPWM register block into the panel and latch it.
// Inputs: GPIO interface, hardware mapping, 1-based register index, row bits.
// Outputs: None.
// Side effects: Drives RGB/LAT/CLK and leaves the requested row on A-E.
void spwm_send_register(GPIO *io, const HardwareMapping &h,
                        uint8_t spwm_register_index, uint8_t spwm_row,
                        int spwm_space_clocks) {
  SPWM_Config &spwm_config = spwm_get_runtime_state().config;
  const SPWM_Register_Data *spwm_register_data =
      spwm_config.spwm_get_register_data(spwm_register_index);
  if (spwm_register_data == nullptr || spwm_register_data->word_count == 0 ||
      spwm_register_data->words == nullptr) {
    return;
  }

  const SPWM_Register_Output_Masks spwm_masks =
      spwm_get_register_output_masks(h);
  SPWM_RGB_Frame spwm_fixed_rgb_frame = {0, 0, 0};
  const bool spwm_has_fixed_rgb_frame =
      spwm_config.spwm_get_fixed_register_rgb_frame(
          spwm_register_index, &spwm_fixed_rgb_frame);
  const SPWM_Pixel_Block_GPIO_Bits spwm_fixed_rgb_gpio_bits =
      spwm_make_rgb_register_gpio_bits(spwm_fixed_rgb_frame, spwm_masks);
  spwm_begin_register_stream(io, h, spwm_masks.all_mask);

  const int spwm_total_clocks = static_cast<int>(
      spwm_register_data->word_count * SPWM_WORD_BIT_COUNT);
  if (spwm_total_clocks <= 0) return;

  const SPWM_Register_Timing &spwm_timing = spwm_register_data->timing;
  const int spwm_latch_clocks =
      spwm_resolve_tail_latch_clocks(spwm_timing, spwm_total_clocks);

  int spwm_clocks_remaining = spwm_total_clocks;
  for (size_t spwm_word_index = 0;
       spwm_word_index < spwm_register_data->word_count;
       ++spwm_word_index) {
    const uint16_t spwm_word = spwm_register_data->words[spwm_word_index];
    for (int spwm_bit = SPWM_WORD_BIT_COUNT - 1; spwm_bit >= 0; --spwm_bit) {
      if (spwm_latch_clocks > 0 &&
          spwm_clocks_remaining == spwm_latch_clocks) {
        io->SetBits(h.strobe);
      }

      const gpio_bits_t spwm_data_bits =
          spwm_has_fixed_rgb_frame
              ? spwm_fixed_rgb_gpio_bits.word_gpio_bits[spwm_bit]
              : ((spwm_word & (1u << spwm_bit)) ? spwm_masks.all_mask : 0);
      io->WriteMaskedBits(spwm_data_bits, spwm_masks.all_mask);
      io->SetBits(h.clock);
      io->ClearBits(h.clock);
      --spwm_clocks_remaining;
    }
  }

  spwm_send_register_extra_lat_clocks(io, h, spwm_masks.all_mask, spwm_timing);
  spwm_end_register_stream(io, h, spwm_masks.all_mask, spwm_row,
                           spwm_space_clocks);
}

// Shift the rotating RGB register block used once per frame by FM6373-style
// panels, then emit any extra LAT postamble clocks for that slot.
// Purpose: Shift the active panel's rotating RGB control register for the
// current frame.
// Inputs: GPIO interface, hardware mapping, 1-based register index, row bits.
// Outputs: None.
// Side effects: Advances the RGB register sequence and drives GPIO lines.
void spwm_send_rgb_register(GPIO *io, const HardwareMapping &h,
                            size_t spwm_register_index, uint8_t spwm_row,
                            int spwm_space_clocks) {
  SPWM_Config &spwm_config = spwm_get_runtime_state().config;
  if (!spwm_config.spwm_has_rgb_register(spwm_register_index)) return;

  const size_t spwm_repeat_count = spwm_config.spwm_register_repeat_count();
  if (spwm_repeat_count == 0) return;

  const SPWM_RGB_Frame spwm_rgb_frame =
      spwm_config.spwm_next_rgb_frame(spwm_register_index);
  const SPWM_Register_Timing &spwm_timing =
      spwm_config.spwm_get_register_timing(spwm_register_index);
  const SPWM_Register_Output_Masks spwm_masks =
      spwm_get_register_output_masks(h);
  const SPWM_Pixel_Block_GPIO_Bits spwm_rgb_gpio_bits =
      spwm_make_rgb_register_gpio_bits(spwm_rgb_frame, spwm_masks);

  spwm_begin_register_stream(io, h, spwm_masks.all_mask);

  const int spwm_total_clocks =
      static_cast<int>(spwm_repeat_count * SPWM_WORD_BIT_COUNT);
  const int spwm_latch_clocks =
      spwm_resolve_tail_latch_clocks(spwm_timing, spwm_total_clocks);

  int spwm_clocks_remaining = spwm_total_clocks;
  for (size_t spwm_repeat_index = 0;
       spwm_repeat_index < spwm_repeat_count;
       ++spwm_repeat_index) {
    for (int spwm_bit = SPWM_WORD_BIT_COUNT - 1;
         spwm_bit >= 0;
         --spwm_bit) {
      if (spwm_latch_clocks > 0 &&
          spwm_clocks_remaining == spwm_latch_clocks) {
        io->SetBits(h.strobe);
      }

      io->WriteMaskedBits(spwm_rgb_gpio_bits.word_gpio_bits[spwm_bit],
                          spwm_masks.all_mask);
      io->SetBits(h.clock);
      io->ClearBits(h.clock);
      --spwm_clocks_remaining;
    }
  }

  spwm_send_register_extra_lat_clocks(io, h, spwm_masks.all_mask, spwm_timing);
  spwm_end_register_stream(io, h, spwm_masks.all_mask, spwm_row,
                           spwm_space_clocks);
}

// Run the panel startup script made up of LAT bursts and register uploads.
void spwm_emit_init_sequence(GPIO *io, const HardwareMapping &h) {
  SPWM_Runtime_State &spwm_runtime_state = spwm_get_runtime_state();
  if (spwm_runtime_state.init_sequence.steps == nullptr ||
      spwm_runtime_state.init_sequence.step_count == 0) {
    spwm_runtime_state.init_sequence = spwm_get_initial_init_sequence();
  }
  // Resolve the fallback script before validating a queued profile against the
  // slots this init sequence will actually emit.
  spwm_apply_pending_rgb_register_profile();
  spwm_apply_pending_fixed_register_profile();

  for (size_t spwm_step_index = 0;
       spwm_step_index < spwm_runtime_state.init_sequence.step_count;
       ++spwm_step_index) {
    const SPWM_Init_Step &spwm_step =
        spwm_runtime_state.init_sequence.steps[spwm_step_index];
    switch (spwm_step.type) {
      case SPWM_INIT_STEP_LAT_PULSES:
        spwm_send_lat_pulses(io, h, spwm_step.row, spwm_step.value,
                             spwm_step.space_clocks);
        break;
      case SPWM_INIT_STEP_REGISTER:
        spwm_send_register(io, h, spwm_step.value, spwm_step.row,
                           spwm_step.space_clocks);
        if (spwm_fixed_register_profile_being_emitted != nullptr &&
            spwm_step.value > 0 &&
            spwm_step.value <= SPWM_FORCE_REGISTER_COUNT) {
          spwm_fixed_register_profile_remaining_mask &=
              ~static_cast<uint32_t>(1u << (spwm_step.value - 1));
        }
        break;
      case SPWM_INIT_STEP_RGB_REGISTER:
        spwm_send_rgb_register(io, h, spwm_step.value, spwm_step.row,
                               spwm_step.space_clocks);
        if (spwm_rgb_register_profile_being_emitted != nullptr &&
            spwm_rgb_register_profile_words_remaining > 0 &&
            spwm_step.value ==
                spwm_rgb_register_profile_being_emitted->register_index) {
          --spwm_rgb_register_profile_words_remaining;
        }
        break;
      default:
        break;
    }
  }

  if (spwm_rgb_register_profile_being_emitted != nullptr &&
      spwm_rgb_register_profile_words_remaining == 0) {
    spwm_last_emitted_rgb_register_profile.store(
        spwm_rgb_register_profile_being_emitted, std::memory_order_release);
    spwm_rgb_register_profile_being_emitted = nullptr;
  }

  if (spwm_fixed_register_profile_being_emitted != nullptr &&
      spwm_fixed_register_profile_remaining_mask == 0) {
    spwm_last_emitted_fixed_register_profile.store(
        spwm_fixed_register_profile_being_emitted,
        std::memory_order_release);
    spwm_fixed_register_profile_being_emitted = nullptr;
  }
}

// -----------------------
// Frame-level OE tracking
// -----------------------
// Start collecting OE gap measurements for the new frame when auto-tuning is
// enabled.
void spwm_auto_tune_begin_frame(SPWM_Auto_Tune_Frame_State *spwm_state) {
  if (spwm_state == nullptr) return;

  SPWM_Auto_Tune_Control &spwm_control = spwm_get_auto_tune_control();
  spwm_state->enabled = spwm_control.enabled;
  spwm_state->last_oe_end_ns = 0;
  spwm_state->last_oe_section = SPWM_AUTO_TUNE_SECTION_NONE;
}

// When enough frames have been sampled, compare upload and free-run gaps and
// nudge the post-upload timing toward the upload timing.
void spwm_auto_tune_end_frame(SPWM_Auto_Tune_Frame_State *spwm_state) {
  if (spwm_state == nullptr || !spwm_state->enabled) return;

  SPWM_Auto_Tune_Control &spwm_control = spwm_get_auto_tune_control();
  if (!spwm_control.enabled) return;

  ++spwm_control.frames_collected;
  if (spwm_control.frames_collected < spwm_control.window_frames) return;

  if (spwm_control.gap_count[SPWM_AUTO_TUNE_SECTION_UPLOAD] == 0) {
    spwm_reset_auto_tune_window(&spwm_control);
    return;
  }

  const uint64_t spwm_upload_average_gap_ns =
      spwm_control.gap_sum_ns[SPWM_AUTO_TUNE_SECTION_UPLOAD] /
      spwm_control.gap_count[SPWM_AUTO_TUNE_SECTION_UPLOAD];
  const int spwm_oe_clks = spwm_get_panel_settings().oe_clk_length;

  if (spwm_control.gap_count[SPWM_AUTO_TUNE_SECTION_FREE] > 0) {
    const uint64_t spwm_free_average_gap_ns =
        spwm_control.gap_sum_ns[SPWM_AUTO_TUNE_SECTION_FREE] /
        spwm_control.gap_count[SPWM_AUTO_TUNE_SECTION_FREE];
    spwm_apply_auto_tune_step(spwm_upload_average_gap_ns,
                              spwm_free_average_gap_ns,
                              spwm_oe_clks,
                              spwm_control.max_step_clks,
                              &spwm_control.current_oe_after_upload_clk_count);
  }

  spwm_reset_auto_tune_window(&spwm_control);
}

// Record the idle gap before a new OE pulse when the previous pulse belonged to
// the same scan section.
void spwm_auto_tune_on_oe_pulse_start(
    SPWM_Auto_Tune_Frame_State *spwm_state,
    SPWM_Auto_Tune_Section spwm_section) {
  if (spwm_state == nullptr || !spwm_state->enabled) return;
  if (spwm_section == SPWM_AUTO_TUNE_SECTION_NONE) return;
  if (spwm_state->last_oe_end_ns == 0 ||
      spwm_state->last_oe_section != spwm_section) {
    return;
  }

  SPWM_Auto_Tune_Control &spwm_control = spwm_get_auto_tune_control();
  if (!spwm_control.enabled) return;

  const uint64_t spwm_now_ns = spwm_get_monotonic_nanos();
  spwm_control.gap_sum_ns[spwm_section] +=
      spwm_now_ns - spwm_state->last_oe_end_ns;
  spwm_control.gap_count[spwm_section] += 1;
}

// Remember when the current OE pulse ended so the next pulse can measure its
// gap.
void spwm_auto_tune_on_oe_pulse_end(
    SPWM_Auto_Tune_Frame_State *spwm_state,
    SPWM_Auto_Tune_Section spwm_section) {
  if (spwm_state == nullptr || !spwm_state->enabled) return;
  if (spwm_section == SPWM_AUTO_TUNE_SECTION_NONE) {
    spwm_state->last_oe_end_ns = 0;
    spwm_state->last_oe_section = SPWM_AUTO_TUNE_SECTION_NONE;
    return;
  }

  spwm_state->last_oe_end_ns = spwm_get_monotonic_nanos();
  spwm_state->last_oe_section = spwm_section;
}

// Prepare an OE pulse that will stay active for the next N clock pulses.
void spwm_arm_oe_gate(SPWM_OE_Gate_State *spwm_gate, int spwm_clocks) {
  if (spwm_gate == nullptr) return;
  if (spwm_clocks < 0) spwm_clocks = 0;

  spwm_gate->remaining = spwm_clocks;
  spwm_gate->active = false;
  if (spwm_clocks == 0) spwm_gate->capture_start_time = false;
}

// Return whether an OE burst has already been armed and still needs clocks.
bool spwm_oe_gate_is_pending(const SPWM_OE_Gate_State *spwm_gate) {
  return spwm_gate != nullptr &&
         (spwm_gate->remaining > 0 || spwm_gate->active);
}

// Write the RGB data bits, optionally gate or pulse OE, and then emit one
// clock pulse.
void spwm_clock_pulse(GPIO *io, const HardwareMapping &h,
                      gpio_bits_t spwm_out_bits,
                      gpio_bits_t spwm_write_mask,
                      SPWM_OE_Gate_State *spwm_gate) {
  io->WriteMaskedBits(spwm_out_bits, spwm_write_mask);

  if (spwm_gate != nullptr && spwm_gate->remaining > 0 && !spwm_gate->active) {
    spwm_gate->active = true;
    if (spwm_gate->capture_start_time) {
      spwm_record_initial_oe_pulse_start();
      spwm_gate->capture_start_time = false;
    }
  }

  if (spwm_gate != nullptr && spwm_gate->active && spwm_gate->remaining > 0) {
    io->SetBits(h.output_enable);
  }

  io->SetBits(h.clock);
  io->ClearBits(h.clock);

  if (spwm_gate != nullptr && spwm_gate->active && spwm_gate->remaining > 0) {
    if (spwm_gate->pulse_each_clock) {
      io->ClearBits(h.output_enable);
    }
    if (--spwm_gate->remaining == 0) {
      if (!spwm_gate->pulse_each_clock) {
        io->ClearBits(h.output_enable);
      }
      spwm_gate->active = false;
      spwm_auto_tune_on_oe_pulse_end(spwm_gate->auto_tune,
                                     spwm_gate->section);
    }
  }
}

// Clamp scan timing inputs into a sane range for the selected row period.
// -------------------------
// Row scan timing helpers
// -------------------------
void spwm_normalize_scan_timing(int spwm_row_clks,
                                int *spwm_setup_clks,
                                int *spwm_oe_clks) {
  if (spwm_setup_clks == nullptr || spwm_oe_clks == nullptr) return;

  if (*spwm_setup_clks < 0) *spwm_setup_clks = 0;
  if (spwm_row_clks > 0 && *spwm_setup_clks > spwm_row_clks) {
    *spwm_setup_clks = spwm_row_clks;
  }

  if (*spwm_oe_clks < 0) *spwm_oe_clks = 0;
  if (spwm_row_clks > 0 && *spwm_oe_clks > spwm_row_clks) {
    *spwm_oe_clks = spwm_row_clks;
  }
}

// Convert a setup distance into the scan phase where the row address should
// advance.
int spwm_advance_phase(int spwm_row_clks, int spwm_setup_clks) {
  if (spwm_row_clks <= 0) return 0;
  return (spwm_setup_clks == 0) ? 0 : (spwm_row_clks - spwm_setup_clks);
}

// Resolve the blank-clock A/B pulse positions once for one scan timing
// configuration. Upload, free-run, and alignment each own a separately
// refreshed config, so runtime panel settings remain frame-consistent.
SPWM_Shiftreg_Waveform_Context spwm_make_shiftreg_waveform_context(
    int spwm_row_clks, int spwm_advance_phase) {
  SPWM_Shiftreg_Waveform_Context spwm_context = {
      spwm_get_runtime_state().row_address_type, 0, 0, 0, 0};
  if (spwm_row_clks <= spwm_advance_phase) return spwm_context;

  spwm_context.blank_clks = spwm_row_clks - spwm_advance_phase;
  if (spwm_context.row_address_type ==
      SPWM_ROW_ADDRESS_TYPE_0_DIRECT_AE) {
    return spwm_context;
  }
  const SPWM_Panel_Settings &spwm_settings = spwm_get_panel_settings();
  int spwm_a_pulse_clks =
      spwm_settings.shiftreg_row_select_a_pulse_clk_count;
  if (spwm_a_pulse_clks <= 0) return spwm_context;
  if (spwm_a_pulse_clks > spwm_context.blank_clks) {
    spwm_a_pulse_clks = spwm_context.blank_clks;
  }

  const int spwm_max_start_clk =
      spwm_context.blank_clks - spwm_a_pulse_clks;
  spwm_context.a_pulse_start =
      spwm_settings.shiftreg_row_select_a_pulse_centered
          ? spwm_max_start_clk / 2
          : std::min(spwm_settings.shiftreg_row_select_a_pulse_start_clk,
                     spwm_max_start_clk);
  spwm_context.a_pulse_end =
      spwm_context.a_pulse_start + spwm_a_pulse_clks;
  spwm_context.type2_b_pulse_start = std::max(
      0, spwm_context.a_pulse_start - SPWM_TYPE2_ROW_SELECT_B_LEAD_CLKS);
  return spwm_context;
}

// Resolve when the next OE burst should be armed for the active scan mode.
// Type-2 shift-register row select delays the OE arm point so the trailing OE
// clock overlaps the first Channel B blank-clock pulse. Captures from the
// working controller show Channel B starting two clocks before Channel A.
int spwm_compute_oe_arm_phase(int spwm_advance_phase,
                              int spwm_oe_clks,
                              bool spwm_row_before_oe,
                              const SPWM_Shiftreg_Waveform_Context
                                  &spwm_shiftreg_waveform) {
  if (spwm_oe_clks <= 0 || spwm_row_before_oe) {
    return 0;
  }

  if (spwm_shiftreg_waveform.row_address_type !=
      SPWM_ROW_ADDRESS_TYPE_2_SHIFTREG_AB_BLANK_CLOCK) {
    return 0;
  }

  if (spwm_shiftreg_waveform.blank_clks <= 0 ||
      spwm_shiftreg_waveform.a_pulse_end <=
          spwm_shiftreg_waveform.a_pulse_start) {
    return 0;
  }

  const int spwm_arm_phase =
      spwm_advance_phase + spwm_shiftreg_waveform.type2_b_pulse_start -
      (spwm_oe_clks - 1);
  return std::max(0, spwm_arm_phase);
}

// Refresh cached scan phases and row-select geometry after mutating a scan
// config field such as row_before_oe.
void spwm_refresh_scan_config_derived_fields(
    SPWM_Scan_Config *spwm_scan_config) {
  if (spwm_scan_config == nullptr) return;

  spwm_scan_config->shiftreg_waveform =
      spwm_make_shiftreg_waveform_context(spwm_scan_config->row_clks,
                                          spwm_scan_config->advance_phase);
  spwm_scan_config->oe_arm_phase =
      spwm_compute_oe_arm_phase(spwm_scan_config->advance_phase,
                                spwm_scan_config->oe_clks,
                                spwm_scan_config->row_before_oe,
                                spwm_scan_config->shiftreg_waveform);
}

// Build a normalized scan-timing description for upload or free-run scanning.
SPWM_Scan_Config spwm_make_scan_config(int spwm_row_clks,
                                       int spwm_setup_clks,
                                       int spwm_oe_clks,
                                       bool spwm_skip_first_oe,
                                       bool spwm_row_before_oe) {
  spwm_normalize_scan_timing(spwm_row_clks, &spwm_setup_clks, &spwm_oe_clks);
  const int spwm_resolved_advance_phase =
      spwm_advance_phase(spwm_row_clks, spwm_setup_clks);

  SPWM_Scan_Config spwm_scan_config = {
      spwm_row_clks,
      spwm_resolved_advance_phase,
      0,
      spwm_oe_clks,
      spwm_skip_first_oe,
      spwm_row_before_oe,
      {0, 0, 0, 0, 0},
  };
  spwm_refresh_scan_config_derived_fields(&spwm_scan_config);
  return spwm_scan_config;
}

// Return how many blank clocks remain between row advance and the next OE burst
// for the active OE schedule.
int spwm_row_blank_clk_count(const SPWM_Scan_Config &spwm_scan_config) {
  return spwm_scan_config.shiftreg_waveform.blank_clks;
}

// Return true when the active scan blank phase falls inside the requested pulse
// window.
bool spwm_blank_phase_in_pulse_window(int spwm_blank_phase,
                                      int spwm_pulse_start_clk,
                                      int spwm_pulse_end_clk) {
  return spwm_blank_phase >= spwm_pulse_start_clk &&
         spwm_blank_phase < spwm_pulse_end_clk;
}

bool spwm_uses_shiftreg_ab_blank_clock(int spwm_row_address_type) {
  return spwm_row_address_type ==
         SPWM_ROW_ADDRESS_TYPE_2_SHIFTREG_AB_BLANK_CLOCK;
}

// Resolve the A/B/C waveform for one blank-clock phase. Type 1 only emits the
// A pulse and adds C on the real wrap. Type 2 adds an earlier B pulse and, on
// wrap, mirrors that wider B pulse onto C.
gpio_bits_t spwm_resolve_shiftreg_row_bits(
    const HardwareMapping &h,
    const SPWM_Shiftreg_Waveform_Context &spwm_shiftreg_waveform,
    int spwm_blank_phase, bool spwm_wrapped_to_row_zero) {
  if (spwm_shiftreg_waveform.blank_clks <= 0) return 0;

  const bool spwm_in_a_pulse = spwm_blank_phase_in_pulse_window(
      spwm_blank_phase, spwm_shiftreg_waveform.a_pulse_start,
      spwm_shiftreg_waveform.a_pulse_end);
  const bool spwm_type2_transport =
      spwm_uses_shiftreg_ab_blank_clock(
          spwm_shiftreg_waveform.row_address_type);

  gpio_bits_t spwm_row_bits = 0;
  if (spwm_type2_transport) {
    const bool spwm_in_b_pulse = spwm_blank_phase_in_pulse_window(
        spwm_blank_phase, spwm_shiftreg_waveform.type2_b_pulse_start,
        spwm_shiftreg_waveform.a_pulse_end);
    if (spwm_in_b_pulse) {
      spwm_row_bits |= h.b;
      if (spwm_wrapped_to_row_zero) {
        spwm_row_bits |= h.c;
      }
    }
  }

  if (spwm_in_a_pulse) {
    spwm_row_bits |= h.a;
    if (!spwm_type2_transport && spwm_wrapped_to_row_zero) {
      spwm_row_bits |= h.c;
    }
  }

  return spwm_row_bits;
}

// Drive the shift-register row-select waveform during the blank clocks before
// the next OE burst.
//
// Type 1:
// Channel A carries a configurable pulse inside the blanking window, while
// Channel C is added on the wrap cycle from the last row back to row 0.
//
// Type 2:
// Channel B starts before A and ends on the same clock. On the wrap cycle,
// Channel C mirrors that longer B-width pulse.
void spwm_row_shiftreg_drive_blanking(GPIO *io, const HardwareMapping &h,
                                      const SPWM_Scan_Config &spwm_scan_config,
                                      SPWM_Scan_State *spwm_scan_state) {
  if (io == nullptr || spwm_scan_state == nullptr) return;

  const gpio_bits_t spwm_row_mask = h.a | h.b | h.c;
  gpio_bits_t spwm_row_bits = 0;
  const int spwm_blank_clks = spwm_row_blank_clk_count(spwm_scan_config);
  if (spwm_blank_clks > 0 &&
      spwm_scan_state->phase >= spwm_scan_config.advance_phase &&
      spwm_scan_state->phase < spwm_scan_config.row_clks) {
    const int spwm_blank_phase =
        spwm_scan_state->phase - spwm_scan_config.advance_phase;
    spwm_row_bits = spwm_resolve_shiftreg_row_bits(
        h, spwm_scan_config.shiftreg_waveform, spwm_blank_phase,
        spwm_scan_state->wrapped_to_row_zero);
  }

  // The blank-clock row-select waveforms only need level changes when the
  // pulse starts or ends, so avoid re-writing A/B/C on clocks where the state
  // holds.
  if (!spwm_scan_state->shiftreg_row_bits_valid ||
      spwm_scan_state->shiftreg_row_bits != spwm_row_bits) {
    io->WriteMaskedBits(spwm_row_bits, spwm_row_mask);
    spwm_scan_state->shiftreg_row_bits = spwm_row_bits;
    spwm_scan_state->shiftreg_row_bits_valid = true;
  }
}

// Return the cached scan phase where the next OE burst should be armed.
int spwm_resolve_oe_arm_phase(const SPWM_Scan_Config &spwm_scan_config) {
  return spwm_scan_config.oe_arm_phase;
}

// Arm the next OE burst once the scan reaches the phase where the visible
// clocks for the active row should begin, unless the first burst is still
// intentionally being skipped or an earlier burst is still in flight.
void spwm_scan_pre_clock_maybe_arm_oe(
    int spwm_phase, const SPWM_Scan_Config &spwm_scan_config,
    SPWM_OE_Gate_State *spwm_oe_gate,
    SPWM_Scan_State *spwm_scan_state) {
  if (spwm_scan_state == nullptr ||
      spwm_phase != spwm_scan_config.oe_arm_phase ||
      spwm_scan_config.oe_clks <= 0 ||
      spwm_oe_gate_is_pending(spwm_oe_gate)) {
    return;
  }

  if (!spwm_scan_config.skip_first_oe || spwm_scan_state->oe_primed) {
    spwm_auto_tune_on_oe_pulse_start(
        spwm_oe_gate != nullptr ? spwm_oe_gate->auto_tune : nullptr,
        spwm_oe_gate != nullptr ? spwm_oe_gate->section
                                : SPWM_AUTO_TUNE_SECTION_NONE);
    spwm_arm_oe_gate(spwm_oe_gate, spwm_scan_config.oe_clks);
  } else {
    spwm_scan_state->oe_primed = true;
  }
}

// Advance to the next logical row and wrap back to row 0 at the end of the
// scan group.
void spwm_advance_scan_row(SPWM_Scan_State *spwm_scan_state,
                           int spwm_double_rows) {
  if (spwm_scan_state == nullptr || spwm_double_rows <= 0) return;

  // Clear the wrap marker for ordinary row advances. It will be asserted only
  // for the single transition from the last row back to row 0.
  spwm_scan_state->wrapped_to_row_zero = false;
  if (++spwm_scan_state->row >= spwm_double_rows) {
    spwm_scan_state->row = 0;
    spwm_scan_state->wrapped_to_row_zero = true;
  }
}

// Direct SPWM path: set the row directly, then run the SPWM clocks for that
// row period.
bool spwm_scan_pre_clock_direct(GPIO *io, const HardwareMapping &h,
                                RowAddressSetter *spwm_row_setter,
                                int spwm_double_rows,
                                const SPWM_Scan_Config &spwm_scan_config,
                                SPWM_OE_Gate_State *spwm_oe_gate,
                                SPWM_Scan_State *spwm_scan_state) {
  if (spwm_row_setter == nullptr || spwm_scan_state == nullptr ||
      spwm_scan_config.row_clks <= 0) {
    return false;
  }

  (void)h;
  const int spwm_phase = spwm_scan_state->phase;
  bool spwm_advanced_row = false;

  if (spwm_scan_config.row_before_oe) {
    if (spwm_phase == spwm_scan_config.advance_phase) {
      spwm_advance_scan_row(spwm_scan_state, spwm_double_rows);
      spwm_row_setter->SetRowAddress(io, spwm_scan_state->row);
      spwm_advanced_row = true;
    }

    spwm_scan_pre_clock_maybe_arm_oe(spwm_phase, spwm_scan_config,
                                     spwm_oe_gate, spwm_scan_state);
  } else {
    spwm_scan_pre_clock_maybe_arm_oe(spwm_phase, spwm_scan_config,
                                     spwm_oe_gate, spwm_scan_state);

    if (spwm_phase == spwm_scan_config.advance_phase) {
      spwm_advance_scan_row(spwm_scan_state, spwm_double_rows);
      spwm_row_setter->SetRowAddress(io, spwm_scan_state->row);
      spwm_advanced_row = true;
    }
  }

  return spwm_advanced_row;
}

// Shiftreg-clock-select SPWM path: the blank clocks themselves carry the row
// select waveform, so row advance and row signalling happen inside scan timing
// instead of through SetRowAddress().
bool spwm_scan_pre_clock_shiftreg(GPIO *io, const HardwareMapping &h,
                                  RowAddressSetter *spwm_row_setter,
                                  int spwm_double_rows,
                                  const SPWM_Scan_Config &spwm_scan_config,
                                  SPWM_OE_Gate_State *spwm_oe_gate,
                                  SPWM_Scan_State *spwm_scan_state) {
  if (spwm_row_setter == nullptr || spwm_scan_state == nullptr ||
      spwm_scan_config.row_clks <= 0) {
    return false;
  }

  const int spwm_phase = spwm_scan_state->phase;
  bool spwm_advanced_row = false;

  if (spwm_scan_config.row_before_oe) {
    if (spwm_phase == spwm_scan_config.advance_phase) {
      spwm_advance_scan_row(spwm_scan_state, spwm_double_rows);
      spwm_advanced_row = true;
    }

    spwm_row_shiftreg_drive_blanking(io, h, spwm_scan_config,
                                     spwm_scan_state);
    spwm_scan_pre_clock_maybe_arm_oe(spwm_phase, spwm_scan_config,
                                     spwm_oe_gate, spwm_scan_state);
  } else {
    spwm_scan_pre_clock_maybe_arm_oe(spwm_phase, spwm_scan_config,
                                     spwm_oe_gate, spwm_scan_state);

    if (spwm_phase == spwm_scan_config.advance_phase) {
      spwm_advance_scan_row(spwm_scan_state, spwm_double_rows);
      spwm_advanced_row = true;
    }

    spwm_row_shiftreg_drive_blanking(io, h, spwm_scan_config,
                                     spwm_scan_state);
  }

  return spwm_advanced_row;
}

// Advance the scan phase after one clock and wrap back to the start of the row
// period when needed.
void spwm_scan_post_clock(const SPWM_Scan_Config &spwm_scan_config,
                          SPWM_Scan_State *spwm_scan_state) {
  if (spwm_scan_state == nullptr || spwm_scan_config.row_clks <= 0) return;
  if (++spwm_scan_state->phase >= spwm_scan_config.row_clks) {
    spwm_scan_state->phase = 0;
  }
}

// Finish any partially emitted OE pulse after RGB upload has stopped issuing new
// clocks.
void spwm_drain_pending_oe_gate(GPIO *io, const HardwareMapping &h,
                                gpio_bits_t spwm_data_mask,
                                SPWM_OE_Gate_State *spwm_oe_gate,
                                const SPWM_Scan_Config &spwm_scan_config,
                                SPWM_Scan_State *spwm_scan_state) {
  if (spwm_oe_gate == nullptr || spwm_scan_state == nullptr) return;
  if (!(spwm_oe_gate->remaining > 0 || spwm_oe_gate->active)) return;

  const int spwm_drain_clks = spwm_oe_gate->remaining;
  for (int spwm_clk = 0; spwm_clk < spwm_drain_clks; ++spwm_clk) {
    spwm_clock_pulse(io, h, 0, spwm_data_mask, spwm_oe_gate);
    spwm_scan_post_clock(spwm_scan_config, spwm_scan_state);
  }
  io->ClearBits(h.output_enable);
}

// Build the per-clock GPIO payload for one uploaded grayscale word by masking
// each stored framebuffer bitplane word down to the six RGB pins used by the
// SPWM upload path.
SPWM_Pixel_Block_GPIO_Bits spwm_repack_pixel_block_gpio_bits(
    const gpio_bits_t *spwm_pixel_base,
    int spwm_bitplane_stride,
    int spwm_start_bit,
    int spwm_active_bits,
    gpio_bits_t spwm_rgb_mask) {
  SPWM_Pixel_Block_GPIO_Bits spwm_block_gpio_bits = {{0}};
  if (spwm_pixel_base == nullptr || spwm_active_bits <= 0) {
    return spwm_block_gpio_bits;
  }

  const int spwm_clamped_pwm_bits =
      spwm_active_bits > SPWM_WORD_BIT_COUNT ? SPWM_WORD_BIT_COUNT
                                             : spwm_active_bits;
  const int spwm_word_bit_base = SPWM_WORD_BIT_COUNT - spwm_clamped_pwm_bits;
  for (int spwm_source_bit = 0;
       spwm_source_bit < spwm_clamped_pwm_bits;
       ++spwm_source_bit) {
    spwm_block_gpio_bits.word_gpio_bits[spwm_word_bit_base + spwm_source_bit] =
        spwm_pixel_base[(spwm_start_bit + spwm_source_bit) *
                        spwm_bitplane_stride] &
        spwm_rgb_mask;
  }

  return spwm_block_gpio_bits;
}

// Snapshot the immutable pin and destination choices used throughout one
// framebuffer upload. Pin order preserves parallel-chain and color identity.
SPWM_RGB_Routing_Context spwm_make_rgb_routing_context(
    const HardwareMapping &h, int spwm_data_layout) {
  SPWM_RGB_Routing_Context spwm_context = {
      spwm_get_parallel_chains() * SPWM_RGB_COLOR_COUNT,
      {
          h.p0_r1, h.p0_g1, h.p0_b1,
          h.p1_r1, h.p1_g1, h.p1_b1,
          h.p2_r1, h.p2_g1, h.p2_b1,
          h.p3_r1, h.p3_g1, h.p3_b1,
          h.p4_r1, h.p4_g1, h.p4_b1,
          h.p5_r1, h.p5_g1, h.p5_b1,
      },
      {
          h.p0_r2, h.p0_g2, h.p0_b2,
          h.p1_r2, h.p1_g2, h.p1_b2,
          h.p2_r2, h.p2_g2, h.p2_b2,
          h.p3_r2, h.p3_g2, h.p3_b2,
          h.p4_r2, h.p4_g2, h.p4_b2,
          h.p5_r2, h.p5_g2, h.p5_b2,
      },
      {},
      spwm_data_layout == SPWM_DATA_LAYOUT_FULL_HEIGHT_SERIAL_BOTH ||
          spwm_data_layout == SPWM_DATA_LAYOUT_FULL_HEIGHT_SERIAL_RGB1,
      spwm_data_layout == SPWM_DATA_LAYOUT_FULL_HEIGHT_SERIAL_BOTH ||
          spwm_data_layout == SPWM_DATA_LAYOUT_FULL_HEIGHT_SERIAL_RGB2,
      spwm_data_layout == SPWM_DATA_LAYOUT_FULL_HEIGHT_LEFT_RGB2,
  };
  for (int spwm_signal = 0; spwm_signal < spwm_context.signal_count;
       ++spwm_signal) {
    spwm_context.rgb1_rgb2_bits[spwm_signal] =
        spwm_context.rgb1_bits[spwm_signal] |
        spwm_context.rgb2_bits[spwm_signal];
  }
  return spwm_context;
}

// Full-height 64S panels use RGB1 and RGB2 as separate data paths. Select one
// row-source pin map, then route its chain/color signals to one output map.
gpio_bits_t spwm_route_selected_row_rgb_bits(
    gpio_bits_t spwm_out_bits,
    const gpio_bits_t *spwm_source_bits,
    const gpio_bits_t *spwm_destination_bits,
    int spwm_signal_count) {
  gpio_bits_t spwm_remapped_bits = 0;
  for (int spwm_signal = 0; spwm_signal < spwm_signal_count;
       ++spwm_signal) {
    if (spwm_out_bits & spwm_source_bits[spwm_signal]) {
      spwm_remapped_bits |= spwm_destination_bits[spwm_signal];
    }
  }
  return spwm_remapped_bits;
}

// Full-width serial panels retain the ordinary column stream instead of
// splitting it into two repeated half-width lanes. The destination map may
// contain one output bus or the precombined RGB1|RGB2 bits used by layout 3.
void spwm_apply_full_height_serial_upload_mapping(
    SPWM_Pixel_Block_GPIO_Bits *spwm_block_gpio_bits,
    const gpio_bits_t *spwm_source_bits,
    const gpio_bits_t *spwm_destination_bits,
    int spwm_signal_count,
    int spwm_first_populated_word_bit) {
  if (spwm_block_gpio_bits == nullptr) return;

  for (int spwm_bit = spwm_first_populated_word_bit;
       spwm_bit < SPWM_WORD_BIT_COUNT; ++spwm_bit) {
    spwm_block_gpio_bits->word_gpio_bits[spwm_bit] =
        spwm_route_selected_row_rgb_bits(
            spwm_block_gpio_bits->word_gpio_bits[spwm_bit],
            spwm_source_bits, spwm_destination_bits, spwm_signal_count);
  }
}

// Combine matching columns from the left and right framebuffer halves. Rows
// 0..double_rows-1 read their colors from the framebuffer RGB1 bits; later rows
// read RGB2. The selected layout decides which physical RGB lane carries each
// horizontal half.
void spwm_combine_split_double_row_upload_blocks(
    SPWM_Pixel_Block_GPIO_Bits *spwm_left_block_gpio_bits,
    const SPWM_Pixel_Block_GPIO_Bits &spwm_right_block_gpio_bits,
    const gpio_bits_t *spwm_source_bits,
    const gpio_bits_t *spwm_left_destination_bits,
    const gpio_bits_t *spwm_right_destination_bits,
    int spwm_signal_count,
    int spwm_first_populated_word_bit) {
  if (spwm_left_block_gpio_bits == nullptr) return;

  for (int spwm_bit = spwm_first_populated_word_bit;
       spwm_bit < SPWM_WORD_BIT_COUNT; ++spwm_bit) {
    const gpio_bits_t spwm_left_bits = spwm_route_selected_row_rgb_bits(
        spwm_left_block_gpio_bits->word_gpio_bits[spwm_bit],
        spwm_source_bits, spwm_left_destination_bits, spwm_signal_count);
    const gpio_bits_t spwm_right_bits = spwm_route_selected_row_rgb_bits(
        spwm_right_block_gpio_bits.word_gpio_bits[spwm_bit],
        spwm_source_bits, spwm_right_destination_bits, spwm_signal_count);
    spwm_left_block_gpio_bits->word_gpio_bits[spwm_bit] =
        spwm_left_bits | spwm_right_bits;
  }
}

// Only select individual physical rows when the caller has expanded the upload
// loop to the full panel height and the framebuffer is exactly two physical
// halves. Partial scan overrides such as 86-row ICND1065L panels with
// --led-spwm-scan=43 retain the existing paired-row upload path.
bool spwm_should_use_full_height_upload(
    const SPWM_Framebuffer_View &spwm_framebuffer_view,
    int spwm_upload_rows,
    int spwm_data_layout) {
  const SPWM_Runtime_State &spwm_runtime_state = spwm_get_runtime_state();
  const int spwm_panel_columns = spwm_runtime_state.panel_columns;
  return spwm_data_layout_uses_full_height_rows(spwm_data_layout) &&
         spwm_row_address_type_uses_blank_clock(
             spwm_runtime_state.row_address_type) &&
         spwm_runtime_state.multiplexing == 0 &&
         spwm_upload_rows == spwm_framebuffer_view.rows &&
         spwm_framebuffer_view.double_rows > 0 &&
         spwm_panel_columns > 0 &&
         (!spwm_data_layout_splits_horizontal_lanes(spwm_data_layout) ||
          spwm_panel_columns % 32 == 0) &&
         spwm_framebuffer_view.columns >= spwm_panel_columns &&
         spwm_framebuffer_view.columns % spwm_panel_columns == 0 &&
         spwm_framebuffer_view.rows ==
             spwm_framebuffer_view.double_rows * 2;
}

// Direct-row-select upload still only needs two scan actions while clocks are
// being emitted: arm OE and update the external row address at the configured
// phase. The active OE style decides which of those happens first.
void spwm_scan_pre_clock_direct_upload(
    GPIO *io, RowAddressSetter *spwm_row_setter, int spwm_double_rows,
    const SPWM_Scan_Config &spwm_scan_config,
    SPWM_OE_Gate_State *spwm_oe_gate,
    SPWM_Scan_State *spwm_scan_state) {
  if (io == nullptr || spwm_row_setter == nullptr || spwm_scan_state == nullptr ||
      spwm_scan_config.row_clks <= 0) {
    return;
  }

  if (spwm_scan_config.row_before_oe) {
    if (spwm_scan_state->phase == spwm_scan_config.advance_phase) {
      spwm_advance_scan_row(spwm_scan_state, spwm_double_rows);
      spwm_row_setter->SetRowAddress(io, spwm_scan_state->row);
    }

    spwm_scan_pre_clock_maybe_arm_oe(
        spwm_scan_state->phase, spwm_scan_config, spwm_oe_gate,
        spwm_scan_state);
    return;
  }

  spwm_scan_pre_clock_maybe_arm_oe(
      spwm_scan_state->phase, spwm_scan_config, spwm_oe_gate, spwm_scan_state);
  if (spwm_scan_state->phase == spwm_scan_config.advance_phase) {
    spwm_advance_scan_row(spwm_scan_state, spwm_double_rows);
    spwm_row_setter->SetRowAddress(io, spwm_scan_state->row);
  }
}

void spwm_finish_shared_initial_oe_if_ready(
    bool *spwm_initial_oe_pending,
    const SPWM_Scan_Config &spwm_upload_scan_config,
    SPWM_OE_Gate_State *spwm_oe_gate,
    SPWM_Scan_State *spwm_scan_state);

// Return the optional LAT-low blank clocks inserted after each normal RGB
// upload LAT pulse.
int spwm_get_rgb_upload_lat_spacer_clk_count() {
  return spwm_get_panel_settings().rgb_upload_lat_spacer_clk_count;
}

void spwm_finish_upload_clock(
    bool spwm_defer_row_scan, bool *spwm_initial_oe_pending,
    const SPWM_Scan_Config &spwm_upload_scan_config,
    SPWM_OE_Gate_State *spwm_oe_gate,
    SPWM_Scan_State *spwm_scan_state) {
  if (spwm_defer_row_scan) {
    spwm_finish_shared_initial_oe_if_ready(
        spwm_initial_oe_pending, spwm_upload_scan_config,
        spwm_oe_gate, spwm_scan_state);
  } else {
    spwm_scan_post_clock(spwm_upload_scan_config, spwm_scan_state);
  }
}

// Walk the framebuffer in logical-row / channel / chip order and hand each
// pre-expanded 16-clock word block to the caller. The direct and shift-register
// upload paths share this traversal and only differ in the per-clock scan/OE
// scheduling wrapped around each block.
template <typename SPWM_Block_Emitter>
void spwm_upload_framebuffer_blocks(
    GPIO *io, const HardwareMapping &h,
    const SPWM_Framebuffer_View &spwm_framebuffer_view,
    gpio_bits_t spwm_rgb_mask,
    int spwm_upload_rows,
    int spwm_data_layout,
    bool spwm_full_height_upload,
    int spwm_chip_count,
    int spwm_channels_per_chip,
    SPWM_Block_Emitter spwm_emit_block) {
  if (io == nullptr || spwm_framebuffer_view.bitplane_buffer == nullptr ||
      spwm_upload_rows <= 0 || spwm_chip_count <= 0 ||
      spwm_channels_per_chip <= 0) {
    return;
  }

  const SPWM_Pixel_Block_GPIO_Bits spwm_zero_block_gpio_bits = {{0}};
  const SPWM_Panel_Settings &spwm_settings = spwm_get_panel_settings();
  const int spwm_last_chip = spwm_chip_count - 1;
  const size_t spwm_row_stride =
      static_cast<size_t>(spwm_framebuffer_view.columns) *
      static_cast<size_t>(spwm_framebuffer_view.stored_bitplanes);
  const int spwm_first_active_bit =
      spwm_framebuffer_view.stored_bitplanes - spwm_framebuffer_view.pwm_bits;
  const int spwm_start_bit =
      std::max(spwm_framebuffer_view.pwm_low_bit, spwm_first_active_bit);
  const int spwm_active_bits =
      spwm_framebuffer_view.stored_bitplanes - spwm_start_bit;
  const int spwm_clamped_active_bits =
      std::max(0, std::min(spwm_active_bits, SPWM_WORD_BIT_COUNT));
  // Repack leaves the leading entries zero. Route only its populated tail;
  // the emitter still clocks every SPWM word position, including those zeros.
  const int spwm_first_populated_word_bit =
      SPWM_WORD_BIT_COUNT - spwm_clamped_active_bits;
  const bool spwm_split_horizontal_lanes =
      spwm_full_height_upload &&
      spwm_data_layout_splits_horizontal_lanes(spwm_data_layout);
  const bool spwm_serial_columns =
      spwm_full_height_upload &&
      spwm_data_layout_uses_serial_columns(spwm_data_layout);
  SPWM_RGB_Routing_Context spwm_routing_context = {};
  if (spwm_full_height_upload) {
    spwm_routing_context =
        spwm_make_rgb_routing_context(h, spwm_data_layout);
  }
  const gpio_bits_t *const spwm_split_left_destination_bits =
      spwm_routing_context.split_left_uses_rgb2
          ? spwm_routing_context.rgb2_bits
          : spwm_routing_context.rgb1_bits;
  const gpio_bits_t *const spwm_split_right_destination_bits =
      spwm_routing_context.split_left_uses_rgb2
          ? spwm_routing_context.rgb1_bits
          : spwm_routing_context.rgb2_bits;
  const bool spwm_serial_uses_both =
      spwm_routing_context.serial_uses_rgb1 &&
      spwm_routing_context.serial_uses_rgb2;
  const gpio_bits_t *const spwm_serial_destination_bits =
      spwm_serial_uses_both
          ? spwm_routing_context.rgb1_rgb2_bits
          : (spwm_routing_context.serial_uses_rgb1
                 ? spwm_routing_context.rgb1_bits
                 : spwm_routing_context.rgb2_bits);
  const int spwm_split_lane_chain_columns =
      spwm_split_horizontal_lanes
          ? spwm_framebuffer_view.columns / 2
          : spwm_framebuffer_view.columns;
  // The framebuffer width includes every chained panel. Split each panel into
  // its own RGB1/RGB2 halves instead of splitting the complete chain once.
  const int spwm_split_panel_columns =
      spwm_split_horizontal_lanes
          ? spwm_get_runtime_state().panel_columns
          : spwm_framebuffer_view.columns;
  const int spwm_split_panel_lane_columns = spwm_split_panel_columns / 2;
  const int spwm_physical_slot_count =
      spwm_chip_count * spwm_channels_per_chip;
  const SPWM_Column_Route spwm_invalid_column_route = {-1, -1};
  std::vector<SPWM_Column_Route> spwm_column_routes(
      static_cast<size_t>(spwm_physical_slot_count),
      spwm_invalid_column_route);

  // Column routing depends only on upload geometry, not on the active row.
  // Preserve sparse blank slots and split-lane ordering while moving the
  // division/modulo and missing-column walk out of the per-row hot loop.
  for (int spwm_physical_column = 0;
       spwm_physical_column < spwm_physical_slot_count;
       ++spwm_physical_column) {
    SPWM_Column_Route &spwm_column_route =
        spwm_column_routes[spwm_physical_column];
    if (spwm_split_horizontal_lanes) {
      const int spwm_lane_chain_column =
          spwm_physical_column % spwm_split_lane_chain_columns;
      const int spwm_panel_index =
          spwm_lane_chain_column / spwm_split_panel_lane_columns;
      const int spwm_panel_lane_column =
          spwm_lane_chain_column % spwm_split_panel_lane_columns;
      spwm_column_route.first_column =
          spwm_panel_index * spwm_split_panel_columns +
          spwm_panel_lane_column;
      spwm_column_route.second_column =
          spwm_column_route.first_column + spwm_split_panel_lane_columns;
      continue;
    }

    const int spwm_visible_column = spwm_map_physical_column_to_visible(
        spwm_physical_column, spwm_settings);
    if (spwm_visible_column >= 0 &&
        spwm_visible_column < spwm_framebuffer_view.columns) {
      spwm_column_route.first_column = spwm_visible_column;
    }
  }

  for (int spwm_row = 0; spwm_row < spwm_upload_rows; ++spwm_row) {
    // Full-height modes reuse the stored double rows, then select RGB1 or RGB2
    // framebuffer data for each physical upload row in the mapping below.
    const int spwm_framebuffer_row =
        spwm_full_height_upload
            ? spwm_row % spwm_framebuffer_view.double_rows
            : spwm_row;
    if (spwm_framebuffer_row < 0 ||
        spwm_framebuffer_row >= spwm_framebuffer_view.double_rows) {
      continue;
    }

    const gpio_bits_t *const spwm_row_base =
        spwm_framebuffer_view.bitplane_buffer +
        spwm_framebuffer_row * spwm_row_stride;
    const gpio_bits_t *const spwm_row_source_bits =
        spwm_row >= spwm_framebuffer_view.double_rows
            ? spwm_routing_context.rgb2_bits
            : spwm_routing_context.rgb1_bits;

    for (int spwm_channel = 0;
         spwm_channel < spwm_channels_per_chip;
         ++spwm_channel) {
      io->ClearBits(h.clock | h.strobe | spwm_rgb_mask);

      for (int spwm_chip = 0; spwm_chip < spwm_chip_count; ++spwm_chip) {
        const int spwm_physical_column =
            spwm_chip * spwm_channels_per_chip + spwm_channel;
        const SPWM_Column_Route &spwm_column_route =
            spwm_column_routes[spwm_physical_column];
        SPWM_Pixel_Block_GPIO_Bits spwm_block_gpio_bits =
            spwm_zero_block_gpio_bits;
        if (spwm_split_horizontal_lanes) {
          // Keep the established full-width timing, but repeat each half-width
          // lane so the final retained words contain both horizontal halves.
          spwm_block_gpio_bits = spwm_repack_pixel_block_gpio_bits(
              spwm_row_base + spwm_column_route.first_column,
              spwm_framebuffer_view.columns,
              spwm_start_bit,
              spwm_active_bits,
              spwm_rgb_mask);
          const SPWM_Pixel_Block_GPIO_Bits spwm_right_block_gpio_bits =
              spwm_repack_pixel_block_gpio_bits(
                  spwm_row_base + spwm_column_route.second_column,
                  spwm_framebuffer_view.columns,
                  spwm_start_bit,
                  spwm_active_bits,
                  spwm_rgb_mask);
          spwm_combine_split_double_row_upload_blocks(
              &spwm_block_gpio_bits, spwm_right_block_gpio_bits,
              spwm_row_source_bits, spwm_split_left_destination_bits,
              spwm_split_right_destination_bits,
              spwm_routing_context.signal_count,
              spwm_first_populated_word_bit);
        } else {
          if (spwm_column_route.first_column >= 0) {
            spwm_block_gpio_bits = spwm_repack_pixel_block_gpio_bits(
                spwm_row_base + spwm_column_route.first_column,
                spwm_framebuffer_view.columns,
                spwm_start_bit,
                spwm_active_bits,
                spwm_rgb_mask);
            if (spwm_serial_columns) {
              spwm_apply_full_height_serial_upload_mapping(
                  &spwm_block_gpio_bits, spwm_row_source_bits,
                  spwm_serial_destination_bits,
                  spwm_routing_context.signal_count,
                  spwm_first_populated_word_bit);
            }
          }
        }
        spwm_emit_block(spwm_block_gpio_bits, spwm_chip == spwm_last_chip);
      }
    }
  }
}

// Purpose: Upload one full framebuffer using the direct-row-select scan path.
// Inputs: Prepared framebuffer words, upload geometry, direct row setter, and
// any pending shared startup-OE state.
// Outputs: None.
// Side effects: Emits RGB/LAT/CLK/OE timing directly to the panel.
//
// In this path the row address is still an explicit control output: scan
// timing decides when the next logical row should become active, then calls
// SetRowAddress() directly. The panel profile still controls OE ordering and
// whether the startup OE burst is shared with upload.
void spwm_upload_framebuffer_direct(
    GPIO *io, const HardwareMapping &h,
    RowAddressSetter *spwm_row_setter,
    const SPWM_Framebuffer_View &spwm_framebuffer_view,
    gpio_bits_t spwm_rgb_mask,
    gpio_bits_t spwm_data_mask,
    int spwm_upload_rows,
    int spwm_data_layout,
    bool spwm_full_height_upload,
    int spwm_chip_count,
    int spwm_channels_per_chip,
    int spwm_word_bits,
    const SPWM_Scan_Config &spwm_upload_scan_config,
    SPWM_OE_Gate_State *spwm_oe_gate,
    SPWM_Scan_State *spwm_scan_state,
    bool *spwm_initial_oe_pending) {
  if (io == nullptr || spwm_row_setter == nullptr ||
      spwm_framebuffer_view.bitplane_buffer == nullptr ||
      spwm_scan_state == nullptr || spwm_initial_oe_pending == nullptr) {
    return;
  }

  const int spwm_lat_spacer_clk_count =
      spwm_get_rgb_upload_lat_spacer_clk_count();
  const auto spwm_emit_lat_spacer = [&]() {
    io->ClearBits(h.strobe | spwm_rgb_mask);
    for (int spwm_clk = 0;
         spwm_clk < spwm_lat_spacer_clk_count;
         ++spwm_clk) {
      const bool spwm_defer_row_scan = *spwm_initial_oe_pending;
      if (!spwm_defer_row_scan) {
        spwm_scan_pre_clock_direct_upload(
            io, spwm_row_setter, spwm_upload_rows, spwm_upload_scan_config,
            spwm_oe_gate, spwm_scan_state);
      }

      spwm_clock_pulse(io, h, 0, spwm_data_mask, spwm_oe_gate);
      spwm_finish_upload_clock(
          spwm_defer_row_scan, spwm_initial_oe_pending,
          spwm_upload_scan_config, spwm_oe_gate, spwm_scan_state);
    }
  };

  spwm_upload_framebuffer_blocks(
      io, h, spwm_framebuffer_view, spwm_rgb_mask, spwm_upload_rows,
      spwm_data_layout, spwm_full_height_upload, spwm_chip_count,
      spwm_channels_per_chip,
      [&](const SPWM_Pixel_Block_GPIO_Bits &spwm_block_gpio_bits,
          bool spwm_is_last_chip) {
        for (int spwm_bit = spwm_word_bits - 1; spwm_bit >= 0; --spwm_bit) {
          const bool spwm_direct_defer_row_scan = *spwm_initial_oe_pending;
          if (!spwm_direct_defer_row_scan) {
            spwm_scan_pre_clock_direct_upload(
                io, spwm_row_setter, spwm_upload_rows, spwm_upload_scan_config,
                spwm_oe_gate, spwm_scan_state);
          }

          const bool spwm_latch = spwm_is_last_chip && spwm_bit == 0;
          if (spwm_latch) io->SetBits(h.strobe);
          spwm_clock_pulse(io, h,
                           spwm_block_gpio_bits.word_gpio_bits[spwm_bit],
                           spwm_data_mask, spwm_oe_gate);
          if (spwm_latch) io->ClearBits(h.strobe);

          spwm_finish_upload_clock(
              spwm_direct_defer_row_scan, spwm_initial_oe_pending,
              spwm_upload_scan_config, spwm_oe_gate, spwm_scan_state);
          if (spwm_latch) spwm_emit_lat_spacer();
        }
      });
}

// Scan-clock row-select upload emits the row-select waveform inside the blank
// clocks. The active panel profile still decides whether OE comes before or
// after that blanking window, but unlike the direct path there is no
// SetRowAddress() call in the hot upload loop.
void spwm_scan_pre_clock_shiftreg_upload(
    GPIO *io, const HardwareMapping &h, int spwm_scan_rows,
    const SPWM_Scan_Config &spwm_scan_config,
    SPWM_OE_Gate_State *spwm_oe_gate,
    SPWM_Scan_State *spwm_scan_state) {
  if (io == nullptr || spwm_scan_state == nullptr ||
      spwm_scan_config.row_clks <= 0) {
    return;
  }

  if (spwm_scan_config.row_before_oe) {
    if (spwm_scan_state->phase == spwm_scan_config.advance_phase) {
      spwm_advance_scan_row(spwm_scan_state, spwm_scan_rows);
    }

    spwm_row_shiftreg_drive_blanking(io, h, spwm_scan_config,
                                     spwm_scan_state);
    spwm_scan_pre_clock_maybe_arm_oe(
        spwm_scan_state->phase, spwm_scan_config, spwm_oe_gate,
        spwm_scan_state);
    return;
  }

  spwm_scan_pre_clock_maybe_arm_oe(
      spwm_scan_state->phase, spwm_scan_config, spwm_oe_gate, spwm_scan_state);
  if (spwm_scan_state->phase == spwm_scan_config.advance_phase) {
    spwm_advance_scan_row(spwm_scan_state, spwm_scan_rows);
  }
  spwm_row_shiftreg_drive_blanking(io, h, spwm_scan_config, spwm_scan_state);
}

// The shared startup burst is consumed by the first upload clocks. Once it
// drains, force the scan state to the row-advance phase so the next clocks
// begin the regular row-setup section for the next row period.
void spwm_finish_shared_initial_oe_if_ready(
    bool *spwm_initial_oe_pending,
    const SPWM_Scan_Config &spwm_upload_scan_config,
    SPWM_OE_Gate_State *spwm_oe_gate,
    SPWM_Scan_State *spwm_scan_state) {
  if (spwm_initial_oe_pending == nullptr || !*spwm_initial_oe_pending ||
      spwm_scan_state == nullptr || spwm_oe_gate_is_pending(spwm_oe_gate)) {
    return;
  }

  *spwm_initial_oe_pending = false;
  spwm_scan_state->phase = spwm_upload_scan_config.advance_phase;
  if (spwm_oe_gate != nullptr) {
    spwm_oe_gate->section = SPWM_AUTO_TUNE_SECTION_UPLOAD;
  }
}

// Purpose: Upload one full framebuffer using the scan-clock row-select path.
// Inputs: Prepared framebuffer words, upload geometry, and shared startup state.
// Outputs: None.
// Side effects: Emits RGB/LAT/CLK/OE timing and advances shift-register row state.
//
// This path treats the blank clocks as part of row selection. The same clocks
// that separate OE windows also generate the blank-clock pulse pattern that
// advances the panel's internal row shift register, so row signalling stays
// tied to the scan clocks even though the panel profile still controls OE
// timing.
void spwm_upload_framebuffer_shiftreg(
    GPIO *io, const HardwareMapping &h,
    const SPWM_Framebuffer_View &spwm_framebuffer_view,
    gpio_bits_t spwm_rgb_mask,
    gpio_bits_t spwm_data_mask,
    int spwm_upload_rows,
    int spwm_data_layout,
    bool spwm_full_height_upload,
    int spwm_scan_rows,
    int spwm_chip_count,
    int spwm_channels_per_chip,
    int spwm_word_bits,
    const SPWM_Scan_Config &spwm_upload_scan_config,
    SPWM_OE_Gate_State *spwm_oe_gate,
    SPWM_Scan_State *spwm_scan_state,
    bool *spwm_initial_oe_pending) {
  if (io == nullptr || spwm_framebuffer_view.bitplane_buffer == nullptr ||
      spwm_scan_state == nullptr || spwm_initial_oe_pending == nullptr) {
    return;
  }

  const int spwm_lat_spacer_clk_count =
      spwm_get_rgb_upload_lat_spacer_clk_count();
  const auto spwm_emit_lat_spacer = [&]() {
    io->ClearBits(h.strobe | spwm_rgb_mask);
    for (int spwm_clk = 0;
         spwm_clk < spwm_lat_spacer_clk_count;
         ++spwm_clk) {
      const bool spwm_defer_row_scan = *spwm_initial_oe_pending;
      if (!spwm_defer_row_scan) {
        spwm_scan_pre_clock_shiftreg_upload(
            io, h, spwm_scan_rows, spwm_upload_scan_config,
            spwm_oe_gate, spwm_scan_state);
      }

      spwm_clock_pulse(io, h, 0, spwm_data_mask, spwm_oe_gate);
      spwm_finish_upload_clock(
          spwm_defer_row_scan, spwm_initial_oe_pending,
          spwm_upload_scan_config, spwm_oe_gate, spwm_scan_state);
    }
  };

  spwm_upload_framebuffer_blocks(
      io, h, spwm_framebuffer_view, spwm_rgb_mask, spwm_upload_rows,
      spwm_data_layout, spwm_full_height_upload, spwm_chip_count,
      spwm_channels_per_chip,
      [&](const SPWM_Pixel_Block_GPIO_Bits &spwm_block_gpio_bits,
          bool spwm_is_last_chip) {
        for (int spwm_bit = spwm_word_bits - 1; spwm_bit >= 0; --spwm_bit) {
          const bool spwm_shiftreg_defer_row_scan = *spwm_initial_oe_pending;
          if (!spwm_shiftreg_defer_row_scan) {
            spwm_scan_pre_clock_shiftreg_upload(
                io, h, spwm_scan_rows, spwm_upload_scan_config,
                spwm_oe_gate, spwm_scan_state);
          }

          const bool spwm_latch = spwm_is_last_chip && spwm_bit == 0;
          if (spwm_latch) io->SetBits(h.strobe);
          spwm_clock_pulse(io, h,
                           spwm_block_gpio_bits.word_gpio_bits[spwm_bit],
                           spwm_data_mask, spwm_oe_gate);
          if (spwm_latch) io->ClearBits(h.strobe);

          spwm_finish_upload_clock(
              spwm_shiftreg_defer_row_scan, spwm_initial_oe_pending,
              spwm_upload_scan_config, spwm_oe_gate, spwm_scan_state);
          if (spwm_latch) spwm_emit_lat_spacer();
        }
      });
}

// Keep scanning rows and emitting OE pulses after upload so the finished frame
// remains visible.
template <SPWM_Scan_Pre_Clock_Handler spwm_scan_pre_clock_handler>
void spwm_free_run_scan(GPIO *io, const HardwareMapping &h,
                        gpio_bits_t spwm_rgb_mask,
                        gpio_bits_t spwm_data_mask,
                        RowAddressSetter *spwm_row_setter,
                        int spwm_double_rows,
                        int spwm_end_of_frame_extra_row_cycles,
                        const SPWM_Scan_Config &spwm_free_scan_config,
                        SPWM_OE_Gate_State *spwm_oe_gate,
                        SPWM_Scan_State *spwm_scan_state) {
  if (spwm_row_setter == nullptr || spwm_end_of_frame_extra_row_cycles <= 0 ||
      spwm_free_scan_config.row_clks <= 0 ||
      spwm_scan_state == nullptr) {
    return;
  }

  io->ClearBits(spwm_rgb_mask | h.strobe);
  spwm_scan_state->phase %= spwm_free_scan_config.row_clks;

  const int64_t spwm_total_hold_clks =
      static_cast<int64_t>(spwm_end_of_frame_extra_row_cycles) *
      static_cast<int64_t>(spwm_double_rows) *
      static_cast<int64_t>(spwm_free_scan_config.row_clks);

  for (int64_t spwm_clk = 0; spwm_clk < spwm_total_hold_clks; ++spwm_clk) {
    spwm_scan_pre_clock_handler(io, h, spwm_row_setter, spwm_double_rows,
                                spwm_free_scan_config, spwm_oe_gate,
                                spwm_scan_state);
    spwm_clock_pulse(io, h, 0, spwm_data_mask, spwm_oe_gate);
    spwm_scan_post_clock(spwm_free_scan_config, spwm_scan_state);
  }

  if (spwm_oe_gate != nullptr &&
      (spwm_oe_gate->remaining > 0 || spwm_oe_gate->active)) {
    io->ClearBits(spwm_rgb_mask | h.strobe);
    spwm_drain_pending_oe_gate(io, h, spwm_data_mask, spwm_oe_gate,
                               spwm_free_scan_config, spwm_scan_state);
  }
  io->ClearBits(h.output_enable);
}

// Return how many clocks must be emitted after a shift-register wrap so the
// delayed row-0 pulse has actually occurred.
int spwm_get_shiftreg_wrap_completion_clks(
    const SPWM_Scan_Config &spwm_scan_config) {
  return spwm_scan_config.shiftreg_waveform.a_pulse_end;
}

// Resolve the effective shift-register scan-row count. When no override is
// supplied, keep the historical rows/2 behavior derived from upload rows.
int spwm_get_effective_scan_rows(bool spwm_blank_clock_row_transport,
                                 int spwm_upload_rows) {
  if (!spwm_blank_clock_row_transport) return spwm_upload_rows;

  const int spwm_scan_rows_override = spwm_get_runtime_state().scan_rows;
  return spwm_scan_rows_override > 0 ? spwm_scan_rows_override
                                     : spwm_upload_rows;
}

// Continue scanning until the state machine lands on a clean row wrap before
// the next frame begins. Shift-register row select needs extra clocks after the
// wrap so the delayed row-0 wrap pulse is actually emitted before stopping.
template <SPWM_Scan_Pre_Clock_Handler spwm_scan_pre_clock_handler>
void spwm_align_frame_end_to_row_wrap(
    GPIO *io, const HardwareMapping &h,
    gpio_bits_t spwm_rgb_mask, gpio_bits_t spwm_data_mask,
    RowAddressSetter *spwm_row_setter, int spwm_double_rows,
    const SPWM_Scan_Config &spwm_align_scan_config,
    SPWM_OE_Gate_State *spwm_oe_gate,
    SPWM_Scan_State *spwm_scan_state) {
  if (spwm_row_setter == nullptr || spwm_double_rows <= 1 ||
      spwm_scan_state == nullptr || spwm_align_scan_config.row_clks <= 0) {
    return;
  }

  io->ClearBits(spwm_rgb_mask | h.strobe);
  io->ClearBits(h.output_enable);
  spwm_scan_state->phase %= spwm_align_scan_config.row_clks;

  const int spwm_last_row = spwm_double_rows - 1;
  const int spwm_row_address_type = spwm_get_runtime_state().row_address_type;
  const bool spwm_blank_clock_row_transport =
      spwm_row_address_type_uses_blank_clock(spwm_row_address_type);
  const bool spwm_type2_shiftreg_align =
      spwm_blank_clock_row_transport &&
      spwm_row_address_type == SPWM_ROW_ADDRESS_TYPE_2_SHIFTREG_AB_BLANK_CLOCK;
  const int spwm_wrap_completion_clks =
      spwm_blank_clock_row_transport
          ? spwm_get_shiftreg_wrap_completion_clks(spwm_align_scan_config)
          : 0;
  bool spwm_wrapped = false;
  int spwm_wrap_elapsed_clks = -1;
  const int64_t spwm_max_align_clks =
      static_cast<int64_t>(spwm_double_rows) *
      static_cast<int64_t>(spwm_align_scan_config.row_clks) +
      static_cast<int64_t>(spwm_align_scan_config.row_clks);

  for (int64_t spwm_clk = 0; spwm_clk < spwm_max_align_clks; ++spwm_clk) {
    const int spwm_previous_row = spwm_scan_state->row;
    SPWM_OE_Gate_State *spwm_align_oe_gate = spwm_oe_gate;

    // SPWM_ROW_ADDRESS_TYPE_2_SHIFTREG_AB_BLANK_CLOCK - Exception
    if (spwm_type2_shiftreg_align && spwm_wrapped) {
      // Keep the type-2 wrap timing, but do not light the wrapped row-0
      // cleanup cycle. The ordinary last-row A/B pulse should still keep its
      // OE window; only the tail wrap pulse itself needs to stay dark.
      spwm_align_oe_gate = nullptr;
    }
    const bool spwm_advanced_row = spwm_scan_pre_clock_handler(
        io, h, spwm_row_setter, spwm_double_rows, spwm_align_scan_config,
        spwm_align_oe_gate, spwm_scan_state);

    if (spwm_advanced_row && spwm_previous_row == spwm_last_row &&
        spwm_scan_state->row == 0) {
      spwm_wrapped = true;
      spwm_wrap_elapsed_clks = 0;
    }

    spwm_clock_pulse(io, h, 0, spwm_data_mask, spwm_align_oe_gate);
    spwm_scan_post_clock(spwm_align_scan_config, spwm_scan_state);
    if (spwm_wrap_elapsed_clks >= 0) {
      ++spwm_wrap_elapsed_clks;
    }

    if (spwm_wrapped &&
        spwm_wrap_elapsed_clks >= spwm_wrap_completion_clks &&
        !(spwm_oe_gate != nullptr &&
          (spwm_oe_gate->active || spwm_oe_gate->remaining > 0))) {
      break;
    }
  }

  spwm_drain_pending_oe_gate(io, h, spwm_data_mask, spwm_oe_gate,
                             spwm_align_scan_config, spwm_scan_state);
  io->ClearBits(h.output_enable);
}

// Return the width of the one-shot OE pulse emitted immediately after the init
// register blocks.
int spwm_get_first_oe_clk_length() {
  return spwm_get_panel_settings().first_oe_clk_length;
}

// Return how many extra full row cycles to display after RGB upload finishes.
int spwm_get_end_of_frame_extra_row_cycles() {
  return spwm_get_panel_settings().end_of_frame_extra_row_cycles;
}

// Return the optional microsecond delay inserted after each frame.
int spwm_get_frame_end_sleep_us() {
  return spwm_get_panel_settings().frame_end_sleep_us;
}

// Return the row period used while RGB data is still being shifted into the
// panel.
int spwm_get_oe_during_upload_clk_count() {
  return spwm_get_panel_settings().oe_during_upload_clk_count;
}

// Return the row period used after upload, including any auto-tuned adjustment.
int spwm_get_oe_after_upload_clk_count() {
  SPWM_Auto_Tune_Control &spwm_control = spwm_get_auto_tune_control();
  return spwm_control.enabled
             ? spwm_control.current_oe_after_upload_clk_count
             : spwm_get_panel_settings().oe_after_upload_clk_count;
}

// Return how far before the row boundary the row-advance/OE schedule starts.
int spwm_get_oe_clk_look_behind() {
  return spwm_get_panel_settings().oe_clk_look_behind;
}

// Return the width of the repeating OE pulses.
int spwm_get_oe_clk_length() {
  return spwm_get_panel_settings().oe_clk_length;
}

// The SPWM framebuffer path drives the RGB data pins for every active parallel
// chain. CLK is added separately when building the full write mask used by the
// clock helper.
gpio_bits_t spwm_get_framebuffer_rgb_mask(const HardwareMapping &h) {
  const int spwm_parallel_chains = spwm_get_parallel_chains();
  const gpio_bits_t spwm_chain_masks[] = {
      h.p0_r1 | h.p0_g1 | h.p0_b1 | h.p0_r2 | h.p0_g2 | h.p0_b2,
      h.p1_r1 | h.p1_g1 | h.p1_b1 | h.p1_r2 | h.p1_g2 | h.p1_b2,
      h.p2_r1 | h.p2_g1 | h.p2_b1 | h.p2_r2 | h.p2_g2 | h.p2_b2,
      h.p3_r1 | h.p3_g1 | h.p3_b1 | h.p3_r2 | h.p3_g2 | h.p3_b2,
      h.p4_r1 | h.p4_g1 | h.p4_b1 | h.p4_r2 | h.p4_g2 | h.p4_b2,
      h.p5_r1 | h.p5_g1 | h.p5_b1 | h.p5_r2 | h.p5_g2 | h.p5_b2,
  };

  gpio_bits_t spwm_rgb_mask = 0;
  for (int spwm_chain = 0; spwm_chain < spwm_parallel_chains; ++spwm_chain) {
    spwm_rgb_mask |= spwm_chain_masks[spwm_chain];
  }
  return spwm_rgb_mask;
}

// FM6373-style OE advances the row shortly before the next burst, while
// FM6363-style OE uses the whole non-OE window as setup time. This stays tied
// to the panel profile even when row transport is overridden.
SPWM_OE_Style spwm_get_active_oe_style() {
  const SPWM_Panel_Settings &spwm_settings = spwm_get_panel_settings();
  return spwm_settings.oe_style;
}

bool spwm_oe_style_uses_row_before_oe(SPWM_OE_Style spwm_oe_style) {
  switch (spwm_oe_style) {
    case SPWM_OE_STYLE_FM6363:
      return true;
    case SPWM_OE_STYLE_FM6373:
    default:
      return false;
  }
}

bool spwm_oe_style_shares_initial_oe_with_upload(
    SPWM_OE_Style spwm_oe_style) {
  switch (spwm_oe_style) {
    case SPWM_OE_STYLE_FM6363:
      return true;
    case SPWM_OE_STYLE_FM6373:
    default:
      return false;
  }
}

bool spwm_oe_style_pulse_each_clock(SPWM_OE_Style spwm_oe_style) {
  switch (spwm_oe_style) {
    case SPWM_OE_STYLE_FM6363:
      return true;
    case SPWM_OE_STYLE_FM6373:
    default:
      return false;
  }
}

int spwm_resolve_scan_setup_clks(SPWM_OE_Style spwm_oe_style,
                                 int spwm_row_clks) {
  if (spwm_oe_style_uses_row_before_oe(spwm_oe_style)) {
    return std::max(0, spwm_row_clks - spwm_get_oe_clk_length());
  }
  return spwm_get_oe_clk_look_behind();
}

// Build the normalized scan description for either the upload phase or the
// post-upload free-run phase using the panel-tied OE schedule.
SPWM_Scan_Config spwm_make_runtime_scan_config(
    SPWM_OE_Style spwm_oe_style,
    int spwm_row_clks,
    bool spwm_skip_first_oe) {
  return spwm_make_scan_config(
      spwm_row_clks,
      spwm_resolve_scan_setup_clks(spwm_oe_style, spwm_row_clks),
      spwm_get_oe_clk_length(),
      spwm_skip_first_oe,
      spwm_oe_style_uses_row_before_oe(spwm_oe_style));
}

// Arm and optionally consume the one-shot startup OE burst that follows the
// panel init script.
//
// There are two startup modes:
// - Standalone initial OE burst: emit the startup clocks here before any
//   framebuffer data is uploaded. The later upload phase resumes as though the
//   startup burst began at the same scan phase as a normal OE burst, so the
//   first-to-second OE spacing matches the steady-state row period even when
//   the startup pulse width differs from the repeating OE pulses.
// - Shared initial OE with upload: arm the startup OE pulse here, but let the
//   first real upload clocks consume it. This keeps scan phase aligned for
//   panel profiles whose first visible OE burst is expected to overlap upload.
//
// The active panel profile selects which form to use. Row transport is
// resolved separately by the chosen row setter.
bool spwm_start_initial_oe_phase(
    GPIO *io, const HardwareMapping &h,
    gpio_bits_t spwm_rgb_mask, gpio_bits_t spwm_data_mask,
    int spwm_init_oe_clks,
    const SPWM_Scan_Config &spwm_upload_scan_config,
    bool spwm_blank_clock_row_transport,
    bool spwm_share_initial_oe_with_upload,
    SPWM_OE_Gate_State *spwm_oe_gate,
    SPWM_Scan_State *spwm_scan_state) {
  if (spwm_oe_gate == nullptr || spwm_scan_state == nullptr) return false;
  if (spwm_init_oe_clks <= 0) {
    spwm_oe_gate->section = SPWM_AUTO_TUNE_SECTION_UPLOAD;
    return false;
  }

  io->ClearBits(spwm_rgb_mask | h.strobe | h.clock);
  spwm_wait_until_initial_oe_pulse_target();
  spwm_oe_gate->capture_start_time = true;
  spwm_arm_oe_gate(spwm_oe_gate, spwm_init_oe_clks);

  if (spwm_share_initial_oe_with_upload) {
    // Keep the startup burst pending. The first upload clocks will consume it,
    // and the regular shift-register scan waveform will only begin after it
    // has drained.
    if (spwm_upload_scan_config.oe_clks > 0) {
      spwm_scan_state->oe_primed = true;
    }
    return true;
  }

  // Direct-row path: consume the startup burst immediately before any
  // framebuffer words are shifted.
  for (int spwm_clk = 0; spwm_clk < spwm_init_oe_clks; ++spwm_clk) {
    spwm_clock_pulse(io, h, 0, spwm_data_mask, spwm_oe_gate);
  }

  if (spwm_upload_scan_config.row_clks > 0 &&
      spwm_upload_scan_config.oe_clks > 0) {
    const int spwm_regular_oe_arm_phase =
        spwm_resolve_oe_arm_phase(spwm_upload_scan_config);
    (void)spwm_blank_clock_row_transport;
    // Account for the actual standalone startup clocks already emitted above.
    // Some profiles use a wider first OE burst than the repeating OE window,
    // and the scan phase must stay aligned to those real GPIO clocks.
    spwm_scan_state->phase =
        (spwm_regular_oe_arm_phase + spwm_init_oe_clks) %
        spwm_upload_scan_config.row_clks;
    spwm_scan_state->oe_primed = true;
  }

  spwm_oe_gate->section = SPWM_AUTO_TUNE_SECTION_UPLOAD;
  return false;
}

}  // namespace

// -----------------------------
// Public runtime configuration
// -----------------------------
bool spwm_request_rgb_register_profile(
    const SPWM_RGB_Register_Profile_View *spwm_profile) {
  if (!spwm_is_valid_rgb_register_profile(spwm_profile)) return false;
  spwm_last_rejected_rgb_register_profile.store(nullptr,
                                                 std::memory_order_release);
  spwm_pending_rgb_register_profile.store(spwm_profile,
                                          std::memory_order_release);
  return true;
}

const SPWM_RGB_Register_Profile_View *
spwm_get_last_emitted_rgb_register_profile() {
  return spwm_last_emitted_rgb_register_profile.load(std::memory_order_acquire);
}

const SPWM_RGB_Register_Profile_View *
spwm_get_last_rejected_rgb_register_profile() {
  return spwm_last_rejected_rgb_register_profile.load(
      std::memory_order_acquire);
}

bool spwm_request_fixed_register_profile(
    const SPWM_Fixed_Register_Profile_View *spwm_profile) {
  if (!spwm_is_valid_fixed_register_profile(spwm_profile)) return false;
  spwm_last_rejected_fixed_register_profile.store(
      nullptr, std::memory_order_release);
  spwm_pending_fixed_register_profile.store(spwm_profile,
                                            std::memory_order_release);
  return true;
}

const SPWM_Fixed_Register_Profile_View *
spwm_get_last_emitted_fixed_register_profile() {
  return spwm_last_emitted_fixed_register_profile.load(
      std::memory_order_acquire);
}

const SPWM_Fixed_Register_Profile_View *
spwm_get_last_rejected_fixed_register_profile() {
  return spwm_last_rejected_fixed_register_profile.load(
      std::memory_order_acquire);
}

// Return true when the selected row-address transport is driven by the SPWM
// blank-clock scan waveform instead of direct row output pins.
bool spwm_row_address_type_uses_blank_clock(int spwm_row_address_type) {
  return spwm_row_address_type ==
             SPWM_ROW_ADDRESS_TYPE_1_SHIFTREG_BLANK_CLOCK ||
         spwm_row_address_type ==
             SPWM_ROW_ADDRESS_TYPE_2_SHIFTREG_AB_BLANK_CLOCK;
}

// Return true when this SPWM panel/mode combination needs the expanded row
// validation range.
bool spwm_uses_extended_row_range(const char *spwm_panel_type,
                                  int spwm_row_address_type) {
  return spwm_is_panel_type(spwm_panel_type) &&
         spwm_row_address_type_uses_blank_clock(spwm_row_address_type);
}

RowAddressSetter *spwm_create_blank_clock_row_select_setter(
    const HardwareMapping &h) {
  return new SPWMBlankClockRowSelectSetter(h);
}

// Return true when the requested panel type resolves to a known SPWM profile.
bool spwm_is_panel_type(const char *spwm_panel_type) {
  return spwm_find_panel_profile(spwm_panel_type) != nullptr;
}

// Only profiles with an RGB register init step have rotating per-frame words
// that can be replaced by --led-spwm-force-register.
bool spwm_panel_supports_forced_register(const char *spwm_panel_type) {
  const SPWM_Panel_Profile *spwm_profile =
      spwm_find_panel_profile(spwm_panel_type);
  if (spwm_profile == nullptr || spwm_profile->init_sequence.steps == nullptr) {
    return false;
  }

  for (size_t spwm_step_index = 0;
       spwm_step_index < spwm_profile->init_sequence.step_count;
       ++spwm_step_index) {
    if (spwm_profile->init_sequence.steps[spwm_step_index].type ==
        SPWM_INIT_STEP_RGB_REGISTER) {
      return true;
    }
  }
  return false;
}

// Resolve a numbered option against the selected profile's init sequence.
SPWM_Register_Slot_Type spwm_get_panel_register_slot_type(
    const char *spwm_panel_type, size_t spwm_register_index) {
  const SPWM_Panel_Profile *spwm_profile =
      spwm_find_panel_profile(spwm_panel_type);
  if (spwm_register_index == 0 || spwm_profile == nullptr ||
      spwm_profile->init_sequence.steps == nullptr) {
    return SPWM_REGISTER_SLOT_NONE;
  }

  for (size_t spwm_step_index = 0;
       spwm_step_index < spwm_profile->init_sequence.step_count;
       ++spwm_step_index) {
    const SPWM_Init_Step &spwm_step =
        spwm_profile->init_sequence.steps[spwm_step_index];
    if (spwm_step.value != spwm_register_index) continue;
    if (spwm_step.type == SPWM_INIT_STEP_RGB_REGISTER) {
      return SPWM_REGISTER_SLOT_RGB;
    }
    if (spwm_step.type == SPWM_INIT_STEP_REGISTER) {
      return SPWM_REGISTER_SLOT_FIXED;
    }
  }
  return SPWM_REGISTER_SLOT_NONE;
}

int spwm_resolve_data_layout(const char *spwm_panel_type,
                             int spwm_data_layout) {
  if (spwm_data_layout != SPWM_DATA_LAYOUT_PROFILE_DEFAULT) {
    return spwm_data_layout;
  }
  const SPWM_Panel_Profile *spwm_profile =
      spwm_find_panel_profile(spwm_panel_type);
  return spwm_profile != nullptr
             ? spwm_profile->settings.default_data_layout
             : SPWM_DATA_LAYOUT_PROFILE_DEFAULT;
}

// Select the active SPWM runtime profile and report whether the panel should
// route framebuffer refresh through the SPWM path.
bool spwm_initialize_panel_type(const char *spwm_panel_type, int spwm_columns,
                                int spwm_panel_columns,
                                int spwm_row_address_type,
                                int spwm_scan_rows,
                                int spwm_data_layout,
                                int spwm_register_config,
                                int spwm_multiplexing,
                                const char *spwm_force_register,
                                const char *const *spwm_force_registers,
                                size_t spwm_force_register_count) {
  spwm_set_enabled(false);
  spwm_configure_panel_type(spwm_panel_type, spwm_columns, spwm_panel_columns,
                            spwm_row_address_type, spwm_scan_rows,
                            spwm_data_layout, spwm_register_config,
                            spwm_multiplexing, spwm_force_register,
                            spwm_force_registers,
                            spwm_force_register_count);

  if (spwm_panel_type == nullptr || *spwm_panel_type == '\0') return false;
  if (!spwm_is_panel_type(spwm_panel_type)) return false;

  spwm_set_enabled(true);
  return true;
}

// Load the chosen panel profile, record the requested row transport override,
// apply environment overrides, rebuild the runtime register layout for the
// active panel width, and then apply any forced register values.
void spwm_configure_panel_type(const char *spwm_panel_type, int spwm_columns,
                               int spwm_panel_columns,
                               int spwm_row_address_type,
                               int spwm_scan_rows,
                               int spwm_data_layout,
                               int spwm_register_config,
                               int spwm_multiplexing,
                               const char *spwm_force_register,
                               const char *const *spwm_force_registers,
                               size_t spwm_force_register_count) {
  SPWM_Runtime_State &spwm_runtime_state = spwm_get_runtime_state();
  SPWM_Auto_Tune_Control &spwm_auto_tune_control =
      spwm_get_auto_tune_control_storage();
  const SPWM_Panel_Profile *spwm_profile =
      spwm_find_panel_profile(spwm_panel_type);
  const SPWM_Panel_Profile &spwm_default_profile =
      spwm_get_default_panel_profile();

  spwm_runtime_state.panel_settings =
      spwm_profile != nullptr
          ? spwm_resolve_profile_settings(*spwm_profile, spwm_columns)
          : spwm_resolve_profile_settings(spwm_default_profile, spwm_columns);
  spwm_runtime_state.row_address_type = spwm_row_address_type;
  spwm_runtime_state.scan_rows = spwm_scan_rows;
  spwm_runtime_state.panel_columns =
      spwm_panel_columns > 0
          ? spwm_panel_columns
          : spwm_runtime_state.panel_settings.default_columns;
  spwm_runtime_state.data_layout = spwm_data_layout;
  spwm_runtime_state.multiplexing = spwm_multiplexing;

  spwm_apply_panel_env_overrides(&spwm_runtime_state.panel_settings);
  spwm_auto_tune_control.loaded = false;

  if (spwm_profile != nullptr && spwm_profile->create_config != nullptr) {
    spwm_runtime_state.config = spwm_profile->create_config(
        spwm_runtime_state.panel_settings, spwm_columns,
        spwm_row_address_type, spwm_register_config);
  } else if (spwm_default_profile.create_config != nullptr) {
    spwm_runtime_state.config = spwm_default_profile.create_config(
        spwm_runtime_state.panel_settings, spwm_columns,
        spwm_row_address_type, spwm_register_config);
  }

  // Override precedence is deliberate: selected built-in/catalog profile,
  // then the unnumbered rotating shortcut, then numbered per-slot values.
  // This lets a numbered option replace the same rotating slot last.
  std::array<std::vector<uint16_t>, 3> spwm_forced_register_channels;
  if (spwm_force_register != nullptr &&
      spwm_parse_forced_rgb_register_words(
          spwm_force_register, &spwm_forced_register_channels)) {
    spwm_runtime_state.config.spwm_force_rgb_register_words(
        spwm_forced_register_channels);
  }

  if (spwm_force_registers != nullptr) {
    for (size_t spwm_register_index = 0;
         spwm_register_index < spwm_force_register_count;
         ++spwm_register_index) {
      if (spwm_force_registers[spwm_register_index] == nullptr) continue;

      if (spwm_runtime_state.config.spwm_has_rgb_register(
              spwm_register_index + 1)) {
        std::array<std::vector<uint16_t>, 3>
            spwm_numbered_register_channels;
        if (spwm_parse_forced_rgb_register_words(
                spwm_force_registers[spwm_register_index],
                &spwm_numbered_register_channels)) {
          spwm_runtime_state.config.spwm_force_register_rgb_words(
              spwm_register_index + 1, spwm_numbered_register_channels);
        }
      } else {
        // Preserve the legacy shared-word path. Only channel-labelled fixed
        // values need the physical R/G/B lane override storage.
        std::vector<uint16_t> spwm_numbered_register_words;
        if (spwm_parse_forced_register_words(
                spwm_force_registers[spwm_register_index],
                &spwm_numbered_register_words) &&
            spwm_numbered_register_words.size() == 1) {
          spwm_runtime_state.config.spwm_force_register_words(
              spwm_register_index + 1, spwm_numbered_register_words);
          continue;
        }

        std::array<std::vector<uint16_t>, 3>
            spwm_numbered_register_channels;
        if (!spwm_parse_forced_rgb_register_words(
                spwm_force_registers[spwm_register_index],
                &spwm_numbered_register_channels) ||
            spwm_numbered_register_channels[0].size() != 1) {
          continue;
        }
        const uint16_t spwm_fixed_register_channels[3] = {
            spwm_numbered_register_channels[0][0],
            spwm_numbered_register_channels[1][0],
            spwm_numbered_register_channels[2][0],
        };
        spwm_runtime_state.config.spwm_force_fixed_register_rgb_words(
            spwm_register_index + 1, spwm_fixed_register_channels);
      }
    }
  }

  if (spwm_profile != nullptr &&
      spwm_profile->init_sequence.steps != nullptr &&
      spwm_profile->init_sequence.step_count > 0) {
    spwm_runtime_state.init_sequence = spwm_profile->init_sequence;
  } else {
    spwm_runtime_state.init_sequence = spwm_default_profile.init_sequence;
  }
}

// Return the currently active SPWM panel settings.
const SPWM_Panel_Settings &spwm_get_panel_settings() {
  return spwm_get_runtime_state().panel_settings;
}

// Return how many parallel RGB chains the active SPWM session should drive.
int spwm_get_parallel_chains() {
  const int spwm_parallel_chains =
      spwm_get_runtime_state().active_parallel_chains;
  if (spwm_parallel_chains < 1) return 1;
  if (spwm_parallel_chains > 6) return 6;
  return spwm_parallel_chains;
}

// Select how many parallel RGB chains the SPWM upload path should include.
void spwm_set_parallel_chains(int spwm_parallel_chains) {
  SPWM_Runtime_State &spwm_runtime_state = spwm_get_runtime_state();
  if (spwm_parallel_chains < 1) {
    spwm_runtime_state.active_parallel_chains = 1;
  } else if (spwm_parallel_chains > 6) {
    spwm_runtime_state.active_parallel_chains = 6;
  } else {
    spwm_runtime_state.active_parallel_chains = spwm_parallel_chains;
  }
}

// Resolve the effective upload geometry from runtime framebuffer values and
// panel defaults.
SPWM_Upload_Geometry spwm_resolve_upload_geometry(int spwm_rows,
                                                  int spwm_columns,
                                                  int spwm_double_rows) {
  const SPWM_Panel_Settings &spwm_settings = spwm_get_panel_settings();

  SPWM_Upload_Geometry spwm_geometry;
  spwm_geometry.rows =
      spwm_resolve_positive_or_fallback(spwm_rows, spwm_settings.default_rows);
  spwm_geometry.columns = spwm_resolve_positive_or_fallback(
      spwm_columns, spwm_settings.default_columns);
  spwm_geometry.double_rows =
      spwm_double_rows > 0 ? spwm_double_rows
                           : (spwm_geometry.rows > 0 ? spwm_geometry.rows / 2 : 0);
  spwm_geometry.channels_per_chip = spwm_resolve_positive_or_fallback(
      spwm_settings.upload_channels_per_chip, 16);
  spwm_geometry.word_bits = spwm_resolve_positive_or_fallback(
      spwm_settings.upload_word_bits, SPWM_WORD_BIT_COUNT);
  if (spwm_geometry.word_bits > SPWM_WORD_BIT_COUNT) {
    spwm_geometry.word_bits = SPWM_WORD_BIT_COUNT;
  }
  spwm_geometry.chips =
      spwm_settings.upload_chip_count > 0
          ? spwm_settings.upload_chip_count
          : spwm_ceil_div_positive(spwm_geometry.columns,
                                   spwm_geometry.channels_per_chip);
  spwm_geometry.clocks_per_block =
      spwm_geometry.chips * spwm_geometry.word_bits;
  return spwm_geometry;
}

// Return whether SPWM mode is currently enabled.
bool spwm_is_enabled() {
  return spwm_get_runtime_state().enabled;
}

// Enable or disable SPWM mode. Disabling requires the refresh thread to be
// stopped and starts the next matrix session with no stale diagnostic request
// or completion state.
void spwm_set_enabled(bool spwm_is_enabled_value) {
  if (!spwm_is_enabled_value) spwm_reset_register_profile_state();
  spwm_get_runtime_state().enabled = spwm_is_enabled_value;
}

// Reset both the measured init-OE timestamp and any pending target deadline.
void spwm_reset_frame_phase_lock() {
  spwm_clear_initial_oe_pulse_start_nanos();
  spwm_set_initial_oe_pulse_target_nanos(0);
}

// Start a new phase-locked frame by discarding the previous init-OE sample.
void spwm_prepare_frame_phase_lock() {
  spwm_clear_initial_oe_pulse_start_nanos();
}

// Map the captured init-OE phase back onto the outer refresh loop and arm the
// desired init-OE deadline for the next frame.
uint64_t spwm_resolve_frame_phase_lock_deadline(uint64_t spwm_frame_start_ns,
                                                uint64_t spwm_frame_period_ns) {
  const uint64_t spwm_default_deadline =
      spwm_frame_start_ns + spwm_frame_period_ns;
  if (spwm_frame_period_ns == 0) {
    spwm_set_initial_oe_pulse_target_nanos(0);
    return spwm_frame_start_ns;
  }

  const uint64_t spwm_init_oe_start_ns =
      spwm_take_initial_oe_pulse_start_nanos();
  if (spwm_init_oe_start_ns < spwm_frame_start_ns) {
    spwm_set_initial_oe_pulse_target_nanos(0);
    return spwm_default_deadline;
  }

  const uint64_t spwm_pre_init_offset_ns =
      spwm_init_oe_start_ns - spwm_frame_start_ns;
  const uint64_t spwm_next_init_target_ns =
      spwm_init_oe_start_ns + spwm_frame_period_ns;
  spwm_set_initial_oe_pulse_target_nanos(spwm_next_init_target_ns);

  return (spwm_next_init_target_ns > spwm_pre_init_offset_ns)
             ? (spwm_next_init_target_ns - spwm_pre_init_offset_ns)
             : spwm_frame_start_ns;
}

// Clear any captured timestamp for the leading OE pulse.
void spwm_clear_initial_oe_pulse_start_nanos() {
  spwm_get_runtime_state().last_initial_oe_start_ns = 0;
}

// Store the target monotonic timestamp for the next leading OE pulse.
void spwm_set_initial_oe_pulse_target_nanos(uint64_t spwm_target_ns) {
  spwm_get_runtime_state().target_initial_oe_start_ns = spwm_target_ns;
}

// Return and clear the captured timestamp for the most recent leading OE pulse.
uint64_t spwm_take_initial_oe_pulse_start_nanos() {
  SPWM_Runtime_State &spwm_runtime_state = spwm_get_runtime_state();
  const uint64_t spwm_captured_value =
      spwm_runtime_state.last_initial_oe_start_ns;
  spwm_runtime_state.last_initial_oe_start_ns = 0;
  return spwm_captured_value;
}

// Capture when the leading OE pulse actually began.
void spwm_record_initial_oe_pulse_start() {
  spwm_get_runtime_state().last_initial_oe_start_ns = spwm_get_monotonic_nanos();
}

// Block until the stored leading-OE target time arrives.
void spwm_wait_until_initial_oe_pulse_target() {
  SPWM_Runtime_State &spwm_runtime_state = spwm_get_runtime_state();
  const uint64_t spwm_deadline_ns =
      spwm_runtime_state.target_initial_oe_start_ns;
  spwm_runtime_state.target_initial_oe_start_ns = 0;
  if (spwm_deadline_ns == 0) return;
  spwm_wait_until_monotonic_nanos(spwm_deadline_ns);
}

// ---------------------
// Frame upload routine
// ---------------------
// Emit one complete SPWM frame: init sequence, leading OE burst, RGB upload,
// post-upload scanning, row-wrap alignment, and optional end-of-frame delay.
// Purpose: Emit one complete SPWM frame for supported SPWM panel profiles.
// Inputs: GPIO interface, hardware mapping, row setter, and prepared bitplanes.
// Outputs: None.
// Side effects: Drives the full init/upload/free-run frame timing on GPIO lines.
void spwm_dump_to_matrix(GPIO *io, const HardwareMapping &h,
                         RowAddressSetter *spwm_row_setter,
                         const SPWM_Framebuffer_View &spwm_framebuffer_view) {
  if (io == nullptr || spwm_row_setter == nullptr ||
      spwm_framebuffer_view.bitplane_buffer == nullptr) {
    return;
  }

  // Build the masks used throughout the frame upload and resolve the
  // logical upload geometry for the active panel. For a 128x64 panel this
  // typically means 32 logical upload rows (top and bottom halves together).
  const gpio_bits_t spwm_rgb_mask = spwm_get_framebuffer_rgb_mask(h);
  const gpio_bits_t spwm_data_mask = spwm_rgb_mask | h.clock;

  const SPWM_Upload_Geometry spwm_upload_geometry =
      spwm_resolve_upload_geometry(spwm_framebuffer_view.rows,
                                   spwm_framebuffer_view.columns,
                                   spwm_framebuffer_view.double_rows);
  const int spwm_chip_count = spwm_upload_geometry.chips;
  const int spwm_channels_per_chip =
      spwm_upload_geometry.channels_per_chip;
  const int spwm_word_bits = spwm_upload_geometry.word_bits;
  const int spwm_upload_rows =
      spwm_upload_geometry.double_rows > 0
          ? spwm_upload_geometry.double_rows
          : spwm_framebuffer_view.double_rows;

  // Prepare per-frame timing state. The same scan state is reused during
  // the initial OE pulse, RGB upload, and the post-upload display phase.
  SPWM_Auto_Tune_Frame_State spwm_auto_tune_state = {
      false, 0, SPWM_AUTO_TUNE_SECTION_NONE};
  spwm_auto_tune_begin_frame(&spwm_auto_tune_state);

  const int spwm_row_address_type = spwm_get_runtime_state().row_address_type;
  const bool spwm_blank_clock_row_transport =
      spwm_row_address_type_uses_blank_clock(spwm_row_address_type);
  const int spwm_effective_scan_rows =
      spwm_get_effective_scan_rows(spwm_blank_clock_row_transport,
                                   spwm_upload_rows);
  const int spwm_data_layout = spwm_get_active_data_layout();
  // Expand to one upload per physical row only for a selected full-height
  // layout. Paired-row panels such as an 86-row ICND1065L with scan 43 keep the
  // normal double-row upload count.
  const bool spwm_full_height_upload =
      spwm_should_use_full_height_upload(spwm_framebuffer_view,
                                         spwm_effective_scan_rows,
                                         spwm_data_layout);
  const int spwm_effective_upload_rows =
      spwm_full_height_upload ? spwm_effective_scan_rows : spwm_upload_rows;
  const SPWM_OE_Style spwm_oe_style = spwm_get_active_oe_style();
  const bool spwm_type2_shiftreg_transport =
      spwm_uses_shiftreg_ab_blank_clock(spwm_row_address_type);
  const bool spwm_pulse_oe_each_clock =
      !spwm_type2_shiftreg_transport &&
      spwm_oe_style_pulse_each_clock(spwm_oe_style);
  // Type-2 row transport expects one continuous OE burst whose last clock
  // overlaps the first Channel B row-select clock.
  SPWM_OE_Gate_State spwm_oe_gate = {0, false, &spwm_auto_tune_state,
                                     SPWM_AUTO_TUNE_SECTION_NONE,
                                     spwm_pulse_oe_each_clock,
                                     false};
  SPWM_Scan_State spwm_scan_state = {0, 0, false, false, 0, false};
  const int spwm_init_oe_clks = spwm_get_first_oe_clk_length();
  const SPWM_Scan_Config spwm_upload_scan_config =
      spwm_make_runtime_scan_config(spwm_oe_style,
                                    spwm_get_oe_during_upload_clk_count(),
                                    true);

  // Start the frame with the panel-specific init script. FM6373-style panels
  // use a simple direct-row init sequence, while FM6363 adds per-register LAT
  // postambles.
  io->ClearBits(h.output_enable);
  spwm_emit_init_sequence(io, h);

  // Both paths start from logical row 0.
  //
  // Direct path:
  // SetRowAddress() actively drives the external row pins to row 0.
  //
  // Shift-register path:
  // SetRowAddress() only clears/reserves the row pins. The real row change is
  // emitted later by the blank-clock waveform during scan timing.
  spwm_row_setter->SetRowAddress(io, spwm_scan_state.row);

  // The panel profile decides whether the startup OE burst is standalone or
  // shared with the first upload clocks. Row-select transport stays
  // overrideable independently through --led-spwm-row-addr-type.
  const bool spwm_share_initial_oe_with_upload =
      spwm_oe_style_shares_initial_oe_with_upload(spwm_oe_style);
  bool spwm_initial_oe_pending = spwm_start_initial_oe_phase(
      io, h, spwm_rgb_mask, spwm_data_mask, spwm_init_oe_clks,
      spwm_upload_scan_config, spwm_blank_clock_row_transport,
      spwm_share_initial_oe_with_upload,
      &spwm_oe_gate, &spwm_scan_state);

  // Upload the RGB data for each logical row. Each logical row contains both
  // halves at once (R1/G1/B1 for the top half and R2/G2/B2 for the bottom
  // half). The upload walks channel index first, then chip index, which
  // matches the panel driver layout across the width.
  //
  // Direct path:
  // row stepping happens through explicit SetRowAddress() calls.
  //
  // Shift-register path:
  // row stepping happens through the blank-clock waveform, so upload and scan
  // phase must stay tightly synchronized. --led-spwm-scan can extend the wrap
  // count beyond upload rows for panels such as 1/43-scan ICND1065L modules.
  // A selected full-height data layout splits the normal double-row
  // framebuffer into one upload row per physical row for 64S panels.
  if (!spwm_blank_clock_row_transport) {
    spwm_upload_framebuffer_direct(
        io, h, spwm_row_setter, spwm_framebuffer_view, spwm_rgb_mask,
        spwm_data_mask, spwm_upload_rows, spwm_data_layout,
        spwm_full_height_upload, spwm_chip_count, spwm_channels_per_chip,
        spwm_word_bits, spwm_upload_scan_config,
        &spwm_oe_gate, &spwm_scan_state, &spwm_initial_oe_pending);
  } else {
    spwm_upload_framebuffer_shiftreg(
        io, h, spwm_framebuffer_view, spwm_rgb_mask, spwm_data_mask,
        spwm_effective_upload_rows, spwm_data_layout,
        spwm_full_height_upload, spwm_effective_scan_rows, spwm_chip_count,
        spwm_channels_per_chip,
        spwm_word_bits, spwm_upload_scan_config,
        &spwm_oe_gate, &spwm_scan_state, &spwm_initial_oe_pending);
  }

  // No more RGB words are being shifted, but an OE pulse may still be in
  // flight. Drain it before switching to the free-running display phase.
  io->ClearBits(spwm_rgb_mask | h.strobe);
  spwm_drain_pending_oe_gate(io, h, spwm_data_mask, &spwm_oe_gate,
                             spwm_upload_scan_config, &spwm_scan_state);
  io->ClearBits(h.output_enable);

  const int spwm_end_of_frame_extra_row_cycles =
      spwm_get_end_of_frame_extra_row_cycles();
  const int spwm_oe_after_upload_clk_count =
      spwm_get_oe_after_upload_clk_count();
  const SPWM_Scan_Config spwm_free_scan_config =
      spwm_make_runtime_scan_config(spwm_oe_style,
                                    spwm_oe_after_upload_clk_count,
                                    false);

  // After upload, keep scanning rows and pulsing OE so the completed frame
  // stays visible for the configured hold period.
  spwm_oe_gate.section = SPWM_AUTO_TUNE_SECTION_FREE;
  if (spwm_blank_clock_row_transport) {
    spwm_free_run_scan<spwm_scan_pre_clock_shiftreg>(
        io, h, spwm_rgb_mask, spwm_data_mask, spwm_row_setter,
        spwm_effective_scan_rows, spwm_end_of_frame_extra_row_cycles,
        spwm_free_scan_config, &spwm_oe_gate, &spwm_scan_state);
  } else {
    spwm_free_run_scan<spwm_scan_pre_clock_direct>(
        io, h, spwm_rgb_mask, spwm_data_mask, spwm_row_setter,
        spwm_effective_scan_rows, spwm_end_of_frame_extra_row_cycles,
        spwm_free_scan_config, &spwm_oe_gate, &spwm_scan_state);
  }

  // Finish on a clean row-wrap boundary so the next frame starts from a
  // predictable scan position.
  SPWM_Scan_Config spwm_align_scan_config = spwm_free_scan_config;
  // Ensures TYPE_2_SHIFTREG_AB_BLANK_CLOCK - OE stays aligned with Channel A/B pulse.
  if (spwm_row_address_type !=
      SPWM_ROW_ADDRESS_TYPE_2_SHIFTREG_AB_BLANK_CLOCK) {
    // TYPE_2_SHIFTREG_AB_BLANK_CLOCK relies on the normal FM6373-style OE/B overlap to keep the final
    // wrap pulse aligned with the rest of the frame, so preserve that runtime timing instead of forcing the row advance ahead of OE here.
    spwm_align_scan_config.row_before_oe = true;
    spwm_refresh_scan_config_derived_fields(&spwm_align_scan_config);
  }
  spwm_oe_gate.section = SPWM_AUTO_TUNE_SECTION_FREE;
  if (spwm_blank_clock_row_transport) {
    spwm_align_frame_end_to_row_wrap<spwm_scan_pre_clock_shiftreg>(
        io, h, spwm_rgb_mask, spwm_data_mask, spwm_row_setter,
        spwm_effective_scan_rows, spwm_align_scan_config,
        &spwm_oe_gate, &spwm_scan_state);
  } else {
    spwm_align_frame_end_to_row_wrap<spwm_scan_pre_clock_direct>(
        io, h, spwm_rgb_mask, spwm_data_mask, spwm_row_setter,
        spwm_effective_scan_rows, spwm_align_scan_config,
        &spwm_oe_gate, &spwm_scan_state);
  }

  // Return the shared control lines to an idle state before leaving the
  // frame. This also makes the next frame start from a known baseline.
  io->ClearBits(spwm_rgb_mask | h.strobe | h.clock);
  io->WriteMaskedBits(0, h.a | h.b | h.c | h.d | h.e);

  // Feed the measured gaps back into the auto-tuner, then apply any optional
  // inter-frame sleep requested by the panel profile.
  spwm_auto_tune_end_frame(&spwm_auto_tune_state);

  const int spwm_frame_end_sleep_us = spwm_get_frame_end_sleep_us();
  if (spwm_frame_end_sleep_us > 0) usleep(spwm_frame_end_sleep_us);
}

}  // namespace internal
}  // namespace rgb_matrix
