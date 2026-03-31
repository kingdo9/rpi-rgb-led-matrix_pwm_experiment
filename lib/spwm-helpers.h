// -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
#ifndef RPI_RGBMATRIX_SPWM_HELPERS_H
#define RPI_RGBMATRIX_SPWM_HELPERS_H

#include <array>
#include <cstddef>
#include <stdint.h>
#include <vector>

#include "gpio.h"
#include "hardware-mapping.h"

namespace rgb_matrix {
namespace internal {

class RowAddressSetter;

// Derived geometry for the active panel upload path: logical rows, chips across,
// and clocks per grayscale block.
struct SPWM_Upload_Geometry {
  int rows;
  int columns;
  int double_rows;
  int chips;
  int channels_per_chip;
  int word_bits;
  int clocks_per_block;
};

// Panel-tied OE timing style.
enum SPWM_OE_Style {
  SPWM_OE_STYLE_FM6373 = 0,
  SPWM_OE_STYLE_FM6363 = 1,
};

// SPWM-only row-address transport selected by --led-spwm-row-addr-type.
enum SPWM_Row_Address_Type {
  SPWM_ROW_ADDRESS_TYPE_0_DIRECT_AE = 0,
  SPWM_ROW_ADDRESS_TYPE_1_SHIFTREG_BLANK_CLOCK = 1,
};

struct SPWM_Panel_Settings {
  int default_rows;               // Default panel row count for this profile.
  int default_columns;            // Default panel column count for this profile.
  int upload_channels_per_chip;   // Driver outputs per cascaded chip.
  int upload_word_bits;           // Bits shifted per grayscale word.
  int upload_chip_count;          // 0 => derive from columns/channels_per_chip.
  int end_of_frame_extra_row_cycles;  // SPWM_END_OF_FRAME_EXTRA_ROW_CYCLES
  int frame_end_sleep_us;         // SPWM_FRAME_END_SLEEP_US
  int first_oe_clk_length;        // SPWM_FIRST_OE_CLK_LENGTH
  int oe_clk_length;              // SPWM_OE_CLK_LENGTH
  int oe_clk_look_behind;         // SPWM_OE_CLK_LOOK_BEHIND
  SPWM_OE_Style oe_style;         // Panel-tied OE timing profile.
  
  bool auto_tune_oe_gaps;         // SPWM_AUTO_TUNE_OE_GAPS
  int auto_tune_frames;           // SPWM_AUTO_TUNE_FRAMES
  int auto_tune_max_step_clks;    // SPWM_AUTO_TUNE_MAX_STEP_CLKS
  
  
  
  int oe_during_upload_clk_count; // SPWM_OE_DURING_UPLOAD_CLK_COUNT
  int oe_after_upload_clk_count;  // SPWM_OE_AFTER_UPLOAD_CLK_COUNT
  
  
  // Shift-register blank-clock row-select Channel A pulse controls.
  // These defaults are attached to --led-spwm-row-addr-type=1 and can still
  // be overridden through the environment.
  int shiftreg_row_select_a_pulse_clk_count;  // SPWM_SHIFT_REG_ROW_SELECT_A_PULSE_CLK_COUNT
  bool shiftreg_row_select_a_pulse_centered;  // SPWM_SHIFT_REG_ROW_SELECT_A_PULSE_CENTERED
  int shiftreg_row_select_a_pulse_start_clk;  // SPWM_SHIFT_REG_ROW_SELECT_A_PULSE_START_CLK
  
};

// Register timing expressed as one or more LAT-high sections, optionally
// separated by a fixed LAT-low spacer. The first LAT section is also the tail
// latch window that overlaps the shifted data clocks.
struct SPWM_Register_Timing {
  const uint8_t *lat_clocks;
  size_t lat_count;
  int lat_space_clocks;
};

// Static register payload plus the timing used to latch it into the panel.
struct SPWM_Register_Data {
  const uint16_t *words;
  size_t word_count;
  SPWM_Register_Timing timing;
};

// One R/G/B word triple emitted by a panel's rotating RGB register block.
struct SPWM_RGB_Frame {
  uint16_t r;
  uint16_t g;
  uint16_t b;
};

// Owns the per-panel register upload data and the per-frame RGB sequence state.
class SPWM_Config {
 public:
  // Allocate storage for all register slots and remember the panel defaults that
  // govern expansion and latch timing.
  SPWM_Config(size_t spwm_register_count,
              const SPWM_Register_Timing &spwm_default_timing,
              size_t spwm_register_repeat_count);

  // Return the static payload for a 1-based register slot, or nullptr if the
  // slot is invalid.
  const SPWM_Register_Data *spwm_get_register_data(
      size_t spwm_register_index) const;
  // Store a fixed register payload. Short payloads are expanded to the panel
  // repeat count when needed.
  void spwm_add_register(size_t spwm_register_index,
                         const std::vector<uint16_t> &spwm_words,
                         const SPWM_Register_Timing *spwm_timing = nullptr);
  // Store the rotating RGB register block used by FM6373-style panels.
  void spwm_add_rgb_register(
      size_t spwm_register_index,
      const std::array<std::vector<uint16_t>, 3> &spwm_channel_sequences,
      const SPWM_Register_Timing &spwm_timing);

  // Return true when the given register slot is backed by a rotating RGB
  // sequence instead of a fixed payload.
  bool spwm_has_rgb_register(size_t spwm_register_index) const;

  // Return the configured register timing for the slot, falling back to the
  // panel default when the slot is empty or invalid.
  const SPWM_Register_Timing &spwm_get_register_timing(
      size_t spwm_register_index) const;

  // Expose how many 16-bit words each register block should cover across the
  // chained driver chips.
  size_t spwm_register_repeat_count() const { return register_repeat_count_; }

  // Return the next R/G/B register triple for the slot and wrap to the start
  // when the sequence ends.
  SPWM_RGB_Frame spwm_next_rgb_frame(size_t spwm_register_index);

 private:
  // Normalize a fixed register payload to the configured repeat count.
  std::vector<uint16_t> spwm_expand_words_for_register(
      const std::vector<uint16_t> &spwm_words) const;

  struct SPWM_RGB_Register {
    bool present = false;
    std::array<std::vector<uint16_t>, 3> channel_sequences{};
    size_t sequence_index = 0;
    size_t sequence_length = 0;
    SPWM_Register_Timing timing = {nullptr, 0, 0};
  };

  SPWM_Register_Timing default_timing_;
  size_t register_repeat_count_;
  std::vector<std::vector<uint16_t> > register_words_;
  std::vector<SPWM_Register_Data> register_data_;
  std::vector<SPWM_RGB_Register> rgb_registers_;
};

// Return true when `panel_type` matches one of the built-in SPWM panel profiles.
bool spwm_is_panel_type(const char *panel_type);

// Select the active SPWM profile for `panel_type` and enable SPWM refresh when
// the panel type is handled by the SPWM path.
bool spwm_initialize_panel_type(const char *panel_type, int columns,
                                int spwm_row_address_type);

// Load the selected panel profile into the runtime state, apply row-select
// transport defaults, rebuild any width-dependent register layout, and then
// apply environment overrides.
void spwm_configure_panel_type(const char *panel_type, int columns,
                               int spwm_row_address_type);

// Return the currently active panel settings after profile selection and
// override handling.
const SPWM_Panel_Settings &spwm_get_panel_settings();

// Track how many parallel RGB output groups the active SPWM session should
// drive.
int spwm_get_parallel_chains();
void spwm_set_parallel_chains(int spwm_parallel_chains);

// Resolve upload dimensions and derived driver layout from runtime framebuffer
// values plus the active panel defaults.
SPWM_Upload_Geometry spwm_resolve_upload_geometry(int rows, int columns,
                                                  int double_rows);

// Return whether the framebuffer refresh path is currently using SPWM handling.
bool spwm_is_enabled();

// Enable or disable SPWM handling in the refresh pipeline.
void spwm_set_enabled(bool spwm_enabled);

// Reset all captured phase-lock timestamps before the refresh loop starts.
void spwm_reset_frame_phase_lock();

// Clear the captured init-OE timestamp before starting a phase-locked frame.
void spwm_prepare_frame_phase_lock();

// Resolve the next outer-loop wake-up deadline from the captured init-OE phase
// and arm the target time for the following frame.
uint64_t spwm_resolve_frame_phase_lock_deadline(uint64_t frame_start_ns,
                                                uint64_t frame_period_ns);

// Phase-lock helpers shared with the refresh thread.
// Clear any previously captured timestamp for the one-shot leading OE pulse.
void spwm_clear_initial_oe_pulse_start_nanos();

// Request that the next leading OE pulse start at the supplied monotonic-clock
// deadline.
void spwm_set_initial_oe_pulse_target_nanos(uint64_t spwm_target_ns);

// Read and clear the captured timestamp for the most recent leading OE pulse.
uint64_t spwm_take_initial_oe_pulse_start_nanos();

// Capture the actual start time of the leading OE pulse for phase locking.
void spwm_record_initial_oe_pulse_start();

// Block until the requested leading-OE start time is reached.
void spwm_wait_until_initial_oe_pulse_target();

// Read-only view of the already prepared framebuffer bitplanes passed into the
// SPWM upload routine.
struct SPWM_Framebuffer_View {
  const gpio_bits_t *bitplane_buffer;
  int rows;
  int columns;
  int double_rows;
  int pwm_bits;
  int stored_bitplanes;
};

// Emit one full SPWM frame using the already prepared framebuffer bitplanes.
void spwm_dump_to_matrix(GPIO *io, const HardwareMapping &h,
                         RowAddressSetter *row_setter,
                         const SPWM_Framebuffer_View &spwm_framebuffer_view);

}  // namespace internal
}  // namespace rgb_matrix

#endif  // RPI_RGBMATRIX_SPWM_HELPERS_H
