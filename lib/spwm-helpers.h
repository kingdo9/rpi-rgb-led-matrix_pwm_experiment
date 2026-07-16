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
  SPWM_ROW_ADDRESS_TYPE_2_SHIFTREG_AB_BLANK_CLOCK = 2,
};

// Resolved framebuffer-to-panel routing for an SPWM upload. The public value
// 0 requests the panel profile default; profiles that leave it at 0 keep the
// normal paired vertical RGB1/RGB2 upload.
enum SPWM_Data_Layout {
  SPWM_DATA_LAYOUT_PROFILE_DEFAULT = 0,
  SPWM_DATA_LAYOUT_FULL_HEIGHT_LEFT_RGB2 = 1,
  SPWM_DATA_LAYOUT_FULL_HEIGHT_LEFT_RGB1 = 2,
};

// The current SPWM profiles expose at most six 1-based register slots.
constexpr size_t SPWM_FORCE_REGISTER_COUNT = 6;

enum SPWM_Register_Slot_Type {
  SPWM_REGISTER_SLOT_NONE = 0,
  SPWM_REGISTER_SLOT_FIXED,
  SPWM_REGISTER_SLOT_RGB,
};

struct SPWM_Panel_Settings {
  static const int kMaxMissingColumnSlots = 4;

  int default_rows;               // Default panel row count for this profile.
  int default_columns;            // Default panel column count for this profile.
  int default_data_layout;        // SPWM_Data_Layout selected by option 0.
  int upload_channels_per_chip;   // Driver outputs per cascaded chip.
  int upload_word_bits;           // Bits shifted per grayscale word.
  int upload_chip_count;          // 0 => derive from columns/channels_per_chip.
  int rgb_upload_lat_spacer_clk_count;  // SPWM_RGB_UPLOAD_LAT_SPACER_CLK_COUNT
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
  // These defaults are attached to the blank-clock row-select transports and
  // can still be overridden through the environment.
  int shiftreg_row_select_a_pulse_clk_count;  // SPWM_SHIFT_REG_ROW_SELECT_A_PULSE_CLK_COUNT
  bool shiftreg_row_select_a_pulse_centered;  // SPWM_SHIFT_REG_ROW_SELECT_A_PULSE_CENTERED
  int shiftreg_row_select_a_pulse_start_clk;  // SPWM_SHIFT_REG_ROW_SELECT_A_PULSE_START_CLK

  // Some panels expose fewer visible columns than the physical shift chain.
  // These positions are physical clock slots that should stay blank during RGB
  // upload and not consume a visible framebuffer column.
  int missing_column_count;
  int missing_column_positions[kMaxMissingColumnSlots];
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

// One R/G/B word triple emitted on the panel's color data lanes.
struct SPWM_RGB_Frame {
  uint16_t r;
  uint16_t g;
  uint16_t b;
};

// Non-owning view of one diagnostic RGB register profile. The view and its
// channel arrays must remain valid and unchanged for the lifetime of the
// process.
struct SPWM_RGB_Register_Profile_View {
  const char *name;
  size_t register_index;
  const uint16_t *channel_words[3];
  size_t channel_word_counts[3];
};

// Structurally validate and queue a diagnostic profile for the refresh thread
// to apply at the next init-sequence boundary. Compatibility with the active
// panel slot is checked on that thread. Only one pending request is supported;
// a later request replaces it without publishing a superseded result. Status
// is identified by profile pointer, not by a per-request generation, so
// requeueing the current last-emitted pointer is not a fresh acknowledgement.
bool spwm_request_rgb_register_profile(
    const SPWM_RGB_Register_Profile_View *profile);

// Return the most recent queued profile whose complete rotating sequence was
// emitted by completed panel init sequences, or nullptr if none has completed.
const SPWM_RGB_Register_Profile_View *
spwm_get_last_emitted_rgb_register_profile();

// Return the rejection result for the latest RGB request. A structurally valid
// new request clears this first, so nullptr means no latest rejection is known.
const SPWM_RGB_Register_Profile_View *
spwm_get_last_rejected_rgb_register_profile();

// One fixed register slot with its physical red, green, and blue lane words.
// RCFGX packages can configure these lanes differently even though the slot is
// emitted once per init sequence rather than as a rotating register block.
struct SPWM_Fixed_Register_Profile_Entry {
  size_t register_index;
  uint16_t channel_words[3];
};

// Non-owning view of one diagnostic fixed-register profile. The view and its
// entries must remain valid and unchanged for the lifetime of the process.
struct SPWM_Fixed_Register_Profile_View {
  const char *name;
  const SPWM_Fixed_Register_Profile_Entry *entries;
  size_t entry_count;
};

// Structurally validate and queue a fixed-register profile for the refresh
// thread. Every entry must also correspond to a fixed slot emitted by the
// active panel's init sequence; that compatibility is checked on the thread.
// It has the same one-pending-request and pointer-status limits as RGB profiles.
bool spwm_request_fixed_register_profile(
    const SPWM_Fixed_Register_Profile_View *profile);

// Return the most recent queued fixed profile whose selected registers were
// all emitted by a completed panel init sequence.
const SPWM_Fixed_Register_Profile_View *
spwm_get_last_emitted_fixed_register_profile();

// Return the rejection result for the latest fixed request. A structurally
// valid new request clears this first, so nullptr means no rejection is known.
const SPWM_Fixed_Register_Profile_View *
spwm_get_last_rejected_fixed_register_profile();

// Owns the per-panel register upload data and the per-frame RGB sequence state.
class SPWM_Config {
 public:
  // Allocate storage for all register slots and remember the panel defaults that
  // govern expansion and latch timing.
  SPWM_Config(size_t spwm_register_count,
              const SPWM_Register_Timing &spwm_default_timing,
              size_t spwm_register_repeat_count);

  // register_data_ points into register_words_, so copying would leave the
  // copied views pointing at the source object. Moving transfers the owning
  // vectors and their views together.
  SPWM_Config(const SPWM_Config &) = delete;
  SPWM_Config &operator=(const SPWM_Config &) = delete;
  SPWM_Config(SPWM_Config &&) = default;
  SPWM_Config &operator=(SPWM_Config &&) = default;

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

  // Replace every rotating RGB register with one shared sequence while
  // preserving each register slot's panel-specific latch timing.
  bool spwm_force_rgb_register_words(
      const std::vector<uint16_t> &spwm_words);

  // Replace every rotating RGB register with distinct, equally sized channel
  // sequences while preserving each register slot's latch timing.
  bool spwm_force_rgb_register_words(
      const std::array<std::vector<uint16_t>, 3> &spwm_channel_sequences);

  // Replace one existing register slot without changing whether the profile
  // treats it as fixed or as a rotating RGB sequence.
  bool spwm_force_register_words(
      size_t spwm_register_index,
      const std::vector<uint16_t> &spwm_words);

  // Replace one rotating RGB slot with distinct, equally sized channel
  // sequences without changing its panel-specific latch timing.
  bool spwm_force_register_rgb_words(
      size_t spwm_register_index,
      const std::array<std::vector<uint16_t>, 3> &spwm_channel_sequences);

  // Replace one existing fixed slot with distinct words for the physical
  // red, green, and blue data lanes while preserving its latch timing.
  bool spwm_force_fixed_register_rgb_words(
      size_t spwm_register_index, const uint16_t spwm_channel_words[3]);

  // Return the channel-specific override for a fixed slot.
  bool spwm_get_fixed_register_rgb_frame(
      size_t spwm_register_index, SPWM_RGB_Frame *spwm_rgb_frame) const;

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
    // The cursor advances once per RGB-register init step, allowing a long
    // register block to rotate across consecutive panel init sequences.
    bool present = false;
    std::array<std::vector<uint16_t>, 3> channel_sequences{};
    size_t sequence_index = 0;
    size_t sequence_length = 0;
    SPWM_Register_Timing timing = {nullptr, 0, 0};
  };

  struct SPWM_Fixed_RGB_Register {
    // Channel-specific diagnostic values are a sidecar to the normal fixed
    // payload, whose word count and latch timing remain authoritative.
    bool present = false;
    SPWM_RGB_Frame frame = {0, 0, 0};
  };

  SPWM_Register_Timing default_timing_;
  size_t register_repeat_count_;
  std::vector<std::vector<uint16_t> > register_words_;
  std::vector<SPWM_Register_Data> register_data_;
  std::vector<SPWM_RGB_Register> rgb_registers_;
  std::vector<SPWM_Fixed_RGB_Register> fixed_rgb_registers_;
};

// Return true when `panel_type` matches one of the built-in SPWM panel profiles.
bool spwm_is_panel_type(const char *panel_type);

// Return true when the selected panel profile has a rotating RGB register
// block that --led-spwm-force-register can replace.
bool spwm_panel_supports_forced_register(const char *panel_type);

// Return the selected profile's type for a 1-based register slot.
SPWM_Register_Slot_Type spwm_get_panel_register_slot_type(
    const char *panel_type, size_t spwm_register_index);

// Parse a non-empty comma-separated sequence of 16-bit C-style integer words.
// Whitespace, including newlines, is allowed around values and commas.
bool spwm_parse_forced_register_words(
    const char *spwm_value, std::vector<uint16_t> *spwm_words);

// Parse either one shared word list or an RGB-labelled value. Labelled values
// contain either one channel (copied to all three) or equally sized R/G/B
// lists, for example "R:0x0000;G:0x0100;B:0x0200".
bool spwm_parse_forced_rgb_register_words(
    const char *spwm_value,
    std::array<std::vector<uint16_t>, 3> *spwm_channel_sequences);

// Return true when this SPWM row-address transport advances rows through the
// blank-clock scan waveform instead of direct A-E row outputs.
bool spwm_row_address_type_uses_blank_clock(int spwm_row_address_type);

// Resolve option 0 to the selected panel profile's default data layout.
int spwm_resolve_data_layout(const char *panel_type, int spwm_data_layout);

// Return true when this SPWM option combination allows larger-than-64 row panels.
bool spwm_uses_extended_row_range(const char *panel_type,
                                  int spwm_row_address_type);

// Create the SPWM-only row setter used by blank-clock row-select transports.
// Ownership transfers to the caller.
RowAddressSetter *spwm_create_blank_clock_row_select_setter(
    const HardwareMapping &h);

// Select the active SPWM profile for `panel_type` and enable SPWM refresh when
// the panel type is handled by the SPWM path.
bool spwm_initialize_panel_type(const char *panel_type, int columns,
                                int panel_columns,
                                int spwm_row_address_type,
                                int spwm_scan_rows,
                                int spwm_data_layout,
                                int spwm_register_config,
                                int multiplexing,
                                const char *spwm_force_register,
                                const char *const *spwm_force_registers,
                                size_t spwm_force_register_count);

// Load the selected panel profile into the runtime state, apply environment
// overrides, rebuild its register layout, and apply forced register values.
void spwm_configure_panel_type(const char *panel_type, int columns,
                               int panel_columns,
                               int spwm_row_address_type,
                               int spwm_scan_rows,
                               int spwm_data_layout,
                               int spwm_register_config,
                               int multiplexing,
                               const char *spwm_force_register,
                               const char *const *spwm_force_registers,
                               size_t spwm_force_register_count);

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

// Enable or disable SPWM handling in the refresh pipeline. The refresh thread
// must be stopped before disabling because teardown also resets its diagnostic
// profile progress state.
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
  int pwm_low_bit;
};

// Emit one full SPWM frame using the already prepared framebuffer bitplanes.
void spwm_dump_to_matrix(GPIO *io, const HardwareMapping &h,
                         RowAddressSetter *row_setter,
                         const SPWM_Framebuffer_View &spwm_framebuffer_view);

}  // namespace internal
}  // namespace rgb_matrix

#endif  // RPI_RGBMATRIX_SPWM_HELPERS_H
