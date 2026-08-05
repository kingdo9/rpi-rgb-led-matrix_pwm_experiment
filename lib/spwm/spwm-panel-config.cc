// -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
#include "spwm-panel-config.h"
#include "spwm-panel-registers.h"

#include <algorithm>
#include <string.h>
#include <strings.h>

namespace rgb_matrix {
namespace internal {

namespace {

// Return true when the requested panel type begins with the expected profile
// name.
bool spwm_panel_type_matches(const char *spwm_panel_type,
                             const char *spwm_expected_panel_type) {
  if (spwm_panel_type == nullptr || spwm_expected_panel_type == nullptr) {
    return false;
  }

  return strncasecmp(spwm_panel_type, spwm_expected_panel_type,
                     strlen(spwm_expected_panel_type)) == 0;
}

// Wrap a compile-time init-step array in the lightweight runtime view used by
// the SPWM upload code.
template <size_t N>
SPWM_Init_Sequence spwm_make_init_sequence(
    const SPWM_Init_Step (&spwm_steps)[N]) {
  SPWM_Init_Sequence spwm_init_sequence = {spwm_steps, N};
  return spwm_init_sequence;
}

// Apply one sparse physical-column layout to a profile when the resolved width
// matches. This keeps odd receiver-chain wiring quirks reusable across panel
// definitions instead of open-coding the same slot list in each profile.
template <size_t N>
void spwm_apply_missing_column_layout(
    SPWM_Panel_Settings *spwm_settings, int spwm_columns,
    int spwm_expected_columns, const int (&spwm_missing_columns)[N]) {
  if (spwm_settings == nullptr || spwm_columns != spwm_expected_columns) {
    return;
  }

  const int spwm_missing_column_count = std::min(
      static_cast<int>(N), SPWM_Panel_Settings::kMaxMissingColumnSlots);
  spwm_settings->missing_column_count = spwm_missing_column_count;
  for (int spwm_column = 0; spwm_column < spwm_missing_column_count;
       ++spwm_column) {
    spwm_settings->missing_column_positions[spwm_column] =
        spwm_missing_columns[spwm_column];
  }
}

static const int SPWM_SPARSE_172_COLUMN_LAYOUT[] = {20, 52, 100, 148};

// Build the shared SPWM settings baseline. Panel profiles can then override
// only the values that differ. Default based on FM6373, including the shared
// shift-register Channel A pulse geometry used by row-address types 1 and 2.
SPWM_Panel_Settings spwm_make_default_panel_settings() {
  SPWM_Panel_Settings spwm_settings = {};
  spwm_settings.default_rows = 64;
  spwm_settings.default_columns = 128;
  spwm_settings.default_data_layout =
      SPWM_DATA_LAYOUT_FULL_HEIGHT_LEFT_RGB2;
  spwm_settings.upload_channels_per_chip = 16;
  spwm_settings.upload_word_bits = 16;
  spwm_settings.upload_chip_count = 0;  // derive from columns / channels_per_chip
  spwm_settings.rgb_upload_lat_spacer_clk_count = 0;
  spwm_settings.auto_tune_oe_gaps = true;
  spwm_settings.auto_tune_frames = 20;
  spwm_settings.auto_tune_max_step_clks = 50;
  spwm_settings.first_oe_clk_length = 12;
  spwm_settings.end_of_frame_extra_row_cycles = 1;
  spwm_settings.frame_end_sleep_us = 300;
  spwm_settings.oe_during_upload_clk_count = 112;
  spwm_settings.oe_after_upload_clk_count = 112;
  spwm_settings.oe_clk_look_behind = 16;
  spwm_settings.oe_clk_length = 4;
  spwm_settings.shiftreg_row_select_a_pulse_clk_count = 2;
  spwm_settings.shiftreg_row_select_a_pulse_start_clk = 0;
  spwm_settings.shiftreg_row_select_a_pulse_centered = true;
  spwm_settings.oe_style = SPWM_OE_STYLE_FM6373;
  spwm_settings.missing_column_count = 0;
  return spwm_settings;
}

// -------------------------------------------------------------------------------------------------
// FM6373 profile definition.
// Keep panel-specific geometry, timing defaults, and init sequence together
// here; register payloads live in spwm-panel-registers.cc.
// -------------------------------------------------------------------------------------------------

// Use named field assignments instead of positional aggregate initialization so
// panel-setting edits stay readable in the C++11 build as this struct evolves.
static const SPWM_Panel_Settings SPWM_FM6373_SETTINGS = []() {
  // Default panel setting based on FM6373
  SPWM_Panel_Settings spwm_settings = spwm_make_default_panel_settings();
  return spwm_settings;
}();

// FM6373 frame start: emit LAT3, LAT11, and LAT14 before the register blocks.
// The shared init emitter starts the leading OE burst after this final leading
// LAT, while the measured spacers preserve the remaining frame cadence.

static const SPWM_Init_Step SPWM_FM6373_INIT_STEPS[] = {
    // LAT pulses | Row lines left at 0 | Spacer CLKs.
    {SPWM_INIT_STEP_LAT_PULSES, 3, 0, 12},
    {SPWM_INIT_STEP_LAT_PULSES, 11, 0, 3},
    {SPWM_INIT_STEP_LAT_PULSES, 14, 0, 9},
    {SPWM_INIT_STEP_REGISTER, 1, 0, 0},
    {SPWM_INIT_STEP_REGISTER, 2, 0, 0},
    {SPWM_INIT_STEP_RGB_REGISTER, 3, 0, 0},
    {SPWM_INIT_STEP_REGISTER, 4, 0, 0},
    {SPWM_INIT_STEP_REGISTER, 5, 0, 0,}
};

static const SPWM_Init_Sequence SPWM_FM6373_INIT_SEQUENCE =
    spwm_make_init_sequence(SPWM_FM6373_INIT_STEPS);

// -------------------------------------------------------------------------------------------------
// ICND1065L profile definition.
// This follows the FM6373/SPWM upload structure but uses ICND1065L-specific
// timing and sparse-column settings.
// -------------------------------------------------------------------------------------------------

static const SPWM_Panel_Settings SPWM_ICND1065L_SETTINGS = []() {
  SPWM_Panel_Settings spwm_settings = spwm_make_default_panel_settings();
  spwm_settings.auto_tune_oe_gaps = false;
  spwm_settings.auto_tune_frames = 0;
  spwm_settings.auto_tune_max_step_clks = 0;
  spwm_settings.first_oe_clk_length = 12;
  spwm_settings.end_of_frame_extra_row_cycles = 1;
  spwm_settings.frame_end_sleep_us = 200;
  spwm_settings.oe_during_upload_clk_count = 52;
  spwm_settings.oe_after_upload_clk_count = 52;
  spwm_settings.oe_clk_look_behind = 0;
  spwm_settings.oe_clk_length = 4;
  spwm_settings.rgb_upload_lat_spacer_clk_count = 9;
  spwm_settings.shiftreg_row_select_a_pulse_clk_count = 2;
  spwm_settings.oe_style = SPWM_OE_STYLE_FM6373;
  return spwm_settings;
}();

// ICND1065L emits LAT3, LAT11, and LAT14 before its register blocks. The shared
// init emitter starts the leading OE burst after this final leading LAT.
static const SPWM_Init_Step SPWM_ICND1065L_INIT_STEPS[] = {
    // LAT pulses | Row lines left at 0 | Spacer CLKs.
    {SPWM_INIT_STEP_LAT_PULSES, 3, 0, 12},
    {SPWM_INIT_STEP_LAT_PULSES, 11, 0, 3},
    {SPWM_INIT_STEP_LAT_PULSES, 14, 0, 9},
    {SPWM_INIT_STEP_REGISTER, 1, 0, 0},
    {SPWM_INIT_STEP_REGISTER, 2, 0, 0},
    {SPWM_INIT_STEP_RGB_REGISTER, 3, 0, 0},
    {SPWM_INIT_STEP_REGISTER, 4, 0, 0},
    {SPWM_INIT_STEP_REGISTER, 5, 0, 0},
};

static const SPWM_Init_Sequence SPWM_ICND1065L_INIT_SEQUENCE =
    spwm_make_init_sequence(SPWM_ICND1065L_INIT_STEPS);

// Some 172-column ICND1065L receivers still shift 176 physical RGB slots per
// row. Four internal chain positions are not bonded to LEDs, so the uploader
// has to skip those physical slots instead of padding only at the tail.
SPWM_Panel_Settings spwm_resolve_icnd1065l_settings(int spwm_columns) {
  SPWM_Panel_Settings spwm_settings = SPWM_ICND1065L_SETTINGS;
  const int spwm_resolved_columns =
      spwm_columns > 0 ? spwm_columns : spwm_settings.default_columns;
  if (spwm_resolved_columns == 172) {
    // Together with its synchronized frame start, the 172x86/43-scan panel
    // needs at least 12 complete post-upload scans; 11 leaves its multiplexed
    // halves on different moving frames.
    spwm_settings.end_of_frame_extra_row_cycles = 12;
  }
  spwm_apply_missing_column_layout(&spwm_settings, spwm_resolved_columns, 172,
                                   SPWM_SPARSE_172_COLUMN_LAYOUT);
  return spwm_settings;
}

// -------------------------------------------------------------------------------------------------
// SM16380SH profile definition.
// This stays close to the FM6373 upload path but uses a shorter init script,
// one extra fixed register, and a different leading-OE length.
// -------------------------------------------------------------------------------------------------

static const SPWM_Panel_Settings SPWM_SM16380SH_SETTINGS = []() {
  SPWM_Panel_Settings spwm_settings = spwm_make_default_panel_settings();
  spwm_settings.first_oe_clk_length = 10;
  spwm_settings.oe_clk_look_behind = 28;
  spwm_settings.oe_during_upload_clk_count = 112;
  spwm_settings.oe_after_upload_clk_count = 112;
  spwm_settings.rgb_upload_lat_spacer_clk_count = 9;
  spwm_settings.shiftreg_row_select_a_pulse_clk_count = 5;
  return spwm_settings;
}();

// SM16380SH frame start: emit LAT bursts of 3 and 14 clocks, each with an
// optional trailing LAT-low spacer count, then stream register blocks 1-6 with
// block 3 coming from the selected rotating RGB register sequence.
static const SPWM_Init_Step SPWM_SM16380SH_INIT_STEPS[] = {
    // LAT pulses | Row lines left at 0 | Spacer CLKs.
    {SPWM_INIT_STEP_LAT_PULSES, 3, 0, 6},
    {SPWM_INIT_STEP_LAT_PULSES, 14, 0, 8},
    {SPWM_INIT_STEP_REGISTER, 1, 0, 8},
    {SPWM_INIT_STEP_REGISTER, 2, 0, 8},
    {SPWM_INIT_STEP_RGB_REGISTER, 3, 0, 8},
    {SPWM_INIT_STEP_REGISTER, 4, 0, 8},
    {SPWM_INIT_STEP_REGISTER, 5, 0, 8},
    {SPWM_INIT_STEP_REGISTER, 6, 0, 0},
};

static const SPWM_Init_Sequence SPWM_SM16380SH_INIT_SEQUENCE =
    spwm_make_init_sequence(SPWM_SM16380SH_INIT_STEPS);

// -------------------------------------------------------------------------------------------------
// FM6363 profile definition.
// FM6363 uses the DP32020A-style shift-register receiver, so register uploads
// carry longer, per-register LAT timing. Its default OE profile also expects
// the FM6363/DP32020A blank-clock-aligned timing even if row select is
// overridden to direct A-E writes later.
// -------------------------------------------------------------------------------------------------

static const SPWM_Panel_Settings SPWM_FM6363_SETTINGS = []() {
  SPWM_Panel_Settings spwm_settings = spwm_make_default_panel_settings();
  spwm_settings.auto_tune_oe_gaps = false;
  spwm_settings.auto_tune_frames = 0;
  spwm_settings.auto_tune_max_step_clks = 0;
  spwm_settings.first_oe_clk_length = 78;
  spwm_settings.end_of_frame_extra_row_cycles = 1;
  spwm_settings.frame_end_sleep_us = 100;
  spwm_settings.oe_during_upload_clk_count = 212;
  spwm_settings.oe_after_upload_clk_count = 212;
  spwm_settings.oe_clk_look_behind = 0;
  spwm_settings.oe_clk_length = 74;
  spwm_settings.oe_style = SPWM_OE_STYLE_FM6363;
  return spwm_settings;
}();

// FM6363 frame start: emit the wake-up LAT bursts, each with an optional
// trailing LAT-low spacer count, then stream the five fixed control registers
// with their per-register LAT postambles.
static const SPWM_Init_Step SPWM_FM6363_INIT_STEPS[] = {
    // LAT pulses | Row lines left at 0 | Spacer CLKs.
    {SPWM_INIT_STEP_LAT_PULSES, 14, 0, 0},
    {SPWM_INIT_STEP_LAT_PULSES, 12, 0, 0},
    {SPWM_INIT_STEP_LAT_PULSES, 3, 0, 0},
    {SPWM_INIT_STEP_LAT_PULSES, 14, 0, 0},
    {SPWM_INIT_STEP_REGISTER, 1, 0, 0},
    {SPWM_INIT_STEP_REGISTER, 2, 0, 0},
    {SPWM_INIT_STEP_REGISTER, 3, 0, 0},
    {SPWM_INIT_STEP_REGISTER, 4, 0, 0},
    {SPWM_INIT_STEP_REGISTER, 5, 0, 0},
};

static const SPWM_Init_Sequence SPWM_FM6363_INIT_SEQUENCE =
    spwm_make_init_sequence(SPWM_FM6363_INIT_STEPS);

// -------------------------------------------------------------------------------------------------
// FM6353 profile definition.
// FM6353 shares the FM6373-style OE schedule (DMD_STM32 derives it from
// DMD_RGB_SPWM_DRIVER, not the FM6363 base). Distinguishing features vs
// FM6373/FM6363: 138 GCLK pulses per row, 5 fixed config registers, and a
// 14-clock PRE_ACT before each register write during init.
// -------------------------------------------------------------------------------------------------

static const SPWM_Panel_Settings SPWM_FM6353_SETTINGS = []() {
  // FM6353's internal row counter advances on GCLK *edges*, not on a held
  // OE level. That means the chip needs OE pulsed once per CLK during the
  // OE window (FM6363 style: pulse_each_clock = true), not asserted high
  // across the whole burst (FM6373 style). Width of that pulse train must
  // match the GCLK_NUM that DMD_STM32 documents for this chip: 138 pulses
  // per row.
  SPWM_Panel_Settings spwm_settings = spwm_make_default_panel_settings();
  spwm_settings.auto_tune_oe_gaps = false;
  spwm_settings.auto_tune_frames = 0;
  spwm_settings.auto_tune_max_step_clks = 0;
  spwm_settings.first_oe_clk_length = 78;
  spwm_settings.end_of_frame_extra_row_cycles = 1;
  spwm_settings.frame_end_sleep_us = 100;
  spwm_settings.oe_during_upload_clk_count = 212;
  spwm_settings.oe_after_upload_clk_count = 212;
  spwm_settings.oe_clk_look_behind = 0;
  spwm_settings.oe_clk_length = 138;
  spwm_settings.oe_style = SPWM_OE_STYLE_FM6363;
  return spwm_settings;
}();

// DMD_STM32 load_config_regs() emits 14-clock pre-active, 12-clock enable, and
// 3-clock vsync LAT bursts before streaming the five fixed control registers.
// Each register write is preceded by another 14-clock LAT PRE_ACT. The
// register timing then uses 2, 4, 6, 8, or 10 tail-latch clocks to address
// physical reg2 through reg10.
static const SPWM_Init_Step SPWM_FM6353_INIT_STEPS[] = {
    {SPWM_INIT_STEP_LAT_PULSES, 14, 0, 0},  // pre-active
    {SPWM_INIT_STEP_LAT_PULSES, 12, 0, 0},  // enable all output
    {SPWM_INIT_STEP_LAT_PULSES,  3, 0, 0},  // vsync
    {SPWM_INIT_STEP_LAT_PULSES, 14, 0, 0},
    {SPWM_INIT_STEP_REGISTER,    1, 0, 0},
    {SPWM_INIT_STEP_LAT_PULSES, 14, 0, 0},
    {SPWM_INIT_STEP_REGISTER,    2, 0, 0},
    {SPWM_INIT_STEP_LAT_PULSES, 14, 0, 0},
    {SPWM_INIT_STEP_REGISTER,    3, 0, 0},
    {SPWM_INIT_STEP_LAT_PULSES, 14, 0, 0},
    {SPWM_INIT_STEP_REGISTER,    4, 0, 0},
    {SPWM_INIT_STEP_LAT_PULSES, 14, 0, 0},
    {SPWM_INIT_STEP_REGISTER,    5, 0, 0},
};

static const SPWM_Init_Sequence SPWM_FM6353_INIT_SEQUENCE =
    spwm_make_init_sequence(SPWM_FM6353_INIT_STEPS);

// This table describes panel-tied behavior only: init sequence, register
// factory, default OE timing, and panel geometry defaults. Register payloads
// live in spwm-panel-registers.cc. The runtime row transport still comes from
// --led-spwm-row-addr-type.
static const SPWM_Panel_Profile SPWM_PANEL_PROFILES[] = {
    {"fm6373",
     SPWM_FM6373_SETTINGS,
     spwm_create_fm6373_config,
     SPWM_FM6373_INIT_SEQUENCE},
    {"icnd1065l",
     SPWM_ICND1065L_SETTINGS,
     spwm_create_icnd1065l_config,
     SPWM_ICND1065L_INIT_SEQUENCE},
    {"sm16380sh",
     SPWM_SM16380SH_SETTINGS,
     spwm_create_sm16380sh_config,
     SPWM_SM16380SH_INIT_SEQUENCE},
    {"fm6363",
     SPWM_FM6363_SETTINGS,
     spwm_create_fm6363_config,
     SPWM_FM6363_INIT_SEQUENCE},
    {"fm6353",
     SPWM_FM6353_SETTINGS,
     spwm_create_fm6353_config,
     SPWM_FM6353_INIT_SEQUENCE},
};

}  // namespace

// Return the first profile in the built-in table, which is also the fallback
// when no explicit match is found.
const SPWM_Panel_Profile &spwm_get_default_panel_profile() {
  // Keep the preferred fallback profile first in SPWM_PANEL_PROFILES.
  return SPWM_PANEL_PROFILES[0];
}

// Look up a panel profile by name using the same prefix match used by the
// runtime panel-type option.
const SPWM_Panel_Profile *spwm_find_panel_profile(const char *spwm_panel_type) {
  for (const SPWM_Panel_Profile &spwm_profile : SPWM_PANEL_PROFILES) {
    if (spwm_panel_type_matches(spwm_panel_type, spwm_profile.panel_type)) {
      return &spwm_profile;
    }
  }
  return nullptr;
}

SPWM_Panel_Settings spwm_resolve_profile_settings(
    const SPWM_Panel_Profile &spwm_profile, int spwm_columns) {
  if (spwm_profile.panel_type != nullptr &&
      strcasecmp(spwm_profile.panel_type, "icnd1065l") == 0) {
    return spwm_resolve_icnd1065l_settings(spwm_columns);
  }
  return spwm_profile.settings;
}

}  // namespace internal
}  // namespace rgb_matrix
