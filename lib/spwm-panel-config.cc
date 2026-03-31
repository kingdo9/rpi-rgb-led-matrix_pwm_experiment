// -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
#include "spwm-panel-config.h"

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

// Copy a compile-time register payload array into a vector for runtime
// storage.
template <size_t N>
std::vector<uint16_t> spwm_make_words(const uint16_t (&spwm_words)[N]) {
  return std::vector<uint16_t>(spwm_words, spwm_words + N);
}

// Wrap a compile-time init-step array in the lightweight runtime view used by
// the SPWM upload code.
template <size_t N>
SPWM_Init_Sequence spwm_make_init_sequence(
    const SPWM_Init_Step (&spwm_steps)[N]) {
  SPWM_Init_Sequence spwm_init_sequence = {spwm_steps, N};
  return spwm_init_sequence;
}

// Wrap one register timing description in the runtime struct consumed by the
// upload helpers. The first LAT width stays aligned with the tail-latch window
// that overlaps the final data clocks, while any later entries become
// post-data LAT sections.
template <size_t N>
SPWM_Register_Timing spwm_make_register_timing(
    const uint8_t (&spwm_lat_clocks)[N],
    int spwm_lat_space_clocks = 0) {
  SPWM_Register_Timing spwm_timing = {
      spwm_lat_clocks,
      N,
      spwm_lat_space_clocks,
  };
  return spwm_timing;
}

// Build the shared SPWM settings baseline. Panel profiles can then override
// only the values that differ. Default based on FM6373
SPWM_Panel_Settings spwm_make_default_panel_settings() {
  SPWM_Panel_Settings spwm_settings = {};
  spwm_settings.default_rows = 64;
  spwm_settings.default_columns = 128;
  spwm_settings.upload_channels_per_chip = 16;
  spwm_settings.upload_word_bits = 16;
  spwm_settings.upload_chip_count = 0;  // derive from columns / channels_per_chip
  spwm_settings.auto_tune_oe_gaps = true;
  spwm_settings.auto_tune_frames = 20;
  spwm_settings.auto_tune_max_step_clks = 50;
  spwm_settings.first_oe_clk_length = 12;
  spwm_settings.end_of_frame_extra_row_cycles = 20;
  spwm_settings.frame_end_sleep_us = 300;
  spwm_settings.oe_during_upload_clk_count = 112;
  spwm_settings.oe_after_upload_clk_count = 112;
  spwm_settings.oe_clk_look_behind = 16;
  spwm_settings.oe_clk_length = 4;
  spwm_settings.oe_style = SPWM_OE_STYLE_FM6373;
  return spwm_settings;
}

// Resolve how many cascaded driver chips a register block has to cover for the
// active panel width.
size_t spwm_resolve_register_repeat_count(
    const SPWM_Panel_Settings &spwm_settings, int spwm_columns) {
  if (spwm_settings.upload_chip_count > 0) {
    return static_cast<size_t>(spwm_settings.upload_chip_count);
  }

  const int spwm_resolved_columns =
      spwm_columns > 0 ? spwm_columns : spwm_settings.default_columns;
  const int spwm_channels_per_chip =
      spwm_settings.upload_channels_per_chip > 0
          ? spwm_settings.upload_channels_per_chip
          : 16;
  if (spwm_resolved_columns <= 0 || spwm_channels_per_chip <= 0) {
    return 0;
  }

  return static_cast<size_t>(
      (spwm_resolved_columns + spwm_channels_per_chip - 1) /
      spwm_channels_per_chip);
}

// -------------------------------------------------------------------------------------------------
// FM6373 profile definition.
// Keep panel-specific geometry, timing defaults, register payloads, and init sequence together here
// so adding a new SPWM panel mostly means dropping in another block like this.
// -------------------------------------------------------------------------------------------------

static const size_t SPWM_FM6373_REGISTER_COUNT = 5;
static const uint8_t SPWM_FM6373_REGISTER_SEND_LAT[][1] = {
    {5},
    {5},
    {5},
    {5},
    {5},
};
static const SPWM_Register_Timing SPWM_FM6373_REGISTER_TIMINGS[] = {
    spwm_make_register_timing(SPWM_FM6373_REGISTER_SEND_LAT[0]),
    spwm_make_register_timing(SPWM_FM6373_REGISTER_SEND_LAT[1]),
    spwm_make_register_timing(SPWM_FM6373_REGISTER_SEND_LAT[2]),
    spwm_make_register_timing(SPWM_FM6373_REGISTER_SEND_LAT[3]),
    spwm_make_register_timing(SPWM_FM6373_REGISTER_SEND_LAT[4]),
};

// Use named field assignments instead of positional aggregate initialization so
// panel-setting edits stay readable in the C++11 build as this struct evolves.
static const SPWM_Panel_Settings SPWM_FM6373_SETTINGS = []() {
  // Default panel setting based on FM6373
  SPWM_Panel_Settings spwm_settings = spwm_make_default_panel_settings();
  return spwm_settings;
}();
static const uint16_t SPWM_FM6373_REGISTER1_WORD = 0x00AA;
static const uint16_t SPWM_FM6373_REGISTER2_WORD = 0x01AA;
static const uint16_t SPWM_FM6373_REGISTER4_WORD = 0x0055;
static const uint16_t SPWM_FM6373_REGISTER5_WORD = 0x0155;

// Register block 3 carries the per-frame RGB control words. Each new
// frame advances one entry in the R/G/B sequences below.
static const uint16_t SPWM_FM6373_BLOCK3_SEQ_R[] = {
    0x0000, 0x0100, 0x021f, 0x033f, 0x0402, 0x0508, 0x0602, 0x0720,
    0x0820, 0x0900, 0x0a00, 0x0b00, 0x0c01, 0x0d01, 0x0e04, 0x0f01,
    0x10c2, 0x1121, 0x1201, 0x1300, 0x1400, 0x1500, 0x1600, 0x17f0,
    0x181f, 0x1900, 0x1a1f, 0x1b10, 0x1c2a, 0x1d0a, 0x1e42, 0x1f04,
    0x2008, 0x2101, 0x221c, 0x7000, 0x7100, 0x7200, 0x7300, 0x7400,
    0xf000, 0xf100, 0xf200, 0xf300, 0xf400, 0xf500, 0x2300,
};

static const uint16_t SPWM_FM6373_BLOCK3_SEQ_G[] = {
    0x0000, 0x0100, 0x021f, 0x033f, 0x0402, 0x0508, 0x0602, 0x0720,
    0x0820, 0x0900, 0x0a00, 0x0b00, 0x0c08, 0x0d01, 0x0e04, 0x0f01,
    0x10c2, 0x1121, 0x1201, 0x1300, 0x1400, 0x1500, 0x1600, 0x17f0,
    0x181f, 0x1950, 0x1a1f, 0x1b10, 0x1c2a, 0x1d0a, 0x1e46, 0x1f20,
    0x2008, 0x2101, 0x221c, 0x7000, 0x7100, 0x7200, 0x7300, 0x7400,
    0xf000, 0xf100, 0xf200, 0xf300, 0xf400, 0xf500, 0x2300,
};

static const uint16_t SPWM_FM6373_BLOCK3_SEQ_B[] = {
    0x0000, 0x0100, 0x021f, 0x033f, 0x0402, 0x0508, 0x0602, 0x0720,
    0x0820, 0x0900, 0x0a00, 0x0b00, 0x0c08, 0x0d01, 0x0e04, 0x0f01,
    0x10c2, 0x1121, 0x1201, 0x1300, 0x1400, 0x1500, 0x1600, 0x17f0,
    0x182f, 0x1900, 0x1a1f, 0x1b10, 0x1c2a, 0x1d0a, 0x1e48, 0x1f20,
    0x2010, 0x2101, 0x221c, 0x7000, 0x7100, 0x7200, 0x7300, 0x7400,
    0xf000, 0xf100, 0xf200, 0xf300, 0xf400, 0xf500, 0x2300,
};

// FM6373 frame start: emit LAT bursts of 3, 11, and 14 clocks, then
// stream register blocks 1-5 with block 3 coming from the rotating RGB
// register sequence above.
static const SPWM_Init_Step SPWM_FM6373_INIT_STEPS[] = {
    {SPWM_INIT_STEP_LAT_PULSES, 3, 0},
    {SPWM_INIT_STEP_LAT_PULSES, 11, 0},
    {SPWM_INIT_STEP_LAT_PULSES, 14, 0},
    {SPWM_INIT_STEP_REGISTER, 1, 0},
    {SPWM_INIT_STEP_REGISTER, 2, 0},
    {SPWM_INIT_STEP_RGB_REGISTER, 3, 0},
    {SPWM_INIT_STEP_REGISTER, 4, 0},
    {SPWM_INIT_STEP_REGISTER, 5, 0},
};

static const SPWM_Init_Sequence SPWM_FM6373_INIT_SEQUENCE =
    spwm_make_init_sequence(SPWM_FM6373_INIT_STEPS);

// Purpose: Build the FM6373 register layout for the active panel width.
// Inputs: Panel timing/settings and the resolved column count.
// Outputs: A runtime register bundle with repeated fixed words and RGB sequence data.
// Side effects: None.
SPWM_Config spwm_create_fm6373_config(const SPWM_Panel_Settings &spwm_settings,
                                      int spwm_columns) {
  SPWM_Config spwm_config(SPWM_FM6373_REGISTER_COUNT,
                          SPWM_FM6373_REGISTER_TIMINGS[0],
                          spwm_resolve_register_repeat_count(spwm_settings,
                                                             spwm_columns));

  // Register blocks 1, 2, 4, and 5 are fixed every frame.
  spwm_config.spwm_add_register(1, {SPWM_FM6373_REGISTER1_WORD},
                                &SPWM_FM6373_REGISTER_TIMINGS[0]);
  spwm_config.spwm_add_register(2, {SPWM_FM6373_REGISTER2_WORD},
                                &SPWM_FM6373_REGISTER_TIMINGS[1]);
  // Register block 3 advances through one RGB triple per frame.
  spwm_config.spwm_add_rgb_register(
      3,
      {spwm_make_words(SPWM_FM6373_BLOCK3_SEQ_R),
       spwm_make_words(SPWM_FM6373_BLOCK3_SEQ_G),
       spwm_make_words(SPWM_FM6373_BLOCK3_SEQ_B)},
      SPWM_FM6373_REGISTER_TIMINGS[2]);
  spwm_config.spwm_add_register(4, {SPWM_FM6373_REGISTER4_WORD},
                                &SPWM_FM6373_REGISTER_TIMINGS[3]);
  spwm_config.spwm_add_register(5, {SPWM_FM6373_REGISTER5_WORD},
                                &SPWM_FM6373_REGISTER_TIMINGS[4]);

  return spwm_config;
}

// -------------------------------------------------------------------------------------------------
// SM16380SH profile definition.
// This stays close to the FM6373 upload path but uses a shorter init script,
// one extra fixed register, and a different leading-OE length.
// -------------------------------------------------------------------------------------------------

static const size_t SPWM_SM16380SH_REGISTER_COUNT = 6;
static const uint8_t SPWM_SM16380SH_REGISTER_SEND_LAT[][1] = {
    {5},
    {5},
    {5},
    {5},
    {5},
    {5},
};
static const SPWM_Register_Timing SPWM_SM16380SH_REGISTER_TIMINGS[] = {
    spwm_make_register_timing(SPWM_SM16380SH_REGISTER_SEND_LAT[0]),
    spwm_make_register_timing(SPWM_SM16380SH_REGISTER_SEND_LAT[1]),
    spwm_make_register_timing(SPWM_SM16380SH_REGISTER_SEND_LAT[2]),
    spwm_make_register_timing(SPWM_SM16380SH_REGISTER_SEND_LAT[3]),
    spwm_make_register_timing(SPWM_SM16380SH_REGISTER_SEND_LAT[4]),
    spwm_make_register_timing(SPWM_SM16380SH_REGISTER_SEND_LAT[5]),
};

static const SPWM_Panel_Settings SPWM_SM16380SH_SETTINGS = []() {
  SPWM_Panel_Settings spwm_settings = spwm_make_default_panel_settings();
  spwm_settings.first_oe_clk_length = 10;  
  spwm_settings.end_of_frame_extra_row_cycles = 27;
  spwm_settings.oe_clk_look_behind = 16;
  return spwm_settings;
}();
static const uint16_t SPWM_SM16380SH_REGISTER1_WORD = 0x00AA;
static const uint16_t SPWM_SM16380SH_REGISTER2_WORD = 0x01AA;
static const uint16_t SPWM_SM16380SH_REGISTER4_WORD = 0xF003;
static const uint16_t SPWM_SM16380SH_REGISTER5_WORD = 0x0055;
static const uint16_t SPWM_SM16380SH_REGISTER6_WORD = 0x0155;

// Register block 3 carries the per-frame RGB control words captured for the
// SM16380SH panel. Each new frame advances one entry in the R/G/B sequences
// below.
static const uint16_t SPWM_SM16380SH_BLOCK3_SEQ_R[] = {
    0x0000, 0x021f, 0x0300, 0x0400, 0x0500, 0x0600, 0x0750, 0x0800, 0x0900,
    0x0a02, 0x0b0c, 0x0c08, 0x0d00, 0x0e05, 0x0f00, 0x1000, 0x1100, 0x1200,
    0x1300, 0x1414, 0x1500, 0x1630, 0x1700, 0x1801, 0x1904, 0x1a03, 0x1b14,
    0x1c12, 0x1d00, 0x1e00, 0x1f0c, 0x2000, 0x2200,
};

static const uint16_t SPWM_SM16380SH_BLOCK3_SEQ_G[] = {
    0x0000, 0x021f, 0x0300, 0x0400, 0x0500, 0x0600, 0x0751, 0x0800, 0x0900,
    0x0a02, 0x0b0c, 0x0c18, 0x0d00, 0x0e05, 0x0f00, 0x1000, 0x1100, 0x1200,
    0x1300, 0x1422, 0x1500, 0x1630, 0x1700, 0x1801, 0x1903, 0x1a01, 0x1b14,
    0x1c8f, 0x1d00, 0x1e00, 0x1f0c, 0x2000, 0x2200,
};

static const uint16_t SPWM_SM16380SH_BLOCK3_SEQ_B[] = {
    0x0000, 0x021f, 0x0300, 0x0400, 0x0500, 0x0600, 0x0753, 0x0800, 0x0900,
    0x0a02, 0x0b0c, 0x0c30, 0x0d00, 0x0e05, 0x0f00, 0x1000, 0x1100, 0x1200,
    0x1300, 0x1432, 0x1500, 0x1630, 0x1700, 0x1801, 0x1903, 0x1a01, 0x1b14,
    0x1c8f, 0x1d00, 0x1e00, 0x1f0c, 0x2000, 0x2200,
};

// SM16380SH frame start: emit LAT bursts of 3 and 14 clocks, then stream
// register blocks 1-6 with block 3 coming from the rotating RGB register
// sequence above.
static const SPWM_Init_Step SPWM_SM16380SH_INIT_STEPS[] = {
    {SPWM_INIT_STEP_LAT_PULSES, 3, 0},
    {SPWM_INIT_STEP_LAT_PULSES, 14, 0},
    {SPWM_INIT_STEP_REGISTER, 1, 0},
    {SPWM_INIT_STEP_REGISTER, 2, 0},
    {SPWM_INIT_STEP_RGB_REGISTER, 3, 0},
    {SPWM_INIT_STEP_REGISTER, 4, 0},
    {SPWM_INIT_STEP_REGISTER, 5, 0},
    {SPWM_INIT_STEP_REGISTER, 6, 0},
};

static const SPWM_Init_Sequence SPWM_SM16380SH_INIT_SEQUENCE =
    spwm_make_init_sequence(SPWM_SM16380SH_INIT_STEPS);

// Purpose: Build the SM16380SH register layout for the active panel width.
// Inputs: Panel timing/settings and the resolved column count.
// Outputs: A runtime register bundle with repeated fixed words and RGB sequence data.
// Side effects: None.
SPWM_Config spwm_create_sm16380sh_config(
    const SPWM_Panel_Settings &spwm_settings, int spwm_columns) {
  SPWM_Config spwm_config(SPWM_SM16380SH_REGISTER_COUNT,
                          SPWM_SM16380SH_REGISTER_TIMINGS[0],
                          spwm_resolve_register_repeat_count(spwm_settings,
                                                             spwm_columns));

  spwm_config.spwm_add_register(1, {SPWM_SM16380SH_REGISTER1_WORD},
                                &SPWM_SM16380SH_REGISTER_TIMINGS[0]);
  spwm_config.spwm_add_register(2, {SPWM_SM16380SH_REGISTER2_WORD},
                                &SPWM_SM16380SH_REGISTER_TIMINGS[1]);
  spwm_config.spwm_add_rgb_register(
      3,
      {spwm_make_words(SPWM_SM16380SH_BLOCK3_SEQ_R),
       spwm_make_words(SPWM_SM16380SH_BLOCK3_SEQ_G),
       spwm_make_words(SPWM_SM16380SH_BLOCK3_SEQ_B)},
      SPWM_SM16380SH_REGISTER_TIMINGS[2]);
  spwm_config.spwm_add_register(4, {SPWM_SM16380SH_REGISTER4_WORD},
                                &SPWM_SM16380SH_REGISTER_TIMINGS[3]);
  spwm_config.spwm_add_register(5, {SPWM_SM16380SH_REGISTER5_WORD},
                                &SPWM_SM16380SH_REGISTER_TIMINGS[4]);
  spwm_config.spwm_add_register(6, {SPWM_SM16380SH_REGISTER6_WORD},
                                &SPWM_SM16380SH_REGISTER_TIMINGS[5]);

  return spwm_config;
}

// -------------------------------------------------------------------------------------------------
// FM6363 profile definition.
// FM6363 uses the DP32020A-style shift-register receiver, so register uploads
// carry longer, per-register LAT timing. Its default OE profile also expects
// the FM6363/DP32020A blank-clock-aligned timing even if row select is
// overridden to direct A-E writes later.
// -------------------------------------------------------------------------------------------------

static const size_t SPWM_FM6363_REGISTER_COUNT = 5;
static const int SPWM_FM6363_REGISTER_SEND_LAT_SPACE = 7;
static const uint8_t SPWM_FM6363_REGISTER1_SEND_LAT[] = {4, 14};
static const uint8_t SPWM_FM6363_REGISTER2_SEND_LAT[] = {6, 14};
static const uint8_t SPWM_FM6363_REGISTER3_SEND_LAT[] = {8, 14};
static const uint8_t SPWM_FM6363_REGISTER4_SEND_LAT[] = {10, 14};
static const uint8_t SPWM_FM6363_REGISTER5_SEND_LAT[] = {2};
static const SPWM_Register_Timing SPWM_FM6363_REGISTER_TIMINGS[] = {
    spwm_make_register_timing(SPWM_FM6363_REGISTER1_SEND_LAT,
                              SPWM_FM6363_REGISTER_SEND_LAT_SPACE),
    spwm_make_register_timing(SPWM_FM6363_REGISTER2_SEND_LAT,
                              SPWM_FM6363_REGISTER_SEND_LAT_SPACE),
    spwm_make_register_timing(SPWM_FM6363_REGISTER3_SEND_LAT,
                              SPWM_FM6363_REGISTER_SEND_LAT_SPACE),
    spwm_make_register_timing(SPWM_FM6363_REGISTER4_SEND_LAT,
                              SPWM_FM6363_REGISTER_SEND_LAT_SPACE),
    spwm_make_register_timing(SPWM_FM6363_REGISTER5_SEND_LAT),
};

static const SPWM_Panel_Settings SPWM_FM6363_SETTINGS = []() {
  SPWM_Panel_Settings spwm_settings = spwm_make_default_panel_settings();
  spwm_settings.auto_tune_oe_gaps = false;
  spwm_settings.auto_tune_frames = 0;
  spwm_settings.auto_tune_max_step_clks = 0;
  spwm_settings.first_oe_clk_length = 78;
  spwm_settings.end_of_frame_extra_row_cycles = 3;
  spwm_settings.frame_end_sleep_us = 100;
  spwm_settings.oe_during_upload_clk_count = 500;
  spwm_settings.oe_after_upload_clk_count = 500;
  spwm_settings.oe_clk_look_behind = 0;
  spwm_settings.oe_clk_length = 74;
  spwm_settings.oe_style = SPWM_OE_STYLE_FM6363;
  return spwm_settings;
}();
static const uint16_t SPWM_FM6363_REGISTER1_WORD = 0x1fb0;
static const uint16_t SPWM_FM6363_REGISTER2_WORD = 0xf39c;
static const uint16_t SPWM_FM6363_REGISTER3_WORD = 0x20b6;
static const uint16_t SPWM_FM6363_REGISTER4_WORD = 0x1a00;
static const uint16_t SPWM_FM6363_REGISTER5_WORD = 0x7e08;

// FM6363 frame start: emit the wake-up LAT bursts, then stream the five fixed
// control registers with their per-register LAT postambles.
static const SPWM_Init_Step SPWM_FM6363_INIT_STEPS[] = {
    {SPWM_INIT_STEP_LAT_PULSES, 14, 0},
    {SPWM_INIT_STEP_LAT_PULSES, 12, 0},
    {SPWM_INIT_STEP_LAT_PULSES, 3, 0},
    {SPWM_INIT_STEP_LAT_PULSES, 14, 0},
    {SPWM_INIT_STEP_REGISTER, 1, 0},
    {SPWM_INIT_STEP_REGISTER, 2, 0},
    {SPWM_INIT_STEP_REGISTER, 3, 0},
    {SPWM_INIT_STEP_REGISTER, 4, 0},
    {SPWM_INIT_STEP_REGISTER, 5, 0},
};

static const SPWM_Init_Sequence SPWM_FM6363_INIT_SEQUENCE =
    spwm_make_init_sequence(SPWM_FM6363_INIT_STEPS);

// Purpose: Build the FM6363 register layout for the active panel width.
// Inputs: Panel timing/settings and the resolved column count.
// Outputs: A runtime register bundle with the five fixed FM6363 control words.
// Side effects: None.
SPWM_Config spwm_create_fm6363_config(const SPWM_Panel_Settings &spwm_settings,
                                      int spwm_columns) {
  SPWM_Config spwm_config(SPWM_FM6363_REGISTER_COUNT,
                          SPWM_FM6363_REGISTER_TIMINGS[0],
                          spwm_resolve_register_repeat_count(spwm_settings,
                                                             spwm_columns));

  spwm_config.spwm_add_register(1, {SPWM_FM6363_REGISTER1_WORD},
                                &SPWM_FM6363_REGISTER_TIMINGS[0]);
  spwm_config.spwm_add_register(2, {SPWM_FM6363_REGISTER2_WORD},
                                &SPWM_FM6363_REGISTER_TIMINGS[1]);
  spwm_config.spwm_add_register(3, {SPWM_FM6363_REGISTER3_WORD},
                                &SPWM_FM6363_REGISTER_TIMINGS[2]);
  spwm_config.spwm_add_register(4, {SPWM_FM6363_REGISTER4_WORD},
                                &SPWM_FM6363_REGISTER_TIMINGS[3]);
  spwm_config.spwm_add_register(5, {SPWM_FM6363_REGISTER5_WORD},
                                &SPWM_FM6363_REGISTER_TIMINGS[4]);

  return spwm_config;
}

static const SPWM_Panel_Profile SPWM_PANEL_PROFILES[] = {
    {"fm6373",
     SPWM_FM6373_SETTINGS,
     spwm_create_fm6373_config,
     SPWM_FM6373_INIT_SEQUENCE},
    {"sm16380sh",
     SPWM_SM16380SH_SETTINGS,
     spwm_create_sm16380sh_config,
     SPWM_SM16380SH_INIT_SEQUENCE},
    {"fm6363",
     SPWM_FM6363_SETTINGS,
     spwm_create_fm6363_config,
     SPWM_FM6363_INIT_SEQUENCE},
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

}  // namespace internal
}  // namespace rgb_matrix
