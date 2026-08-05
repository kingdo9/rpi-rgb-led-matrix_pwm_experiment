// -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
#include "spwm-panel-registers.h"
#include "registertest/spwm-register-profile-loader.h"

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <array>
#include <vector>

namespace rgb_matrix {
namespace internal {

namespace {

// Lightweight compile-time views let panel definitions keep literal words in
// static arrays. The factory helpers below copy those views into SPWM_Config's
// owning vectors before the refresh thread starts.
struct SPWM_Word_Sequence {
  const uint16_t *words;
  size_t word_count;
};

struct SPWM_RGB_Word_Sequences {
  SPWM_Word_Sequence r;
  SPWM_Word_Sequence g;
  SPWM_Word_Sequence b;
};

// A register config is an ordered list of fixed or RGB register payloads.
// Keeping the register index on each entry lets future panels put the RGB
// payload on block 2, 3, 4, or mix fixed/RGB blocks differently per variant.
enum SPWM_Register_Config_Entry_Type {
  SPWM_REGISTER_CONFIG_ENTRY_FIXED = 0,
  SPWM_REGISTER_CONFIG_ENTRY_RGB,
};

struct SPWM_Register_Config_Entry {
  size_t register_index;
  SPWM_Register_Config_Entry_Type type;
  uint16_t fixed_word;
  SPWM_RGB_Word_Sequences rgb_words;
};

struct SPWM_Register_Config_Entries {
  const SPWM_Register_Config_Entry *entries;
  size_t entry_count;
};

// Wrap a compile-time word array without copying it.
template <size_t N>
SPWM_Word_Sequence spwm_make_word_sequence(
    const uint16_t (&spwm_words)[N]) {
  const SPWM_Word_Sequence spwm_sequence = {spwm_words, N};
  return spwm_sequence;
}

// Copy a static sequence view into storage owned by the runtime config.
std::vector<uint16_t> spwm_make_words(
    const SPWM_Word_Sequence &spwm_sequence) {
  return std::vector<uint16_t>(
      spwm_sequence.words, spwm_sequence.words + spwm_sequence.word_count);
}

// Wrap a compile-time entry array without duplicating the entry objects.
template <size_t N>
SPWM_Register_Config_Entries spwm_make_register_config_entries(
    const SPWM_Register_Config_Entry (&spwm_entries)[N]) {
  const SPWM_Register_Config_Entries spwm_entry_sequence = {spwm_entries, N};
  return spwm_entry_sequence;
}

// Describe one built-in fixed slot; one word is broadcast to all RGB lanes.
SPWM_Register_Config_Entry spwm_make_fixed_register_config_entry(
    size_t spwm_register_index, uint16_t spwm_word) {
  const SPWM_RGB_Word_Sequences spwm_no_rgb_words = {
      {nullptr, 0}, {nullptr, 0}, {nullptr, 0}};
  const SPWM_Register_Config_Entry spwm_entry = {
      spwm_register_index,
      SPWM_REGISTER_CONFIG_ENTRY_FIXED,
      spwm_word,
      spwm_no_rgb_words,
  };
  return spwm_entry;
}

// Describe one built-in rotating slot with independent physical R/G/B words.
SPWM_Register_Config_Entry spwm_make_rgb_register_config_entry(
    size_t spwm_register_index,
    const SPWM_RGB_Word_Sequences &spwm_rgb_words) {
  const SPWM_Register_Config_Entry spwm_entry = {
      spwm_register_index,
      SPWM_REGISTER_CONFIG_ENTRY_RGB,
      0,
      spwm_rgb_words,
  };
  return spwm_entry;
}

// Wrap a compile-time LAT timing array in the non-owning timing view.
template <size_t N>
SPWM_Register_Timing spwm_make_register_timing(
    const uint8_t (&spwm_lat_clocks)[N],
    int spwm_lat_space_clocks = 0) {
  const SPWM_Register_Timing spwm_timing = {
      spwm_lat_clocks,
      N,
      spwm_lat_space_clocks,
  };
  return spwm_timing;
}

// Derive how many cascaded driver chips each register word must cover.
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

  // One register word is shifted into every cascaded driver chip. Round up for
  // profiles whose visible width does not fill the last physical chip.
  return static_cast<size_t>(
      (spwm_resolved_columns + spwm_channels_per_chip - 1) /
      spwm_channels_per_chip);
}

// Copy a static RGB entry into the owning runtime config with its slot timing.
void spwm_add_rgb_register_config_entry(
    SPWM_Config *spwm_config, size_t spwm_register_index,
    const SPWM_RGB_Word_Sequences &spwm_sequences,
    const SPWM_Register_Timing &spwm_timing) {
  spwm_config->spwm_add_rgb_register(
      spwm_register_index,
      {spwm_make_words(spwm_sequences.r),
       spwm_make_words(spwm_sequences.g),
       spwm_make_words(spwm_sequences.b)},
      spwm_timing);
}

// Materialize the built-in main config with per-slot latch timing. Catalog
// profiles are overlaid only after this complete baseline has been built.
template <size_t RegisterCount>
SPWM_Config spwm_create_register_config(
    const SPWM_Panel_Settings &spwm_settings, int spwm_columns,
    const SPWM_Register_Timing (&spwm_register_timings)[RegisterCount],
    const SPWM_Register_Config_Entries &spwm_entries) {
  SPWM_Config spwm_config(RegisterCount, spwm_register_timings[0],
                          spwm_resolve_register_repeat_count(spwm_settings,
                                                             spwm_columns));
  for (size_t spwm_index = 0; spwm_index < spwm_entries.entry_count;
       ++spwm_index) {
    const SPWM_Register_Config_Entry &spwm_entry =
        spwm_entries.entries[spwm_index];
    if (spwm_entry.register_index == 0 ||
        spwm_entry.register_index > RegisterCount) {
      continue;
    }

    const SPWM_Register_Timing &spwm_timing =
        spwm_register_timings[spwm_entry.register_index - 1];
    if (spwm_entry.type == SPWM_REGISTER_CONFIG_ENTRY_RGB) {
      spwm_add_rgb_register_config_entry(&spwm_config, spwm_entry.register_index,
                                         spwm_entry.rgb_words, spwm_timing);
    } else {
      spwm_config.spwm_add_register(spwm_entry.register_index,
                                    {spwm_entry.fixed_word}, &spwm_timing);
    }
  }
  return spwm_config;
}

// Overlay one catalog rotating profile during initial configuration. This
// is synchronous and distinct from Demo 15's refresh-thread request hand-off.
bool spwm_apply_rgb_register_profile(
    SPWM_Config *spwm_config,
    const SPWM_RGB_Register_Profile_View &spwm_profile) {
  if (spwm_config == nullptr || spwm_profile.register_index == 0) return false;

  const size_t spwm_word_count = spwm_profile.channel_word_counts[0];
  if (spwm_word_count == 0) return false;

  std::array<std::vector<uint16_t>, 3> spwm_channel_sequences;
  for (size_t spwm_channel = 0; spwm_channel < 3; ++spwm_channel) {
    if (spwm_profile.channel_words[spwm_channel] == nullptr ||
        spwm_profile.channel_word_counts[spwm_channel] != spwm_word_count) {
      return false;
    }
    spwm_channel_sequences[spwm_channel].assign(
        spwm_profile.channel_words[spwm_channel],
        spwm_profile.channel_words[spwm_channel] + spwm_word_count);
  }

  return spwm_config->spwm_force_register_rgb_words(
      spwm_profile.register_index, spwm_channel_sequences);
}

// Overlay every fixed slot from one catalog profile as an atomic logical
// unit: validate the complete profile before changing the config.
bool spwm_apply_fixed_register_profile(
    SPWM_Config *spwm_config,
    const SPWM_Fixed_Register_Profile_View &spwm_profile) {
  if (spwm_config == nullptr || spwm_profile.entries == nullptr ||
      spwm_profile.entry_count == 0) {
    return false;
  }

  // Validate every slot before changing any of them so a catalog profile is
  // applied to the main config as one complete unit.
  for (size_t spwm_entry_index = 0;
       spwm_entry_index < spwm_profile.entry_count;
       ++spwm_entry_index) {
    const size_t spwm_register_index =
        spwm_profile.entries[spwm_entry_index].register_index;
    const SPWM_Register_Data *spwm_register_data =
        spwm_config->spwm_get_register_data(spwm_register_index);
    if (spwm_register_data == nullptr || spwm_register_data->words == nullptr ||
        spwm_register_data->word_count == 0 ||
        spwm_config->spwm_has_rgb_register(spwm_register_index)) {
      return false;
    }
  }

  for (size_t spwm_entry_index = 0;
       spwm_entry_index < spwm_profile.entry_count;
       ++spwm_entry_index) {
    const SPWM_Fixed_Register_Profile_Entry &spwm_entry =
        spwm_profile.entries[spwm_entry_index];
    if (!spwm_config->spwm_force_fixed_register_rgb_words(
            spwm_entry.register_index, spwm_entry.channel_words)) {
      return false;
    }
  }
  return true;
}

// -------------------------------------------------------------------------------------------------
// FM6373 register definition.
// -------------------------------------------------------------------------------------------------

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
// Keep these built-in main arrays synchronized with catalog regtype1 so
// --led-spwm-register-config values -1, 0, and 1 remain equivalent.
static const uint16_t SPWM_FM6373_REGISTER_BLOCK3_SEQ_R[] = {
    0x0000, 0x0100, 0x021f, 0x033f, 0x0402, 0x0508, 0x0602, 0x0720,
    0x0820, 0x0900, 0x0a00, 0x0b00, 0x0c01, 0x0d01, 0x0e04, 0x0f01,
    0x10c2, 0x1121, 0x1201, 0x1300, 0x1400, 0x1500, 0x1600, 0x17f0,
    0x181f, 0x1900, 0x1a1f, 0x1b10, 0x1c2a, 0x1d0a, 0x1e42, 0x1f04,
    0x2008, 0x2101, 0x221c, 0x7000, 0x7100, 0x7200, 0x7300, 0x7400,
    0xf000, 0xf100, 0xf200, 0xf300, 0xf400, 0xf500, 0x2300,
};

static const uint16_t SPWM_FM6373_REGISTER_BLOCK3_SEQ_G[] = {
    0x0000, 0x0100, 0x021f, 0x033f, 0x0402, 0x0508, 0x0602, 0x0720,
    0x0820, 0x0900, 0x0a00, 0x0b00, 0x0c08, 0x0d01, 0x0e04, 0x0f01,
    0x10c2, 0x1121, 0x1201, 0x1300, 0x1400, 0x1500, 0x1600, 0x17f0,
    0x181f, 0x1950, 0x1a1f, 0x1b10, 0x1c2a, 0x1d0a, 0x1e46, 0x1f20,
    0x2008, 0x2101, 0x221c, 0x7000, 0x7100, 0x7200, 0x7300, 0x7400,
    0xf000, 0xf100, 0xf200, 0xf300, 0xf400, 0xf500, 0x2300,
};

static const uint16_t SPWM_FM6373_REGISTER_BLOCK3_SEQ_B[] = {
    0x0000, 0x0100, 0x021f, 0x033f, 0x0402, 0x0508, 0x0602, 0x0720,
    0x0820, 0x0900, 0x0a00, 0x0b00, 0x0c08, 0x0d01, 0x0e04, 0x0f01,
    0x10c2, 0x1121, 0x1201, 0x1300, 0x1400, 0x1500, 0x1600, 0x17f0,
    0x182f, 0x1900, 0x1a1f, 0x1b10, 0x1c2a, 0x1d0a, 0x1e48, 0x1f20,
    0x2010, 0x2101, 0x221c, 0x7000, 0x7100, 0x7200, 0x7300, 0x7400,
    0xf000, 0xf100, 0xf200, 0xf300, 0xf400, 0xf500, 0x2300,
};

static const SPWM_RGB_Word_Sequences
    SPWM_FM6373_REGISTER_BLOCK3_SEQUENCES = {
    spwm_make_word_sequence(SPWM_FM6373_REGISTER_BLOCK3_SEQ_R),
    spwm_make_word_sequence(SPWM_FM6373_REGISTER_BLOCK3_SEQ_G),
    spwm_make_word_sequence(SPWM_FM6373_REGISTER_BLOCK3_SEQ_B),
};

static const SPWM_Register_Config_Entry
    SPWM_FM6373_REGISTER_ENTRIES[] = {
    spwm_make_fixed_register_config_entry(1, 0x00AA),
    spwm_make_fixed_register_config_entry(2, 0x01AA),
    spwm_make_rgb_register_config_entry(
        3, SPWM_FM6373_REGISTER_BLOCK3_SEQUENCES),
    spwm_make_fixed_register_config_entry(4, 0x0055),
    spwm_make_fixed_register_config_entry(5, 0x0155),
};

// -------------------------------------------------------------------------------------------------
// ICND1065L register definition.
// -------------------------------------------------------------------------------------------------

static const uint8_t SPWM_ICND1065L_REGISTER_SEND_LAT[][1] = {
    {5},
    {5},
    {5},
    {5},
    {5},
};
static const SPWM_Register_Timing SPWM_ICND1065L_REGISTER_TIMINGS[] = {
    spwm_make_register_timing(SPWM_ICND1065L_REGISTER_SEND_LAT[0]),
    spwm_make_register_timing(SPWM_ICND1065L_REGISTER_SEND_LAT[1]),
    spwm_make_register_timing(SPWM_ICND1065L_REGISTER_SEND_LAT[2]),
    spwm_make_register_timing(SPWM_ICND1065L_REGISTER_SEND_LAT[3]),
    spwm_make_register_timing(SPWM_ICND1065L_REGISTER_SEND_LAT[4]),
};
// Keep these built-in main arrays synchronized with catalog regtype1, which is
// the diagnostic copy of this built-in main block.
static const uint16_t SPWM_ICND1065L_REGISTER_BLOCK3_SEQ_R[] = {
    0x0000, 0x026a, 0x0322, 0x0412, 0x0500, 0x0601, 0x0712, 0x0c10,
    0x0d02, 0x0e84, 0x0f01, 0x1040, 0x1127, 0x1800, 0x1926, 0x1c60,
    0x1d02, 0x1e71, 0x2040, 0x2101, 0x2380, 0x74a0,
};

static const uint16_t SPWM_ICND1065L_REGISTER_BLOCK3_SEQ_G[] = {
    0x0000, 0x026a, 0x0322, 0x0412, 0x0500, 0x0601, 0x0712, 0x0c10,
    0x0d04, 0x0e84, 0x0f01, 0x1040, 0x1127, 0x1800, 0x1908, 0x1c60,
    0x1d02, 0x1e92, 0x2060, 0x2101, 0x2305, 0x74a0,
};

static const uint16_t SPWM_ICND1065L_REGISTER_BLOCK3_SEQ_B[] = {
    0x0000, 0x026a, 0x0322, 0x0412, 0x0500, 0x0601, 0x0712, 0x0c10,
    0x0d03, 0x0e84, 0x0f11, 0x1040, 0x1127, 0x1800, 0x190a, 0x1c60,
    0x1d02, 0x1eb5, 0x2060, 0x2101, 0x2300, 0x74a0,
};

static const SPWM_RGB_Word_Sequences
    SPWM_ICND1065L_REGISTER_BLOCK3_SEQUENCES = {
    spwm_make_word_sequence(SPWM_ICND1065L_REGISTER_BLOCK3_SEQ_R),
    spwm_make_word_sequence(SPWM_ICND1065L_REGISTER_BLOCK3_SEQ_G),
    spwm_make_word_sequence(SPWM_ICND1065L_REGISTER_BLOCK3_SEQ_B),
};

static const SPWM_Register_Config_Entry
    SPWM_ICND1065L_REGISTER_ENTRIES[] = {
    spwm_make_fixed_register_config_entry(1, 0x00AA),
    spwm_make_fixed_register_config_entry(2, 0x01AA),
    spwm_make_rgb_register_config_entry(
        3, SPWM_ICND1065L_REGISTER_BLOCK3_SEQUENCES),
    spwm_make_fixed_register_config_entry(4, 0x0055),
    spwm_make_fixed_register_config_entry(5, 0x0155),
};

// -------------------------------------------------------------------------------------------------
// SM16380SH register definition.
// -------------------------------------------------------------------------------------------------

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
// Keep these built-in main arrays synchronized with catalog regtype1, which is
// the diagnostic copy of this built-in main block.
static const uint16_t SPWM_SM16380SH_REGISTER_BLOCK3_SEQ_R[] = {
    0x021f, 0x0300, 0x0400, 0x0500, 0x0600, 0x078c, 0x0800, 0x0900,
    0x0a02, 0x0b0c, 0x0c08, 0x0d00, 0x0e05, 0x0f00, 0x1000, 0x1100,
    0x1200, 0x1308, 0x1414, 0x1500, 0x1630, 0x1700, 0x1801, 0x1904,
    0x1a03, 0x1b14, 0x1c12, 0x1d00, 0x1e00, 0x1f0c, 0x2000, 0x2200,
};

static const uint16_t SPWM_SM16380SH_REGISTER_BLOCK3_SEQ_G[] = {
    0x021f, 0x0300, 0x0400, 0x0500, 0x0600, 0x078c, 0x0800, 0x0900,
    0x0a02, 0x0b0c, 0x0c18, 0x0d00, 0x0e05, 0x0f00, 0x1000, 0x1100,
    0x1200, 0x1308, 0x1422, 0x1500, 0x1630, 0x1700, 0x1801, 0x1903,
    0x1a01, 0x1b14, 0x1c8f, 0x1d00, 0x1e00, 0x1f0c, 0x2000, 0x2200,
};

static const uint16_t SPWM_SM16380SH_REGISTER_BLOCK3_SEQ_B[] = {
    0x021f, 0x0300, 0x0400, 0x0500, 0x0600, 0x078c, 0x0800, 0x0900,
    0x0a02, 0x0b0c, 0x0c30, 0x0d00, 0x0e05, 0x0f00, 0x1000, 0x1100,
    0x1200, 0x1308, 0x1432, 0x1500, 0x1630, 0x1700, 0x1801, 0x1903,
    0x1a01, 0x1b14, 0x1c8f, 0x1d00, 0x1e00, 0x1f0c, 0x2000, 0x2200,
};

static const SPWM_RGB_Word_Sequences
    SPWM_SM16380SH_REGISTER_BLOCK3_SEQUENCES = {
    spwm_make_word_sequence(SPWM_SM16380SH_REGISTER_BLOCK3_SEQ_R),
    spwm_make_word_sequence(SPWM_SM16380SH_REGISTER_BLOCK3_SEQ_G),
    spwm_make_word_sequence(SPWM_SM16380SH_REGISTER_BLOCK3_SEQ_B),
};

static const SPWM_Register_Config_Entry
    SPWM_SM16380SH_REGISTER_ENTRIES[] = {
    spwm_make_fixed_register_config_entry(1, 0x00AA),
    spwm_make_fixed_register_config_entry(2, 0x01AA),
    spwm_make_rgb_register_config_entry(
        3, SPWM_SM16380SH_REGISTER_BLOCK3_SEQUENCES),
    spwm_make_fixed_register_config_entry(4, 0xF003),
    spwm_make_fixed_register_config_entry(5, 0x0055),
    spwm_make_fixed_register_config_entry(6, 0x0155),
};

// -------------------------------------------------------------------------------------------------
// FM6363 register definition.
// -------------------------------------------------------------------------------------------------

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
// Keep this built-in baseline synchronized with catalog regtype1 so the first
// runtime profile remains the exact diagnostic equivalent.
static const SPWM_Register_Config_Entry
    SPWM_FM6363_REGISTER_ENTRIES[] = {
    spwm_make_fixed_register_config_entry(1, 0x1fb0),
    spwm_make_fixed_register_config_entry(2, 0xf39c),
    spwm_make_fixed_register_config_entry(3, 0x20b6),
    spwm_make_fixed_register_config_entry(4, 0x1a00),
    spwm_make_fixed_register_config_entry(5, 0x7e08),
};

// -------------------------------------------------------------------------------------------------
// FM6353 register definition.
// -------------------------------------------------------------------------------------------------

// Runtime slots follow DMD_STM32's reg2, reg4, reg6, reg8, reg10 order.
// PRE_ACT is emitted separately by the panel init sequence, so these are the
// tail-latch widths that select the physical register being written.
static const uint8_t SPWM_FM6353_REGISTER_SEND_LAT[][1] = {
    {2},
    {4},
    {6},
    {8},
    {10},
};
static const SPWM_Register_Timing SPWM_FM6353_REGISTER_TIMINGS[] = {
    spwm_make_register_timing(SPWM_FM6353_REGISTER_SEND_LAT[0]),
    spwm_make_register_timing(SPWM_FM6353_REGISTER_SEND_LAT[1]),
    spwm_make_register_timing(SPWM_FM6353_REGISTER_SEND_LAT[2]),
    spwm_make_register_timing(SPWM_FM6353_REGISTER_SEND_LAT[3]),
    spwm_make_register_timing(SPWM_FM6353_REGISTER_SEND_LAT[4]),
};

// DMD_STM32: conf_6353[] = {0x0008, 0x1f70, 0x6707, 0x40f7, 0x0040}; physical
// reg4 is patched there as ((nRows-1) << 8) | (0x1f70 & 0xFF). It is baked in
// here for the 1/32 scan case (nRows = 32).
// Keep these five values synchronized with catalog regtype1 so the first
// runtime profile remains the exact diagnostic equivalent.
static const SPWM_Register_Config_Entry
    SPWM_FM6353_REGISTER_ENTRIES[] = {
    spwm_make_fixed_register_config_entry(1, 0x0008),
    spwm_make_fixed_register_config_entry(2, 0x1f70),
    spwm_make_fixed_register_config_entry(3, 0x6707),
    spwm_make_fixed_register_config_entry(4, 0x40f7),
    spwm_make_fixed_register_config_entry(5, 0x0040),
};

const SPWM_Loaded_Register_Profile *spwm_get_selected_runtime_profile(
    const char *spwm_panel_type, int spwm_register_config) {
  if (spwm_register_config <= 0) return nullptr;

  std::string error;
  const SPWM_Register_Profile_File *const catalog =
      spwm_get_register_profile_file(spwm_panel_type, &error);
  if (catalog == nullptr) {
    fprintf(stderr, "Unable to load %s register profiles: %s\n",
            spwm_panel_type, error.c_str());
    return nullptr;
  }
  const size_t profile_index =
      static_cast<size_t>(spwm_register_config - 1);
  const SPWM_Loaded_Register_Profile *const profile =
      catalog->profile(profile_index);
  if (profile == nullptr) {
    fprintf(stderr,
            "%s register profile %d is outside the available range 1..%zu.\n",
            spwm_panel_type, spwm_register_config, catalog->profile_count());
  }
  return profile;
}

const SPWM_RGB_Register_Profile_View *spwm_get_selected_rgb_profile(
    const char *spwm_panel_type, int spwm_register_config) {
  const SPWM_Loaded_Register_Profile *const profile =
      spwm_get_selected_runtime_profile(spwm_panel_type,
                                        spwm_register_config);
  if (profile == nullptr) return nullptr;
  const SPWM_RGB_Register_Profile_View *const rgb_profile =
      profile->rgb_profile();
  if (rgb_profile == nullptr) {
    fprintf(stderr, "%s register-profile catalog has the wrong payload kind.\n",
            spwm_panel_type);
  }
  return rgb_profile;
}

const SPWM_Fixed_Register_Profile_View *spwm_get_selected_fixed_profile(
    const char *spwm_panel_type, int spwm_register_config) {
  const SPWM_Loaded_Register_Profile *const profile =
      spwm_get_selected_runtime_profile(spwm_panel_type,
                                        spwm_register_config);
  if (profile == nullptr) return nullptr;
  const SPWM_Fixed_Register_Profile_View *const fixed_profile =
      profile->fixed_profile();
  if (fixed_profile == nullptr) {
    fprintf(stderr, "%s register-profile catalog has the wrong payload kind.\n",
            spwm_panel_type);
  }
  return fixed_profile;
}

}  // namespace

// Keep option validation tied to the exact runtime catalog selected later by
// the panel factories below.
size_t spwm_get_panel_register_profile_count(const char *spwm_panel_type) {
  std::string error;
  const SPWM_Register_Profile_File *const catalog =
      spwm_get_register_profile_file(spwm_panel_type, &error);
  return catalog != nullptr ? catalog->profile_count() : 0;
}

// All factories build the built-in main config first. A positive selector then
// overlays identically numbered runtime-catalog data; row transport does not
// alter register selection.
SPWM_Config spwm_create_fm6373_config(
    const SPWM_Panel_Settings &spwm_settings, int spwm_columns,
    int /*spwm_row_address_type*/, int spwm_register_config) {
  SPWM_Config spwm_config = spwm_create_register_config(
      spwm_settings, spwm_columns, SPWM_FM6373_REGISTER_TIMINGS,
      spwm_make_register_config_entries(SPWM_FM6373_REGISTER_ENTRIES));
  const SPWM_RGB_Register_Profile_View *spwm_profile =
      spwm_get_selected_rgb_profile("fm6373", spwm_register_config);
  if (spwm_profile != nullptr) {
    spwm_apply_rgb_register_profile(&spwm_config, *spwm_profile);
  }
  return spwm_config;
}

SPWM_Config spwm_create_icnd1065l_config(
    const SPWM_Panel_Settings &spwm_settings, int spwm_columns,
    int /*spwm_row_address_type*/, int spwm_register_config) {
  SPWM_Config spwm_config = spwm_create_register_config(
      spwm_settings, spwm_columns, SPWM_ICND1065L_REGISTER_TIMINGS,
      spwm_make_register_config_entries(SPWM_ICND1065L_REGISTER_ENTRIES));
  const SPWM_RGB_Register_Profile_View *spwm_profile =
      spwm_get_selected_rgb_profile("icnd1065l", spwm_register_config);
  if (spwm_profile != nullptr) {
    spwm_apply_rgb_register_profile(&spwm_config, *spwm_profile);
  }
  return spwm_config;
}

SPWM_Config spwm_create_sm16380sh_config(
    const SPWM_Panel_Settings &spwm_settings, int spwm_columns,
    int /*spwm_row_address_type*/, int spwm_register_config) {
  SPWM_Config spwm_config = spwm_create_register_config(
      spwm_settings, spwm_columns, SPWM_SM16380SH_REGISTER_TIMINGS,
      spwm_make_register_config_entries(SPWM_SM16380SH_REGISTER_ENTRIES));
  const SPWM_RGB_Register_Profile_View *spwm_profile =
      spwm_get_selected_rgb_profile("sm16380sh", spwm_register_config);
  if (spwm_profile != nullptr) {
    spwm_apply_rgb_register_profile(&spwm_config, *spwm_profile);
  }
  return spwm_config;
}

SPWM_Config spwm_create_fm6363_config(
    const SPWM_Panel_Settings &spwm_settings, int spwm_columns,
    int /*spwm_row_address_type*/, int spwm_register_config) {
  SPWM_Config spwm_config = spwm_create_register_config(
      spwm_settings, spwm_columns, SPWM_FM6363_REGISTER_TIMINGS,
      spwm_make_register_config_entries(SPWM_FM6363_REGISTER_ENTRIES));
  const SPWM_Fixed_Register_Profile_View *spwm_profile =
      spwm_get_selected_fixed_profile("fm6363", spwm_register_config);
  if (spwm_profile != nullptr) {
    spwm_apply_fixed_register_profile(&spwm_config, *spwm_profile);
  }
  return spwm_config;
}

SPWM_Config spwm_create_fm6353_config(
    const SPWM_Panel_Settings &spwm_settings, int spwm_columns,
    int /*spwm_row_address_type*/, int spwm_register_config) {
  SPWM_Config spwm_config = spwm_create_register_config(
      spwm_settings, spwm_columns, SPWM_FM6353_REGISTER_TIMINGS,
      spwm_make_register_config_entries(SPWM_FM6353_REGISTER_ENTRIES));
  const SPWM_Fixed_Register_Profile_View *spwm_profile =
      spwm_get_selected_fixed_profile("fm6353", spwm_register_config);
  if (spwm_profile != nullptr) {
    spwm_apply_fixed_register_profile(&spwm_config, *spwm_profile);
  }
  return spwm_config;
}

}  // namespace internal
}  // namespace rgb_matrix
