// -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
#include "spwm-panel-registers.h"

#include <stddef.h>
#include <stdint.h>
#include <vector>

namespace rgb_matrix {
namespace internal {

namespace {

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

struct SPWM_Register_Config {
  int register_config;
  SPWM_Register_Config_Entries entries;
};

template <size_t N>
SPWM_Word_Sequence spwm_make_word_sequence(
    const uint16_t (&spwm_words)[N]) {
  const SPWM_Word_Sequence spwm_sequence = {spwm_words, N};
  return spwm_sequence;
}

std::vector<uint16_t> spwm_make_words(
    const SPWM_Word_Sequence &spwm_sequence) {
  return std::vector<uint16_t>(
      spwm_sequence.words, spwm_sequence.words + spwm_sequence.word_count);
}

template <size_t N>
SPWM_Register_Config_Entries spwm_make_register_config_entries(
    const SPWM_Register_Config_Entry (&spwm_entries)[N]) {
  const SPWM_Register_Config_Entries spwm_entry_sequence = {spwm_entries, N};
  return spwm_entry_sequence;
}

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

template <size_t N>
const SPWM_Register_Config &spwm_find_register_config(
    const SPWM_Register_Config (&spwm_register_configs)[N],
    int spwm_register_config) {
  for (size_t spwm_index = 0; spwm_index < N; ++spwm_index) {
    if (spwm_register_configs[spwm_index].register_config ==
        spwm_register_config) {
      return spwm_register_configs[spwm_index];
    }
  }
  return spwm_register_configs[0];
}

int spwm_resolve_default_register_config(int spwm_register_config) {
  return spwm_register_config >= 0 ? spwm_register_config : 0;
}

template <size_t RegisterCount, size_t ConfigCount>
SPWM_Config spwm_create_selected_register_config(
    const SPWM_Panel_Settings &spwm_settings, int spwm_columns,
    const SPWM_Register_Timing (&spwm_register_timings)[RegisterCount],
    const SPWM_Register_Config (&spwm_register_configs)[ConfigCount],
    int spwm_register_config) {
  const SPWM_Register_Config &spwm_selected_register_config =
      spwm_find_register_config(
          spwm_register_configs,
          spwm_resolve_default_register_config(spwm_register_config));
  return spwm_create_register_config(spwm_settings, spwm_columns,
                                     spwm_register_timings,
                                     spwm_selected_register_config.entries);
}

int spwm_resolve_sm16380sh_register_config(int spwm_row_address_type,
                                           int spwm_register_config) {
  if (spwm_register_config >= 0) return spwm_register_config;
  return spwm_row_address_type == SPWM_ROW_ADDRESS_TYPE_0_DIRECT_AE ? 0 : 1;
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
static const uint16_t SPWM_FM6373_REGISTER_CONFIG0_BLOCK3_SEQ_R[] = {
    0x0000, 0x0100, 0x021f, 0x033f, 0x0402, 0x0508, 0x0602, 0x0720,
    0x0820, 0x0900, 0x0a00, 0x0b00, 0x0c01, 0x0d01, 0x0e04, 0x0f01,
    0x10c2, 0x1121, 0x1201, 0x1300, 0x1400, 0x1500, 0x1600, 0x17f0,
    0x181f, 0x1900, 0x1a1f, 0x1b10, 0x1c2a, 0x1d0a, 0x1e42, 0x1f04,
    0x2008, 0x2101, 0x221c, 0x7000, 0x7100, 0x7200, 0x7300, 0x7400,
    0xf000, 0xf100, 0xf200, 0xf300, 0xf400, 0xf500, 0x2300,
};

static const uint16_t SPWM_FM6373_REGISTER_CONFIG0_BLOCK3_SEQ_G[] = {
    0x0000, 0x0100, 0x021f, 0x033f, 0x0402, 0x0508, 0x0602, 0x0720,
    0x0820, 0x0900, 0x0a00, 0x0b00, 0x0c08, 0x0d01, 0x0e04, 0x0f01,
    0x10c2, 0x1121, 0x1201, 0x1300, 0x1400, 0x1500, 0x1600, 0x17f0,
    0x181f, 0x1950, 0x1a1f, 0x1b10, 0x1c2a, 0x1d0a, 0x1e46, 0x1f20,
    0x2008, 0x2101, 0x221c, 0x7000, 0x7100, 0x7200, 0x7300, 0x7400,
    0xf000, 0xf100, 0xf200, 0xf300, 0xf400, 0xf500, 0x2300,
};

static const uint16_t SPWM_FM6373_REGISTER_CONFIG0_BLOCK3_SEQ_B[] = {
    0x0000, 0x0100, 0x021f, 0x033f, 0x0402, 0x0508, 0x0602, 0x0720,
    0x0820, 0x0900, 0x0a00, 0x0b00, 0x0c08, 0x0d01, 0x0e04, 0x0f01,
    0x10c2, 0x1121, 0x1201, 0x1300, 0x1400, 0x1500, 0x1600, 0x17f0,
    0x182f, 0x1900, 0x1a1f, 0x1b10, 0x1c2a, 0x1d0a, 0x1e48, 0x1f20,
    0x2010, 0x2101, 0x221c, 0x7000, 0x7100, 0x7200, 0x7300, 0x7400,
    0xf000, 0xf100, 0xf200, 0xf300, 0xf400, 0xf500, 0x2300,
};

static const SPWM_RGB_Word_Sequences
    SPWM_FM6373_REGISTER_CONFIG0_BLOCK3_SEQUENCES = {
    spwm_make_word_sequence(SPWM_FM6373_REGISTER_CONFIG0_BLOCK3_SEQ_R),
    spwm_make_word_sequence(SPWM_FM6373_REGISTER_CONFIG0_BLOCK3_SEQ_G),
    spwm_make_word_sequence(SPWM_FM6373_REGISTER_CONFIG0_BLOCK3_SEQ_B),
};

// Config 1 starts from the MRV412/Nova 64S FM6373+DP32020B capture from a
// 128x64 P2.5HE64AV2B module. The 64S scan/subfield/tail values are kept from
// that capture, while selected color/analog balance slots stay at config 0
// values to test the blue-tint behavior seen with the raw capture payload.
static const uint16_t SPWM_FM6373_REGISTER_CONFIG1_BLOCK3_SEQ_R[] = {
    0x0000, 0x0100, 0x023f, 0x033f, 0x0402, 0x0508, 0x0602, 0x0710,
    0x0810, 0x0900, 0x0a00, 0x0b00, 0x0c01, 0x0d03, 0x0e02, 0x0f11,
    0x10c2, 0x1121, 0x1201, 0x1300, 0x1400, 0x1500, 0x1600, 0x17f0,
    0x181f, 0x1900, 0x1a1f, 0x1b10, 0x1cbe, 0x1d0e, 0x1e42, 0x1f24,
    0x2008, 0x2101, 0x221c, 0x5400, 0x5400, 0x5400, 0x5400, 0x5400,
    0x5403, 0x5403, 0x5403, 0x5403, 0x5403, 0x5403, 0x2300,
};

static const uint16_t SPWM_FM6373_REGISTER_CONFIG1_BLOCK3_SEQ_G[] = {
    0x0000, 0x0100, 0x023f, 0x033f, 0x0402, 0x0508, 0x0602, 0x0710,
    0x0810, 0x0900, 0x0a00, 0x0b00, 0x0c08, 0x0d03, 0x0e04, 0x0f11,
    0x10c2, 0x1121, 0x1201, 0x1300, 0x1400, 0x1500, 0x1600, 0x17f0,
    0x181f, 0x1950, 0x1a1f, 0x1b10, 0x1cbe, 0x1d0e, 0x1e46, 0x1f20,
    0x2008, 0x2101, 0x221c, 0x5400, 0x5400, 0x5400, 0x5400, 0x5400,
    0x5403, 0x5403, 0x5403, 0x5403, 0x5403, 0x5403, 0x2300,
};

static const uint16_t SPWM_FM6373_REGISTER_CONFIG1_BLOCK3_SEQ_B[] = {
    0x0000, 0x0100, 0x023f, 0x033f, 0x0402, 0x0508, 0x0602, 0x0710,
    0x0810, 0x0900, 0x0a00, 0x0b00, 0x0c08, 0x0d01, 0x0e04, 0x0f11,
    0x10c2, 0x1121, 0x1201, 0x1300, 0x1400, 0x1500, 0x1600, 0x17f0,
    0x182f, 0x1900, 0x1a1f, 0x1b10, 0x1cbe, 0x1d0e, 0x1e48, 0x1f20,
    0x2010, 0x2101, 0x221c, 0x5400, 0x5400, 0x5400, 0x5400, 0x5400,
    0x5403, 0x5403, 0x5403, 0x5403, 0x5403, 0x5403, 0x2300,
};

static const SPWM_RGB_Word_Sequences
    SPWM_FM6373_REGISTER_CONFIG1_BLOCK3_SEQUENCES = {
    spwm_make_word_sequence(SPWM_FM6373_REGISTER_CONFIG1_BLOCK3_SEQ_R),
    spwm_make_word_sequence(SPWM_FM6373_REGISTER_CONFIG1_BLOCK3_SEQ_G),
    spwm_make_word_sequence(SPWM_FM6373_REGISTER_CONFIG1_BLOCK3_SEQ_B),
};

static const SPWM_Register_Config_Entry
    SPWM_FM6373_REGISTER_CONFIG0_ENTRIES[] = {
    spwm_make_fixed_register_config_entry(1, 0x00AA),
    spwm_make_fixed_register_config_entry(2, 0x01AA),
    spwm_make_rgb_register_config_entry(
        3, SPWM_FM6373_REGISTER_CONFIG0_BLOCK3_SEQUENCES),
    spwm_make_fixed_register_config_entry(4, 0x0055),
    spwm_make_fixed_register_config_entry(5, 0x0155),
};

static const SPWM_Register_Config_Entry
    SPWM_FM6373_REGISTER_CONFIG1_ENTRIES[] = {
    spwm_make_fixed_register_config_entry(1, 0x00AA),
    spwm_make_fixed_register_config_entry(2, 0x01AA),
    spwm_make_rgb_register_config_entry(
        3, SPWM_FM6373_REGISTER_CONFIG1_BLOCK3_SEQUENCES),
    spwm_make_fixed_register_config_entry(4, 0x0055),
    spwm_make_fixed_register_config_entry(5, 0x0155),
};

static const SPWM_Register_Config SPWM_FM6373_REGISTER_CONFIGS[] = {
    {0, spwm_make_register_config_entries(SPWM_FM6373_REGISTER_CONFIG0_ENTRIES)},
    {1, spwm_make_register_config_entries(SPWM_FM6373_REGISTER_CONFIG1_ENTRIES)},
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
static const uint16_t SPWM_ICND1065L_REGISTER_CONFIG0_BLOCK3_SEQ_R[] = {
    0x0000, 0x026a, 0x0322, 0x0412, 0x0500, 0x0601, 0x0712, 0x0c10,
    0x0d02, 0x0e84, 0x0f01, 0x1040, 0x1127, 0x1800, 0x1926, 0x1c60,
    0x1d02, 0x1e71, 0x2040, 0x2101, 0x2380, 0x74a0,
};

static const uint16_t SPWM_ICND1065L_REGISTER_CONFIG0_BLOCK3_SEQ_G[] = {
    0x0000, 0x026a, 0x0322, 0x0412, 0x0500, 0x0601, 0x0712, 0x0c10,
    0x0d04, 0x0e84, 0x0f01, 0x1040, 0x1127, 0x1800, 0x1908, 0x1c60,
    0x1d02, 0x1e92, 0x2060, 0x2101, 0x2305, 0x74a0,
};

static const uint16_t SPWM_ICND1065L_REGISTER_CONFIG0_BLOCK3_SEQ_B[] = {
    0x0000, 0x026a, 0x0322, 0x0412, 0x0500, 0x0601, 0x0712, 0x0c10,
    0x0d03, 0x0e84, 0x0f11, 0x1040, 0x1127, 0x1800, 0x190a, 0x1c60,
    0x1d02, 0x1eb5, 0x2060, 0x2101, 0x2300, 0x74a0,
};

static const SPWM_RGB_Word_Sequences
    SPWM_ICND1065L_REGISTER_CONFIG0_BLOCK3_SEQUENCES = {
    spwm_make_word_sequence(SPWM_ICND1065L_REGISTER_CONFIG0_BLOCK3_SEQ_R),
    spwm_make_word_sequence(SPWM_ICND1065L_REGISTER_CONFIG0_BLOCK3_SEQ_G),
    spwm_make_word_sequence(SPWM_ICND1065L_REGISTER_CONFIG0_BLOCK3_SEQ_B),
};

static const SPWM_Register_Config_Entry
    SPWM_ICND1065L_REGISTER_CONFIG0_ENTRIES[] = {
    spwm_make_fixed_register_config_entry(1, 0x00AA),
    spwm_make_fixed_register_config_entry(2, 0x01AA),
    spwm_make_rgb_register_config_entry(
        3, SPWM_ICND1065L_REGISTER_CONFIG0_BLOCK3_SEQUENCES),
    spwm_make_fixed_register_config_entry(4, 0x0055),
    spwm_make_fixed_register_config_entry(5, 0x0155),
};

// CONFIG1 keeps the proven CONFIG0 tuning but programs the ICND1065L for
// 1/32 scan. RegValue[0] bits 5:0 contain scan_count - 1, so the 1/43 word
// 0x026a becomes 0x025f for 1/32 in each RGB sequence.
static const uint16_t SPWM_ICND1065L_REGISTER_CONFIG1_BLOCK3_SEQ_R[] = {
    0x0000, 0x025f, 0x0322, 0x0412, 0x0500, 0x0601, 0x0712, 0x0c10,
    0x0d02, 0x0e84, 0x0f01, 0x1040, 0x1127, 0x1800, 0x1926, 0x1c60,
    0x1d02, 0x1e71, 0x2040, 0x2101, 0x2380, 0x74a0,
};

static const uint16_t SPWM_ICND1065L_REGISTER_CONFIG1_BLOCK3_SEQ_G[] = {
    0x0000, 0x025f, 0x0322, 0x0412, 0x0500, 0x0601, 0x0712, 0x0c10,
    0x0d04, 0x0e84, 0x0f01, 0x1040, 0x1127, 0x1800, 0x1908, 0x1c60,
    0x1d02, 0x1e92, 0x2060, 0x2101, 0x2305, 0x74a0,
};

static const uint16_t SPWM_ICND1065L_REGISTER_CONFIG1_BLOCK3_SEQ_B[] = {
    0x0000, 0x025f, 0x0322, 0x0412, 0x0500, 0x0601, 0x0712, 0x0c10,
    0x0d03, 0x0e84, 0x0f11, 0x1040, 0x1127, 0x1800, 0x190a, 0x1c60,
    0x1d02, 0x1eb5, 0x2060, 0x2101, 0x2300, 0x74a0,
};

static const SPWM_RGB_Word_Sequences
    SPWM_ICND1065L_REGISTER_CONFIG1_BLOCK3_SEQUENCES = {
    spwm_make_word_sequence(SPWM_ICND1065L_REGISTER_CONFIG1_BLOCK3_SEQ_R),
    spwm_make_word_sequence(SPWM_ICND1065L_REGISTER_CONFIG1_BLOCK3_SEQ_G),
    spwm_make_word_sequence(SPWM_ICND1065L_REGISTER_CONFIG1_BLOCK3_SEQ_B),
};

static const SPWM_Register_Config_Entry
    SPWM_ICND1065L_REGISTER_CONFIG1_ENTRIES[] = {
    spwm_make_fixed_register_config_entry(1, 0x00AA),
    spwm_make_fixed_register_config_entry(2, 0x01AA),
    spwm_make_rgb_register_config_entry(
        3, SPWM_ICND1065L_REGISTER_CONFIG1_BLOCK3_SEQUENCES),
    spwm_make_fixed_register_config_entry(4, 0x0055),
    spwm_make_fixed_register_config_entry(5, 0x0155),
};

static const SPWM_Register_Config SPWM_ICND1065L_REGISTER_CONFIGS[] = {
    {0, spwm_make_register_config_entries(
            SPWM_ICND1065L_REGISTER_CONFIG0_ENTRIES)},
    {1, spwm_make_register_config_entries(
            SPWM_ICND1065L_REGISTER_CONFIG1_ENTRIES)},
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
// REGISTER_CONFIG0 is the existing direct-A/B/C/D/E SM16380SH block used by
// --led-spwm-register-config=0. REGISTER_CONFIG1 is the shift-register variant
// used by --led-spwm-register-config=1, changing register 0x07 to 0x072c.
static const uint16_t SPWM_SM16380SH_REGISTER_CONFIG0_BLOCK3_SEQ_R[] = {
    0x021f, 0x0300, 0x0400, 0x0500, 0x0600, 0x078c, 0x0800, 0x0900,
    0x0a02, 0x0b0c, 0x0c08, 0x0d00, 0x0e05, 0x0f00, 0x1000, 0x1100,
    0x1200, 0x1308, 0x1414, 0x1500, 0x1630, 0x1700, 0x1801, 0x1904,
    0x1a03, 0x1b14, 0x1c12, 0x1d00, 0x1e00, 0x1f0c, 0x2000, 0x2200,
};

static const uint16_t SPWM_SM16380SH_REGISTER_CONFIG0_BLOCK3_SEQ_G[] = {
    0x021f, 0x0300, 0x0400, 0x0500, 0x0600, 0x078c, 0x0800, 0x0900,
    0x0a02, 0x0b0c, 0x0c18, 0x0d00, 0x0e05, 0x0f00, 0x1000, 0x1100,
    0x1200, 0x1308, 0x1422, 0x1500, 0x1630, 0x1700, 0x1801, 0x1903,
    0x1a01, 0x1b14, 0x1c8f, 0x1d00, 0x1e00, 0x1f0c, 0x2000, 0x2200,
};

static const uint16_t SPWM_SM16380SH_REGISTER_CONFIG0_BLOCK3_SEQ_B[] = {
    0x021f, 0x0300, 0x0400, 0x0500, 0x0600, 0x078c, 0x0800, 0x0900,
    0x0a02, 0x0b0c, 0x0c30, 0x0d00, 0x0e05, 0x0f00, 0x1000, 0x1100,
    0x1200, 0x1308, 0x1432, 0x1500, 0x1630, 0x1700, 0x1801, 0x1903,
    0x1a01, 0x1b14, 0x1c8f, 0x1d00, 0x1e00, 0x1f0c, 0x2000, 0x2200,
};

static const uint16_t SPWM_SM16380SH_REGISTER_CONFIG1_BLOCK3_SEQ_R[] = {
    0x021f, 0x0300, 0x0400, 0x0500, 0x0600, 0x072c, 0x0800, 0x0900,
    0x0a02, 0x0b0c, 0x0c08, 0x0d00, 0x0e05, 0x0f00, 0x1000, 0x1100,
    0x1200, 0x1308, 0x1414, 0x1500, 0x1630, 0x1700, 0x1801, 0x1904,
    0x1a03, 0x1b14, 0x1c12, 0x1d00, 0x1e00, 0x1f0c, 0x2000, 0x2200,
};

static const uint16_t SPWM_SM16380SH_REGISTER_CONFIG1_BLOCK3_SEQ_G[] = {
    0x021f, 0x0300, 0x0400, 0x0500, 0x0600, 0x072c, 0x0800, 0x0900,
    0x0a02, 0x0b0c, 0x0c18, 0x0d00, 0x0e05, 0x0f00, 0x1000, 0x1100,
    0x1200, 0x1308, 0x1422, 0x1500, 0x1630, 0x1700, 0x1801, 0x1903,
    0x1a01, 0x1b14, 0x1c8f, 0x1d00, 0x1e00, 0x1f0c, 0x2000, 0x2200,
};

static const uint16_t SPWM_SM16380SH_REGISTER_CONFIG1_BLOCK3_SEQ_B[] = {
    0x021f, 0x0300, 0x0400, 0x0500, 0x0600, 0x072c, 0x0800, 0x0900,
    0x0a02, 0x0b0c, 0x0c30, 0x0d00, 0x0e05, 0x0f00, 0x1000, 0x1100,
    0x1200, 0x1308, 0x1432, 0x1500, 0x1630, 0x1700, 0x1801, 0x1903,
    0x1a01, 0x1b14, 0x1c8f, 0x1d00, 0x1e00, 0x1f0c, 0x2000, 0x2200,
};

static const SPWM_RGB_Word_Sequences
    SPWM_SM16380SH_REGISTER_CONFIG0_BLOCK3_SEQUENCES = {
    spwm_make_word_sequence(SPWM_SM16380SH_REGISTER_CONFIG0_BLOCK3_SEQ_R),
    spwm_make_word_sequence(SPWM_SM16380SH_REGISTER_CONFIG0_BLOCK3_SEQ_G),
    spwm_make_word_sequence(SPWM_SM16380SH_REGISTER_CONFIG0_BLOCK3_SEQ_B),
};

static const SPWM_RGB_Word_Sequences
    SPWM_SM16380SH_REGISTER_CONFIG1_BLOCK3_SEQUENCES = {
    spwm_make_word_sequence(SPWM_SM16380SH_REGISTER_CONFIG1_BLOCK3_SEQ_R),
    spwm_make_word_sequence(SPWM_SM16380SH_REGISTER_CONFIG1_BLOCK3_SEQ_G),
    spwm_make_word_sequence(SPWM_SM16380SH_REGISTER_CONFIG1_BLOCK3_SEQ_B),
};

static const SPWM_Register_Config_Entry
    SPWM_SM16380SH_REGISTER_CONFIG0_ENTRIES[] = {
    spwm_make_fixed_register_config_entry(1, 0x00AA),
    spwm_make_fixed_register_config_entry(2, 0x01AA),
    spwm_make_rgb_register_config_entry(
        3, SPWM_SM16380SH_REGISTER_CONFIG0_BLOCK3_SEQUENCES),
    spwm_make_fixed_register_config_entry(4, 0xF003),
    spwm_make_fixed_register_config_entry(5, 0x0055),
    spwm_make_fixed_register_config_entry(6, 0x0155),
};

static const SPWM_Register_Config_Entry
    SPWM_SM16380SH_REGISTER_CONFIG1_ENTRIES[] = {
    spwm_make_fixed_register_config_entry(1, 0x00AA),
    spwm_make_fixed_register_config_entry(2, 0x01AA),
    spwm_make_rgb_register_config_entry(
        3, SPWM_SM16380SH_REGISTER_CONFIG1_BLOCK3_SEQUENCES),
    spwm_make_fixed_register_config_entry(4, 0xF003),
    spwm_make_fixed_register_config_entry(5, 0x0055),
    spwm_make_fixed_register_config_entry(6, 0x0155),
};

static const SPWM_Register_Config SPWM_SM16380SH_REGISTER_CONFIGS[] = {
    {0, spwm_make_register_config_entries(
            SPWM_SM16380SH_REGISTER_CONFIG0_ENTRIES)},
    {1, spwm_make_register_config_entries(
            SPWM_SM16380SH_REGISTER_CONFIG1_ENTRIES)},
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
static const SPWM_Register_Config_Entry
    SPWM_FM6363_REGISTER_CONFIG0_ENTRIES[] = {
    spwm_make_fixed_register_config_entry(1, 0x1fb0),
    spwm_make_fixed_register_config_entry(2, 0xf39c),
    spwm_make_fixed_register_config_entry(3, 0x20b6),
    spwm_make_fixed_register_config_entry(4, 0x1a00),
    spwm_make_fixed_register_config_entry(5, 0x7e08),
};

static const SPWM_Register_Config SPWM_FM6363_REGISTER_CONFIGS[] = {
    {0, spwm_make_register_config_entries(SPWM_FM6363_REGISTER_CONFIG0_ENTRIES)},
};

// -------------------------------------------------------------------------------------------------
// FM6353 register definition.
// -------------------------------------------------------------------------------------------------

static const uint8_t SPWM_FM6353_REGISTER_SEND_LAT[][1] = {
    {14},
    {14},
    {14},
    {14},
    {14},
};
static const SPWM_Register_Timing SPWM_FM6353_REGISTER_TIMINGS[] = {
    spwm_make_register_timing(SPWM_FM6353_REGISTER_SEND_LAT[0]),
    spwm_make_register_timing(SPWM_FM6353_REGISTER_SEND_LAT[1]),
    spwm_make_register_timing(SPWM_FM6353_REGISTER_SEND_LAT[2]),
    spwm_make_register_timing(SPWM_FM6353_REGISTER_SEND_LAT[3]),
    spwm_make_register_timing(SPWM_FM6353_REGISTER_SEND_LAT[4]),
};

// DMD_STM32: conf_6353[] = {0x0008, 0x1f70, 0x6707, 0x40f7, 0x0040}; reg 2 is
// patched at runtime as ((nRows-1) << 8) | (0x1f70 & 0xFF). Baked in for the
// 1/32 scan case (nRows = 32) like the FM6363 profile does for its own panel.
static const SPWM_Register_Config_Entry
    SPWM_FM6353_REGISTER_CONFIG0_ENTRIES[] = {
    spwm_make_fixed_register_config_entry(1, 0x0008),
    spwm_make_fixed_register_config_entry(2, 0x1f70),
    spwm_make_fixed_register_config_entry(3, 0x6707),
    spwm_make_fixed_register_config_entry(4, 0x40f7),
    spwm_make_fixed_register_config_entry(5, 0x0040),
};

static const SPWM_Register_Config SPWM_FM6353_REGISTER_CONFIGS[] = {
    {0, spwm_make_register_config_entries(SPWM_FM6353_REGISTER_CONFIG0_ENTRIES)},
};

}  // namespace

SPWM_Config spwm_create_fm6373_config(
    const SPWM_Panel_Settings &spwm_settings, int spwm_columns,
    int /*spwm_row_address_type*/, int spwm_register_config) {
  return spwm_create_selected_register_config(
      spwm_settings, spwm_columns, SPWM_FM6373_REGISTER_TIMINGS,
      SPWM_FM6373_REGISTER_CONFIGS, spwm_register_config);
}

SPWM_Config spwm_create_icnd1065l_config(
    const SPWM_Panel_Settings &spwm_settings, int spwm_columns,
    int /*spwm_row_address_type*/, int spwm_register_config) {
  return spwm_create_selected_register_config(
      spwm_settings, spwm_columns, SPWM_ICND1065L_REGISTER_TIMINGS,
      SPWM_ICND1065L_REGISTER_CONFIGS, spwm_register_config);
}

SPWM_Config spwm_create_sm16380sh_config(
    const SPWM_Panel_Settings &spwm_settings, int spwm_columns,
    int spwm_row_address_type, int spwm_register_config) {
  const int spwm_resolved_register_config =
      spwm_resolve_sm16380sh_register_config(spwm_row_address_type,
                                             spwm_register_config);
  return spwm_create_selected_register_config(
      spwm_settings, spwm_columns, SPWM_SM16380SH_REGISTER_TIMINGS,
      SPWM_SM16380SH_REGISTER_CONFIGS, spwm_resolved_register_config);
}

SPWM_Config spwm_create_fm6363_config(
    const SPWM_Panel_Settings &spwm_settings, int spwm_columns,
    int /*spwm_row_address_type*/, int spwm_register_config) {
  return spwm_create_selected_register_config(
      spwm_settings, spwm_columns, SPWM_FM6363_REGISTER_TIMINGS,
      SPWM_FM6363_REGISTER_CONFIGS, spwm_register_config);
}

SPWM_Config spwm_create_fm6353_config(
    const SPWM_Panel_Settings &spwm_settings, int spwm_columns,
    int /*spwm_row_address_type*/, int spwm_register_config) {
  return spwm_create_selected_register_config(
      spwm_settings, spwm_columns, SPWM_FM6353_REGISTER_TIMINGS,
      SPWM_FM6353_REGISTER_CONFIGS, spwm_register_config);
}

}  // namespace internal
}  // namespace rgb_matrix
