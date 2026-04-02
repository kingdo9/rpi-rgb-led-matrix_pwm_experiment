// -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
#ifndef RGBMATRIX_SPWM_PANEL_CONFIG_H
#define RGBMATRIX_SPWM_PANEL_CONFIG_H

#include "spwm-helpers.h"

namespace rgb_matrix {
namespace internal {

// Factory used by a panel profile to build its runtime register layout.
using SPWM_Config_Factory = SPWM_Config (*)(
    const SPWM_Panel_Settings &settings, int columns);

// Kinds of startup actions that can appear in a panel init sequence.
enum SPWM_Init_Step_Type {
  SPWM_INIT_STEP_LAT_PULSES = 0,
  SPWM_INIT_STEP_REGISTER,
  SPWM_INIT_STEP_RGB_REGISTER,
};

// One startup action: either a LAT pulse burst, a fixed register upload, or the
// per-frame RGB register block.
struct SPWM_Init_Step {
  SPWM_Init_Step_Type type;
  uint8_t value;  // LAT pulse count or register index.
  uint8_t row;    // Row bits to leave on A-E after the step completes.
  uint8_t space_clocks;  // Extra LAT-low spacer clocks after the step.
};

// Ordered list of init steps emitted before regular RGB data upload begins.
struct SPWM_Init_Sequence {
  const SPWM_Init_Step *steps;  // Backing storage for the init steps.
  size_t step_count;          // Number of entries in `steps`.
};

// Complete description of an SPWM-capable panel type, including geometry, init
// sequence, and register payload factory.
struct SPWM_Panel_Profile {
  const char *panel_type;    // Name matched against --led-panel-type.
  SPWM_Panel_Settings settings;   // Default geometry and timing for the panel.
  SPWM_Config_Factory create_config;  // Builds the register payload set.
  SPWM_Init_Sequence init_sequence;  // Startup script emitted each frame.
};

// Return the built-in fallback SPWM panel profile.
const SPWM_Panel_Profile &spwm_get_default_panel_profile();

// Return the matching SPWM panel profile for `panel_type`, if any.
const SPWM_Panel_Profile *spwm_find_panel_profile(const char *panel_type);

// Resolve the effective settings for one profile at the requested width so
// panel-specific geometry quirks can stay co-located with the profile.
SPWM_Panel_Settings spwm_resolve_profile_settings(
    const SPWM_Panel_Profile &profile, int columns);

}  // namespace internal
}  // namespace rgb_matrix

#endif  // RGBMATRIX_SPWM_PANEL_CONFIG_H
