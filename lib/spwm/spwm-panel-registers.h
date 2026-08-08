// -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
#ifndef RGBMATRIX_SPWM_PANEL_REGISTERS_H
#define RGBMATRIX_SPWM_PANEL_REGISTERS_H

#include "spwm-panel-config.h"

namespace rgb_matrix {
namespace internal {

// Return how many runtime-catalog regtype profiles can be selected for this
// panel. Positive --led-spwm-register-config values use this 1-based range.
size_t spwm_get_panel_register_profile_count(const char *spwm_panel_type);

// Build a panel's owning register config from its built-in main payload, then
// overlay catalog regtypeN when spwm_register_config is positive. Register
// selection is independent of row-address transport, so that argument is kept
// only as part of the shared panel-factory signature.
SPWM_Config spwm_create_fm6373_config(
    const SPWM_Panel_Settings &spwm_settings, int spwm_columns,
    int spwm_row_address_type, int spwm_register_config);
SPWM_Config spwm_create_icnd1065l_config(
    const SPWM_Panel_Settings &spwm_settings, int spwm_columns,
    int spwm_row_address_type, int spwm_register_config);
SPWM_Config spwm_create_sm16380sh_config(
    const SPWM_Panel_Settings &spwm_settings, int spwm_columns,
    int spwm_row_address_type, int spwm_register_config);
SPWM_Config spwm_create_fm6363_config(
    const SPWM_Panel_Settings &spwm_settings, int spwm_columns,
    int spwm_row_address_type, int spwm_register_config);
SPWM_Config spwm_create_fm6353_config(
    const SPWM_Panel_Settings &spwm_settings, int spwm_columns,
    int spwm_row_address_type, int spwm_register_config);
SPWM_Config spwm_create_icnd2153_config(
    const SPWM_Panel_Settings &spwm_settings, int spwm_columns,
    int spwm_row_address_type, int spwm_register_config);

}  // namespace internal
}  // namespace rgb_matrix

#endif  // RGBMATRIX_SPWM_PANEL_REGISTERS_H
