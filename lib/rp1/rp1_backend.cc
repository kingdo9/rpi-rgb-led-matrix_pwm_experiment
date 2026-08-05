// -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
#include "rp1_backend.h"

#include "rp1_pio_backend.h"
#include "rp1_rio_backend.h"
#include "../hardware-mapping.h"
#include "../spwm/spwm-helpers.h"

#include <stdio.h>

namespace rgb_matrix {
namespace internal {
namespace {

static void SetError(std::string *error, const std::string &message) {
  if (error != NULL) *error = message;
}

}  // namespace

bool Rp1BackendSelectForOptions(bool do_gpio_init, int rp1_pio,
                                int gpio_slowdown,
                                const char *hardware_mapping,
                                int row_address_type, int parallel,
                                const char *panel_type,
                                bool *use_dedicated_backend,
                                std::string *error) {
  if (use_dedicated_backend != NULL) *use_dedicated_backend = false;

  if (rp1_pio != 0 && rp1_pio != 1) {
    char buffer[128];
    snprintf(buffer, sizeof(buffer),
             "--led-rp1-pio=%d is outside usable range 0..1\n", rp1_pio);
    SetError(error, buffer);
    return false;
  }

  Rp1RioSetEnabled(rp1_pio == 0);

  const bool is_spwm_panel = spwm_is_panel_type(panel_type);
  const bool use_rp1_rio = do_gpio_init && !is_spwm_panel &&
      Rp1RioShouldActivate(hardware_mapping, row_address_type, parallel);
  const bool use_rp1_pio = !use_rp1_rio && do_gpio_init && !is_spwm_panel &&
      !Rp1RioBackendRequested() &&
      Rp1PioShouldActivate(hardware_mapping, row_address_type, parallel);

  if (use_rp1_rio) {
    Rp1RioSetGpioSlowdown(gpio_slowdown);
  }
  if (use_rp1_pio) {
    Rp1PioSetGpioSlowdown(gpio_slowdown);
  }

  const bool pi5_backend_available =
      Rp1PioPlatformDetected() || Rp1RioPlatformDetected();
  if (do_gpio_init && pi5_backend_available && is_spwm_panel &&
      !Rp1RioBackendRequested()) {
    char buffer[512];
    snprintf(buffer, sizeof(buffer),
             "SPWM panel type '%s' on Pi 5-family boards currently supports "
             "the RP1 RIO GPIO path only.\n"
             "SPWM over RP1 PIO is not supported; omit --led-rp1-pio "
             "or use --led-rp1-pio=0 on Pi 5-family boards.\n",
             panel_type != NULL ? panel_type : "");
    SetError(error, buffer);
    return false;
  }

  if (do_gpio_init && pi5_backend_available && !is_spwm_panel &&
      !use_rp1_pio && !use_rp1_rio) {
    if (Rp1RioBackendRequested()) {
      SetError(error,
               "Pi 5-family RP1 RIO backend is selected, but this non-SPWM "
               "configuration is outside the currently supported RIO cases.\n"
               "RIO supports non-SPWM panels for mappings "
               "regular/adafruit-hat/adafruit-hat-pwm/classic and "
               "--led-row-addr-type=0, 1, 2, 3, 4, or 5.\n"
               "For non-SPWM configurations supported by PIO, use "
               "--led-rp1-pio=1 to select RP1 PIO.\n");
    } else {
      SetError(error,
               "Pi 5-family RP1 PIO backend is selected, but this non-SPWM "
               "configuration is not supported by PIO yet.\n"
               "Supported in PIO mode for now: mappings "
               "regular/regular-pi1/adafruit-hat/adafruit-hat-pwm/classic and "
               "--led-row-addr-type=0, 1, 2, 3, 4, or 5.\n");
    }
    return false;
  }

  if (use_dedicated_backend != NULL) {
    *use_dedicated_backend = use_rp1_rio || use_rp1_pio;
  }
  return true;
}

bool Rp1BackendIsActive() {
  return Rp1RioIsActive() || Rp1PioIsActive();
}

bool Rp1BackendInitIfNeeded(const HardwareMapping &mapping, int double_rows,
                            int parallel, int pwm_lsb_nanoseconds,
                            int dither_bits, int row_address_type,
                            const char *panel_type) {
  if (spwm_is_panel_type(panel_type)) return false;

  if (Rp1RioShouldActivate(mapping.name, row_address_type, parallel)) {
    Rp1RioInitOrDie(mapping, double_rows, parallel, pwm_lsb_nanoseconds,
                    dither_bits, row_address_type);
    return true;
  }

  if (Rp1PioShouldActivate(mapping.name, row_address_type, parallel)) {
    Rp1PioInitOrDie(mapping, double_rows, parallel, pwm_lsb_nanoseconds,
                    dither_bits, row_address_type);
    return true;
  }

  return false;
}

bool Rp1BackendInitializePanelsIfActive(const HardwareMapping &mapping,
                                        const char *panel_type, int columns) {
  if (Rp1RioIsActive()) {
    Rp1RioInitializePanels(mapping, panel_type, columns);
    return true;
  }
  if (Rp1PioIsActive()) {
    Rp1PioInitializePanels(mapping, panel_type, columns);
    return true;
  }
  return false;
}

bool Rp1BackendDumpFramebufferIfActive(Framebuffer *framebuffer,
                                       int pwm_low_bit) {
  if (Rp1RioIsActive()) {
    Rp1RioDumpFramebuffer(framebuffer, pwm_low_bit);
    return true;
  }
  if (Rp1PioIsActive()) {
    Rp1PioDumpFramebuffer(framebuffer, pwm_low_bit);
    return true;
  }
  return false;
}

void Rp1BackendDeinit() {
  Rp1RioDeinit();
  Rp1PioDeinit();
}

}  // namespace internal
}  // namespace rgb_matrix
