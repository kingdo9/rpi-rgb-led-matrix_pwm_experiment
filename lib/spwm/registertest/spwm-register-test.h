// -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
#ifndef RGBMATRIX_SPWM_REGISTER_TEST_H
#define RGBMATRIX_SPWM_REGISTER_TEST_H

#include <stdint.h>

namespace rgb_matrix {

class RGBMatrix;

namespace internal {

// Scene policy applied while one confirmed register profile remains active.
enum SPWM_Register_Test_Pattern {
  SPWM_REGISTER_TEST_PATTERN_GRADIENT = 0,
  SPWM_REGISTER_TEST_PATTERN_ALIGN,
  SPWM_REGISTER_TEST_PATTERN_CYCLE,
};

// Return true when Demo 15 has generated register profiles for this panel.
bool SupportsSPWMRegisterTest(const char *panel_type);

// Parse "32", "1/32", or a comma-separated list into a 1-to-64 scan mask.
// A zero mask means that every extracted scan rate is eligible.
bool ParseSPWMRegisterTestScanFilter(const char *value,
                                     uint64_t *scan_filter);

// Navigate the selected panel's generated register profiles until interrupted.
void RunSPWMRegisterTest(RGBMatrix *matrix, const char *panel_type,
                         SPWM_Register_Test_Pattern pattern,
                         uint64_t scan_filter,
                         volatile bool *interrupt_received);

}  // namespace internal
}  // namespace rgb_matrix

#endif  // RGBMATRIX_SPWM_REGISTER_TEST_H
