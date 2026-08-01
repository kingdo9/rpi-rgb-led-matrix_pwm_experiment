// -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
#ifndef RGBMATRIX_SPWM_REGISTER_TEST_H
#define RGBMATRIX_SPWM_REGISTER_TEST_H

#include <stddef.h>
#include <stdint.h>

namespace rgb_matrix {

class RGBMatrix;

namespace internal {

// Scene policy applied while one confirmed register profile remains active.
enum SPWM_Register_Test_Pattern {
  SPWM_REGISTER_TEST_PATTERN_GRADIENT = 0,
  SPWM_REGISTER_TEST_PATTERN_ALIGN,
  SPWM_REGISTER_TEST_PATTERN_CYCLE,
  SPWM_REGISTER_TEST_PATTERN_TEXTSCROLL,
  SPWM_REGISTER_TEST_PATTERN_TEAR,
};

// Catalog-backed snapshot of the last profile whose test scene was displayed.
// Keeping only its index lets Demo 15 defer the verbose result until after
// matrix teardown without retaining a pointer into refresh-thread state.
struct SPWM_Register_Test_Result {
  SPWM_Register_Test_Result()
      : final_output_ready(false), has_displayed_profile(false),
        profile_index(0) {}

  bool final_output_ready;
  bool has_displayed_profile;
  size_t profile_index;
};

// Return true when Demo 15 supports a runtime profile catalog for this panel.
bool SupportsSPWMRegisterTest(const char *panel_type);

// Parse "32", "1/32", or a comma-separated list into a 1-to-64 scan mask.
// A zero mask means that every extracted scan rate is eligible.
bool ParseSPWMRegisterTestScanFilter(const char *value,
                                     uint64_t *scan_filter);

// Navigate the selected panel's runtime register catalog until interrupted.
// TEXTSCROLL moves by text_scroll_step_pixels per presented refresh frame.
// text_scroll_middle_only keeps each new pass in the middle display band.
// TEAR moves its center-seam marker once per presented refresh frame.
// Return false when the catalog cannot be tested or a profile fails to apply.
bool RunSPWMRegisterTest(RGBMatrix *matrix, const char *panel_type,
                         SPWM_Register_Test_Pattern pattern,
                         int text_scroll_step_pixels,
                         bool text_scroll_middle_only,
                         uint64_t scan_filter,
                         volatile bool *interrupt_received,
                         SPWM_Register_Test_Result *result);

// Print the result captured by RunSPWMRegisterTest(). Call this only after the
// RGBMatrix has been destroyed so refresh diagnostics cannot follow it.
void PrintSPWMRegisterTestResult(const char *panel_type,
                                 const SPWM_Register_Test_Result &result);

}  // namespace internal
}  // namespace rgb_matrix

#endif  // RGBMATRIX_SPWM_REGISTER_TEST_H
