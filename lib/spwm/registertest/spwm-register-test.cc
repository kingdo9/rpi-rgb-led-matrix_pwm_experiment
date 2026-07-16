// -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
#include "spwm/registertest/spwm-register-test.h"

#include "graphics.h"
#include "led-matrix.h"
#include "spwm-helpers.h"
#include "spwm/registertest/fm6353/fm6353-register-profiles.generated.h"
#include "spwm/registertest/fm6363/fm6363-register-profiles.generated.h"
#include "spwm/registertest/fm6373/fm6373-register-profiles.generated.h"
#include "spwm/registertest/icnd1065l/icnd1065l-register-profiles.generated.h"
#include "spwm/registertest/sm16380sh/sm16380sh-register-profiles.generated.h"

#include <chrono>
#include <ctype.h>
#include <deque>
#include <poll.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <termios.h>
#include <unistd.h>
#include <vector>

namespace rgb_matrix {
namespace internal {
namespace {

const int kSceneDurationMs = 4000;

enum RegisterTestPanelFamily {
  REGISTER_TEST_PANEL_NONE = 0,
  REGISTER_TEST_PANEL_FM6353,
  REGISTER_TEST_PANEL_FM6363,
  REGISTER_TEST_PANEL_FM6373,
  REGISTER_TEST_PANEL_ICND1065L,
  REGISTER_TEST_PANEL_SM16380SH,
};

struct TinyGlyph {
  char character;
  uint8_t rows[5];
};

// One decoded terminal command. Commands are queued individually so repeated
// marks and mixed input such as RIGHT then M retain their original order.
struct RegisterTestInput {
  RegisterTestInput()
      : profile_step(0), toggle_mark(false), confirm_marks(false) {}

  bool HasAction() const {
    return profile_step != 0 || toggle_mark || confirm_marks;
  }

  int profile_step;
  bool toggle_mark;
  bool confirm_marks;
};

// Read arrows, M, and Enter without blocking the refresh demo. ISIG stays
// enabled so Ctrl-C continues to reach demo-main's signal handler.
class TerminalRegisterTestInput {
 public:
  TerminalRegisterTestInput() : enabled_(false), escape_state_(0) {
    if (!isatty(STDIN_FILENO) ||
        tcgetattr(STDIN_FILENO, &original_settings_) != 0) {
      return;
    }

    struct termios raw_settings = original_settings_;
    raw_settings.c_lflag &=
        ~static_cast<tcflag_t>(ICANON | ECHO);
    raw_settings.c_cc[VMIN] = 0;
    raw_settings.c_cc[VTIME] = 0;
    enabled_ =
        tcsetattr(STDIN_FILENO, TCSANOW, &raw_settings) == 0;
  }

  ~TerminalRegisterTestInput() {
    if (enabled_) {
      tcsetattr(STDIN_FILENO, TCSANOW, &original_settings_);
    }
  }

  bool enabled() const { return enabled_; }

  // Return one complete semantic command, retaining any later commands from
  // the same read for subsequent calls. Partial escape sequences survive split
  // reads but are cleared when polling reaches a deadline or input error.
  RegisterTestInput WaitForInput(int timeout_ms) {
    if (!pending_actions_.empty()) return TakeNextAction();

    RegisterTestInput input;
    if (!enabled_) {
      usleep(timeout_ms * 1000);
      return input;
    }

    const std::chrono::steady_clock::time_point deadline =
        std::chrono::steady_clock::now() +
        std::chrono::milliseconds(timeout_ms);
    while (true) {
      const std::chrono::steady_clock::time_point now =
          std::chrono::steady_clock::now();
      const int remaining_ms = static_cast<int>(
          std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now)
              .count());
      if (remaining_ms <= 0) {
        escape_state_ = 0;
        return input;
      }

      struct pollfd input_poll = {STDIN_FILENO, POLLIN, 0};
      const int poll_result = poll(&input_poll, 1, remaining_ms);
      if (poll_result <= 0) {
        escape_state_ = 0;
        return input;
      }
      if ((input_poll.revents & (POLLHUP | POLLERR | POLLNVAL)) != 0) {
        // A disconnected terminal can make poll() return immediately forever.
        // Retain the termios-restoration state but preserve the normal scene
        // cadence instead of spinning and redrawing continuously.
        escape_state_ = 0;
        usleep(remaining_ms * 1000);
        return input;
      }
      if ((input_poll.revents & POLLIN) == 0) continue;

      char input_bytes[32];
      const ssize_t input_count =
          read(STDIN_FILENO, input_bytes, sizeof(input_bytes));
      if (input_count <= 0) {
        escape_state_ = 0;
        return input;
      }

      for (ssize_t input_index = 0;
           input_index < input_count;
           ++input_index) {
        ConsumeByte(input_bytes[input_index]);
      }
      if (!pending_actions_.empty()) return TakeNextAction();
    }
  }

 private:
  RegisterTestInput TakeNextAction() {
    const RegisterTestInput input = pending_actions_.front();
    pending_actions_.pop_front();
    return input;
  }

  void QueueProfileStep(int profile_step) {
    RegisterTestInput input;
    input.profile_step = profile_step;
    pending_actions_.push_back(input);
  }

  void QueueMarkToggle() {
    RegisterTestInput input;
    input.toggle_mark = true;
    pending_actions_.push_back(input);
  }

  void QueueMarkConfirmation() {
    RegisterTestInput input;
    input.confirm_marks = true;
    pending_actions_.push_back(input);
  }

  // Decode common CSI/SS3 left and right arrow sequences while preserving a
  // partial escape sequence across terminal reads.
  void ConsumeByte(char input_byte) {
    const unsigned char byte = static_cast<unsigned char>(input_byte);
    if (escape_state_ == 0) {
      if (byte == 0x1b) {
        escape_state_ = 1;
      } else if (input_byte == 'm' || input_byte == 'M') {
        QueueMarkToggle();
      } else if (input_byte == '\r' || input_byte == '\n') {
        QueueMarkConfirmation();
      }
      return;
    }

    if (escape_state_ == 1) {
      if (input_byte == '[' || input_byte == 'O') {
        escape_state_ = 2;
      } else {
        escape_state_ = byte == 0x1b ? 1 : 0;
      }
      return;
    }

    if (input_byte == 'D') {
      escape_state_ = 0;
      QueueProfileStep(-1);
      return;
    }
    if (input_byte == 'C') {
      escape_state_ = 0;
      QueueProfileStep(1);
      return;
    }

    // Keep consuming numeric modifier parameters such as ESC [ 1 ; 5 C.
    if ((input_byte >= '0' && input_byte <= '9') || input_byte == ';') {
      return;
    }
    escape_state_ = byte == 0x1b ? 1 : 0;
  }

  bool enabled_;
  int escape_state_;
  struct termios original_settings_;
  std::deque<RegisterTestInput> pending_actions_;
};

// Three-pixel-wide glyphs keep both profile-label lines compact and avoid a
// runtime dependency on an external BDF font.
const TinyGlyph kTinyGlyphs[] = {
    {' ', {0x0, 0x0, 0x0, 0x0, 0x0}},
    {'0', {0x7, 0x5, 0x5, 0x5, 0x7}},
    {'1', {0x2, 0x6, 0x2, 0x2, 0x7}},
    {'2', {0x7, 0x1, 0x7, 0x4, 0x7}},
    {'3', {0x7, 0x1, 0x7, 0x1, 0x7}},
    {'4', {0x5, 0x5, 0x7, 0x1, 0x1}},
    {'5', {0x7, 0x4, 0x7, 0x1, 0x7}},
    {'6', {0x7, 0x4, 0x7, 0x5, 0x7}},
    {'7', {0x7, 0x1, 0x2, 0x2, 0x2}},
    {'8', {0x7, 0x5, 0x7, 0x5, 0x7}},
    {'9', {0x7, 0x5, 0x7, 0x1, 0x7}},
    {'C', {0x7, 0x4, 0x4, 0x4, 0x7}},
    {'D', {0x6, 0x5, 0x5, 0x5, 0x6}},
    {'E', {0x7, 0x4, 0x6, 0x4, 0x7}},
    {'F', {0x7, 0x4, 0x6, 0x4, 0x4}},
    {'G', {0x7, 0x4, 0x5, 0x5, 0x7}},
    {'H', {0x5, 0x5, 0x7, 0x5, 0x5}},
    {'I', {0x7, 0x2, 0x2, 0x2, 0x7}},
    {'L', {0x4, 0x4, 0x4, 0x4, 0x7}},
    {'M', {0x5, 0x7, 0x7, 0x5, 0x5}},
    {'N', {0x5, 0x7, 0x7, 0x7, 0x5}},
    {'P', {0x6, 0x5, 0x6, 0x4, 0x4}},
    {'R', {0x6, 0x5, 0x6, 0x5, 0x5}},
    {'S', {0x7, 0x4, 0x7, 0x1, 0x7}},
    {'T', {0x7, 0x2, 0x2, 0x2, 0x2}},
    {'Y', {0x5, 0x5, 0x2, 0x2, 0x2}},
};

// Panel-type matching intentionally accepts case-insensitive suffix variants,
// mirroring the core SPWM profile lookup (for example "fm6373-something").
RegisterTestPanelFamily GetRegisterTestPanelFamily(const char *panel_type) {
  if (panel_type == nullptr) return REGISTER_TEST_PANEL_NONE;
  if (strncasecmp(panel_type, "fm6353", 6) == 0) {
    return REGISTER_TEST_PANEL_FM6353;
  }
  if (strncasecmp(panel_type, "fm6363", 6) == 0) {
    return REGISTER_TEST_PANEL_FM6363;
  }
  if (strncasecmp(panel_type, "fm6373", 6) == 0) {
    return REGISTER_TEST_PANEL_FM6373;
  }
  if (strncasecmp(panel_type, "icnd1065l", 9) == 0) {
    return REGISTER_TEST_PANEL_ICND1065L;
  }
  if (strncasecmp(panel_type, "sm16380sh", 9) == 0) {
    return REGISTER_TEST_PANEL_SM16380SH;
  }
  return REGISTER_TEST_PANEL_NONE;
}

// Bit N-1 represents scan rate 1/N. Zero is reserved for the unfiltered "all"
// selection, so valid input must otherwise set at least one bit.
bool ParseRegisterTestScanFilterValue(const char *value,
                                      uint64_t *scan_filter) {
  if (value == nullptr || scan_filter == nullptr || *value == '\0') {
    return false;
  }
  if (strcasecmp(value, "all") == 0) {
    *scan_filter = 0;
    return true;
  }

  uint64_t parsed_filter = 0;
  const char *cursor = value;
  while (*cursor != '\0') {
    while (isspace(static_cast<unsigned char>(*cursor))) ++cursor;
    if (cursor[0] == '1' && cursor[1] == '/') cursor += 2;
    if (!isdigit(static_cast<unsigned char>(*cursor))) return false;

    char *number_end = nullptr;
    const long scan_rows = strtol(cursor, &number_end, 10);
    if (number_end == cursor || scan_rows < 1 || scan_rows > 64) return false;
    parsed_filter |= static_cast<uint64_t>(1ULL << (scan_rows - 1));
    cursor = number_end;

    while (isspace(static_cast<unsigned char>(*cursor))) ++cursor;
    if (*cursor == '\0') break;
    if (*cursor != ',') return false;
    ++cursor;
    while (isspace(static_cast<unsigned char>(*cursor))) ++cursor;
    if (*cursor == '\0') return false;
  }

  *scan_filter = parsed_filter;
  return parsed_filter != 0;
}

// Merged profiles can list several scan rates in one metadata string. Include
// the profile when any embedded 1-to-64 value is selected by the filter.
bool ScanMetadataMatchesFilter(const char *scan_metadata,
                               uint64_t scan_filter) {
  if (scan_filter == 0) return true;
  if (scan_metadata == nullptr) return false;

  const char *cursor = scan_metadata;
  while (*cursor != '\0') {
    while (*cursor != '\0' &&
           !isdigit(static_cast<unsigned char>(*cursor))) {
      ++cursor;
    }
    if (*cursor == '\0') break;

    char *number_end = nullptr;
    const long scan_rows = strtol(cursor, &number_end, 10);
    if (number_end == cursor) return false;
    if (scan_rows >= 1 && scan_rows <= 64 &&
        (scan_filter & static_cast<uint64_t>(1ULL << (scan_rows - 1))) != 0) {
      return true;
    }
    cursor = number_end;
  }
  return false;
}

// Preserve generated-table indices so filtered navigation still displays and
// reports the original regtype number rather than a filtered-list offset.
template <typename Profile>
std::vector<size_t> CollectMatchingProfileIndices(
    const Profile *profiles, size_t profile_count, uint64_t scan_filter) {
  std::vector<size_t> profile_indices;
  for (size_t profile_index = 0;
       profile_index < profile_count;
       ++profile_index) {
    if (ScanMetadataMatchesFilter(profiles[profile_index].scan_type,
                                  scan_filter)) {
      profile_indices.push_back(profile_index);
    }
  }
  return profile_indices;
}

void PrintScanFilter(uint64_t scan_filter) {
  if (scan_filter == 0) {
    printf("ALL");
    return;
  }

  bool printed_scan = false;
  for (int scan_rows = 1; scan_rows <= 64; ++scan_rows) {
    if ((scan_filter & static_cast<uint64_t>(1ULL << (scan_rows - 1))) == 0) {
      continue;
    }
    printf("%s1/%d", printed_scan ? "," : "", scan_rows);
    printed_scan = true;
  }
}

// Print the selected/total count once and prevent a runner from indexing an
// empty filtered list.
bool PrintProfileFilterSummary(const char *panel_label,
                               size_t total_profile_count,
                               const std::vector<size_t> &profile_indices,
                               uint64_t scan_filter) {
  printf("%s register test: %zu of %zu unique profiles selected; scan filter: ",
         panel_label, profile_indices.size(), total_profile_count);
  PrintScanFilter(scan_filter);
  printf(".\n");
  if (profile_indices.empty()) {
    printf("No %s register profiles match the requested scan filter.\n",
           panel_label);
  }
  fflush(stdout);
  return !profile_indices.empty();
}

bool WasInterrupted(const volatile bool *interrupt_received) {
  return interrupt_received != nullptr && *interrupt_received;
}

// ---------------------------
// Compact on-panel test scenes
// ---------------------------
const uint8_t *FindTinyGlyph(char character) {
  for (size_t glyph_index = 0;
       glyph_index < sizeof(kTinyGlyphs) / sizeof(kTinyGlyphs[0]);
       ++glyph_index) {
    if (kTinyGlyphs[glyph_index].character == character) {
      return kTinyGlyphs[glyph_index].rows;
    }
  }
  return kTinyGlyphs[0].rows;
}

int TinyTextWidth(const char *text) {
  const size_t character_count = text != nullptr ? strlen(text) : 0;
  return character_count == 0 ? 0 : static_cast<int>(character_count * 4 - 1);
}

void FillRectangle(Canvas *canvas, int x, int y, int width, int height,
                   const Color &color) {
  for (int draw_y = y; draw_y < y + height; ++draw_y) {
    for (int draw_x = x; draw_x < x + width; ++draw_x) {
      canvas->SetPixel(draw_x, draw_y, color.r, color.g, color.b);
    }
  }
}

void DrawTinyText(Canvas *canvas, int x, int y, const char *text,
                  const Color &color) {
  if (text == nullptr) return;

  for (const char *cursor = text; *cursor != '\0'; ++cursor, x += 4) {
    const uint8_t *rows = FindTinyGlyph(*cursor);
    for (int row = 0; row < 5; ++row) {
      for (int column = 0; column < 3; ++column) {
        if ((rows[row] & (1u << (2 - column))) != 0) {
          canvas->SetPixel(x + column, y + row,
                           color.r, color.g, color.b);
        }
      }
    }
  }
}

void DrawProfileLabel(Canvas *canvas, const char *panel_label,
                      size_t one_based_profile_index) {
  char profile_label[32];
  snprintf(profile_label, sizeof(profile_label), "REG %zu",
           one_based_profile_index);

  const int label_width =
      TinyTextWidth(panel_label) > TinyTextWidth(profile_label)
          ? TinyTextWidth(panel_label)
          : TinyTextWidth(profile_label);
  FillRectangle(canvas, 1, 1, label_width + 2, 13, Color(0, 0, 0));
  DrawTinyText(canvas, 2, 2, panel_label, Color(255, 255, 255));
  DrawTinyText(canvas, 2, 8, profile_label, Color(255, 255, 0));
}

void DrawAlignmentScene(Canvas *canvas, const char *panel_label,
                        size_t one_based_profile_index) {
  const int width = canvas->width() - 1;
  const int height = canvas->height() - 1;
  canvas->Clear();
  if (width < 0 || height < 0) return;

  // Keep the Demo 3 test pattern unchanged: four colored borders and two
  // diagonals make row, column, and RGB-lane alignment faults easy to spot.
  DrawLine(canvas, 0, 0,      width, 0,      Color(255, 0, 0));
  DrawLine(canvas, 0, height, width, height, Color(255, 255, 0));
  DrawLine(canvas, 0, 0,      0,     height, Color(0, 0, 255));
  DrawLine(canvas, width, 0,  width, height, Color(0, 255, 0));
  DrawLine(canvas, 0, 0,      width, height, Color(255, 255, 255));
  DrawLine(canvas, 0, height, width, 0,      Color(255, 0, 255));

  DrawProfileLabel(canvas, panel_label, one_based_profile_index);
}

void HueColor(int hue, int *red, int *green, int *blue) {
  const int segment = hue / 256;
  const int offset = hue % 256;
  switch (segment) {
    case 0: *red = 255;          *green = offset;       *blue = 0; break;
    case 1: *red = 255 - offset; *green = 255;          *blue = 0; break;
    case 2: *red = 0;            *green = 255;          *blue = offset; break;
    case 3: *red = 0;            *green = 255 - offset; *blue = 255; break;
    case 4: *red = offset;       *green = 0;            *blue = 255; break;
    default:
      *red = 255;
      *green = 0;
      *blue = 255 - offset;
      break;
  }
}

void DrawGradientScene(Canvas *canvas, const char *panel_label,
                       size_t one_based_profile_index) {
  const int width = canvas->width();
  const int height = canvas->height();
  canvas->Clear();
  if (width <= 0 || height <= 0) return;

  for (int y = 0; y < height; ++y) {
    const int brightness =
        height > 1 ? 255 * (height - 1 - y) / (height - 1) : 255;
    for (int x = 0; x < width; ++x) {
      const int hue = width > 1 ? x * 1535 / (width - 1) : 0;
      int red = 0;
      int green = 0;
      int blue = 0;
      HueColor(hue, &red, &green, &blue);
      canvas->SetPixel(x, y,
                       red * brightness / 255,
                       green * brightness / 255,
                       blue * brightness / 255);
    }
  }

  DrawProfileLabel(canvas, panel_label, one_based_profile_index);
}

void ShowProfileScene(RGBMatrix *matrix, FrameCanvas **offscreen,
                      const char *panel_label, size_t profile_index,
                      bool show_gradient) {
  if (show_gradient) {
    DrawGradientScene(*offscreen, panel_label, profile_index + 1);
  } else {
    DrawAlignmentScene(*offscreen, panel_label, profile_index + 1);
  }
  *offscreen = matrix->SwapOnVSync(*offscreen);
}

// Wrap an arbitrary signed movement across a non-empty profile list.
size_t MoveProfileIndex(size_t profile_index, size_t profile_count,
                        int profile_step) {
  if (profile_count == 0 || profile_step == 0) return profile_index;
  if (profile_step > 0) {
    return (profile_index + static_cast<size_t>(profile_step) % profile_count) %
           profile_count;
  }
  const size_t backward_step =
      static_cast<size_t>(-profile_step) % profile_count;
  return (profile_index + profile_count - backward_step) % profile_count;
}

// Translate a generated-table index into a position in a filtered/finalist
// list, then move with wraparound while returning the original table index.
size_t MoveWithinProfileIndices(
    size_t profile_index, const std::vector<size_t> &profile_indices,
    int profile_step) {
  if (profile_indices.empty() || profile_step == 0) return profile_index;

  size_t position = 0;
  while (position < profile_indices.size() &&
         profile_indices[position] != profile_index) {
    ++position;
  }
  if (position == profile_indices.size()) return profile_indices[0];
  return profile_indices[MoveProfileIndex(position, profile_indices.size(),
                                          profile_step)];
}

// Before Enter, navigation uses the scan-eligible set and marks are mutable.
// After Enter, the marked indices become an immutable finalist set.
class RegisterTestSelection {
 public:
  RegisterTestSelection(size_t profile_count,
                        const std::vector<size_t> &eligible_indices)
      : marked_profiles_(profile_count, 0),
        eligible_indices_(eligible_indices), finalized_(false) {}

  bool IsMarked(size_t profile_index) const {
    return profile_index < marked_profiles_.size() &&
           marked_profiles_[profile_index] != 0;
  }

  size_t MarkedCount() const {
    size_t marked_count = 0;
    for (size_t profile_index = 0;
         profile_index < marked_profiles_.size();
         ++profile_index) {
      if (marked_profiles_[profile_index] != 0) ++marked_count;
    }
    return marked_count;
  }

  void ToggleMark(size_t profile_index, const char *profile_name) {
    if (finalized_) {
      printf("\n[M] Final selection is locked; marks can no longer change.\n");
      fflush(stdout);
      return;
    }
    if (profile_index >= marked_profiles_.size()) return;

    marked_profiles_[profile_index] = marked_profiles_[profile_index] == 0;
    printf("\n>>> [M] %s: %s (%zu marked good) <<<\n",
           marked_profiles_[profile_index] != 0 ? "MARKED GOOD" : "UNMARKED",
           profile_name != nullptr ? profile_name : "unknown",
           MarkedCount());
    fflush(stdout);
  }

  bool Finalize(size_t profile_index, size_t *selected_profile_index) {
    if (finalized_) {
      printf("\n[ENTER] Final selection is already locked.\n");
      fflush(stdout);
      return false;
    }

    finalist_indices_.clear();
    for (size_t index = 0; index < marked_profiles_.size(); ++index) {
      if (marked_profiles_[index] != 0) finalist_indices_.push_back(index);
    }
    if (finalist_indices_.empty()) {
      printf("\n!!! [ENTER] No profiles are marked. Press [M] on at least one "
             "good profile first. !!!\n");
      fflush(stdout);
      return false;
    }

    finalized_ = true;
    if (selected_profile_index != nullptr) {
      *selected_profile_index =
          IsMarked(profile_index) ? profile_index : finalist_indices_[0];
    }
    printf("\n============================================================\n"
           " FINAL SELECTION LOCKED: %zu marked-good profile%s\n",
           finalist_indices_.size(),
           finalist_indices_.size() == 1 ? "" : "s");
    printf(" PROFILE NUMBERS:");
    for (size_t finalist_index = 0;
         finalist_index < finalist_indices_.size();
         ++finalist_index) {
      printf(" %zu", finalist_indices_[finalist_index] + 1);
    }
    printf("\n"
           " LEFT/RIGHT now moves only between these finalists.\n"
           " CTRL-C prints the CLI config for the displayed finalist.\n"
           "============================================================\n");
    fflush(stdout);
    return true;
  }

  size_t Move(size_t profile_index, int profile_step) const {
    if (!finalized_) {
      return MoveWithinProfileIndices(profile_index, eligible_indices_,
                                      profile_step);
    }
    if (finalist_indices_.empty() || profile_step == 0) return profile_index;

    size_t finalist_position = 0;
    while (finalist_position < finalist_indices_.size() &&
           finalist_indices_[finalist_position] != profile_index) {
      ++finalist_position;
    }
    if (finalist_position == finalist_indices_.size()) {
      return finalist_indices_[0];
    }
    const size_t next_position = MoveProfileIndex(
        finalist_position, finalist_indices_.size(), profile_step);
    return finalist_indices_[next_position];
  }

  void PrintStatus(size_t profile_index) const {
    printf("  selection: %s; marked good: %zu; current: %s\n",
           finalized_ ? "FINALISTS ONLY"
                      : (eligible_indices_.size() == marked_profiles_.size()
                             ? "ALL PROFILES"
                             : "SCAN FILTERED"),
           MarkedCount(), IsMarked(profile_index) ? "MARKED" : "not marked");
    fflush(stdout);
  }

 private:
  std::vector<uint8_t> marked_profiles_;
  std::vector<size_t> eligible_indices_;
  std::vector<size_t> finalist_indices_;
  bool finalized_;
};

const char *RegisterTestPatternDescription(
    SPWM_Register_Test_Pattern pattern) {
  switch (pattern) {
    case SPWM_REGISTER_TEST_PATTERN_ALIGN:
      return "ALIGNMENT ONLY";
    case SPWM_REGISTER_TEST_PATTERN_CYCLE:
      return "ALIGNMENT + COLOR GRADIENT (alternates every 4 seconds)";
    case SPWM_REGISTER_TEST_PATTERN_GRADIENT:
    default:
      return "COLOR GRADIENT ONLY";
  }
}

// Print the controls prominently before any large per-profile metadata output.
void PrintRegisterTestControls(SPWM_Register_Test_Pattern pattern,
                               uint64_t scan_filter) {
  printf("\n"
         "============================================================\n"
         "                 REGISTER TEST CONTROLS\n");
  printf("  >>> DISPLAY PATTERN: %s <<<\n",
         RegisterTestPatternDescription(pattern));
  printf("  >>> SCAN FILTER: ");
  PrintScanFilter(scan_filter);
  printf(" <<<\n");
  printf("  LEFT / RIGHT : previous or next register profile\n"
         "  [M]          : mark/unmark the displayed profile as good\n"
         "  [ENTER]      : lock marks and browse only the finalists\n"
         "  [CTRL-C]     : quit and print the displayed CLI config\n"
         "============================================================\n\n");
  fflush(stdout);
}

// Handle rendering and terminal interaction only after the refresh thread has
// confirmed the selected register payload. Marking does not reload it;
// navigation returns the next index for the caller to queue and confirm.
size_t WaitForProfileInteraction(
    RGBMatrix *matrix, FrameCanvas **offscreen,
    TerminalRegisterTestInput *terminal_input, const char *panel_label,
    const char *profile_name, size_t profile_index,
    RegisterTestSelection *selection, SPWM_Register_Test_Pattern pattern,
    volatile bool *interrupt_received) {
  bool show_gradient = pattern != SPWM_REGISTER_TEST_PATTERN_ALIGN;
  const bool alternate_scenes =
      pattern == SPWM_REGISTER_TEST_PATTERN_CYCLE;
  while (!WasInterrupted(interrupt_received)) {
    ShowProfileScene(matrix, offscreen, panel_label, profile_index,
                     show_gradient);
    const RegisterTestInput input =
        terminal_input->WaitForInput(kSceneDurationMs);
    // A signal can arrive while poll() is waiting or while decoded actions
    // remain queued. Do not mutate the selection after the user asked to quit.
    if (WasInterrupted(interrupt_received)) break;
    if (!input.HasAction()) {
      if (alternate_scenes) show_gradient = !show_gradient;
      continue;
    }

    if (input.toggle_mark) {
      selection->ToggleMark(profile_index, profile_name);
    }
    if (input.confirm_marks) {
      size_t selected_profile_index = profile_index;
      if (selection->Finalize(profile_index, &selected_profile_index) &&
          selected_profile_index != profile_index) {
        return selected_profile_index;
      }
      continue;
    }
    if (input.profile_step != 0) {
      const size_t next_profile_index =
          selection->Move(profile_index, input.profile_step);
      if (next_profile_index != profile_index) return next_profile_index;
    }
  }
  return profile_index;
}

// A rotating profile completes after its entire word sequence has been sent
// across consecutive init sequences. Stop early if the refresh thread rejects
// it as incompatible with the active panel instead of waiting indefinitely.
bool WaitUntilRGBProfileEmitted(
    const SPWM_RGB_Register_Profile_View *requested_profile,
    volatile bool *interrupt_received) {
  while (!WasInterrupted(interrupt_received)) {
    if (spwm_get_last_emitted_rgb_register_profile() == requested_profile) {
      return true;
    }
    if (spwm_get_last_rejected_rgb_register_profile() == requested_profile) {
      fprintf(stderr,
              "Refresh thread rejected RGB register profile '%s' for the "
              "active panel config.\n",
              requested_profile->name);
      return false;
    }
    usleep(1000);
  }
  return false;
}

// A fixed profile completes once every selected fixed slot has appeared in a
// completed init sequence. Report active-panel incompatibility immediately.
bool WaitUntilFixedProfileEmitted(
    const SPWM_Fixed_Register_Profile_View *requested_profile,
    volatile bool *interrupt_received) {
  while (!WasInterrupted(interrupt_received)) {
    if (spwm_get_last_emitted_fixed_register_profile() == requested_profile) {
      return true;
    }
    if (spwm_get_last_rejected_fixed_register_profile() == requested_profile) {
      fprintf(stderr,
              "Refresh thread rejected fixed register profile '%s' for the "
              "active panel config.\n",
              requested_profile->name);
      return false;
    }
    usleep(1000);
  }
  return false;
}

void PrintFM6373ProfileSummary(const FM6373_Register_Test_Profile &profile,
                               size_t profile_index) {
  printf("\n[%zu/%zu] Testing %s\n",
         profile_index + 1, FM6373_REGISTER_TEST_PROFILE_COUNT,
         profile.register_profile.name);
  printf("  source: %s\n", profile.source_path);
  printf("  register: %zu, chip code: %d, scan: %s, duplicate sources: %zu\n",
         profile.register_profile.register_index, profile.chip_code,
         profile.scan_type, profile.duplicate_source_count);
  printf("  use the controls above to evaluate this profile\n");
  fflush(stdout);
}

// Locate metadata by view identity; generated tables use static-lifetime views
// so pointer identity is the unambiguous link between runtime and Demo 15 data.
const FM6373_Register_Test_Profile *FindFM6373GeneratedProfile(
    const SPWM_RGB_Register_Profile_View *register_profile,
    size_t *profile_index) {
  for (size_t index = 0; index < FM6373_REGISTER_TEST_PROFILE_COUNT; ++index) {
    if (&FM6373_REGISTER_TEST_PROFILES[index].register_profile ==
        register_profile) {
      if (profile_index != nullptr) *profile_index = index;
      return &FM6373_REGISTER_TEST_PROFILES[index];
    }
  }
  return nullptr;
}

void PrintRGBChannelArray(const SPWM_RGB_Register_Profile_View &profile,
                          size_t channel_index, char channel_suffix) {
  printf("static const uint16_t %s_%c[] = {\n",
         profile.name, channel_suffix);
  for (size_t word_index = 0;
       word_index < profile.channel_word_counts[channel_index];
       ++word_index) {
    if (word_index % 8 == 0) printf("    ");
    printf("0x%04x", static_cast<unsigned int>(
                          profile.channel_words[channel_index][word_index]));
    if (word_index + 1 < profile.channel_word_counts[channel_index]) {
      printf(",");
    }
    if (word_index % 8 == 7 ||
        word_index + 1 == profile.channel_word_counts[channel_index]) {
      printf("\n");
    } else {
      printf(" ");
    }
  }
  printf("};\n\n");
}

// Print a shell-compatible comma-separated word list. Newlines are safe while
// the caller keeps the entire option value inside quotes.
void PrintCLIWordList(const uint16_t *words, size_t word_count) {
  // The wrapping newlines stay inside the quoted option value, where the
  // force-register parser treats them as whitespace.
  for (size_t word_index = 0; word_index < word_count; ++word_index) {
    if (word_index > 0) {
      printf("%s", word_index % 8 == 0 ? ",\n    " : ", ");
    }
    printf("0x%04x", static_cast<unsigned int>(words[word_index]));
  }
}

void PrintRGBCLIOption(const SPWM_RGB_Register_Profile_View &profile,
                       bool use_numbered_option) {
  printf("--led-spwm-force-register");
  if (use_numbered_option) printf("%zu", profile.register_index);
  printf("=\"R:");
  PrintCLIWordList(profile.channel_words[0], profile.channel_word_counts[0]);
  printf(";\n    G:");
  PrintCLIWordList(profile.channel_words[1], profile.channel_word_counts[1]);
  printf(";\n    B:");
  PrintCLIWordList(profile.channel_words[2], profile.channel_word_counts[2]);
  printf("\"\n");
}

// Generated tables are zero-based; the CLI and regtype names are one-based.
void PrintGeneratedProfileCLISelector(size_t generated_profile_index) {
  printf("Short CLI selector for this generated profile:\n"
         "--led-spwm-register-config=%zu\n\n",
         generated_profile_index + 1);
}

void PrintRGBProfileConfig(const SPWM_RGB_Register_Profile_View &profile) {
  printf("Register slot: %zu\n\n", profile.register_index);

  PrintRGBChannelArray(profile, 0, 'r');
  PrintRGBChannelArray(profile, 1, 'g');
  PrintRGBChannelArray(profile, 2, 'b');

  printf("To reproduce the full payload explicitly, append either force "
         "option below to your normal panel command:\n\n");
  PrintRGBCLIOption(profile, false);
  printf("\n");
  PrintRGBCLIOption(profile, true);
}

// Use only the last refresh-confirmed pointer so quitting during a new profile
// upload reports the previous complete payload rather than a partial request.
void PrintLastEmittedFM6373Profile() {
  const SPWM_RGB_Register_Profile_View *const last_profile =
      spwm_get_last_emitted_rgb_register_profile();
  if (last_profile == nullptr) {
    printf("\nNo FM6373 register profile was confirmed as emitted.\n");
    return;
  }

  size_t generated_profile_index = 0;
  const FM6373_Register_Test_Profile *const generated_profile =
      FindFM6373GeneratedProfile(last_profile, &generated_profile_index);
  printf("\nConfirmed last-emitted FM6373 register profile: %s",
         last_profile->name);
  if (generated_profile != nullptr) {
    printf(" (%zu/%zu)\n", generated_profile_index + 1,
           FM6373_REGISTER_TEST_PROFILE_COUNT);
    printf("Source: %s\n", generated_profile->source_path);
    PrintGeneratedProfileCLISelector(generated_profile_index);
  } else {
    printf("\n");
  }
  PrintRGBProfileConfig(*last_profile);
  fflush(stdout);
}

// Every family runner follows the same state transition: filter candidates,
// queue one static profile, wait for confirmed emission, then render and accept
// input. On exit, report the last fully emitted profile rather than a merely
// queued selection.
void RunFM6373Profiles(RGBMatrix *matrix,
                       TerminalRegisterTestInput *terminal_input,
                       SPWM_Register_Test_Pattern pattern,
                       uint64_t scan_filter,
                       volatile bool *interrupt_received) {
  const std::vector<size_t> profile_indices = CollectMatchingProfileIndices(
      FM6373_REGISTER_TEST_PROFILES, FM6373_REGISTER_TEST_PROFILE_COUNT,
      scan_filter);
  if (!PrintProfileFilterSummary("FM6373",
                                 FM6373_REGISTER_TEST_PROFILE_COUNT,
                                 profile_indices, scan_filter)) {
    return;
  }

  FrameCanvas *offscreen = matrix->CreateFrameCanvas();
  size_t profile_index = profile_indices[0];
  RegisterTestSelection selection(FM6373_REGISTER_TEST_PROFILE_COUNT,
                                  profile_indices);

  while (!WasInterrupted(interrupt_received)) {
    const FM6373_Register_Test_Profile &profile =
        FM6373_REGISTER_TEST_PROFILES[profile_index];
    PrintFM6373ProfileSummary(profile, profile_index);
    selection.PrintStatus(profile_index);

    if (!spwm_request_rgb_register_profile(&profile.register_profile)) {
      fprintf(stderr, "Unable to queue FM6373 register profile '%s'.\n",
              profile.register_profile.name);
      break;
    }
    if (!WaitUntilRGBProfileEmitted(&profile.register_profile,
                                    interrupt_received)) {
      break;
    }
    profile_index = WaitForProfileInteraction(
        matrix, &offscreen, terminal_input, "FM6373",
        profile.register_profile.name, profile_index, &selection,
        pattern, interrupt_received);
  }

  PrintLastEmittedFM6373Profile();
}

// ICND1065L and SM16380SH reuse the rotating-profile flow above but retain
// driver-specific metadata, table identity, and stop-report labels.
void PrintICND1065LProfileSummary(
    const ICND1065L_Register_Test_Profile &profile, size_t profile_index) {
  printf("\n[%zu/%zu] Testing %s\n",
         profile_index + 1, ICND1065L_REGISTER_TEST_PROFILE_COUNT,
         profile.register_profile.name);
  printf("  source: %s\n", profile.source_path);
  printf("  register: %zu, chip code: %d, scan: %s, duplicate sources: %zu\n",
         profile.register_profile.register_index, profile.chip_code,
         profile.scan_type, profile.duplicate_source_count);
  printf("  use the controls above to evaluate this profile\n");
  fflush(stdout);
}

const ICND1065L_Register_Test_Profile *FindICND1065LGeneratedProfile(
    const SPWM_RGB_Register_Profile_View *register_profile,
    size_t *profile_index) {
  for (size_t index = 0;
       index < ICND1065L_REGISTER_TEST_PROFILE_COUNT;
       ++index) {
    if (&ICND1065L_REGISTER_TEST_PROFILES[index].register_profile ==
        register_profile) {
      if (profile_index != nullptr) *profile_index = index;
      return &ICND1065L_REGISTER_TEST_PROFILES[index];
    }
  }
  return nullptr;
}

void PrintLastEmittedICND1065LProfile() {
  const SPWM_RGB_Register_Profile_View *const last_profile =
      spwm_get_last_emitted_rgb_register_profile();
  if (last_profile == nullptr) {
    printf("\nNo ICND1065L register profile was confirmed as emitted.\n");
    return;
  }

  size_t generated_profile_index = 0;
  const ICND1065L_Register_Test_Profile *const generated_profile =
      FindICND1065LGeneratedProfile(last_profile, &generated_profile_index);
  printf("\nConfirmed last-emitted ICND1065L register profile: %s",
         last_profile->name);
  if (generated_profile != nullptr) {
    printf(" (%zu/%zu)\n", generated_profile_index + 1,
           ICND1065L_REGISTER_TEST_PROFILE_COUNT);
    printf("Source: %s\n", generated_profile->source_path);
    PrintGeneratedProfileCLISelector(generated_profile_index);
  } else {
    printf("\n");
  }
  PrintRGBProfileConfig(*last_profile);
  fflush(stdout);
}

void RunICND1065LProfiles(RGBMatrix *matrix,
                          TerminalRegisterTestInput *terminal_input,
                          SPWM_Register_Test_Pattern pattern,
                          uint64_t scan_filter,
                          volatile bool *interrupt_received) {
  const std::vector<size_t> profile_indices = CollectMatchingProfileIndices(
      ICND1065L_REGISTER_TEST_PROFILES,
      ICND1065L_REGISTER_TEST_PROFILE_COUNT, scan_filter);
  if (!PrintProfileFilterSummary("ICND1065L",
                                 ICND1065L_REGISTER_TEST_PROFILE_COUNT,
                                 profile_indices, scan_filter)) {
    return;
  }

  FrameCanvas *offscreen = matrix->CreateFrameCanvas();
  size_t profile_index = profile_indices[0];
  RegisterTestSelection selection(ICND1065L_REGISTER_TEST_PROFILE_COUNT,
                                  profile_indices);

  while (!WasInterrupted(interrupt_received)) {
    const ICND1065L_Register_Test_Profile &profile =
        ICND1065L_REGISTER_TEST_PROFILES[profile_index];
    PrintICND1065LProfileSummary(profile, profile_index);
    selection.PrintStatus(profile_index);

    if (!spwm_request_rgb_register_profile(&profile.register_profile)) {
      fprintf(stderr, "Unable to queue ICND1065L register profile '%s'.\n",
              profile.register_profile.name);
      break;
    }
    if (!WaitUntilRGBProfileEmitted(&profile.register_profile,
                                    interrupt_received)) {
      break;
    }
    profile_index = WaitForProfileInteraction(
        matrix, &offscreen, terminal_input, "ICND1065L",
        profile.register_profile.name, profile_index, &selection,
        pattern, interrupt_received);
  }

  PrintLastEmittedICND1065LProfile();
}

void PrintSM16380SHProfileSummary(
    const SM16380SH_Register_Test_Profile &profile, size_t profile_index) {
  printf("\n[%zu/%zu] Testing %s\n",
         profile_index + 1, SM16380SH_REGISTER_TEST_PROFILE_COUNT,
         profile.register_profile.name);
  printf("  source: %s\n", profile.source_path);
  printf("  register: %zu, chip code: %d, scan: %s, duplicate sources: %zu\n",
         profile.register_profile.register_index, profile.chip_code,
         profile.scan_type, profile.duplicate_source_count);
  printf("  use the controls above to evaluate this profile\n");
  fflush(stdout);
}

const SM16380SH_Register_Test_Profile *FindSM16380SHGeneratedProfile(
    const SPWM_RGB_Register_Profile_View *register_profile,
    size_t *profile_index) {
  for (size_t index = 0;
       index < SM16380SH_REGISTER_TEST_PROFILE_COUNT;
       ++index) {
    if (&SM16380SH_REGISTER_TEST_PROFILES[index].register_profile ==
        register_profile) {
      if (profile_index != nullptr) *profile_index = index;
      return &SM16380SH_REGISTER_TEST_PROFILES[index];
    }
  }
  return nullptr;
}

void PrintLastEmittedSM16380SHProfile() {
  const SPWM_RGB_Register_Profile_View *const last_profile =
      spwm_get_last_emitted_rgb_register_profile();
  if (last_profile == nullptr) {
    printf("\nNo SM16380SH register profile was confirmed as emitted.\n");
    return;
  }

  size_t generated_profile_index = 0;
  const SM16380SH_Register_Test_Profile *const generated_profile =
      FindSM16380SHGeneratedProfile(last_profile, &generated_profile_index);
  printf("\nConfirmed last-emitted SM16380SH register profile: %s",
         last_profile->name);
  if (generated_profile != nullptr) {
    printf(" (%zu/%zu)\n", generated_profile_index + 1,
           SM16380SH_REGISTER_TEST_PROFILE_COUNT);
    printf("Source: %s\n", generated_profile->source_path);
    PrintGeneratedProfileCLISelector(generated_profile_index);
  } else {
    printf("\n");
  }
  PrintRGBProfileConfig(*last_profile);
  fflush(stdout);
}

void RunSM16380SHProfiles(RGBMatrix *matrix,
                          TerminalRegisterTestInput *terminal_input,
                          SPWM_Register_Test_Pattern pattern,
                          uint64_t scan_filter,
                          volatile bool *interrupt_received) {
  const std::vector<size_t> profile_indices = CollectMatchingProfileIndices(
      SM16380SH_REGISTER_TEST_PROFILES,
      SM16380SH_REGISTER_TEST_PROFILE_COUNT, scan_filter);
  if (!PrintProfileFilterSummary("SM16380SH",
                                 SM16380SH_REGISTER_TEST_PROFILE_COUNT,
                                 profile_indices, scan_filter)) {
    return;
  }

  FrameCanvas *offscreen = matrix->CreateFrameCanvas();
  size_t profile_index = profile_indices[0];
  RegisterTestSelection selection(SM16380SH_REGISTER_TEST_PROFILE_COUNT,
                                  profile_indices);

  while (!WasInterrupted(interrupt_received)) {
    const SM16380SH_Register_Test_Profile &profile =
        SM16380SH_REGISTER_TEST_PROFILES[profile_index];
    PrintSM16380SHProfileSummary(profile, profile_index);
    selection.PrintStatus(profile_index);

    if (!spwm_request_rgb_register_profile(&profile.register_profile)) {
      fprintf(stderr, "Unable to queue SM16380SH register profile '%s'.\n",
              profile.register_profile.name);
      break;
    }
    if (!WaitUntilRGBProfileEmitted(&profile.register_profile,
                                    interrupt_received)) {
      break;
    }
    profile_index = WaitForProfileInteraction(
        matrix, &offscreen, terminal_input, "SM16380SH",
        profile.register_profile.name, profile_index, &selection,
        pattern, interrupt_received);
  }

  PrintLastEmittedSM16380SHProfile();
}

// Fixed-register families confirm a mask of emitted slots and print one
// numbered force option per slot instead of the rotating-register shortcut.
void PrintFM6363ProfileSummary(const FM6363_Register_Test_Profile &profile,
                               size_t profile_index) {
  printf("\n[%zu/%zu] Testing %s\n",
         profile_index + 1, FM6363_REGISTER_TEST_PROFILE_COUNT,
         profile.register_profile.name);
  printf("  source: %s\n", profile.source_path);
  printf("  fixed registers: %zu, driver: %s, scan: %s, "
         "duplicate sources: %zu, RegSixth sources omitted: %zu\n",
         profile.register_profile.entry_count,
         profile.source_driver_id, profile.scan_type,
         profile.duplicate_source_count,
         profile.ignored_reg_sixth_source_count);
  printf("  use the controls above to evaluate this profile\n");
  fflush(stdout);
}

const FM6363_Register_Test_Profile *FindFM6363GeneratedProfile(
    const SPWM_Fixed_Register_Profile_View *register_profile,
    size_t *profile_index) {
  for (size_t index = 0; index < FM6363_REGISTER_TEST_PROFILE_COUNT; ++index) {
    if (&FM6363_REGISTER_TEST_PROFILES[index].register_profile ==
        register_profile) {
      if (profile_index != nullptr) *profile_index = index;
      return &FM6363_REGISTER_TEST_PROFILES[index];
    }
  }
  return nullptr;
}

// Reconstruct the selected generated fixed-entry declaration for inspection.
void PrintFixedRegisterEntryArray(
    const SPWM_Fixed_Register_Profile_View &profile) {
  printf("static const SPWM_Fixed_Register_Profile_Entry %s_entries[] = {\n",
         profile.name);
  for (size_t entry_index = 0;
       entry_index < profile.entry_count;
       ++entry_index) {
    const SPWM_Fixed_Register_Profile_Entry &entry =
        profile.entries[entry_index];
    printf("    {%zu, {0x%04x, 0x%04x, 0x%04x}},\n",
           entry.register_index,
           static_cast<unsigned int>(entry.channel_words[0]),
           static_cast<unsigned int>(entry.channel_words[1]),
           static_cast<unsigned int>(entry.channel_words[2]));
  }
  printf("};\n");
}

// Emit a continuation-friendly CLI fragment, using the compact shared form
// when all physical lanes contain the same word.
void PrintFixedRegisterCLIOptions(
    const SPWM_Fixed_Register_Profile_View &profile,
    const char *panel_label) {
  printf("To reproduce the full payload explicitly, append this force-register "
         "fragment to your normal panel command:\n");
  for (size_t entry_index = 0;
       entry_index < profile.entry_count;
       ++entry_index) {
    const SPWM_Fixed_Register_Profile_Entry &entry =
        profile.entries[entry_index];
    printf("--led-spwm-force-register%zu=", entry.register_index);
    if (entry.channel_words[0] == entry.channel_words[1] &&
        entry.channel_words[0] == entry.channel_words[2]) {
      printf("0x%04x", static_cast<unsigned int>(entry.channel_words[0]));
    } else {
      printf("\"R:0x%04x;G:0x%04x;B:0x%04x\"",
             static_cast<unsigned int>(entry.channel_words[0]),
             static_cast<unsigned int>(entry.channel_words[1]),
             static_cast<unsigned int>(entry.channel_words[2]));
    }
    if (entry_index + 1 < profile.entry_count) {
      printf(" %c\n", '\\');
    } else {
      printf("\n");
    }
  }
  printf("%s has no rotating register slot, so the unnumbered "
         "--led-spwm-force-register option does not apply.\n",
         panel_label);
}

void PrintLastEmittedFM6363Profile() {
  const SPWM_Fixed_Register_Profile_View *const last_profile =
      spwm_get_last_emitted_fixed_register_profile();
  if (last_profile == nullptr) {
    printf("\nNo FM6363 register profile was confirmed as emitted.\n");
    return;
  }

  size_t generated_profile_index = 0;
  const FM6363_Register_Test_Profile *const generated_profile =
      FindFM6363GeneratedProfile(last_profile, &generated_profile_index);
  printf("\nConfirmed last-emitted FM6363 register profile: %s",
         last_profile->name);
  if (generated_profile != nullptr) {
    printf(" (%zu/%zu)\n", generated_profile_index + 1,
           FM6363_REGISTER_TEST_PROFILE_COUNT);
    printf("Source: %s\n", generated_profile->source_path);
    PrintGeneratedProfileCLISelector(generated_profile_index);
  } else {
    printf("\n");
  }
  printf("Register words below are ordered R, G, B for each fixed slot.\n\n");
  PrintFixedRegisterEntryArray(*last_profile);
  printf("\n");
  PrintFixedRegisterCLIOptions(*last_profile, "FM6363");
  fflush(stdout);
}

void RunFM6363Profiles(RGBMatrix *matrix,
                       TerminalRegisterTestInput *terminal_input,
                       SPWM_Register_Test_Pattern pattern,
                       uint64_t scan_filter,
                       volatile bool *interrupt_received) {
  const std::vector<size_t> profile_indices = CollectMatchingProfileIndices(
      FM6363_REGISTER_TEST_PROFILES, FM6363_REGISTER_TEST_PROFILE_COUNT,
      scan_filter);
  if (!PrintProfileFilterSummary("FM6363",
                                 FM6363_REGISTER_TEST_PROFILE_COUNT,
                                 profile_indices, scan_filter)) {
    return;
  }

  FrameCanvas *offscreen = matrix->CreateFrameCanvas();
  size_t profile_index = profile_indices[0];
  RegisterTestSelection selection(FM6363_REGISTER_TEST_PROFILE_COUNT,
                                  profile_indices);

  while (!WasInterrupted(interrupt_received)) {
    const FM6363_Register_Test_Profile &profile =
        FM6363_REGISTER_TEST_PROFILES[profile_index];
    PrintFM6363ProfileSummary(profile, profile_index);
    selection.PrintStatus(profile_index);

    if (!spwm_request_fixed_register_profile(&profile.register_profile)) {
      fprintf(stderr, "Unable to queue FM6363 register profile '%s'.\n",
              profile.register_profile.name);
      break;
    }
    if (!WaitUntilFixedProfileEmitted(&profile.register_profile,
                                      interrupt_received)) {
      break;
    }
    profile_index = WaitForProfileInteraction(
        matrix, &offscreen, terminal_input, "FM6363",
        profile.register_profile.name, profile_index, &selection,
        pattern, interrupt_received);
  }

  PrintLastEmittedFM6363Profile();
}

void PrintFM6353ProfileSummary(const FM6353_Register_Test_Profile &profile,
                               size_t profile_index) {
  printf("\n[%zu/%zu] Testing %s\n",
         profile_index + 1, FM6353_REGISTER_TEST_PROFILE_COUNT,
         profile.register_profile.name);
  printf("  source: %s\n", profile.source_path);
  printf("  fixed registers: %zu (physical reg2, reg4, reg6, reg8, reg10), "
         "driver: %s, source scans: %s, duplicate sources: %zu\n",
         profile.register_profile.entry_count,
         profile.source_driver_id, profile.scan_type,
         profile.duplicate_source_count);
  printf("  RCFGX format: %s\n", profile.storage_format);
  printf("  use the controls above to evaluate this profile\n");
  fflush(stdout);
}

const FM6353_Register_Test_Profile *FindFM6353GeneratedProfile(
    const SPWM_Fixed_Register_Profile_View *register_profile,
    size_t *profile_index) {
  for (size_t index = 0; index < FM6353_REGISTER_TEST_PROFILE_COUNT; ++index) {
    if (&FM6353_REGISTER_TEST_PROFILES[index].register_profile ==
        register_profile) {
      if (profile_index != nullptr) *profile_index = index;
      return &FM6353_REGISTER_TEST_PROFILES[index];
    }
  }
  return nullptr;
}

void PrintLastEmittedFM6353Profile() {
  const SPWM_Fixed_Register_Profile_View *const last_profile =
      spwm_get_last_emitted_fixed_register_profile();
  if (last_profile == nullptr) {
    printf("\nNo FM6353 register profile was confirmed as emitted.\n");
    return;
  }

  size_t generated_profile_index = 0;
  const FM6353_Register_Test_Profile *const generated_profile =
      FindFM6353GeneratedProfile(last_profile, &generated_profile_index);
  printf("\nConfirmed last-emitted FM6353 register profile: %s",
         last_profile->name);
  if (generated_profile != nullptr) {
    printf(" (%zu/%zu)\n", generated_profile_index + 1,
           FM6353_REGISTER_TEST_PROFILE_COUNT);
    printf("Source: %s\n", generated_profile->source_path);
    PrintGeneratedProfileCLISelector(generated_profile_index);
  } else {
    printf("\n");
  }
  printf("Register words below are ordered R, G, B for runtime slots 1-5 "
         "(physical reg2, reg4, reg6, reg8, reg10).\n\n");
  PrintFixedRegisterEntryArray(*last_profile);
  printf("\n");
  PrintFixedRegisterCLIOptions(*last_profile, "FM6353");
  fflush(stdout);
}

void RunFM6353Profiles(RGBMatrix *matrix,
                       TerminalRegisterTestInput *terminal_input,
                       SPWM_Register_Test_Pattern pattern,
                       uint64_t scan_filter,
                       volatile bool *interrupt_received) {
  const std::vector<size_t> profile_indices = CollectMatchingProfileIndices(
      FM6353_REGISTER_TEST_PROFILES, FM6353_REGISTER_TEST_PROFILE_COUNT,
      scan_filter);
  if (!PrintProfileFilterSummary("FM6353",
                                 FM6353_REGISTER_TEST_PROFILE_COUNT,
                                 profile_indices, scan_filter)) {
    return;
  }

  FrameCanvas *offscreen = matrix->CreateFrameCanvas();
  size_t profile_index = profile_indices[0];
  RegisterTestSelection selection(FM6353_REGISTER_TEST_PROFILE_COUNT,
                                  profile_indices);

  while (!WasInterrupted(interrupt_received)) {
    const FM6353_Register_Test_Profile &profile =
        FM6353_REGISTER_TEST_PROFILES[profile_index];
    PrintFM6353ProfileSummary(profile, profile_index);
    selection.PrintStatus(profile_index);

    if (!spwm_request_fixed_register_profile(&profile.register_profile)) {
      fprintf(stderr, "Unable to queue FM6353 register profile '%s'.\n",
              profile.register_profile.name);
      break;
    }
    if (!WaitUntilFixedProfileEmitted(&profile.register_profile,
                                      interrupt_received)) {
      break;
    }
    profile_index = WaitForProfileInteraction(
        matrix, &offscreen, terminal_input, "FM6353",
        profile.register_profile.name, profile_index, &selection,
        pattern, interrupt_received);
  }

  PrintLastEmittedFM6353Profile();
}

}  // namespace

bool ParseSPWMRegisterTestScanFilter(const char *value,
                                     uint64_t *scan_filter) {
  return ParseRegisterTestScanFilterValue(value, scan_filter);
}

bool SupportsSPWMRegisterTest(const char *panel_type) {
  return GetRegisterTestPanelFamily(panel_type) != REGISTER_TEST_PANEL_NONE;
}

void RunSPWMRegisterTest(RGBMatrix *matrix, const char *panel_type,
                         SPWM_Register_Test_Pattern pattern,
                         uint64_t scan_filter,
                         volatile bool *interrupt_received) {
  if (matrix == nullptr) return;

  TerminalRegisterTestInput terminal_input;
  PrintRegisterTestControls(pattern, scan_filter);
  if (!terminal_input.enabled()) {
    fprintf(stderr,
            "D15 navigation, marking, and final selection require an "
            "interactive terminal; "
            "the first profile will remain selected.\n");
  }

  switch (GetRegisterTestPanelFamily(panel_type)) {
    case REGISTER_TEST_PANEL_FM6353:
      RunFM6353Profiles(matrix, &terminal_input, pattern, scan_filter,
                        interrupt_received);
      break;
    case REGISTER_TEST_PANEL_FM6363:
      RunFM6363Profiles(matrix, &terminal_input, pattern, scan_filter,
                        interrupt_received);
      break;
    case REGISTER_TEST_PANEL_FM6373:
      RunFM6373Profiles(matrix, &terminal_input, pattern, scan_filter,
                        interrupt_received);
      break;
    case REGISTER_TEST_PANEL_ICND1065L:
      RunICND1065LProfiles(matrix, &terminal_input, pattern, scan_filter,
                           interrupt_received);
      break;
    case REGISTER_TEST_PANEL_SM16380SH:
      RunSM16380SHProfiles(matrix, &terminal_input, pattern, scan_filter,
                           interrupt_received);
      break;
    default:
      fprintf(stderr, "No Demo 15 register profiles for panel type '%s'.\n",
              panel_type != nullptr ? panel_type : "");
      break;
  }
}

}  // namespace internal
}  // namespace rgb_matrix
