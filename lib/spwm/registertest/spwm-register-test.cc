// -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
#include "spwm/registertest/spwm-register-test.h"

#include "graphics.h"
#include "led-matrix.h"
#include "../spwm-helpers.h"
#include "spwm/registertest/spwm-register-profile-loader.h"

#include <chrono>
#include <ctype.h>
#include <deque>
#include <poll.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>

namespace rgb_matrix {
namespace internal {
namespace {

const int kSceneDurationMs = 4000;
const int kEscapeSequenceIdleTimeoutMs = 100;
const char kTextScrollMessage[] = "Register Test";

const char *const kRegisterTestPanelNames[] = {
    "FM6353",
    "FM6363",
    "FM6373",
    "ICND1065L",
    "SM16380SH",
};

struct TinyGlyph {
  char character;
  uint8_t rows[5];
};

struct TextScrollGlyph {
  char character;
  uint8_t rows[6];
};

// Keep one marquee phase across register changes while this Demo 15 run is
// active. Register uploads deliberately pause, rather than reset, that phase.
struct TextScrollState {
  explicit TextScrollState(bool middle_only)
      : x(0), band(middle_only ? 1 : 0), initialized(false),
        middle_only(middle_only) {}

  int x;
  uint8_t band;
  bool initialized;
  bool middle_only;
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

  // Drop commands entered before the newly confirmed profile became visible.
  // ISIG remains enabled, so flushing terminal input does not suppress Ctrl-C.
  void DiscardPendingActions() {
    pending_actions_.clear();
    escape_state_ = 0;
    if (enabled_) tcflush(STDIN_FILENO, TCIFLUSH);
  }

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

  // Check once without delaying the next text-scroll frame. SwapOnVSync()
  // supplies that frame's cadence, including any --led-limit-refresh cap.
  RegisterTestInput PollInput() {
    ExpireStaleEscapeSequence();
    if (!pending_actions_.empty()) return TakeNextAction();

    RegisterTestInput input;
    if (!enabled_) return input;

    struct pollfd input_poll = {STDIN_FILENO, POLLIN, 0};
    const int poll_result = poll(&input_poll, 1, 0);
    if (poll_result <= 0) return input;
    if ((input_poll.revents & (POLLHUP | POLLERR | POLLNVAL)) != 0) {
      escape_state_ = 0;
      return input;
    }
    if ((input_poll.revents & POLLIN) == 0) return input;

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
    return pending_actions_.empty() ? input : TakeNextAction();
  }

 private:
  void ExpireStaleEscapeSequence() {
    if (escape_state_ != 0 &&
        std::chrono::steady_clock::now() >= escape_deadline_) {
      escape_state_ = 0;
    }
  }

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
    if (escape_state_ != 0 || byte == 0x1b) {
      escape_deadline_ =
          std::chrono::steady_clock::now() +
          std::chrono::milliseconds(kEscapeSequenceIdleTimeoutMs);
    }
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
  std::chrono::steady_clock::time_point escape_deadline_;
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

// These marquee letters are embedded from the repository's public-domain
// fonts/5x7.bdf so the register test has no runtime font-file dependency. Its
// empty seventh cell row is omitted so the visible glyph can fill each band.
const TextScrollGlyph kTextScrollGlyphs[] = {
    {'R', {0x1c, 0x12, 0x12, 0x1c, 0x14, 0x12}},
    {'E', {0x1e, 0x10, 0x1c, 0x10, 0x10, 0x1e}},
    {'G', {0x0c, 0x12, 0x10, 0x16, 0x12, 0x0e}},
    {'I', {0x0e, 0x04, 0x04, 0x04, 0x04, 0x0e}},
    {'S', {0x0c, 0x12, 0x08, 0x04, 0x12, 0x0c}},
    {'T', {0x0e, 0x04, 0x04, 0x04, 0x04, 0x04}},
};

// Panel-type matching intentionally accepts case-insensitive suffix variants,
// mirroring the core SPWM profile lookup (for example "fm6373-something").
const char *FindRegisterTestPanelLabel(const char *panel_type) {
  if (panel_type == nullptr) return nullptr;
  for (const char *panel_name : kRegisterTestPanelNames) {
    if (strncasecmp(panel_type, panel_name, strlen(panel_name)) == 0) {
      return panel_name;
    }
  }
  return nullptr;
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

char UppercaseGlyphCharacter(char character) {
  return character >= 'a' && character <= 'z'
             ? static_cast<char>(character - 'a' + 'A')
             : character;
}

uint8_t TextScrollGlyphRow(char character, int row, bool use_slim_font) {
  character = UppercaseGlyphCharacter(character);
  if (!use_slim_font) {
    return row >= 0 && row < 5 ? FindTinyGlyph(character)[row] : 0;
  }

  for (size_t glyph_index = 0;
       glyph_index < sizeof(kTextScrollGlyphs) /
                         sizeof(kTextScrollGlyphs[0]);
       ++glyph_index) {
    if (kTextScrollGlyphs[glyph_index].character == character) {
      return row >= 0 && row < 6
                 ? kTextScrollGlyphs[glyph_index].rows[row]
                 : 0;
    }
  }
  return 0;
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

int TextScrollHeight(int panel_height) {
  if (panel_height <= 0) return 0;
  const int text_height = panel_height / 3;
  return text_height > 0 ? text_height : 1;
}

int TextScrollGlyphWidth(int text_height) {
  const int glyph_width = text_height >= 7
                              ? (text_height * 5 + 3) / 7
                              : (text_height * 3 + 2) / 5;
  return glyph_width > 0 ? glyph_width : 1;
}

int TextScrollSpacing(int text_height) {
  const int spacing =
      text_height >= 7 ? text_height / 7 : text_height / 5;
  return spacing > 0 ? spacing : 1;
}

int TextScrollWidth(int text_height) {
  const size_t character_count = sizeof(kTextScrollMessage) - 1;
  return static_cast<int>(character_count) *
             TextScrollGlyphWidth(text_height) +
         static_cast<int>(character_count - 1) *
             TextScrollSpacing(text_height);
}

void InitializeTextScroll(Canvas *canvas, TextScrollState *state) {
  if (state == nullptr || state->initialized) return;
  state->x = canvas->width();
  state->band = state->middle_only ? 1 : 0;
  state->initialized = true;
}

const Color &TextScrollColor(size_t color_index) {
  static const Color palette[] = {
      Color(255, 255, 255),
      Color(255, 0, 0),
      Color(0, 255, 0),
      Color(0, 0, 255),
      Color(255, 255, 0),
      Color(170, 0, 255),
  };
  return palette[color_index % (sizeof(palette) / sizeof(palette[0]))];
}

void DrawTextScrollScene(Canvas *canvas, const char *panel_label,
                         size_t one_based_profile_index,
                         TextScrollState *state) {
  const int visible_width = canvas->width();
  const int panel_height = canvas->height();
  const int text_height = TextScrollHeight(panel_height);
  canvas->Clear();
  if (state == nullptr || visible_width <= 0 || text_height <= 0) return;

  InitializeTextScroll(canvas, state);
  const bool use_slim_font = text_height >= 7;
  const int source_rows = use_slim_font ? 6 : 5;
  const int source_columns = use_slim_font ? 5 : 3;
  const int glyph_width = TextScrollGlyphWidth(text_height);
  const int spacing = TextScrollSpacing(text_height);
  const int band_top = state->band * panel_height / 3;
  const int band_bottom = (state->band + 1) * panel_height / 3;
  const int render_height = band_bottom - band_top;
  if (render_height <= 0) return;
  int cursor_x = state->x;
  size_t letter_index = 0;

  for (const char *character = kTextScrollMessage;
       *character != '\0'; ++character) {
    if (*character != ' ') {
      if (cursor_x < visible_width && cursor_x + glyph_width > 0) {
        const Color &color = TextScrollColor(letter_index);
        for (int output_y = 0; output_y < render_height; ++output_y) {
          const int source_row = output_y * source_rows / render_height;
          const uint8_t row_bits = TextScrollGlyphRow(
              *character, source_row, use_slim_font);
          for (int output_x = 0; output_x < glyph_width; ++output_x) {
            const int source_column =
                output_x * source_columns / glyph_width;
            const int pixel_x = cursor_x + output_x;
            if (pixel_x >= 0 && pixel_x < visible_width &&
                (row_bits &
                 (1u << (source_columns - 1 - source_column))) != 0) {
              canvas->SetPixel(pixel_x, band_top + output_y,
                               color.r, color.g, color.b);
            }
          }
        }
      }
      ++letter_index;
    }
    cursor_x += glyph_width + spacing;
  }

  DrawProfileLabel(canvas, panel_label, one_based_profile_index);
}

void AdvanceTextScroll(Canvas *canvas, int steps, TextScrollState *state) {
  if (state == nullptr || steps <= 0) return;
  InitializeTextScroll(canvas, state);

  const int visible_width = canvas->width();
  const int text_height = TextScrollHeight(canvas->height());
  const int text_width = TextScrollWidth(text_height);
  if (visible_width <= 0 || text_height <= 0 || text_width <= 0) return;

  const int distance_to_wrap = state->x + text_width;
  if (steps < distance_to_wrap) {
    state->x -= steps;
    return;
  }

  const int travel = visible_width + text_width;
  const int remaining_after_wrap = steps - distance_to_wrap;
  const int completed_passes = 1 + remaining_after_wrap / travel;
  if (!state->middle_only) {
    state->band = static_cast<uint8_t>(
        (state->band + completed_passes % 3) % 3);
  }
  state->x = visible_width - remaining_after_wrap % travel;
}

// Move one red pixel on either side of the framebuffer half boundary through
// a short ping-pong path. A complete frame is a vertical pair; mixing animation
// generations turns it into a diagonal. Keep everything else black so R0/R1
// captures contain only the signal under test.
void DrawTearScene(Canvas *canvas, size_t frame_index) {
  const int width = canvas->width();
  const int height = canvas->height();
  canvas->Clear();
  if (width < 2 || height < 2) return;

  const int travel_columns = width < 8 ? width : 8;
  const int path_length = 2 * (travel_columns - 1);
  const int path_phase = static_cast<int>(
      frame_index % static_cast<size_t>(path_length));
  const int path_offset = path_phase < travel_columns
                              ? path_phase
                              : path_length - path_phase;
  const int x = (width - travel_columns) / 2 + path_offset;
  const int upper_y = height / 2 - 1;
  const int lower_y = height / 2;
  canvas->SetPixel(x, upper_y, 255, 0, 0);
  canvas->SetPixel(x, lower_y, 255, 0, 0);
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
                      SPWM_Register_Test_Pattern pattern,
                      bool show_gradient, size_t tear_frame,
                      TextScrollState *text_scroll_state) {
  switch (pattern) {
    case SPWM_REGISTER_TEST_PATTERN_TEXTSCROLL:
      DrawTextScrollScene(*offscreen, panel_label, profile_index + 1,
                          text_scroll_state);
      break;
    case SPWM_REGISTER_TEST_PATTERN_TEAR:
      DrawTearScene(*offscreen, tear_frame);
      break;
    case SPWM_REGISTER_TEST_PATTERN_ALIGN:
      DrawAlignmentScene(*offscreen, panel_label, profile_index + 1);
      break;
    case SPWM_REGISTER_TEST_PATTERN_CYCLE:
      if (show_gradient) {
        DrawGradientScene(*offscreen, panel_label, profile_index + 1);
      } else {
        DrawAlignmentScene(*offscreen, panel_label, profile_index + 1);
      }
      break;
    case SPWM_REGISTER_TEST_PATTERN_GRADIENT:
    default:
      DrawGradientScene(*offscreen, panel_label, profile_index + 1);
      break;
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

// Translate a catalog index into a position in a filtered/finalist list, then
// move with wraparound while returning the original catalog index.
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
    SPWM_Register_Test_Pattern pattern, bool text_scroll_middle_only) {
  switch (pattern) {
    case SPWM_REGISTER_TEST_PATTERN_ALIGN:
      return "ALIGNMENT ONLY";
    case SPWM_REGISTER_TEST_PATTERN_CYCLE:
      return "ALIGNMENT + COLOR GRADIENT (alternates every 4 seconds)";
    case SPWM_REGISTER_TEST_PATTERN_TEXTSCROLL:
      return text_scroll_middle_only
                 ? "TEXTSCROLL (middle only)"
                 : "TEXTSCROLL (top / middle / bottom)";
    case SPWM_REGISTER_TEST_PATTERN_TEAR:
      return "TEAR (two-pixel moving center-seam marker)";
    case SPWM_REGISTER_TEST_PATTERN_GRADIENT:
    default:
      return "COLOR GRADIENT ONLY";
  }
}

// Print the controls prominently before any large per-profile metadata output.
void PrintRegisterTestControls(SPWM_Register_Test_Pattern pattern,
                               int text_scroll_step_pixels,
                               bool text_scroll_middle_only,
                               uint64_t scan_filter) {
  printf("\n"
         "============================================================\n"
         "                 REGISTER TEST CONTROLS\n");
  printf("  >>> DISPLAY PATTERN: %s <<<\n",
         RegisterTestPatternDescription(pattern, text_scroll_middle_only));
  printf("  >>> SCAN FILTER: ");
  PrintScanFilter(scan_filter);
  printf(" <<<\n");
  if (pattern == SPWM_REGISTER_TEST_PATTERN_TEXTSCROLL) {
    printf("  >>> TEXT SPEED: %d PIXEL%s / REFRESH FRAME <<<\n",
           text_scroll_step_pixels,
           text_scroll_step_pixels == 1 ? "" : "S");
  }
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
    int text_scroll_step_pixels,
    TextScrollState *text_scroll_state,
    volatile bool *interrupt_received, SPWM_Register_Test_Result *result) {
  bool show_gradient = pattern == SPWM_REGISTER_TEST_PATTERN_GRADIENT ||
                       pattern == SPWM_REGISTER_TEST_PATTERN_CYCLE;
  const bool alternate_scenes =
      pattern == SPWM_REGISTER_TEST_PATTERN_CYCLE;
  const bool scroll_text =
      pattern == SPWM_REGISTER_TEST_PATTERN_TEXTSCROLL;
  const bool test_tearing =
      pattern == SPWM_REGISTER_TEST_PATTERN_TEAR;
  const bool animate_scene = scroll_text || test_tearing;
  size_t tear_frame = 0;
  bool current_profile_is_visible = false;
  while (!WasInterrupted(interrupt_received)) {
    ShowProfileScene(matrix, offscreen, panel_label, profile_index,
                     pattern, show_gradient, tear_frame, text_scroll_state);
    if (!current_profile_is_visible) {
      if (result != nullptr) {
        result->has_displayed_profile = true;
        result->profile_index = profile_index;
      }
      // Input accumulated during refresh-thread processing must not skip or
      // mark this profile immediately after its first confirmed display.
      terminal_input->DiscardPendingActions();
      current_profile_is_visible = true;
    }
    // A signal can arrive while drawing or swapping the scene. Avoid entering
    // the timed input wait after the user has already asked to quit.
    if (WasInterrupted(interrupt_received)) break;
    const RegisterTestInput input =
        animate_scene ? terminal_input->PollInput()
                      : terminal_input->WaitForInput(kSceneDurationMs);
    // A signal can arrive while poll() is waiting or while decoded actions
    // remain queued. Do not mutate the selection after the user asked to quit.
    if (WasInterrupted(interrupt_received)) break;
    if (!input.HasAction()) {
      if (scroll_text) {
        AdvanceTextScroll(*offscreen, text_scroll_step_pixels,
                          text_scroll_state);
      } else if (test_tearing) {
        ++tear_frame;
      } else if (alternate_scenes) {
        show_gradient = !show_gradient;
      }
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
enum ProfileEmissionResult {
  PROFILE_EMISSION_COMPLETE = 0,
  PROFILE_EMISSION_INTERRUPTED,
  PROFILE_EMISSION_FAILED,
};

ProfileEmissionResult WaitUntilRGBProfileEmitted(
    const SPWM_RGB_Register_Profile_View *requested_profile,
    volatile bool *interrupt_received) {
  while (true) {
    if (spwm_get_last_emitted_rgb_register_profile() == requested_profile) {
      return PROFILE_EMISSION_COMPLETE;
    }
    if (spwm_get_last_rejected_rgb_register_profile() == requested_profile) {
      fprintf(stderr,
              "Refresh thread rejected RGB register profile '%s' for the "
              "active panel config.\n",
              requested_profile->name);
      return PROFILE_EMISSION_FAILED;
    }
    if (WasInterrupted(interrupt_received)) {
      return PROFILE_EMISSION_INTERRUPTED;
    }
    usleep(1000);
  }
}

// A fixed profile completes once every selected fixed slot has appeared in a
// completed init sequence. Report active-panel incompatibility immediately.
ProfileEmissionResult WaitUntilFixedProfileEmitted(
    const SPWM_Fixed_Register_Profile_View *requested_profile,
    volatile bool *interrupt_received) {
  while (true) {
    if (spwm_get_last_emitted_fixed_register_profile() == requested_profile) {
      return PROFILE_EMISSION_COMPLETE;
    }
    if (spwm_get_last_rejected_fixed_register_profile() == requested_profile) {
      fprintf(stderr,
              "Refresh thread rejected fixed register profile '%s' for the "
              "active panel config.\n",
              requested_profile->name);
      return PROFILE_EMISSION_FAILED;
    }
    if (WasInterrupted(interrupt_received)) {
      return PROFILE_EMISSION_INTERRUPTED;
    }
    usleep(1000);
  }
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

// Catalog indices are zero-based; the CLI and regtype names are one-based.
void PrintCatalogProfileCLISelector(size_t profile_index) {
  printf("Short CLI selector for this catalog profile:\n"
         "--led-spwm-register-config=%zu\n\n",
         profile_index + 1);
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

void PrintProfileSummary(const SPWM_Loaded_Register_Profile &profile,
                         size_t profile_index, size_t profile_count) {
  const SPWM_Register_Profile_Metadata &metadata = profile.metadata();
  printf("\n[%zu/%zu] Testing %s\n",
         profile_index + 1, profile_count, metadata.name.c_str());
  printf("  source: %s\n", metadata.source.c_str());

  const SPWM_RGB_Register_Profile_View *const rgb_profile =
      profile.rgb_profile();
  const SPWM_Fixed_Register_Profile_View *const fixed_profile =
      profile.fixed_profile();
  if (rgb_profile != nullptr) {
    printf("  register: %zu\n", rgb_profile->register_index);
  } else if (fixed_profile != nullptr) {
    printf("  fixed registers: %zu\n", fixed_profile->entry_count);
  }

  printf("  source scans: %s\n", metadata.scan_type.c_str());
  printf("  use the controls above to evaluate this profile\n");
  fflush(stdout);
}

void PrintLastDisplayedRGBProfile(const char *panel_label,
                                  const SPWM_Register_Profile_File &catalog,
                                  const SPWM_Register_Test_Result &result) {
  if (!result.has_displayed_profile) {
    printf("\nNo %s register profile was displayed.\n", panel_label);
    return;
  }

  const size_t profile_index = result.profile_index;
  const SPWM_Loaded_Register_Profile *const loaded_profile =
      catalog.profile(profile_index);
  const SPWM_RGB_Register_Profile_View *const last_profile =
      loaded_profile != nullptr ? loaded_profile->rgb_profile() : nullptr;
  if (last_profile == nullptr) {
    fprintf(stderr, "Unable to recover the final %s register profile.\n",
            panel_label);
    return;
  }
  printf("\nLast displayed %s register profile: %s",
         panel_label, last_profile->name);
  printf(" (%zu/%zu)\n", profile_index + 1, catalog.profile_count());
  printf("Source: %s\n", loaded_profile->metadata().source.c_str());
  PrintRGBProfileConfig(*last_profile);
  printf("\n");
  PrintCatalogProfileCLISelector(profile_index);
  fflush(stdout);
}

void PrintLastDisplayedFixedProfile(
    const char *panel_label, const SPWM_Register_Profile_File &catalog,
    const SPWM_Register_Test_Result &result) {
  if (!result.has_displayed_profile) {
    printf("\nNo %s register profile was displayed.\n", panel_label);
    return;
  }

  const size_t profile_index = result.profile_index;
  const SPWM_Loaded_Register_Profile *const loaded_profile =
      catalog.profile(profile_index);
  const SPWM_Fixed_Register_Profile_View *const last_profile =
      loaded_profile != nullptr ? loaded_profile->fixed_profile() : nullptr;
  if (last_profile == nullptr) {
    fprintf(stderr, "Unable to recover the final %s register profile.\n",
            panel_label);
    return;
  }
  printf("\nLast displayed %s register profile: %s",
         panel_label, last_profile->name);
  printf(" (%zu/%zu)\n", profile_index + 1, catalog.profile_count());
  printf("Source: %s\n", loaded_profile->metadata().source.c_str());
  printf("Register words below are ordered R, G, B for each fixed slot.\n\n");
  PrintFixedRegisterEntryArray(*last_profile);
  printf("\n");
  PrintFixedRegisterCLIOptions(*last_profile, panel_label);
  printf("\n");
  PrintCatalogProfileCLISelector(profile_index);
  fflush(stdout);
}

bool RunRGBProfiles(RGBMatrix *matrix,
                    TerminalRegisterTestInput *terminal_input,
                    const char *panel_label,
                    const SPWM_Register_Profile_File &catalog,
                    SPWM_Register_Test_Pattern pattern,
                    int text_scroll_step_pixels,
                    bool text_scroll_middle_only,
                    uint64_t scan_filter,
                    volatile bool *interrupt_received,
                    SPWM_Register_Test_Result *result) {
  const std::vector<size_t> profile_indices =
      catalog.MatchingProfileIndices(scan_filter);
  if (!PrintProfileFilterSummary(panel_label, catalog.profile_count(),
                                 profile_indices, scan_filter)) {
    return false;
  }

  FrameCanvas *offscreen = matrix->CreateFrameCanvas();
  if (offscreen == nullptr) {
    fprintf(stderr, "Unable to create the %s register-test canvas.\n",
            panel_label);
    return false;
  }
  size_t profile_index = profile_indices[0];
  RegisterTestSelection selection(catalog.profile_count(), profile_indices);
  TextScrollState text_scroll_state(text_scroll_middle_only);
  bool succeeded = true;

  while (!WasInterrupted(interrupt_received)) {
    const SPWM_Loaded_Register_Profile *const loaded_profile =
        catalog.profile(profile_index);
    const SPWM_RGB_Register_Profile_View *const profile =
        loaded_profile != nullptr ? loaded_profile->rgb_profile() : nullptr;
    if (loaded_profile == nullptr || profile == nullptr) {
      fprintf(stderr, "%s catalog profile %zu has no RGB payload.\n",
              panel_label, profile_index + 1);
      succeeded = false;
      break;
    }

    PrintProfileSummary(*loaded_profile, profile_index,
                        catalog.profile_count());
    selection.PrintStatus(profile_index);
    if (!spwm_request_rgb_register_profile(profile)) {
      fprintf(stderr, "Unable to queue %s register profile '%s'.\n",
              panel_label, profile->name);
      succeeded = false;
      break;
    }
    const ProfileEmissionResult emission_result =
        WaitUntilRGBProfileEmitted(profile, interrupt_received);
    if (emission_result != PROFILE_EMISSION_COMPLETE) {
      succeeded = emission_result == PROFILE_EMISSION_INTERRUPTED;
      break;
    }

    profile_index = WaitForProfileInteraction(
        matrix, &offscreen, terminal_input, panel_label, profile->name,
        profile_index, &selection, pattern, text_scroll_step_pixels,
        &text_scroll_state, interrupt_received, result);
  }

  if (result != nullptr) result->final_output_ready = true;
  return succeeded;
}

bool RunFixedProfiles(RGBMatrix *matrix,
                      TerminalRegisterTestInput *terminal_input,
                      const char *panel_label,
                      const SPWM_Register_Profile_File &catalog,
                      SPWM_Register_Test_Pattern pattern,
                      int text_scroll_step_pixels,
                      bool text_scroll_middle_only,
                      uint64_t scan_filter,
                      volatile bool *interrupt_received,
                      SPWM_Register_Test_Result *result) {
  const std::vector<size_t> profile_indices =
      catalog.MatchingProfileIndices(scan_filter);
  if (!PrintProfileFilterSummary(panel_label, catalog.profile_count(),
                                 profile_indices, scan_filter)) {
    return false;
  }

  FrameCanvas *offscreen = matrix->CreateFrameCanvas();
  if (offscreen == nullptr) {
    fprintf(stderr, "Unable to create the %s register-test canvas.\n",
            panel_label);
    return false;
  }
  size_t profile_index = profile_indices[0];
  RegisterTestSelection selection(catalog.profile_count(), profile_indices);
  TextScrollState text_scroll_state(text_scroll_middle_only);
  bool succeeded = true;

  while (!WasInterrupted(interrupt_received)) {
    const SPWM_Loaded_Register_Profile *const loaded_profile =
        catalog.profile(profile_index);
    const SPWM_Fixed_Register_Profile_View *const profile =
        loaded_profile != nullptr ? loaded_profile->fixed_profile() : nullptr;
    if (loaded_profile == nullptr || profile == nullptr) {
      fprintf(stderr, "%s catalog profile %zu has no fixed payload.\n",
              panel_label, profile_index + 1);
      succeeded = false;
      break;
    }

    PrintProfileSummary(*loaded_profile, profile_index,
                        catalog.profile_count());
    selection.PrintStatus(profile_index);
    if (!spwm_request_fixed_register_profile(profile)) {
      fprintf(stderr, "Unable to queue %s register profile '%s'.\n",
              panel_label, profile->name);
      succeeded = false;
      break;
    }
    const ProfileEmissionResult emission_result =
        WaitUntilFixedProfileEmitted(profile, interrupt_received);
    if (emission_result != PROFILE_EMISSION_COMPLETE) {
      succeeded = emission_result == PROFILE_EMISSION_INTERRUPTED;
      break;
    }

    profile_index = WaitForProfileInteraction(
        matrix, &offscreen, terminal_input, panel_label, profile->name,
        profile_index, &selection, pattern, text_scroll_step_pixels,
        &text_scroll_state, interrupt_received, result);
  }

  if (result != nullptr) result->final_output_ready = true;
  return succeeded;
}

}  // namespace

bool ParseSPWMRegisterTestScanFilter(const char *value,
                                     uint64_t *scan_filter) {
  return ParseRegisterTestScanFilterValue(value, scan_filter);
}

bool SupportsSPWMRegisterTest(const char *panel_type) {
  return FindRegisterTestPanelLabel(panel_type) != nullptr;
}

bool RunSPWMRegisterTest(RGBMatrix *matrix, const char *panel_type,
                         SPWM_Register_Test_Pattern pattern,
                         int text_scroll_step_pixels,
                         bool text_scroll_middle_only,
                         uint64_t scan_filter,
                         volatile bool *interrupt_received,
                         SPWM_Register_Test_Result *result) {
  if (result != nullptr) *result = SPWM_Register_Test_Result();
  if (matrix == nullptr) {
    fprintf(stderr, "Demo 15 cannot run without an RGB matrix.\n");
    return false;
  }
  if (pattern == SPWM_REGISTER_TEST_PATTERN_TEXTSCROLL &&
      text_scroll_step_pixels < 1) {
    fprintf(stderr, "Demo 15 TEXTSCROLL speed must be at least 1.\n");
    return false;
  }

  const char *const panel_label = FindRegisterTestPanelLabel(panel_type);
  if (panel_label == nullptr) {
    fprintf(stderr, "No Demo 15 register profiles for panel type '%s'.\n",
            panel_type != nullptr ? panel_type : "");
    return false;
  }

  std::string catalog_error;
  const SPWM_Register_Profile_File *const catalog =
      spwm_get_register_profile_file(panel_type, &catalog_error);
  if (catalog == nullptr) {
    fprintf(stderr, "Unable to load Demo 15 profiles for %s: %s\n",
            panel_label, catalog_error.c_str());
    return false;
  }
  printf("Loaded %zu %s register profiles into memory from %s.\n",
         catalog->profile_count(), panel_label, catalog->data_path().c_str());

  TerminalRegisterTestInput terminal_input;
  PrintRegisterTestControls(pattern, text_scroll_step_pixels,
                            text_scroll_middle_only, scan_filter);
  if (!terminal_input.enabled()) {
    fprintf(stderr,
            "D15 navigation, marking, and final selection require an "
            "interactive terminal; "
            "the first profile will remain selected.\n");
  }

  if (catalog->kind() == SPWM_REGISTER_PROFILE_RGB) {
    return RunRGBProfiles(matrix, &terminal_input, panel_label, *catalog,
                          pattern, text_scroll_step_pixels,
                          text_scroll_middle_only, scan_filter,
                          interrupt_received, result);
  }
  return RunFixedProfiles(matrix, &terminal_input, panel_label, *catalog,
                          pattern, text_scroll_step_pixels,
                          text_scroll_middle_only, scan_filter,
                          interrupt_received, result);
}

void PrintSPWMRegisterTestResult(const char *panel_type,
                                 const SPWM_Register_Test_Result &result) {
  if (!result.final_output_ready) return;

  const char *const panel_label = FindRegisterTestPanelLabel(panel_type);
  if (panel_label == nullptr) return;

  std::string catalog_error;
  const SPWM_Register_Profile_File *const catalog =
      spwm_get_register_profile_file(panel_type, &catalog_error);
  if (catalog == nullptr) {
    fprintf(stderr, "Unable to load Demo 15 profiles for %s: %s\n",
            panel_label, catalog_error.c_str());
    return;
  }
  if (catalog->kind() == SPWM_REGISTER_PROFILE_RGB) {
    PrintLastDisplayedRGBProfile(panel_label, *catalog, result);
  } else {
    PrintLastDisplayedFixedProfile(panel_label, *catalog, result);
  }
}

}  // namespace internal
}  // namespace rgb_matrix
