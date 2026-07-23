// -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
#include "spwm-register-profile-loader.h"

#include <ctype.h>
#include <errno.h>
#include <limits.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>

#include <fstream>
#include <limits>
#include <mutex>
#include <sstream>
#include <utility>

#ifdef RGBMATRIX_SPWM_PROFILE_MODULE_RELATIVE
#include <dlfcn.h>
#endif

#ifndef RGBMATRIX_SPWM_PROFILE_DATA_DIR
#define RGBMATRIX_SPWM_PROFILE_DATA_DIR \
  "/usr/local/share/rgbmatrix/spwm/registertest"
#endif

namespace rgb_matrix {
namespace internal {
namespace {

const char kCatalogMagic[] = "RGBMATRIX_SPWM_PROFILES_V4";
const size_t kCatalogFieldCount = 4;

void SetError(const std::string &message, std::string *error) {
  if (error != nullptr) *error = message;
}

std::vector<std::string> Split(const std::string &value, char delimiter) {
  std::vector<std::string> fields;
  size_t field_begin = 0;
  while (true) {
    const size_t delimiter_pos = value.find(delimiter, field_begin);
    if (delimiter_pos == std::string::npos) {
      fields.push_back(value.substr(field_begin));
      return fields;
    }
    fields.push_back(value.substr(field_begin, delimiter_pos - field_begin));
    field_begin = delimiter_pos + 1;
  }
}

bool ParseUnsigned(const std::string &value, int base,
                   unsigned long long maximum,
                   unsigned long long *parsed_value) {
  if (value.empty() || parsed_value == nullptr) return false;
  for (size_t index = 0; index < value.size(); ++index) {
    const unsigned char character =
        static_cast<unsigned char>(value[index]);
    if ((base == 10 && !isdigit(character)) ||
        (base == 16 && !isxdigit(character))) {
      return false;
    }
  }
  char *end = nullptr;
  errno = 0;
  const unsigned long long parsed = strtoull(value.c_str(), &end, base);
  if (errno != 0 || end == value.c_str() || *end != '\0' ||
      parsed > maximum) {
    return false;
  }
  *parsed_value = parsed;
  return true;
}

bool ParseSize(const std::string &value, size_t *parsed_value) {
  unsigned long long parsed = 0;
  if (!ParseUnsigned(value, 10,
                     std::numeric_limits<size_t>::max(), &parsed)) {
    return false;
  }
  *parsed_value = static_cast<size_t>(parsed);
  return true;
}

bool ParseWord(const std::string &value, uint16_t *parsed_value) {
  unsigned long long parsed = 0;
  if (!ParseUnsigned(value, 16, 0xffff, &parsed)) return false;
  *parsed_value = static_cast<uint16_t>(parsed);
  return true;
}

bool ParseScanMask(const std::string &scan_types, uint64_t *scan_mask) {
  if (scan_mask == nullptr || scan_types.empty()) return false;
  uint64_t parsed_mask = 0;
  const std::vector<std::string> values = Split(scan_types, ',');
  for (size_t index = 0; index < values.size(); ++index) {
    const std::string &value = values[index];
    if (value.compare(0, 5, "Scan_") != 0) return false;
    size_t scan_rows = 0;
    if (!ParseSize(value.substr(5), &scan_rows) ||
        scan_rows < 1 || scan_rows > 64) {
      return false;
    }
    parsed_mask |= static_cast<uint64_t>(1ULL << (scan_rows - 1));
  }
  *scan_mask = parsed_mask;
  return parsed_mask != 0;
}

const char *CanonicalPanelName(const char *panel_type) {
  if (panel_type == nullptr) return nullptr;
  const char *const supported_panels[] = {
      "fm6353", "fm6363", "fm6373", "icnd1065l", "sm16380sh"};
  for (size_t index = 0;
       index < sizeof(supported_panels) / sizeof(supported_panels[0]);
       ++index) {
    const char *const panel_name = supported_panels[index];
    if (strncasecmp(panel_type, panel_name, strlen(panel_name)) == 0) {
      return panel_name;
    }
  }
  return nullptr;
}

SPWM_Register_Profile_Kind ExpectedProfileKind(
    const std::string &panel_name) {
  return panel_name == "fm6353" || panel_name == "fm6363"
             ? SPWM_REGISTER_PROFILE_FIXED
             : SPWM_REGISTER_PROFILE_RGB;
}

bool ValidateRegisterSlots(const std::string &panel_name,
                           const SPWM_Loaded_Register_Profile &profile,
                           std::string *error) {
  const SPWM_RGB_Register_Profile_View *const rgb_profile =
      profile.rgb_profile();
  if (rgb_profile != nullptr) {
    if (spwm_get_panel_register_slot_type(
            panel_name.c_str(), rgb_profile->register_index) ==
        SPWM_REGISTER_SLOT_RGB) {
      return true;
    }
    std::ostringstream message;
    message << "RGB profile register index " << rgb_profile->register_index
            << " does not target an RGB slot for " << panel_name;
    SetError(message.str(), error);
    return false;
  }

  const SPWM_Fixed_Register_Profile_View *const fixed_profile =
      profile.fixed_profile();
  if (fixed_profile == nullptr) {
    SetError("profile has no register payload", error);
    return false;
  }
  for (size_t entry_index = 0; entry_index < fixed_profile->entry_count;
       ++entry_index) {
    const size_t register_index =
        fixed_profile->entries[entry_index].register_index;
    if (spwm_get_panel_register_slot_type(panel_name.c_str(), register_index) !=
        SPWM_REGISTER_SLOT_FIXED) {
      std::ostringstream message;
      message << "fixed profile register index " << register_index
              << " does not target a fixed slot for " << panel_name;
      SetError(message.str(), error);
      return false;
    }
  }

  // Fixed catalog records are complete configurations, so every fixed slot in
  // the selected panel's init sequence must be represented.
  for (size_t register_index = 1;
       register_index <= SPWM_FORCE_REGISTER_COUNT; ++register_index) {
    if (spwm_get_panel_register_slot_type(panel_name.c_str(), register_index) !=
        SPWM_REGISTER_SLOT_FIXED) {
      continue;
    }
    bool found = false;
    for (size_t entry_index = 0; entry_index < fixed_profile->entry_count;
         ++entry_index) {
      if (fixed_profile->entries[entry_index].register_index == register_index) {
        found = true;
        break;
      }
    }
    if (!found) {
      std::ostringstream message;
      message << "fixed profile is missing required register index "
              << register_index << " for " << panel_name;
      SetError(message.str(), error);
      return false;
    }
  }
  return true;
}

std::string JoinPath(const std::string &directory,
                     const std::string &filename) {
  if (directory.empty()) return filename;
  return directory[directory.size() - 1] == '/'
             ? directory + filename
             : directory + "/" + filename;
}

std::string ExecutableDirectory() {
  char executable_path[PATH_MAX + 1];
  const ssize_t path_length =
      readlink("/proc/self/exe", executable_path, PATH_MAX);
  if (path_length <= 0 || path_length > PATH_MAX) return std::string();
  executable_path[path_length] = '\0';
  const char *const final_separator = strrchr(executable_path, '/');
  if (final_separator == nullptr) return std::string();
  return std::string(executable_path,
                     static_cast<size_t>(final_separator - executable_path));
}

// Resolve the module containing this loader so language packages and
// relocatable shared-library installs can find their catalogs without changing
// process environment state.
std::string ModuleDirectory() {
#ifdef RGBMATRIX_SPWM_PROFILE_MODULE_RELATIVE
  Dl_info module_info = {};
  if (dladdr(static_cast<const void *>(kCatalogMagic), &module_info) == 0 ||
      module_info.dli_fname == nullptr) {
    return std::string();
  }
  const char *const final_separator = strrchr(module_info.dli_fname, '/');
  if (final_separator == nullptr) return std::string();
  return std::string(
      module_info.dli_fname,
      static_cast<size_t>(final_separator - module_info.dli_fname));
#else
  return std::string();
#endif
}

void AppendCandidate(const std::string &directory,
                     const std::string &filename,
                     std::vector<std::string> *candidates) {
  if (directory.empty() || candidates == nullptr) return;
  const std::string candidate = JoinPath(directory, filename);
  for (size_t index = 0; index < candidates->size(); ++index) {
    if ((*candidates)[index] == candidate) return;
  }
  candidates->push_back(candidate);
}

}  // namespace

bool SPWM_Loaded_Register_Profile::ParseRGBPayload(
    const std::string &payload, std::string *error) {
  const std::vector<std::string> sections = Split(payload, '|');
  if (sections.size() != 4) {
    SetError("RGB profile payload must contain a register index and three "
             "channel lists", error);
    return false;
  }

  size_t register_index = 0;
  if (!ParseSize(sections[0], &register_index) || register_index < 1 ||
      register_index > SPWM_FORCE_REGISTER_COUNT) {
    SetError("RGB profile register index is outside 1..6", error);
    return false;
  }

  for (size_t channel = 0; channel < 3; ++channel) {
    const std::vector<std::string> words = Split(sections[channel + 1], ',');
    if (words.empty() || (words.size() == 1 && words[0].empty())) {
      SetError("RGB profile channel list is empty", error);
      return false;
    }
    for (size_t word_index = 0; word_index < words.size(); ++word_index) {
      uint16_t word = 0;
      if (!ParseWord(words[word_index], &word)) {
        SetError("RGB profile contains an invalid uint16 word", error);
        return false;
      }
      rgb_words_[channel].push_back(word);
    }
  }
  if (rgb_words_[0].size() != rgb_words_[1].size() ||
      rgb_words_[0].size() != rgb_words_[2].size()) {
    SetError("RGB profile channel word counts do not match", error);
    return false;
  }
  rgb_profile_.register_index = register_index;
  return true;
}

bool SPWM_Loaded_Register_Profile::ParseFixedPayload(
    const std::string &payload, std::string *error) {
  const std::vector<std::string> encoded_entries = Split(payload, ';');
  if (encoded_entries.empty() ||
      (encoded_entries.size() == 1 && encoded_entries[0].empty())) {
    SetError("fixed profile entry list is empty", error);
    return false;
  }

  uint32_t register_mask = 0;
  for (size_t entry_index = 0;
       entry_index < encoded_entries.size(); ++entry_index) {
    const std::vector<std::string> fields =
        Split(encoded_entries[entry_index], ',');
    if (fields.size() != 4) {
      SetError("fixed profile entry must contain slot,R,G,B", error);
      return false;
    }
    size_t register_index = 0;
    if (!ParseSize(fields[0], &register_index) || register_index < 1 ||
        register_index > SPWM_FORCE_REGISTER_COUNT) {
      SetError("fixed profile register index is outside 1..6", error);
      return false;
    }
    const uint32_t register_bit =
        static_cast<uint32_t>(1u << (register_index - 1));
    if ((register_mask & register_bit) != 0) {
      SetError("fixed profile contains a duplicate register index", error);
      return false;
    }

    SPWM_Fixed_Register_Profile_Entry entry = {};
    entry.register_index = register_index;
    for (size_t channel = 0; channel < 3; ++channel) {
      if (!ParseWord(fields[channel + 1], &entry.channel_words[channel])) {
        SetError("fixed profile contains an invalid uint16 word", error);
        return false;
      }
    }
    fixed_entries_.push_back(entry);
    register_mask |= register_bit;
  }
  return true;
}

SPWM_Loaded_Register_Profile::SPWM_Loaded_Register_Profile()
    : kind_(SPWM_REGISTER_PROFILE_RGB), rgb_profile_(), fixed_profile_() {}

const SPWM_RGB_Register_Profile_View *
SPWM_Loaded_Register_Profile::rgb_profile() const {
  return kind_ == SPWM_REGISTER_PROFILE_RGB ? &rgb_profile_ : nullptr;
}

const SPWM_Fixed_Register_Profile_View *
SPWM_Loaded_Register_Profile::fixed_profile() const {
  return kind_ == SPWM_REGISTER_PROFILE_FIXED ? &fixed_profile_ : nullptr;
}

void SPWM_Loaded_Register_Profile::Clear(SPWM_Register_Profile_Kind kind) {
  kind_ = kind;
  metadata_ = SPWM_Register_Profile_Metadata();
  for (size_t channel = 0; channel < 3; ++channel) {
    rgb_words_[channel].clear();
  }
  fixed_entries_.clear();
  rgb_profile_ = SPWM_RGB_Register_Profile_View();
  fixed_profile_ = SPWM_Fixed_Register_Profile_View();
}

void SPWM_Loaded_Register_Profile::RefreshViews() {
  if (kind_ == SPWM_REGISTER_PROFILE_RGB) {
    rgb_profile_.name = metadata_.name.c_str();
    for (size_t channel = 0; channel < 3; ++channel) {
      rgb_profile_.channel_words[channel] = rgb_words_[channel].data();
      rgb_profile_.channel_word_counts[channel] = rgb_words_[channel].size();
    }
  } else {
    fixed_profile_.name = metadata_.name.c_str();
    fixed_profile_.entries = fixed_entries_.data();
    fixed_profile_.entry_count = fixed_entries_.size();
  }
}

SPWM_Register_Profile_File::SPWM_Register_Profile_File()
    : kind_(SPWM_REGISTER_PROFILE_RGB) {}

bool SPWM_Register_Profile_File::Open(const char *panel_type,
                                      std::string *error) {
  const char *const canonical_panel = CanonicalPanelName(panel_type);
  if (canonical_panel == nullptr) {
    SetError("unsupported SPWM register-profile panel", error);
    return false;
  }

  const std::string filename = std::string(canonical_panel) + ".profiles";
  std::vector<std::string> candidates;
  const char *const override_directory =
      getenv("SPWM_PROFILE_DIR");
  if (override_directory != nullptr && *override_directory != '\0') {
    // An explicit caller override is authoritative, including load failures.
    return OpenPath(JoinPath(override_directory, filename), canonical_panel,
                    error);
  }

  const std::string module_directory = ModuleDirectory();
  if (!module_directory.empty()) {
    AppendCandidate(module_directory, filename, &candidates);
    AppendCandidate(module_directory + "/spwm/registertest", filename,
                    &candidates);
#ifdef RGBMATRIX_SPWM_PROFILE_MODULE_DATA_DIR
    // CMake supplies the catalog path relative to its configured library
    // directory, so install-time prefix relocation remains supported.
    AppendCandidate(module_directory + "/" +
                        RGBMATRIX_SPWM_PROFILE_MODULE_DATA_DIR,
                    filename, &candidates);
#endif
  }

  const std::string executable_directory = ExecutableDirectory();
  if (!executable_directory.empty()) {
    AppendCandidate(executable_directory, filename, &candidates);
    AppendCandidate(executable_directory + "/../lib/spwm/registertest/data",
                    filename, &candidates);
    AppendCandidate(executable_directory + "/lib/spwm/registertest/data",
                    filename, &candidates);
    AppendCandidate(executable_directory +
                        "/../share/rgbmatrix/spwm/registertest",
                    filename, &candidates);
  }
  AppendCandidate(RGBMATRIX_SPWM_PROFILE_DATA_DIR, filename, &candidates);

  for (size_t candidate_index = 0;
       candidate_index < candidates.size(); ++candidate_index) {
    std::ifstream probe(candidates[candidate_index].c_str(),
                        std::ios::in | std::ios::binary);
    if (!probe.is_open()) continue;
    probe.close();
    return OpenPath(candidates[candidate_index], canonical_panel, error);
  }

  SetError("unable to locate " + filename +
               "; set SPWM_PROFILE_DIR to the catalog directory",
           error);
  return false;
}

bool SPWM_Register_Profile_File::OpenPath(const std::string &path,
                                          const std::string &panel_name,
                                          std::string *error) {
  std::ifstream input(path.c_str(), std::ios::in | std::ios::binary);
  if (!input.is_open()) {
    SetError("unable to open register-profile catalog: " + path, error);
    return false;
  }

  std::string header;
  if (!std::getline(input, header)) {
    SetError("register-profile catalog is empty: " + path, error);
    return false;
  }
  if (!header.empty() && header[header.size() - 1] == '\r') {
    header.resize(header.size() - 1);
  }
  const std::vector<std::string> header_fields = Split(header, '\t');
  if (header_fields.size() != 4) {
    SetError("register-profile catalog has an invalid header: " + path,
             error);
    return false;
  }
  if (header_fields[0] != kCatalogMagic) {
    SetError("register-profile catalog uses an unsupported schema version: " +
                 path,
             error);
    return false;
  }
  if (header_fields[1] != panel_name) {
    SetError("register-profile catalog header does not match panel: " + path,
             error);
    return false;
  }
  SPWM_Register_Profile_Kind parsed_kind = SPWM_REGISTER_PROFILE_RGB;
  if (header_fields[2] == "rgb") {
    parsed_kind = SPWM_REGISTER_PROFILE_RGB;
  } else if (header_fields[2] == "fixed") {
    parsed_kind = SPWM_REGISTER_PROFILE_FIXED;
  } else {
    SetError("register-profile catalog has an unsupported profile kind: " +
                 path,
             error);
    return false;
  }
  if (parsed_kind != ExpectedProfileKind(panel_name)) {
    SetError("register-profile catalog has the wrong profile kind for " +
                 panel_name + ": " + path,
             error);
    return false;
  }

  size_t declared_count = 0;
  if (!ParseSize(header_fields[3], &declared_count) || declared_count == 0) {
    SetError("register-profile catalog has an invalid profile count: " + path,
             error);
    return false;
  }

  std::vector<std::unique_ptr<SPWM_Loaded_Register_Profile> > loaded_profiles;
  std::vector<uint64_t> loaded_scan_masks;
  std::string line;
  while (std::getline(input, line)) {
    if (!line.empty() && line[line.size() - 1] == '\r') {
      line.resize(line.size() - 1);
    }
    std::unique_ptr<SPWM_Loaded_Register_Profile> profile(
        new SPWM_Loaded_Register_Profile());
    uint64_t scan_mask = 0;
    std::string record_error;
    if (!ParseRecord(line, parsed_kind, profile.get(), &scan_mask,
                     &record_error)) {
      SetError("invalid register-profile record in " + path + ": " +
                   record_error,
               error);
      return false;
    }
    if (!ValidateRegisterSlots(panel_name, *profile, &record_error)) {
      SetError("invalid register-profile record in " + path + ": " +
                   record_error,
               error);
      return false;
    }
    const std::string expected_name =
        panel_name + "_regtype" + std::to_string(loaded_profiles.size() + 1);
    if (profile->metadata().name != expected_name) {
      SetError("register-profile catalog contains an out-of-order name: " +
                   profile->metadata().name,
               error);
      return false;
    }
    loaded_profiles.push_back(std::move(profile));
    loaded_scan_masks.push_back(scan_mask);
  }
  if (!input.eof()) {
    SetError("unable to finish reading register-profile catalog: " + path,
             error);
    return false;
  }
  if (loaded_profiles.size() != declared_count) {
    std::ostringstream message;
    message << "register-profile catalog count mismatch in " << path
            << ": header says " << declared_count << ", found "
            << loaded_profiles.size();
    SetError(message.str(), error);
    return false;
  }

  profiles_.swap(loaded_profiles);
  scan_masks_.swap(loaded_scan_masks);
  kind_ = parsed_kind;
  panel_name_ = panel_name;
  data_path_ = path;
  return true;
}

bool SPWM_Register_Profile_File::ParseRecord(
    const std::string &line, SPWM_Register_Profile_Kind profile_kind,
    SPWM_Loaded_Register_Profile *profile,
    uint64_t *scan_mask, std::string *error) {
  if (profile == nullptr || scan_mask == nullptr) {
    SetError("profile output is unavailable", error);
    return false;
  }

  const std::vector<std::string> fields = Split(line, '\t');
  if (fields.size() != kCatalogFieldCount) {
    SetError("record has an invalid field count", error);
    return false;
  }
  if (!ParseScanMask(fields[2], scan_mask)) {
    SetError("record has invalid scan metadata", error);
    return false;
  }

  profile->Clear(profile_kind);
  profile->metadata_.name = fields[0];
  profile->metadata_.source = fields[1];
  profile->metadata_.scan_type = fields[2];
  if (profile->metadata_.name.empty() || profile->metadata_.source.empty()) {
    SetError("record has invalid basic metadata", error);
    return false;
  }
  const bool payload_valid =
      profile_kind == SPWM_REGISTER_PROFILE_RGB
          ? profile->ParseRGBPayload(fields[3], error)
          : profile->ParseFixedPayload(fields[3], error);
  if (!payload_valid) return false;
  profile->RefreshViews();
  return true;
}

const SPWM_Loaded_Register_Profile *SPWM_Register_Profile_File::profile(
    size_t profile_index) const {
  return profile_index < profiles_.size()
             ? profiles_[profile_index].get()
             : nullptr;
}

std::vector<size_t> SPWM_Register_Profile_File::MatchingProfileIndices(
    uint64_t scan_filter) const {
  std::vector<size_t> profile_indices;
  for (size_t profile_index = 0;
       profile_index < scan_masks_.size(); ++profile_index) {
    if (scan_filter == 0 ||
        (scan_masks_[profile_index] & scan_filter) != 0) {
      profile_indices.push_back(profile_index);
    }
  }
  return profile_indices;
}

const SPWM_Register_Profile_File *spwm_get_register_profile_file(
    const char *panel_type, std::string *error) {
  const char *const canonical_panel = CanonicalPanelName(panel_type);
  if (canonical_panel == nullptr) {
    SetError("unsupported SPWM register-profile panel", error);
    return nullptr;
  }

  const char *const supported_panels[] = {
      "fm6353", "fm6363", "fm6373", "icnd1065l", "sm16380sh"};
  size_t panel_index = 0;
  while (panel_index <
             sizeof(supported_panels) / sizeof(supported_panels[0]) &&
         strcmp(canonical_panel, supported_panels[panel_index]) != 0) {
    ++panel_index;
  }
  if (panel_index ==
      sizeof(supported_panels) / sizeof(supported_panels[0])) {
    SetError("unsupported SPWM register-profile panel", error);
    return nullptr;
  }

  static SPWM_Register_Profile_File catalogs[
      sizeof(supported_panels) / sizeof(supported_panels[0])];
  static std::mutex catalog_mutex;
  const std::lock_guard<std::mutex> lock(catalog_mutex);
  if (catalogs[panel_index].profile_count() == 0) {
    std::string load_error;
    if (!catalogs[panel_index].Open(canonical_panel, &load_error)) {
      SetError(load_error, error);
      return nullptr;
    }
  }
  if (error != nullptr) error->clear();
  return &catalogs[panel_index];
}

}  // namespace internal
}  // namespace rgb_matrix
