// -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
#ifndef RGBMATRIX_SPWM_REGISTER_PROFILE_LOADER_H
#define RGBMATRIX_SPWM_REGISTER_PROFILE_LOADER_H

#include "../spwm-helpers.h"

#include <memory>
#include <stdint.h>
#include <string>
#include <vector>

namespace rgb_matrix {
namespace internal {

enum SPWM_Register_Profile_Kind {
  SPWM_REGISTER_PROFILE_RGB = 0,
  SPWM_REGISTER_PROFILE_FIXED,
};

struct SPWM_Register_Profile_Metadata {
  std::string name;
  std::string source;
  std::string scan_type;
};

// Own one decoded catalog record. Demo 15 retains loaded objects so the
// non-owning refresh-thread views and their pointer identities remain stable.
class SPWM_Loaded_Register_Profile {
 public:
  SPWM_Loaded_Register_Profile();
  SPWM_Loaded_Register_Profile(const SPWM_Loaded_Register_Profile &) = delete;
  SPWM_Loaded_Register_Profile &operator=(
      const SPWM_Loaded_Register_Profile &) = delete;

  SPWM_Register_Profile_Kind kind() const { return kind_; }
  const SPWM_Register_Profile_Metadata &metadata() const { return metadata_; }
  const SPWM_RGB_Register_Profile_View *rgb_profile() const;
  const SPWM_Fixed_Register_Profile_View *fixed_profile() const;

 private:
  friend class SPWM_Register_Profile_File;

  void Clear(SPWM_Register_Profile_Kind kind);
  void RefreshViews();
  bool ParseRGBPayload(const std::string &payload, std::string *error);
  bool ParseFixedPayload(const std::string &payload, std::string *error);

  SPWM_Register_Profile_Kind kind_;
  SPWM_Register_Profile_Metadata metadata_;
  std::vector<uint16_t> rgb_words_[3];
  std::vector<SPWM_Fixed_Register_Profile_Entry> fixed_entries_;
  SPWM_RGB_Register_Profile_View rgb_profile_;
  SPWM_Fixed_Register_Profile_View fixed_profile_;
};

// Load one selected panel's complete catalog into stable owned memory. The
// external catalog removes generated C++ parsing from the build, while the
// decoded register payload is small enough to keep Demo 15 navigation simple.
class SPWM_Register_Profile_File {
 public:
  SPWM_Register_Profile_File();
  SPWM_Register_Profile_File(const SPWM_Register_Profile_File &) = delete;
  SPWM_Register_Profile_File &operator=(
      const SPWM_Register_Profile_File &) = delete;

  bool Open(const char *panel_type, std::string *error);

  SPWM_Register_Profile_Kind kind() const { return kind_; }
  size_t profile_count() const { return profiles_.size(); }
  const SPWM_Loaded_Register_Profile *profile(size_t profile_index) const;
  const std::string &panel_name() const { return panel_name_; }
  const std::string &data_path() const { return data_path_; }
  std::vector<size_t> MatchingProfileIndices(uint64_t scan_filter) const;

 private:
  bool OpenPath(const std::string &path, const std::string &panel_name,
                std::string *error);
  bool ParseRecord(const std::string &line,
                   SPWM_Register_Profile_Kind profile_kind,
                   SPWM_Loaded_Register_Profile *profile,
                   uint64_t *scan_mask, std::string *error);

  SPWM_Register_Profile_Kind kind_;
  std::string panel_name_;
  std::string data_path_;
  std::vector<std::unique_ptr<SPWM_Loaded_Register_Profile> > profiles_;
  std::vector<uint64_t> scan_masks_;
};

// Return the process-lifetime catalog for a supported panel, loading it on
// first use. Core register selection and Demo 15 share this same owned data.
const SPWM_Register_Profile_File *spwm_get_register_profile_file(
    const char *panel_type, std::string *error);

}  // namespace internal
}  // namespace rgb_matrix

#endif  // RGBMATRIX_SPWM_REGISTER_PROFILE_LOADER_H
