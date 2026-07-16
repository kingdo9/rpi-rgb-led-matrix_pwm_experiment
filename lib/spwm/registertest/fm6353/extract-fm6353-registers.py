#!/usr/bin/env python3
"""Extract fixed physical RGB register profiles for FM6353 testing."""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
import sys
from typing import Dict, List, Optional, Sequence, Tuple
import xml.etree.ElementTree as ET
import zipfile


SCRIPT_PATH = Path(__file__).resolve()
REPOSITORY_ROOT = SCRIPT_PATH.parents[4]
SOURCE_DIRECTORY = REPOSITORY_ROOT / "references/fm6353_rcfgx"
OUTPUT_PATH = SCRIPT_PATH.with_name("fm6353-register-profiles.generated.h")

CHANNEL_PROPERTIES = ("RedProperty", "GreenProperty", "BlueProperty")
NATIVE_REGISTER_FIELDS = ("RegFirst", "RegSecond", "RegThird", "RegFourth")
COMPATIBLE_CHANNEL_PREFIXES = ("Red", "Green", "Blue")
# The RCFGX ICN2053-compatible schema stores the LAT2/debug value last. The
# runtime FM6353 slots are ordered by physical address: reg2, 4, 6, 8, 10.
COMPATIBLE_REGISTER_FIELDS = ("Fifth", "First", "Second", "Third", "Special")
NATIVE_DRIVER_NAME = "Chip_FM6353"
COMPATIBLE_DRIVER_NAME = "Chip_ICN2053"
COMPATIBLE_CHIP_CODE = 81
NATIVE_REG2_WORD = 0x0008

RGBWords = Tuple[int, int, int]
FixedRegisterProfile = Tuple[RGBWords, ...]

# Keep the library's built-in reg2/reg4/reg6/reg8/reg10 baseline first in
# Demo 15. Each base word is shared by the physical R/G/B lanes. This literal
# mirrors SPWM_FM6353_REGISTER_ENTRIES in lib/spwm-panel-registers.cc; update
# both locations together. --check does not compare against the core entries.
MAIN_PROFILE: FixedRegisterProfile = (
    (0x0008, 0x0008, 0x0008),
    (0x1F70, 0x1F70, 0x1F70),
    (0x6707, 0x6707, 0x6707),
    (0x40F7, 0x40F7, 0x40F7),
    (0x0040, 0x0040, 0x0040),
)
MAIN_PROFILE_SOURCE = "built-in main FM6353 register config"
MAIN_PROFILE_DRIVER_ID = NATIVE_DRIVER_NAME
MAIN_PROFILE_SCAN_TYPE = "Scan_32"
MAIN_PROFILE_STORAGE_FORMAT = "built-in main"


class ExtractionError(RuntimeError):
    """Raised when an RCFGX package has an unexpected or invalid payload."""


@dataclass(frozen=True)
class ExtractedSource:
    source_path: Path
    register_profile: FixedRegisterProfile
    source_driver_id: str
    scan_type: str
    storage_format: str


@dataclass
class MergedProfile:
    register_profile: FixedRegisterProfile
    sources: List[ExtractedSource]


@dataclass(frozen=True)
class SkippedSource:
    source_path: Path
    source_driver_id: str


def repository_path(path: Path) -> str:
    return path.relative_to(REPOSITORY_ROOT).as_posix()


def parse_decimal_word(text: Optional[str], description: str, source_path: Path) -> int:
    value_text = (text or "").strip()
    try:
        value = int(value_text, 10)
    except ValueError as error:
        raise ExtractionError(
            "%s: %s is not a decimal integer: %r"
            % (repository_path(source_path), description, value_text)
        ) from error
    if value < 0 or value > 0xFFFF:
        raise ExtractionError(
            "%s: %s is outside uint16_t: %d"
            % (repository_path(source_path), description, value)
        )
    return value


def read_xml_root(source_path: Path) -> ET.Element:
    try:
        with zipfile.ZipFile(source_path) as archive:
            xml_names = sorted(
                name for name in archive.namelist() if name.lower().endswith(".xml")
            )
            if len(xml_names) != 1:
                raise ExtractionError(
                    "%s: expected exactly one XML payload, found %d"
                    % (repository_path(source_path), len(xml_names))
                )
            xml_payload = archive.read(xml_names[0])
    except (OSError, zipfile.BadZipFile, KeyError) as error:
        raise ExtractionError(
            "%s: unable to read RCFGX package: %s"
            % (repository_path(source_path), error)
        ) from error

    try:
        root = ET.fromstring(xml_payload)
    except ET.ParseError as error:
        raise ExtractionError(
            "%s: invalid XML payload: %s"
            % (repository_path(source_path), error)
        ) from error
    if root.tag != "ScanBoardProperty":
        raise ExtractionError(
            "%s: unexpected XML root %r"
            % (repository_path(source_path), root.tag)
        )
    return root


def read_driver_id(root: ET.Element, source_path: Path) -> str:
    driver = root.find("./StandardLedModuleProp/DriverChipType")
    if driver is None:
        raise ExtractionError(
            "%s: missing StandardLedModuleProp/DriverChipType"
            % repository_path(source_path)
        )

    chip_code_node = driver.find("ChipCode")
    if chip_code_node is not None:
        chip_code = parse_decimal_word(
            chip_code_node.text, "DriverChipType/ChipCode", source_path
        )
        return "ChipCode=%d" % chip_code

    driver_name = (driver.text or "").strip()
    if not driver_name:
        raise ExtractionError(
            "%s: DriverChipType has neither a name nor ChipCode"
            % repository_path(source_path)
        )
    return driver_name


def read_scan_type(root: ET.Element, source_path: Path) -> str:
    scan_type = (root.findtext("./StandardLedModuleProp/ScanType") or "").strip()
    if not scan_type:
        raise ExtractionError(
            "%s: missing StandardLedModuleProp/ScanType"
            % repository_path(source_path)
        )
    scan_suffix = scan_type[5:] if scan_type.startswith("Scan_") else ""
    if not scan_suffix or not all("0" <= char <= "9" for char in scan_suffix):
        raise ExtractionError(
            "%s: unsupported ScanType %r"
            % (repository_path(source_path), scan_type)
        )
    scan_rows = int(scan_suffix, 10)
    if scan_rows < 1 or scan_rows > 64:
        raise ExtractionError(
            "%s: ScanType is outside the supported 1-to-64 range: %s"
            % (repository_path(source_path), scan_type)
        )
    return scan_type


def read_native_profile(root: ET.Element, source_path: Path) -> FixedRegisterProfile:
    channels: List[Tuple[int, ...]] = []
    for property_name in CHANNEL_PROPERTIES:
        channel_property = root.find("./ChipPropey/%s" % property_name)
        if channel_property is None:
            raise ExtractionError(
                "%s: missing ChipPropey/%s"
                % (repository_path(source_path), property_name)
            )

        words: List[int] = []
        for register_field in NATIVE_REGISTER_FIELDS:
            register_node = channel_property.find(register_field)
            if register_node is None:
                raise ExtractionError(
                    "%s: missing ChipPropey/%s/%s"
                    % (repository_path(source_path), property_name, register_field)
                )
            words.append(
                parse_decimal_word(
                    register_node.text,
                    "%s/%s" % (property_name, register_field),
                    source_path,
                )
            )
        channels.append(tuple(words))

    # Nova's native FM6353 schema exposes physical reg4 through reg10. The
    # basic reg2 word is fixed and comes from the working built-in profile.
    return ((NATIVE_REG2_WORD,) * 3,) + tuple(
        (channels[0][index], channels[1][index], channels[2][index])
        for index in range(len(NATIVE_REGISTER_FIELDS))
    )


def read_compatible_profile(
    root: ET.Element, source_path: Path
) -> Tuple[FixedRegisterProfile, bool]:
    chip_property = root.find("./ChipPropey")
    if chip_property is None:
        raise ExtractionError(
            "%s: missing ChipPropey" % repository_path(source_path)
        )

    register_words: List[RGBWords] = []
    defaulted_fifth = False
    for register_field in COMPATIBLE_REGISTER_FIELDS:
        channel_words: List[int] = []
        missing_channels: List[str] = []
        for channel_prefix in COMPATIBLE_CHANNEL_PREFIXES:
            field_name = "%sRegValueConfig%s" % (channel_prefix, register_field)
            register_node = chip_property.find(field_name)
            if register_node is None:
                missing_channels.append(channel_prefix)
                continue
            channel_words.append(
                parse_decimal_word(register_node.text, field_name, source_path)
            )
        if missing_channels:
            if register_field == "Fifth" and len(missing_channels) == 3:
                # Older ICN2053 packages predate the optional LAT2/debug field;
                # its implicit value is zero in that storage format.
                channel_words = [0, 0, 0]
                defaulted_fifth = True
            else:
                raise ExtractionError(
                    "%s: Config%s is missing for physical channel(s): %s"
                    % (
                        repository_path(source_path),
                        register_field,
                        ", ".join(missing_channels),
                    )
                )
        register_words.append(tuple(channel_words))
    return tuple(register_words), defaulted_fifth


def extract_source(
    source_path: Path,
) -> Tuple[Optional[ExtractedSource], Optional[SkippedSource]]:
    root = read_xml_root(source_path)
    source_driver_id = read_driver_id(root, source_path)

    if source_driver_id == NATIVE_DRIVER_NAME:
        register_profile = read_native_profile(root, source_path)
        storage_format = "native FM6353 (fixed reg2 + RegFirst-RegFourth)"
    elif source_driver_id in (
        COMPATIBLE_DRIVER_NAME,
        "ChipCode=%d" % COMPATIBLE_CHIP_CODE,
    ):
        register_profile, defaulted_fifth = read_compatible_profile(
            root, source_path
        )
        storage_format = "ICN2053-compatible"
        if defaulted_fifth:
            storage_format += " (implicit zero ConfigFifth)"
    else:
        return None, SkippedSource(source_path, source_driver_id)

    # Other-driver packages are intentionally skipped without requiring them
    # to use an FM6353-compatible ScanType schema.
    scan_type = read_scan_type(root, source_path)
    if len(register_profile) != 5:
        raise ExtractionError(
            "%s: expected five mapped register slots, found %d"
            % (repository_path(source_path), len(register_profile))
        )
    return (
        ExtractedSource(
            source_path,
            register_profile,
            source_driver_id,
            scan_type,
            storage_format,
        ),
        None,
    )


# Repository-path order defines public regtype numbering and which duplicate is
# recorded as the canonical source in the generated metadata.
def collect_source_paths() -> List[Path]:
    if not SOURCE_DIRECTORY.is_dir():
        raise ExtractionError(
            "RCFGX source directory is missing: %s"
            % repository_path(SOURCE_DIRECTORY)
        )
    source_paths = sorted(
        SOURCE_DIRECTORY.glob("*.rcfgx"), key=lambda path: repository_path(path)
    )
    if not source_paths:
        raise ExtractionError(
            "no RCFGX packages found in %s" % repository_path(SOURCE_DIRECTORY)
        )
    return source_paths


# Deduplicate by the complete five-slot physical RGB payload; sources[0]
# remains the canonical provenance entry for that generated profile.
def extract_and_merge(
    source_paths: Sequence[Path],
) -> Tuple[List[MergedProfile], List[SkippedSource], int]:
    merged_by_words: Dict[FixedRegisterProfile, MergedProfile] = {}
    skipped_sources: List[SkippedSource] = []
    extracted_source_count = 0

    for source_path in source_paths:
        extracted, skipped = extract_source(source_path)
        if skipped is not None:
            skipped_sources.append(skipped)
            continue
        if extracted is None:
            raise ExtractionError(
                "%s: internal extraction result is empty"
                % repository_path(source_path)
            )

        extracted_source_count += 1
        merged = merged_by_words.get(extracted.register_profile)
        if merged is None:
            merged_by_words[extracted.register_profile] = MergedProfile(
                extracted.register_profile, [extracted]
            )
        else:
            merged.sources.append(extracted)

    return list(merged_by_words.values()), skipped_sources, extracted_source_count


def format_c_string(value: str) -> str:
    return '"%s"' % value.replace("\\", "\\\\").replace('"', '\\"')


def source_comment(prefix: str, source: ExtractedSource) -> str:
    return "// %s: %s; Driver: %s; ScanType: %s; Format: %s" % (
        prefix,
        repository_path(source.source_path),
        source.source_driver_id,
        source.scan_type,
        source.storage_format,
    )


def merged_value(sources: Sequence[ExtractedSource], attribute: str) -> str:
    values = sorted(set(getattr(source, attribute) for source in sources))
    return values[0] if len(values) == 1 else "mixed"


def merged_scan_types(sources: Sequence[ExtractedSource]) -> str:
    scan_types = sorted(
        set(source.scan_type for source in sources),
        key=lambda scan_type: int(scan_type[5:], 10),
    )
    return ",".join(scan_types)


# Emit one shared set of fixed-entry arrays, a lean runtime selector table, and
# a metadata-rich Demo 15 table. Both tables must keep identical regtype order.
def render_header(profiles: Sequence[MergedProfile]) -> str:
    lines = [
        "// -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-",
        "// Generated by extract-fm6353-registers.py. Do not edit by hand.",
        "#ifndef RGBMATRIX_FM6353_REGISTER_PROFILES_GENERATED_H",
        "#define RGBMATRIX_FM6353_REGISTER_PROFILES_GENERATED_H",
        "",
        '#include "../../../spwm-helpers.h"',
        "",
        "namespace rgb_matrix {",
        "namespace internal {",
        "",
        "struct FM6353_Register_Test_Profile {",
        "  SPWM_Fixed_Register_Profile_View register_profile;",
        "  const char *source_path;",
        "  size_t duplicate_source_count;",
        "  const char *source_driver_id;",
        "  const char *scan_type;",
        "  const char *storage_format;",
        "};",
        "",
    ]

    main_profile_name = "fm6353_regtype1"
    lines.append(
        "// %s (runtime slots: reg2, reg4, reg6, reg8, reg10)"
        % main_profile_name
    )
    lines.append("// Built-in main register config; not extracted from RCFGX.")
    lines.append(
        "static const SPWM_Fixed_Register_Profile_Entry %s_entries[] = {"
        % main_profile_name
    )
    for register_index, channel_words in enumerate(MAIN_PROFILE, start=1):
        lines.append(
            "    {%d, {0x%04x, 0x%04x, 0x%04x}},"
            % ((register_index,) + channel_words)
        )
    lines.extend(["};", ""])

    for profile_index, profile in enumerate(profiles, start=2):
        profile_name = "fm6353_regtype%d" % profile_index
        lines.append("// %s (runtime slots: reg2, reg4, reg6, reg8, reg10)" % profile_name)
        lines.append(source_comment("Source", profile.sources[0]))
        for duplicate in profile.sources[1:]:
            lines.append(source_comment("Duplicate", duplicate))
        lines.append(
            "static const SPWM_Fixed_Register_Profile_Entry %s_entries[] = {"
            % profile_name
        )
        for register_index, channel_words in enumerate(
            profile.register_profile, start=1
        ):
            lines.append(
                "    {%d, {0x%04x, 0x%04x, 0x%04x}},"
                % ((register_index,) + channel_words)
            )
        lines.extend(["};", ""])

    lines.append(
        "static const SPWM_Fixed_Register_Profile_View "
        "FM6353_REGISTER_PROFILES[] = {"
    )
    lines.extend(
        [
            "    {%s," % format_c_string(main_profile_name),
            "     %s_entries," % main_profile_name,
            "     sizeof(%s_entries) / sizeof(%s_entries[0])},"
            % (main_profile_name, main_profile_name),
        ]
    )
    for profile_index, profile in enumerate(profiles, start=2):
        profile_name = "fm6353_regtype%d" % profile_index
        lines.extend(
            [
                "    {%s," % format_c_string(profile_name),
                "     %s_entries," % profile_name,
                "     sizeof(%s_entries) / sizeof(%s_entries[0])},"
                % (profile_name, profile_name),
            ]
        )
    lines.extend(
        [
            "};",
            "",
            "static const size_t FM6353_REGISTER_PROFILE_COUNT =",
            "    sizeof(FM6353_REGISTER_PROFILES) /",
            "    sizeof(FM6353_REGISTER_PROFILES[0]);",
            "",
        ]
    )

    lines.append(
        "static const FM6353_Register_Test_Profile "
        "FM6353_REGISTER_TEST_PROFILES[] = {"
    )
    lines.extend(
        [
            "    {{%s," % format_c_string(main_profile_name),
            "      %s_entries," % main_profile_name,
            "      sizeof(%s_entries) / sizeof(%s_entries[0])},"
            % (main_profile_name, main_profile_name),
            "     %s," % format_c_string(MAIN_PROFILE_SOURCE),
            "     0,",
            "     %s," % format_c_string(MAIN_PROFILE_DRIVER_ID),
            "     %s," % format_c_string(MAIN_PROFILE_SCAN_TYPE),
            "     %s}," % format_c_string(MAIN_PROFILE_STORAGE_FORMAT),
        ]
    )
    for profile_index, profile in enumerate(profiles, start=2):
        profile_name = "fm6353_regtype%d" % profile_index
        canonical = profile.sources[0]
        lines.extend(
            [
                "    {{%s," % format_c_string(profile_name),
                "      %s_entries," % profile_name,
                "      sizeof(%s_entries) / sizeof(%s_entries[0])},"
                % (profile_name, profile_name),
                "     %s," % format_c_string(repository_path(canonical.source_path)),
                "     %d," % (len(profile.sources) - 1),
                "     %s," % format_c_string(merged_value(profile.sources, "source_driver_id")),
                "     %s," % format_c_string(merged_scan_types(profile.sources)),
                "     %s}," % format_c_string(merged_value(profile.sources, "storage_format")),
            ]
        )
    lines.extend(
        [
            "};",
            "",
            "static const size_t FM6353_REGISTER_TEST_PROFILE_COUNT =",
            "    sizeof(FM6353_REGISTER_TEST_PROFILES) /",
            "    sizeof(FM6353_REGISTER_TEST_PROFILES[0]);",
            "",
            "}  // namespace internal",
            "}  // namespace rgb_matrix",
            "",
            "#endif  // RGBMATRIX_FM6353_REGISTER_PROFILES_GENERATED_H",
            "",
        ]
    )
    return "\n".join(lines)


def report_summary(
    source_count: int,
    extracted_source_count: int,
    profiles: Sequence[MergedProfile],
    skipped_sources: Sequence[SkippedSource],
) -> None:
    print("RCFGX packages scanned: %d" % source_count)
    print("FM6353/ICN2053-compatible payloads extracted: %d" % extracted_source_count)
    print("unique five-slot physical RGB profiles: %d" % len(profiles))
    print("generated profiles including main: %d" % (len(profiles) + 1))
    print("packages skipped for other drivers: %d" % len(skipped_sources))
    skipped_driver_counts = Counter(
        source.source_driver_id for source in skipped_sources
    )
    for source_driver_id in sorted(skipped_driver_counts):
        print("  %s: %d" % (source_driver_id, skipped_driver_counts[source_driver_id]))


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Extract and merge FM6353 fixed physical RGB profiles."
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="verify that the generated header is current without writing it",
    )
    return parser.parse_args()


def main() -> int:
    arguments = parse_arguments()
    try:
        source_paths = collect_source_paths()
        profiles, skipped_sources, extracted_source_count = extract_and_merge(
            source_paths
        )
        generated_header = render_header(profiles)
    except ExtractionError as error:
        print("error: %s" % error, file=sys.stderr)
        return 1

    report_summary(
        len(source_paths), extracted_source_count, profiles, skipped_sources
    )

    if arguments.check:
        try:
            existing_header = OUTPUT_PATH.read_text(encoding="utf-8")
        except OSError as error:
            print("error: generated header is unavailable: %s" % error, file=sys.stderr)
            return 1
        if existing_header != generated_header:
            print(
                "error: generated header is out of date; run %s"
                % repository_path(SCRIPT_PATH),
                file=sys.stderr,
            )
            return 1
        print("generated header is current: %s" % repository_path(OUTPUT_PATH))
        return 0

    with OUTPUT_PATH.open("w", encoding="utf-8", newline="\n") as output_file:
        output_file.write(generated_header)
    print("wrote %s" % repository_path(OUTPUT_PATH))
    return 0


if __name__ == "__main__":
    sys.exit(main())
