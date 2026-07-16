#!/usr/bin/env python3
"""Extract physical RGB register profiles from ICND1065L RCFGX packages."""

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
SOURCE_DIRECTORY = REPOSITORY_ROOT / "references/icnd1065l_rcfgx"
OUTPUT_PATH = SCRIPT_PATH.with_name("icnd1065l-register-profiles.generated.h")

CHANNEL_PROPERTIES = ("RedProperty", "GreenProperty", "BlueProperty")
CHANNEL_SUFFIXES = ("r", "g", "b")
ICND1065L_CHIP_CODE = 386
REGISTER_INDEX = 3

RGBSequences = Tuple[Tuple[int, ...], Tuple[int, ...], Tuple[int, ...]]

# Keep the library's built-in 1/43 main register block as the first diagnostic
# profile so Demo 15 always tests the normal baseline before alternatives.
# This literal mirrors SPWM_ICND1065L_REGISTER_BLOCK3_SEQ_R/G/B in
# lib/spwm-panel-registers.cc; update both locations together. --check verifies
# this script against its generated header, not against the core arrays.
MAIN_PROFILE: RGBSequences = (
    (
        0x0000, 0x026A, 0x0322, 0x0412, 0x0500, 0x0601, 0x0712, 0x0C10,
        0x0D02, 0x0E84, 0x0F01, 0x1040, 0x1127, 0x1800, 0x1926, 0x1C60,
        0x1D02, 0x1E71, 0x2040, 0x2101, 0x2380, 0x74A0,
    ),
    (
        0x0000, 0x026A, 0x0322, 0x0412, 0x0500, 0x0601, 0x0712, 0x0C10,
        0x0D04, 0x0E84, 0x0F01, 0x1040, 0x1127, 0x1800, 0x1908, 0x1C60,
        0x1D02, 0x1E92, 0x2060, 0x2101, 0x2305, 0x74A0,
    ),
    (
        0x0000, 0x026A, 0x0322, 0x0412, 0x0500, 0x0601, 0x0712, 0x0C10,
        0x0D03, 0x0E84, 0x0F11, 0x1040, 0x1127, 0x1800, 0x190A, 0x1C60,
        0x1D02, 0x1EB5, 0x2060, 0x2101, 0x2300, 0x74A0,
    ),
)
MAIN_PROFILE_SOURCE = "built-in main ICND1065L register config"
MAIN_PROFILE_CHIP_CODE = 386
MAIN_PROFILE_SCAN_TYPE = "Scan_43"

# Preserve the former built-in 1/32-scan CONFIG1 payload as regtype2. It was
# derived from the main 1/43 block and was not extracted from an RCFGX package.
PRESERVED_CONFIG1_PROFILE: RGBSequences = (
    (
        0x0000, 0x025F, 0x0322, 0x0412, 0x0500, 0x0601, 0x0712, 0x0C10,
        0x0D02, 0x0E84, 0x0F01, 0x1040, 0x1127, 0x1800, 0x1926, 0x1C60,
        0x1D02, 0x1E71, 0x2040, 0x2101, 0x2380, 0x74A0,
    ),
    (
        0x0000, 0x025F, 0x0322, 0x0412, 0x0500, 0x0601, 0x0712, 0x0C10,
        0x0D04, 0x0E84, 0x0F01, 0x1040, 0x1127, 0x1800, 0x1908, 0x1C60,
        0x1D02, 0x1E92, 0x2060, 0x2101, 0x2305, 0x74A0,
    ),
    (
        0x0000, 0x025F, 0x0322, 0x0412, 0x0500, 0x0601, 0x0712, 0x0C10,
        0x0D03, 0x0E84, 0x0F11, 0x1040, 0x1127, 0x1800, 0x190A, 0x1C60,
        0x1D02, 0x1EB5, 0x2060, 0x2101, 0x2300, 0x74A0,
    ),
)
PRESERVED_CONFIG1_SOURCE = "preserved former built-in ICND1065L CONFIG1"
PRESERVED_CONFIG1_CHIP_CODE = 386
PRESERVED_CONFIG1_SCAN_TYPE = "Scan_32"


class ExtractionError(RuntimeError):
    """Raised when an RCFGX package has an unexpected or invalid payload."""


@dataclass(frozen=True)
class ExtractedSource:
    source_path: Path
    register_profile: RGBSequences
    chip_code: int
    scan_type: str


@dataclass
class MergedProfile:
    register_profile: RGBSequences
    sources: List[ExtractedSource]


@dataclass(frozen=True)
class SkippedSource:
    source_path: Path
    chip_code: int


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


def read_scan_type(root: ET.Element, source_path: Path) -> str:
    """Read a filterable Scan_N value and enforce Demo 15's 1-to-64 range."""
    scan_type = (root.findtext("./StandardLedModuleProp/ScanType") or "").strip()
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


def read_channel_words(
    root: ET.Element,
    property_name: str,
    reg_group_count: int,
    source_path: Path,
) -> Tuple[int, ...]:
    value_nodes = root.findall(
        "./ChipPropey/%s/RegValue/unsignedShort" % property_name
    )
    if not value_nodes:
        raise ExtractionError(
            "%s: missing ChipPropey/%s/RegValue payload"
            % (repository_path(source_path), property_name)
        )

    stored_words = tuple(
        parse_decimal_word(
            value_node.text,
            "%s/RegValue[%d]" % (property_name, word_index),
            source_path,
        )
        for word_index, value_node in enumerate(value_nodes)
    )
    if len(stored_words) != reg_group_count + 1:
        raise ExtractionError(
            "%s: %s has %d stored words; expected RegGroupCnt %d plus one"
            % (
                repository_path(source_path),
                property_name,
                len(stored_words),
                reg_group_count,
            )
        )

    populated_indices = [
        word_index for word_index, word in enumerate(stored_words) if word != 0
    ]
    if not populated_indices:
        raise ExtractionError(
            "%s: %s register payload is empty"
            % (repository_path(source_path), property_name)
        )
    final_populated_index = populated_indices[-1]

    # ICND1065L packages store populated address words first and pad the
    # remaining register groups with zero. The runtime sequence rotates the
    # storage terminator to the front, matching the library's proven 22-word
    # ICND1065L register-3 representation.
    return (0,) + stored_words[: final_populated_index + 1]


def extract_source(
    source_path: Path,
) -> Tuple[Optional[ExtractedSource], Optional[SkippedSource]]:
    root = read_xml_root(source_path)
    chip_code = parse_decimal_word(
        root.findtext("./StandardLedModuleProp/DriverChipType/ChipCode"),
        "DriverChipType/ChipCode",
        source_path,
    )
    if chip_code != ICND1065L_CHIP_CODE:
        return None, SkippedSource(source_path, chip_code)

    reg_group_count = parse_decimal_word(
        root.findtext("./RegGroupCnt"), "RegGroupCnt", source_path
    )
    register_profile = tuple(
        read_channel_words(root, property_name, reg_group_count, source_path)
        for property_name in CHANNEL_PROPERTIES
    )
    channel_lengths = tuple(len(channel) for channel in register_profile)
    if len(set(channel_lengths)) != 1:
        raise ExtractionError(
            "%s: normalized physical R/G/B lengths differ: %s"
            % (repository_path(source_path), channel_lengths)
        )

    scan_type = read_scan_type(root, source_path)

    return (
        ExtractedSource(source_path, register_profile, chip_code, scan_type),
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


# Deduplicate by the complete normalized physical (R, G, B) payload; sources[0]
# remains the canonical provenance entry for that generated profile.
def extract_and_merge(
    source_paths: Sequence[Path],
) -> Tuple[List[MergedProfile], List[SkippedSource], int]:
    merged_by_words: Dict[RGBSequences, MergedProfile] = {}
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
            continue

        canonical = merged.sources[0]
        if extracted.scan_type != canonical.scan_type:
            raise ExtractionError(
                "%s: duplicate RGB payload has ScanType %s, but %s uses %s"
                % (
                    repository_path(extracted.source_path),
                    extracted.scan_type,
                    repository_path(canonical.source_path),
                    canonical.scan_type,
                )
            )
        merged.sources.append(extracted)

    return list(merged_by_words.values()), skipped_sources, extracted_source_count


def format_c_string(value: str) -> str:
    return '"%s"' % value.replace("\\", "\\\\").replace('"', '\\"')


def format_word_array(name: str, words: Sequence[int]) -> List[str]:
    lines = ["static const uint16_t %s[] = {" % name]
    for offset in range(0, len(words), 8):
        chunk = words[offset : offset + 8]
        lines.append("    %s," % ", ".join("0x%04x" % word for word in chunk))
    lines.append("};")
    return lines


# Emit one shared set of payload arrays, a lean runtime selector table, and a
# metadata-rich Demo 15 table. Both tables must keep identical regtype order.
def render_header(profiles: Sequence[MergedProfile]) -> str:
    lines = [
        "// -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-",
        "// Generated by extract-icnd1065l-registers.py. Do not edit by hand.",
        "#ifndef RGBMATRIX_ICND1065L_REGISTER_PROFILES_GENERATED_H",
        "#define RGBMATRIX_ICND1065L_REGISTER_PROFILES_GENERATED_H",
        "",
        '#include "../../../spwm-helpers.h"',
        "",
        "namespace rgb_matrix {",
        "namespace internal {",
        "",
        "struct ICND1065L_Register_Test_Profile {",
        "  SPWM_RGB_Register_Profile_View register_profile;",
        "  const char *source_path;",
        "  size_t duplicate_source_count;",
        "  int chip_code;",
        "  const char *scan_type;",
        "};",
        "",
    ]

    main_profile_name = "icnd1065l_regtype1"
    lines.append("// %s" % main_profile_name)
    lines.append("// Built-in main register config; not extracted from RCFGX.")
    for suffix, words in zip(CHANNEL_SUFFIXES, MAIN_PROFILE):
        lines.extend(format_word_array("%s_%s" % (main_profile_name, suffix), words))
        lines.append("")

    preserved_profile_name = "icnd1065l_regtype2"
    lines.append("// %s" % preserved_profile_name)
    lines.append("// Preserved former built-in CONFIG1; not extracted from RCFGX.")
    for suffix, words in zip(CHANNEL_SUFFIXES, PRESERVED_CONFIG1_PROFILE):
        lines.extend(format_word_array("%s_%s" % (preserved_profile_name, suffix), words))
        lines.append("")

    for profile_index, profile in enumerate(profiles, start=3):
        profile_name = "icnd1065l_regtype%d" % profile_index
        canonical = profile.sources[0]
        lines.append("// %s" % profile_name)
        lines.append("// Source: %s" % repository_path(canonical.source_path))
        for duplicate in profile.sources[1:]:
            lines.append("// Duplicate: %s" % repository_path(duplicate.source_path))
        for suffix, words in zip(CHANNEL_SUFFIXES, profile.register_profile):
            lines.extend(format_word_array("%s_%s" % (profile_name, suffix), words))
            lines.append("")

    lines.append(
        "static const SPWM_RGB_Register_Profile_View "
        "ICND1065L_REGISTER_PROFILES[] = {"
    )
    main_counts = tuple(len(words) for words in MAIN_PROFILE)
    lines.extend(
        [
            "    {%s," % format_c_string(main_profile_name),
            "     %d," % REGISTER_INDEX,
            "     {%s_r, %s_g, %s_b},"
            % (main_profile_name, main_profile_name, main_profile_name),
            "     {%d, %d, %d}}," % main_counts,
        ]
    )
    preserved_counts = tuple(len(words) for words in PRESERVED_CONFIG1_PROFILE)
    lines.extend(
        [
            "    {%s," % format_c_string(preserved_profile_name),
            "     %d," % REGISTER_INDEX,
            "     {%s_r, %s_g, %s_b},"
            % (preserved_profile_name, preserved_profile_name, preserved_profile_name),
            "     {%d, %d, %d}}," % preserved_counts,
        ]
    )
    for profile_index, profile in enumerate(profiles, start=3):
        profile_name = "icnd1065l_regtype%d" % profile_index
        counts = tuple(len(words) for words in profile.register_profile)
        lines.extend(
            [
                "    {%s," % format_c_string(profile_name),
                "     %d," % REGISTER_INDEX,
                "     {%s_r, %s_g, %s_b},"
                % (profile_name, profile_name, profile_name),
                "     {%d, %d, %d}}," % counts,
            ]
        )
    lines.extend(
        [
            "};",
            "",
            "static const size_t ICND1065L_REGISTER_PROFILE_COUNT =",
            "    sizeof(ICND1065L_REGISTER_PROFILES) /",
            "    sizeof(ICND1065L_REGISTER_PROFILES[0]);",
            "",
        ]
    )

    lines.append(
        "static const ICND1065L_Register_Test_Profile "
        "ICND1065L_REGISTER_TEST_PROFILES[] = {"
    )
    lines.extend(
        [
            "    {{%s," % format_c_string(main_profile_name),
            "      %d," % REGISTER_INDEX,
            "      {%s_r, %s_g, %s_b},"
            % (main_profile_name, main_profile_name, main_profile_name),
            "      {%d, %d, %d}}," % main_counts,
            "     %s," % format_c_string(MAIN_PROFILE_SOURCE),
            "     0,",
            "     %d," % MAIN_PROFILE_CHIP_CODE,
            "     %s}," % format_c_string(MAIN_PROFILE_SCAN_TYPE),
        ]
    )
    lines.extend(
        [
            "    {{%s," % format_c_string(preserved_profile_name),
            "      %d," % REGISTER_INDEX,
            "      {%s_r, %s_g, %s_b},"
            % (preserved_profile_name, preserved_profile_name, preserved_profile_name),
            "      {%d, %d, %d}}," % preserved_counts,
            "     %s," % format_c_string(PRESERVED_CONFIG1_SOURCE),
            "     0,",
            "     %d," % PRESERVED_CONFIG1_CHIP_CODE,
            "     %s}," % format_c_string(PRESERVED_CONFIG1_SCAN_TYPE),
        ]
    )
    for profile_index, profile in enumerate(profiles, start=3):
        profile_name = "icnd1065l_regtype%d" % profile_index
        canonical = profile.sources[0]
        counts = tuple(len(words) for words in profile.register_profile)
        lines.extend(
            [
                "    {{%s," % format_c_string(profile_name),
                "      %d," % REGISTER_INDEX,
                "      {%s_r, %s_g, %s_b},"
                % (profile_name, profile_name, profile_name),
                "      {%d, %d, %d}}," % counts,
                "     %s," % format_c_string(repository_path(canonical.source_path)),
                "     %d," % (len(profile.sources) - 1),
                "     %d," % canonical.chip_code,
                "     %s}," % format_c_string(canonical.scan_type),
            ]
        )
    lines.extend(
        [
            "};",
            "",
            "static const size_t ICND1065L_REGISTER_TEST_PROFILE_COUNT =",
            "    sizeof(ICND1065L_REGISTER_TEST_PROFILES) /",
            "    sizeof(ICND1065L_REGISTER_TEST_PROFILES[0]);",
            "",
            "}  // namespace internal",
            "}  // namespace rgb_matrix",
            "",
            "#endif  // RGBMATRIX_ICND1065L_REGISTER_PROFILES_GENERATED_H",
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
    print("ICND1065L RGB payloads extracted: %d" % extracted_source_count)
    print("unique ICND1065L RGB profiles: %d" % len(profiles))
    print(
        "generated profiles including main and preserved CONFIG1: %d"
        % (len(profiles) + 2)
    )
    print("packages skipped for other drivers: %d" % len(skipped_sources))
    skipped_chip_counts = Counter(source.chip_code for source in skipped_sources)
    for chip_code in sorted(skipped_chip_counts):
        print("  ChipCode=%d: %d" % (chip_code, skipped_chip_counts[chip_code]))


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Extract and merge ICND1065L physical RGB register profiles."
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
            print(
                "error: generated header is unavailable: %s" % error,
                file=sys.stderr,
            )
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

    # Path.write_text() only gained its newline argument in Python 3.10.
    # Path.open() keeps this tool usable on older Raspberry Pi OS releases.
    with OUTPUT_PATH.open("w", encoding="utf-8", newline="\n") as output_file:
        output_file.write(generated_header)
    print("wrote %s" % repository_path(OUTPUT_PATH))
    return 0


if __name__ == "__main__":
    sys.exit(main())
