# AGENTS.md

## Project scope

This repository is a fork of Hzeller's Raspberry Pi RGB LED matrix library. It
adds experimental support for newer SPWM-style LED panel controllers and for
Raspberry Pi 5 GPIO through RP1.

The currently registered SPWM controller profiles are FM6353, FM6363, FM6373,
ICND1065L, and SM16380SH. The SPWM implementation is profile-based so new
controllers can reuse the shared upload, row-address, initialization, and
register-configuration machinery.

Raspberry Pi 5 support has an important boundary:

- Non-SPWM panels can use supported RP1 RIO or RP1 PIO configurations.
- SPWM panels currently support the RP1 RIO GPIO path only. Omit
  `--led-rp1-pio` or use `--led-rp1-pio=0`.
- `--led-rp1-pio=1` is not supported for SPWM panels.

## Non-negotiable agent rules

- Never try to compile the program.
- Never add new test units.
- Keep SPWM and RP1 functionality in their dedicated directories whenever
  possible. Minimize changes to the library's core files.
- Preserve existing comments unless the related behavior changes. Update stale
  comments when changing the behavior they describe.
- Keep core-library changes compatible with C++11 because `lib/Makefile`
  builds the library with `-std=c++11`.
- Never invent register values, latch timing, scan behavior, or panel geometry.
  Require evidence from a datasheet, a known-good implementation, a captured
  waveform, or working hardware.
- Clearly label inferred or unverified hardware values.
- When register initialization is required, provide at most one validated
  built-in main register configuration. Do not invent additional variants.

## Architecture boundaries

### SPWM

The principal SPWM extension points are:

- `lib/spwm/spwm-panel-config.cc` and `.h` contain panel settings, initialization
  sequences, profile lookup, and the `SPWM_PANEL_PROFILES` registry.
- `lib/spwm/spwm-panel-registers.cc` and `.h` contain built-in register payloads,
  per-register latch timing, and register-config factories.
- `lib/spwm/spwm-helpers.cc` and `.h` contain shared SPWM configuration, upload,
  row-address, parsing, and runtime behavior. Keep panel-specific tables out of
  these files.
- `lib/spwm/registertest/` contains the optional runtime profile loader and
  Demo 15 register-tuning tool.
- `lib/spwm/registertest/data/` contains owner-maintained `.profiles` catalogs;
  its `README.md` documents their format.

Use the existing shared functions, data structures, enums, and constants before
adding new abstractions.

Register payload selection is intentionally independent of row-address
transport; do not duplicate register configs solely for different
`--led-spwm-row-addr-type` values without hardware evidence.

The register-test catalogs and Demo 15 integration are not required for ordinary new-controller support.

Only place genuinely controller-specific waveform or protocol code under
`lib/spwm/driver/<canonical-panel-name>/`. Put code under
`lib/spwm/driver/shared/` only when at least two drivers need it or when it is
clearly protocol-generic. Do not create a new driver directory for a register
variant that the existing profile and register mechanisms can express.

### RP1

The principal RP1 files are under `lib/rp1/`:

- `rp1_backend.cc` selects and validates RP1 backends.
- `rp1_rio_backend.cc` and `.h` implement the RIO path.
- `rp1_pio_backend.cc`, `.h`, and `rp1_pio_support.c` implement the PIO path.
- `rp1_spwm_gpio.cc` and `.h` adapt the existing software SPWM upload path to
  Pi 5 GPIO access.

A normal SPWM panel-profile addition should not require changes under
`lib/rp1/`. Change RP1 code only when the shared backend cannot express a
required waveform or GPIO behavior.

Treat `lib/rp1/rp1_pio_vendor/` as vendored code. Do not edit it for ordinary
panel support.

## Decide what kind of SPWM change is needed

Choose the smallest applicable change:

1. An existing controller with different geometry or timing should first use
   existing flags, `SPWM_Panel_Settings`, or a narrowly scoped profile resolver.
2. A new controller using the existing SPWM waveform needs a panel profile,
   initialization sequence, register factory, and, when required, one
   evidence-backed built-in main register configuration.
3. When another user supplies one validated register configuration, use it as
   the built-in main configuration selected by `--led-spwm-register-config=0`.
   Do not duplicate it into a `.profiles` catalog or invent alternatives.
4. A controller requiring a genuinely different waveform or transport may need
   isolated code under `lib/spwm/driver/<canonical-panel-name>/`.
5. Runtime catalogs and Demo 15 are not a normal requirement for a new controller.
6. Shared transport behavior belongs in existing SPWM helpers only when it is
   not tied to one controller, or you think would be used by other controllers.

## Adding a new SPWM controller

### 1. Establish evidence and naming

Before editing code, record:

- Controller and receiver-chip names.
- Panel dimensions, scan ratio, and physical shift-chain geometry.
- Row-address transport and data layout.
- Register slot count and whether each slot is fixed or rotating RGB.
- Register words, latch timing, and ordered initialization pulses.
- OE timing and any controller-specific blanking behavior.
- The source of every hardware value.

Use a lowercase canonical name for `--led-panel-type` and the panel profile.
Runtime lookup uses case-insensitive prefix matching so suffix variants are
accepted. Do not add canonical names that are prefixes of one another without
first resolving the lookup ambiguity. If the repository owner later creates a
catalog, it must use the same canonical name for its key and filename.

### 2. Add the panel profile and initialization sequence

In `lib/spwm/spwm-panel-config.cc`:

- Start from `spwm_make_default_panel_settings()` and override only values that
  differ.
- Define the controller's `SPWM_Panel_Settings`.
- Define the ordered `SPWM_Init_Step` array and `SPWM_Init_Sequence`.
- Use `SPWM_INIT_STEP_REGISTER` for fixed slots and
  `SPWM_INIT_STEP_RGB_REGISTER` for rotating per-channel slots.
- Ensure each register step refers to a register slot supplied by the register
  factory.
- Add the controller to `SPWM_PANEL_PROFILES`.
- Keep FM6373 first unless deliberately changing the fallback profile.
- Add a width- or geometry-specific resolver only when the static settings
  cannot represent the hardware.

Keep register payloads out of `spwm-panel-config.cc`.

### 3. Add one built-in register configuration when required

In `lib/spwm/spwm-panel-registers.h` and `.cc`:

- Declare and implement the controller's `spwm_create_<panel>_config()` factory.
- If the controller requires register initialization, define one evidence-backed
  built-in main configuration. That one logical configuration may contain all
  required register slots and words.
- For each required slot, define its `SPWM_Register_Timing`, LAT sequence, and
  fixed or rotating-RGB `SPWM_Register_Config_Entry`.
- Confirm the register entries, initialization steps, payload kind, and latch
  timing agree.

The built-in main configuration is selected by
`--led-spwm-register-config=0` (and by the internal unset/default value). It is
the only register configuration normally expected from a contributor adding a
controller. If register initialization is required but no validated
configuration is supplied, do not fabricate one; surface the missing evidence
to the repository owner.

### 4. Register-test catalogs and Demo 15

`lib/spwm/registertest/` is owner-maintained diagnostic tooling. Its runtime
`.profiles` catalogs provide curated diagnostic register configurations, often
the main configuration plus alternatives. Positive
`--led-spwm-register-config=N` values select catalog entries, and Demo 15 lets
the owner step through them on connected hardware.

Catalog and Demo 15 work is not required for ordinary new-controller support.
Do not modify catalogs, loader tables, Demo 15 integration, or related help text
unless the repository owner explicitly requests it and supplies or approves the
configurations. Keep a single contributor-supplied configuration solely as
built-in selection `0`; do not create a one-entry catalog or invent alternatives.
For explicitly requested catalog work, follow
`lib/spwm/registertest/data/README.md`.

### 5. Update user-visible names and documentation

Update every applicable general `--led-panel-type` support list and relevant
non-Demo usage example:

- `lib/options-initialize.cc`
- `bindings/python/samples/samplebase.py`
- `README.md`
- `spwm.md`
- `examples-api-use/README.md`
- `utils/README.md`
- `AGENTS.md` (current-support list)

Some of these files also contain Demo 15-specific panel lists. Leave those
lists unchanged unless the repository owner explicitly requested catalog and
Demo 15 integration.

Use `rg` with the new canonical name and with a nearby existing controller name
to find integration points that may have been added since this guide was
written.

Document at least one hardware-verified command with its geometry, row-address
type, scan count, data layout, and register selection. Normal new-controller
documentation should use the built-in main selection
`--led-spwm-register-config=0`.

Keep detailed run and tuning commands in `README.md` and `spwm.md` rather than
duplicating them here.

### 6. Register any new source files with the build systems

No build-manifest change is needed when only the existing centralized SPWM files
are edited.

If a new `.cc` file is added, update all applicable explicit source lists:

- `CMakeLists.txt`
- `lib/CMakeLists.txt`
- `lib/Makefile`
- `examples-api-use/Makefile` when the new source is demo-only

Keep Make object paths, dependency paths, shared-library inputs, and cleanup
paths consistent. Do not add register-test or catalog build integration as part
of ordinary new-controller support.

## Current limits and assumptions

Before adapting hardware that exceeds current behavior, inspect these limits:

- Numbered `--led-spwm-force-registerN` handling currently exposes slots 1
  through 6 via `SPWM_FORCE_REGISTER_COUNT`.
- `SPWM_Panel_Settings` currently stores at most four missing-column positions.
- Panel names use case-insensitive prefix matching.
- Full-height data layouts 1 through 5 require row-address type 1 or 2,
  `--led-spwm-scan` equal to `--led-rows`, and no multiplexing. Split layouts 1
  and 2 also require panel columns divisible by 32.

If new hardware exceeds a limit, make the limit change explicit and review all
consumers. Do not silently truncate values or hide the issue in panel-specific
code.

## Verification

### Static verification required from agents

Because agents must not compile or add tests, use static checks:

- Confirm the canonical panel name is registered in the panel profile, general
  CLI help, bindings, and user documentation. Do not require catalog or Demo 15
  entries.
- Confirm every initialization-sequence register slot exists in the built-in
  register configuration.
- Confirm there is no more than one contributor-provided built-in main register
  configuration and that its fixed versus rotating-RGB slot types agree with
  the initialization sequence.
- Confirm every new source file is present in all applicable Make and CMake
  source lists.
- Confirm factory declarations and definitions use matching signatures.
- Confirm ordinary controller work did not modify `lib/spwm/registertest/`,
  runtime catalogs, or Demo 15 integration without an explicit owner request.
- For explicitly owner-requested catalog work only, confirm the catalog header,
  payload kind, record count, ordering, scan tags, register slots, loader
  tables, and Demo 15 lists are internally consistent.
- Use `rg` to find stale names, missing references, and duplicated supported
  lists.
- Run `git diff --check`.
- Inspect `git status --short` for unintended or untracked files.
- Review the complete diff for unrelated changes.

Report explicitly that compilation and hardware testing were not performed.

### Hardware verification required before claiming support

A human maintainer must compile and perform hardware testing outside the agent
workflow.

A maintainer with the physical panel should record:

- Controller, receiver chip, panel dimensions, and scan ratio.
- Row-address type, data layout, and register-config selection, normally the
  built-in main configuration `0` for a new controller.
- Raspberry Pi model, GPIO mapping, slowdown, and RP1 backend.
- Whether testing used one panel, chained panels, or parallel outputs.

The maintainer should verify:

- Reliable startup and register initialization.
- Stable refresh without tearing, flicker, or top/bottom frame misalignment.
- Brightness, blanking, ghosting, and behavior during sustained refresh.
- Correct row selection, panel width, and all visible columns.
- Correct red, green, and blue routing.
- Relevant direct and shift-register row-address modes.
- Chaining or parallel output when support is claimed.
- Cold starts, repeated process restart, and clean shutdown behavior.
- Raspberry Pi 5 through RP1 RIO for SPWM. Do not claim RP1 PIO support until it
  is separately implemented and verified.

Do not describe a controller or panel as supported until working-hardware
evidence is available.

## General coding guidelines

- Keep original comments where applicable, unless needing to update them.
- Ensure any new function has a comment briefly describing what it does.
- State assumptions before implementation when hardware behavior is unclear.
- Surface multiple plausible interpretations instead of choosing silently.
- Identify the closest existing panel profile before designing new behavior.
- Prefer the minimum change that solves the demonstrated requirement.
- Do not add speculative flexibility or abstractions for one use.
- Touch only files required by the task and match the surrounding style.
- Do not refactor adjacent code or remove pre-existing dead code.
- Remove imports, variables, functions, or files made unused by your own change.
- Add a descriptive comment for every new function.
- Comments should explain hardware behavior and why a value or sequence is
  required; Git history already records that a change occurred.
- For multi-step work, state a short plan with static success criteria and
  verify each item before finishing.
