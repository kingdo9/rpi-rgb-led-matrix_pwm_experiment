# AGENTS.md

Fork of the Hzeller library to control commonly available 128x64, 64x64, 32x32 or 16x32 RGB LED panels with the Raspberry Pi.

The goal of this fork is to try to get new SPWM style led display drivers such as FM6373, FM6363 to be supported. These specific two are now currently supported.
Another goal is to ensure Raspberry Pi5 is supported using the RP1 RIO and PIO modes --led-rp1-pio=0 --led-rp1-pio=1.

* Never try to compile the program
* Please keep comments relevant, keep them as they are in general but also adjust them to make them up-to-date.
* when adding, editing anything related to the new functionality of SPWM displays or Pi5 RP1 , the goal is to keep the code in their separate folders / files and minimise how much we have in the libraries core files. Example keep in ./lib/spwm and ./lib/rp1 where possible.

majority of spwm and rp1 pi5 related files:

SPWM:
lib/spwm/registertest/data/README.md
lib/spwm/registertest/data/fm6353.profiles
lib/spwm/registertest/data/fm6363.profiles
lib/spwm/registertest/data/fm6373.profiles
lib/spwm/registertest/data/icnd1065l.profiles
lib/spwm/registertest/data/sm16380sh.profiles
lib/spwm/registertest/spwm-register-profile-loader.cc
lib/spwm/registertest/spwm-register-profile-loader.h
lib/spwm/registertest/spwm-register-test.cc
lib/spwm/registertest/spwm-register-test.h
lib/spwm/spwm-helpers.cc
lib/spwm/spwm-helpers.h
lib/spwm/spwm-panel-config.cc
lib/spwm/spwm-panel-config.h
lib/spwm/spwm-panel-registers.cc
lib/spwm/spwm-panel-registers.h

RP1:
lib/rp1/rp1_backend.cc
lib/rp1/rp1_backend.h
lib/rp1/rp1_pio_backend.cc
lib/rp1/rp1_pio_backend.h
lib/rp1/rp1_pio_support.c
lib/rp1/rp1_pio_vendor/include/piomatter/protomatter.pio.h
lib/rp1/rp1_pio_vendor/piolib/include/hardware/clocks.h
lib/rp1/rp1_pio_vendor/piolib/include/hardware/gpio.h
lib/rp1/rp1_pio_vendor/piolib/include/hardware/pio.h
lib/rp1/rp1_pio_vendor/piolib/include/hardware/pio_instructions.h
lib/rp1/rp1_pio_vendor/piolib/include/hardware/regs/proc_pio.h
lib/rp1/rp1_pio_vendor/piolib/include/pio_platform.h
lib/rp1/rp1_pio_vendor/piolib/include/piolib.h
lib/rp1/rp1_pio_vendor/piolib/include/piolib_priv.h
lib/rp1/rp1_pio_vendor/piolib/include/rp1_pio_if.h
lib/rp1/rp1_pio_vendor/piolib/pio_rp1.c
lib/rp1/rp1_pio_vendor/piolib/piolib.c
lib/rp1/rp1_rio_backend.cc
lib/rp1/rp1_rio_backend.h
lib/rp1/rp1_spwm_gpio.cc
lib/rp1/rp1_spwm_gpio.h

The new style SPWM displays currently use parameter examples like below.
--led-panel-type=fm6373 --led-spwm-row-addr-type=0
--led-panel-type=fm6363 --led-spwm-row-addr-type=1
--led-panel-type=icnd1065l --led-spwm-row-addr-type=2 --led-spwm-scan=43
--led-panel-type=fm6373 --led-spwm-row-addr-type=1 --led-spwm-scan=64 --led-spwm-data-layout=1 --led-spwm-register-config=2
--led-panel-type=fm6363 --led-spwm-row-addr-type=1

If adding any new spwm driver support, please try to use spwm functions, constants that already exist.
If any new functions / files need to be created specific to a panel, please use ./lib/spwm/driver/[driver_panel_name]/ and comment the additions / amendments.
If any new functions / files need to be created which could be shared on future drivers, please use ./lib/spwm/driver/shared/ and comment the additions / amendments.

---

Behavioral guidelines to reduce common LLM coding mistakes. Merge with project-specific instructions as needed.

**Tradeoff:** These guidelines bias toward caution over speed. For trivial tasks, use judgment.
- Never try to compile the program.
- Never add new test units.

## 1. Think Before Coding

**Don't assume. Don't hide confusion. Surface tradeoffs.**

Before implementing:
- If unsure, state your assumptions explicitly.
- If multiple interpretations exist, present them - don't pick silently.
- If a simpler approach exists, say so. Push back when warranted.
- If something is unclear, stop. Name what's confusing. Ask.

## 2. Simplicity First

**Minimum code that solves the problem. Nothing speculative.**

- No features beyond what was asked.
- No abstractions for single-use code.
- No "flexibility" or "configurability" that wasn't requested.
- No error handling for impossible scenarios.
- If you write 200 lines and it could be 50, rewrite it.

Ask yourself: "Would a senior engineer say this is overcomplicated?" If yes, simplify.

## 3. Surgical Changes

**Touch only what you must. Clean up only your own mess.**

When editing existing code:
- Don't "improve" adjacent code, comments, or formatting.
- Don't refactor things that aren't broken.
- Match existing style, even if you'd do it differently.
- If you notice unrelated dead code, mention it - don't delete it.

When your changes create orphans:
- Remove imports/variables/functions that YOUR changes made unused.
- Don't remove pre-existing dead code unless asked.

The test: Every changed line should trace directly to the user's request.

## 4. Goal-Driven Execution

**Define success criteria. Loop until verified.**

Transform tasks into verifiable goals:
- "Add validation" → "Write tests for invalid inputs, then make them pass"
- "Fix the bug" → "Write a test that reproduces it, then make it pass"
- "Refactor X" → "Ensure tests pass before and after"

For multi-step tasks, state a brief plan:
```
1. [Step] → verify: [check]
2. [Step] → verify: [check]
3. [Step] → verify: [check]
```

Strong success criteria let you loop independently. Weak criteria ("make it work") require constant clarification.

## 5. Comments

Add comments where you think it would be beneficial to describe and to help keep the project maintainable.
Any new function please add comment to describe it.

---

**These guidelines are working if:** fewer unnecessary changes in diffs, fewer rewrites due to overcomplication, and clarifying questions come before implementation rather than after mistakes.
