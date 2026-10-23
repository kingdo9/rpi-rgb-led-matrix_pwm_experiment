// -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
// RP1 physical addresses exceed 32 bits, including on 32-bit userspace.
#ifndef _LARGEFILE64_SOURCE
#define _LARGEFILE64_SOURCE
#endif

#include "rp1_spwm_gpio.h"

#include <fcntl.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <unistd.h>

namespace rgb_matrix {
namespace internal {
namespace {

#define GPIO_BIT(x) (1ull << x)

// SPWM-on-Pi5 reuses the existing software SPWM upload path, but GPIO writes
// are backed by RP1 RIO registers instead of the legacy BCM GPIO block.
static const uint32_t kRp1GpioFunctionSysRio = 5;
static const uint32_t kRp1PadFastDrive = 0x15;
static const off64_t kRp1RioPeripheralBase = 0x1f000d0000ll;
static const size_t kRp1RioMapSizeBytes = 0x40000;
static const uint32_t kRp1GpioOffsetWords = 0x00000 / 4;
static const uint32_t kRp1RioOffsetWords = 0x10000 / 4;
static const uint32_t kRp1PadOffsetWords = 0x20000 / 4;
static const uint32_t kRp1GpioCount = 28;

struct Rp1GpioCtrlRegs {
  volatile uint32_t status;
  volatile uint32_t ctrl;
};

struct Rp1RioRegs {
  volatile uint32_t Out;
  volatile uint32_t OE;
  volatile uint32_t In;
  volatile uint32_t InSync;
};

static volatile uint32_t *s_RP1_registers = NULL;
static Rp1GpioCtrlRegs *s_RP1_gpio_regs = NULL;
static volatile uint32_t *s_RP1_pad_regs = NULL;
static Rp1RioRegs *s_RP1_rio_out = NULL;
static Rp1RioRegs *s_RP1_rio_set = NULL;
static Rp1RioRegs *s_RP1_rio_clr = NULL;

static volatile uint32_t *TryMmapRp1Registers(const char *path, off64_t offset) {
  const int fd = open(path, O_RDWR | O_SYNC);
  if (fd < 0) return NULL;

  void *map = mmap64(NULL, kRp1RioMapSizeBytes, PROT_READ | PROT_WRITE,
                   MAP_SHARED, fd, offset);
  close(fd);
  if (map == MAP_FAILED) return NULL;
  return static_cast<volatile uint32_t *>(map);
}

static bool MmapAllRp1RioRegistersOnce() {
  if (s_RP1_registers != NULL) return true;

  static const struct {
    const char *path;
    off64_t offset;
  } candidates[] = {
      {"/dev/gpiomem0", 0},
      {"/dev/gpiomem0", kRp1RioPeripheralBase},
      {"/dev/mem", kRp1RioPeripheralBase},
  };

  for (size_t i = 0; i < sizeof(candidates) / sizeof(candidates[0]); ++i) {
    s_RP1_registers =
        TryMmapRp1Registers(candidates[i].path, candidates[i].offset);
    if (s_RP1_registers != NULL) break;
  }
  if (s_RP1_registers == NULL) {
    fprintf(stderr,
            "Pi 5-family RP1 RIO GPIO: could not map RP1 GPIO registers via "
            "/dev/gpiomem0 or /dev/mem.\n");
    return false;
  }

  s_RP1_gpio_regs =
      reinterpret_cast<Rp1GpioCtrlRegs *>(
          const_cast<uint32_t *>(s_RP1_registers + kRp1GpioOffsetWords));
  s_RP1_pad_regs =
      const_cast<uint32_t *>(s_RP1_registers + kRp1PadOffsetWords + 1);
  s_RP1_rio_out =
      reinterpret_cast<Rp1RioRegs *>(
          const_cast<uint32_t *>(s_RP1_registers + kRp1RioOffsetWords));
  s_RP1_rio_set =
      reinterpret_cast<Rp1RioRegs *>(
          const_cast<uint32_t *>(s_RP1_registers + kRp1RioOffsetWords +
                                 (0x2000 / 4)));
  s_RP1_rio_clr =
      reinterpret_cast<Rp1RioRegs *>(
          const_cast<uint32_t *>(s_RP1_registers + kRp1RioOffsetWords +
                                 (0x3000 / 4)));
  return true;
}

static void ValidateRp1GpioRangeOrQuit(const char *label, gpio_bits_t bits) {
  const gpio_bits_t rp1_supported_bits = GPIO_BIT(kRp1GpioCount) - 1;
  const gpio_bits_t unavailable_bits = bits & ~rp1_supported_bits;
  if (unavailable_bits) {
    fprintf(stderr,
            "Pi 5-family RP1 RIO GPIO supports GPIO bits 0..27; "
            "%s mask 0x%llx uses unavailable bits 0x%llx.\n",
            label, (unsigned long long)bits,
            (unsigned long long)unavailable_bits);
    abort();
  }
}

}  // namespace

bool Rp1SpwmGpioInit(Rp1SpwmGpioRegisters *registers) {
  if (!MmapAllRp1RioRegistersOnce()) {
    return false;
  }

  if (registers != NULL) {
    registers->write_bits = &s_RP1_rio_out->Out;
    registers->set_bits = &s_RP1_rio_set->Out;
    registers->clear_bits = &s_RP1_rio_clr->Out;
    registers->read_bits = &s_RP1_rio_out->In;
  }
  return true;
}

bool Rp1SpwmGpioIsInitialized() {
  return s_RP1_registers != NULL;
}

gpio_bits_t Rp1SpwmGpioInitOutputs(gpio_bits_t outputs,
                                   bool adafruit_pwm_transition_hack_needed) {
  if (!Rp1SpwmGpioIsInitialized()) {
    fprintf(stderr, "Attempt to init RP1 RIO outputs but not yet Init()-ialized.\n");
    return 0;
  }

  ValidateRp1GpioRangeOrQuit("output", outputs);

  if (adafruit_pwm_transition_hack_needed) {
    s_RP1_rio_clr->OE = GPIO_BIT(4) | GPIO_BIT(18);
  }

  for (int b = 0; b < static_cast<int>(kRp1GpioCount); ++b) {
    if (outputs & GPIO_BIT(b)) {
      s_RP1_gpio_regs[b].ctrl = kRp1GpioFunctionSysRio;
      s_RP1_pad_regs[b] = kRp1PadFastDrive;
      s_RP1_rio_set->OE = GPIO_BIT(b);
    }
  }
  return outputs;
}

gpio_bits_t Rp1SpwmGpioRequestInputs(gpio_bits_t inputs) {
  if (!Rp1SpwmGpioIsInitialized()) {
    fprintf(stderr, "Attempt to init RP1 RIO inputs but not yet Init()-ialized.\n");
    return 0;
  }

  ValidateRp1GpioRangeOrQuit("input", inputs);
  for (int b = 0; b < static_cast<int>(kRp1GpioCount); ++b) {
    if (inputs & GPIO_BIT(b)) {
      s_RP1_gpio_regs[b].ctrl = kRp1GpioFunctionSysRio;
      s_RP1_rio_clr->OE = GPIO_BIT(b);
    }
  }
  return inputs;
}

}  // namespace internal
}  // namespace rgb_matrix
