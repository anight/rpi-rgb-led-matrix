// -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
// Copyright (C) 2013 Henner Zeller <h.zeller@acm.org>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation version 2.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://gnu.org/licenses/gpl-2.0.txt>

#define __STDC_FORMAT_MACROS
#include <inttypes.h>

#include "gpio-h3.h"

#include <assert.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>
#include <errno.h>

#define H3_MMAP1_BASE         0x01c20000 /* for gpio and ccu */
#define H3_MMAP1_SIZE             0x1000
#define H3_MMAP1_OFFSET_GPIO       0x800
#define H3_MMAP1_OFFSET_CCU          0x0

#define H3_MMAP2_BASE         0x01c60000 /* for hstimer */
#define H3_MMAP2_SIZE             0x1000
#define H3_MMAP2_OFFSET_HSTIMER      0x0


#define H3_GPIO_PIN_PORTA(pin) (0x00 + pin)
#define H3_GPIO_PIN_PORTB(pin) (0x20 + pin)
#define H3_GPIO_PIN_PORTC(pin) (0x40 + pin)
#define H3_GPIO_PIN_PORTD(pin) (0x60 + pin)
#define H3_GPIO_PIN_PORTE(pin) (0x80 + pin)
#define H3_GPIO_PIN_PORTF(pin) (0xa0 + pin)
#define H3_GPIO_PIN_PORTG(pin) (0xc0 + pin)

#define PIN_UNUSED (-1)

namespace rgb_matrix {

/* xxx: keep in sync with union IoBits layout from framebuffer-internal.h */
const uint32_t GPIO::kValidBits
= ((1 <<  0) | (1 <<  1) | (1 <<  2) | (1 <<  3) |
   (1 <<  6) | (1 <<  7) | (1 <<  8) | (1 <<  9) |
   (1 << 10) | (1 << 11) | (1 << 12) | (1 << 13) |
   (1 << 14) | (1 << 15) | (1 << 16) | (1 << 17) |
   (1 << 18) | (1 << 19) | (1 << 20) | (1 << 21) |
   (1 << 22) | (1 << 23) | (1 << 24) | (1 << 25) |
   (1 << 26) | (1 << 27)
);

GPIO::GPIO() : output_bits_(0), gpio_ports_(NULL), hstimer_regs_(NULL) {
}

void GPIO::ConfOutput_(unsigned pin) {
  unsigned bank = pin >> 5;
  unsigned index = (pin & 0x1f) >> 3;
  unsigned offset = ((pin & 0x1f) & 0x7) << 2;

  volatile struct h3_gpio_bank *b = gpio_ports_ + bank;

  const unsigned cfg_output = 1;

  uint32_t cfg = b->cfg[index];
  cfg &= ~(0xf << offset);
  cfg |= cfg_output << offset;

  b->cfg[index] = cfg;
}

uint32_t GPIO::InitOutputs(uint32_t outputs) {
  assert(gpio_ports_ != NULL);

  outputs &= kValidBits;   // Sanitize input.
  output_bits_ = outputs;

  /* xxx: keep in sync with union IoBits layout from framebuffer-internal.h */
  static int pins[32] = {
    H3_GPIO_PIN_PORTA(0),  // 0
    H3_GPIO_PIN_PORTA(1),  // 1
    H3_GPIO_PIN_PORTA(2),  // 2
    H3_GPIO_PIN_PORTA(3),  // 3
    PIN_UNUSED,            // 4
    PIN_UNUSED,            // 5
    H3_GPIO_PIN_PORTA(6),  // 6
    H3_GPIO_PIN_PORTA(7),  // 7
    H3_GPIO_PIN_PORTA(8),  // 8
    H3_GPIO_PIN_PORTA(9),  // 9
    H3_GPIO_PIN_PORTA(10), // 10
    H3_GPIO_PIN_PORTA(11), // 11
    H3_GPIO_PIN_PORTA(12), // 12
    H3_GPIO_PIN_PORTA(13), // 13
    H3_GPIO_PIN_PORTA(14), // 14
    H3_GPIO_PIN_PORTC(0),  // 15
    H3_GPIO_PIN_PORTC(1),  // 16
    H3_GPIO_PIN_PORTC(2),  // 17
    H3_GPIO_PIN_PORTC(3),  // 18
    H3_GPIO_PIN_PORTC(4),  // 19
    H3_GPIO_PIN_PORTA(20), // 20
    H3_GPIO_PIN_PORTA(21), // 21
    H3_GPIO_PIN_PORTC(7),  // 22
    H3_GPIO_PIN_PORTG(6),  // 23
    H3_GPIO_PIN_PORTG(7),  // 24
    H3_GPIO_PIN_PORTG(8),  // 25
    H3_GPIO_PIN_PORTG(9),  // 26
    H3_GPIO_PIN_PORTD(14), // 27
    PIN_UNUSED,            // 28
    PIN_UNUSED,            // 29
    PIN_UNUSED,            // 30
    PIN_UNUSED,            // 31
  };

  for (unsigned b = 0; b < 32; ++b) {
    if (outputs & (1 << b)) {
      assert(pins[b] != PIN_UNUSED);
      ConfOutput_(pins[b]);
    }
  }
  return output_bits_;
}

bool GPIO::Init() {
  int fd = open("/dev/mem", O_RDWR | O_SYNC);
  if (0 > fd) {
    fprintf(stderr, "open(\"/dev/mem\") failed: %s (%d)", strerror(errno), errno);
    return false;
  }

  void *mem;

  mem = mmap(NULL, H3_MMAP1_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, H3_MMAP1_BASE);
  if (MAP_FAILED == mem) {
    fprintf(stderr, "mmap() failed: %s (%d)\n", strerror(errno), errno);
    return false;
  }

  gpio_ports_ = (volatile struct h3_gpio_bank *) ((char *) mem + H3_MMAP1_OFFSET_GPIO);

  struct ccu_s {
    uint32_t stuff[0x60 / 4];
    uint32_t bus_clk_gating_reg0;
    uint32_t stuff2[(0x2c0 - 0x64) / 4];
    uint32_t bus_soft_rst_reg0;
  };

  volatile struct ccu_s *ccu = (volatile struct ccu_s *) ((char *) mem + H3_MMAP1_OFFSET_CCU);

  ccu->bus_clk_gating_reg0 |= (1 << 19);
  ccu->bus_soft_rst_reg0 |= (1 << 19);

  mem = mmap(NULL, H3_MMAP2_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, H3_MMAP2_BASE);
  if (MAP_FAILED == mem) {
    fprintf(stderr, "mmap() failed: %s (%d)\n", strerror(errno), errno);
    return false;
  }

  close(fd);

  hstimer_regs_ = (volatile struct h3_hstimer_regs *) ((char *) mem + H3_MMAP2_OFFSET_HSTIMER);

  hstimer_regs_->tmr_ctrl_reg = 0;

  return true;
}

/*
 * We support also other pinouts that don't have the OE- on the hardware
 * PWM output pin, so we need to provide (impefect) 'manual' timing as well.
 * Hence all various sleep_nano() implementations depending on the hardware.
 */

// --- PinPulser. Private implementation parts.
namespace {

class TimerBasedPinPulser : public PinPulser {
public:
  TimerBasedPinPulser(GPIO *io, uint32_t bits,
                      const std::vector<int> &nano_specs)
    : io_(io), bits_(bits), nano_specs_(nano_specs) {}

  virtual void SendPulse(int time_spec_number) {
    io_->ClearBits(bits_);
    io_->StartTimer(nano_specs_[time_spec_number]);
  }

  virtual void WaitPulseFinished() {
    io_->WaitTimer();
    io_->SetBits(bits_);
  }


private:
  GPIO *const io_;
  const uint32_t bits_;
  const std::vector<int> nano_specs_;
};

} // end anonymous namespace

// Public PinPulser factory
PinPulser *PinPulser::Create(GPIO *io, uint32_t gpio_mask,
                             const std::vector<int> &nano_wait_spec) {
  // The only implementation so far.
  return new TimerBasedPinPulser(io, gpio_mask, nano_wait_spec);
}

} // namespace rgb_matrix
