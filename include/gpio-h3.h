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

#ifndef RPI_GPIO_H3_H
#define RPI_GPIO_H3_H

#include <stdint.h>

#include <vector>

// Putting this in our namespace to not collide with other things called like
// this.
namespace rgb_matrix {

struct h3_gpio_bank {
  uint32_t cfg[4];
  uint32_t dat;
  uint32_t drv[2];
  uint32_t pull[2];
};

struct h3_hstimer_regs {
  uint32_t tmr_irq_en_reg;
  uint32_t tmr_irq_stas_reg;
  uint32_t tmr_reserved01[2];
  uint32_t tmr_ctrl_reg;
  uint32_t tmr_intv_lo_reg;
  uint32_t tmr_intv_hi_reg;
  uint32_t tmr_curnt_lo_reg;
  uint32_t tmr_curnt_hi_reg;
};

// For now, everything is initialized as output.
class GPIO {
 public:
  // Available bits that actually have pins.
  static const uint32_t kValidBits;

  GPIO();

  // Initialize before use. Returns 'true' if successful, 'false' otherwise
  // (e.g. due to a permission problem).
  bool Init();

  // Initialize outputs.
  // Returns the bits that are actually set.
  uint32_t InitOutputs(uint32_t outputs);

  // Set the bits that are '1' in the output. Leave the rest untouched.
  inline void SetBits(uint32_t value) {
    if (!value) return;

    uint32_t porta_value = (value >> porta_shift_) & porta_mask_;
    if (porta_value) {
      gpio_ports_[0 /* A */].dat |= porta_value;
    }

    uint32_t portc_value = (value >> portc_shift_) & portc_mask_;
    if (portc_value) {
      gpio_ports_[2 /* C */].dat |= portc_value;
    }

    uint32_t portd_value = (value >> portd_shift_) & portd_mask_;
    if (portd_value) {
      gpio_ports_[3 /* D */].dat |= portd_value;
    }

    uint32_t portg_value = (value >> portg_shift_) & portg_mask_;
    if (portg_value) {
      gpio_ports_[6 /* G */].dat |= portg_value;
    }
  }

  // Clear the bits that are '1' in the output. Leave the rest untouched.
  inline void ClearBits(uint32_t value) {
    if (!value) return;

    uint32_t porta_value = (value >> porta_shift_) & porta_mask_;
    if (porta_value) {
      gpio_ports_[0 /* A */].dat &= ~porta_value;
    }

    uint32_t portc_value = (value >> portc_shift_) & portc_mask_;
    if (portc_value) {
      gpio_ports_[2 /* C */].dat &= ~portc_value;
    }

    uint32_t portd_value = (value >> portd_shift_) & portd_mask_;
    if (portd_value) {
      gpio_ports_[3 /* D */].dat &= ~portd_value;
    }

    uint32_t portg_value = (value >> portg_shift_) & portg_mask_;
    if (portg_value) {
      gpio_ports_[6 /* G */].dat &= ~portg_value;
    }
  }

  // Write all the bits of "value" mentioned in "mask". Leave the rest untouched.
  inline void WriteMaskedBits(uint32_t value, uint32_t mask) {
    ClearBits(~value & mask);
    SetBits(value & mask);
  }

  inline void Write(uint32_t value) { WriteMaskedBits(value, output_bits_); }

  inline void StartTimer(unsigned ns) {
    hstimer_regs_->tmr_intv_lo_reg = ns;
    hstimer_regs_->tmr_ctrl_reg = 0b10000011;
  }

  inline void WaitTimer() {
    while ((hstimer_regs_->tmr_ctrl_reg & 1) != 0);
  }

 private:
  uint32_t output_bits_;
  volatile struct h3_gpio_bank *gpio_ports_;
  volatile struct h3_hstimer_regs *hstimer_regs_;

  /* xxx: keep in sync with union IoBits layout from framebuffer-internal.h */
  static const uint32_t porta_mask_  = 0x0307fcf;
  static const uint32_t porta_shift_ = 0;
  static const uint32_t portc_mask_  = 0x000009f;
  static const uint32_t portc_shift_ = 15;
  static const uint32_t portd_mask_  = 0x0004000;
  static const uint32_t portd_shift_ = 27 - 14;
  static const uint32_t portg_mask_  = 0x00003c0;
  static const uint32_t portg_shift_ = 23 - 6;

  void ConfOutput_(unsigned pin);
};

// A PinPulser is a utility class that pulses a GPIO pin. There can be various
// implementations.
class PinPulser {
public:
  // Factory for a PinPulser. Chooses the right implementation depending
  // on the context (CPU and which pins are affected).
  // "gpio_mask" is the mask that should be output (since we only
  //   need negative pulses, this is what it does)
  // "nano_wait_spec" contains a list of time periods we'd like
  //   invoke later. This can be used to pre-process timings if needed.
  static PinPulser *Create(GPIO *io, uint32_t gpio_mask,
                           const std::vector<int> &nano_wait_spec);

  virtual ~PinPulser() {}

  // Send a pulse with a given length (index into nano_wait_spec array).
  virtual void SendPulse(int time_spec_number) = 0;

  // If SendPulse() is asynchronously implemented, wait for pulse to finish.
  virtual void WaitPulseFinished() {}
};

}  // end namespace rgb_matrix
#endif  // RPI_GPIO_H
