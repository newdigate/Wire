/* WireIMXRT1176.cpp - TwoWire (I2C) for the NXP MIMXRT1176 (RT1170-EVKB), LPI2C.
 *
 * Dedicated implementation for the imxrt1176 Arduino core: written against
 * the documented Arduino Wire API, register logic from this project's
 * HW-verified RT1176 LPI2C bring-up (developed from the NXP MCUXpresso SDK,
 * BSD-3-Clause). Not derived from the LGPL Arduino TwoWire/twi files in this
 * repository -- see LICENSE.md.
 *
 * Copyright (c) 2026 Nicholas Newdigate
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include "WireIMXRT1176.h"

#if defined(__IMXRT1176__)

#include <string.h>

// Register logic + bit macros live ONCE in the shared C core lpi2c1176.{h,c}
// (master paths Phase 3.3, slave init Phase 4.2) — consumed by this class
// (addresses from imxrt1176.h via hardware.hw) and by the CM4 gate images
// (same addresses as literals). This file keeps the Arduino API surface,
// buffers, the NVIC hookup, and the CM7 slave ISR body below.

#include <stddef.h>

// The shared C core's master overlay must equal the core header's.
static_assert(offsetof(lpi2c1176_regs_t, MCR)    == offsetof(IMXRT_LPI2C_t, MCR),    "MCR");
static_assert(offsetof(lpi2c1176_regs_t, MSR)    == offsetof(IMXRT_LPI2C_t, MSR),    "MSR");
static_assert(offsetof(lpi2c1176_regs_t, MCFGR1) == offsetof(IMXRT_LPI2C_t, MCFGR1), "MCFGR1");
static_assert(offsetof(lpi2c1176_regs_t, MCCR0)  == offsetof(IMXRT_LPI2C_t, MCCR0),  "MCCR0");
static_assert(offsetof(lpi2c1176_regs_t, MTDR)   == offsetof(IMXRT_LPI2C_t, MTDR),   "MTDR");
static_assert(offsetof(lpi2c1176_regs_t, MRDR)   == offsetof(IMXRT_LPI2C_t, MRDR),   "MRDR");

void TwoWire::begin() {
	lpi2c1176_begin(mp(), &hardware.hw, clock_hz);
}

void TwoWire::end() { lpi2c1176_end(mp(), &hardware.hw); }

void TwoWire::begin(uint8_t address) {
	is_slave = true; slave_addr = address;
	s_rx_len = 0; s_rx_idx = 0; s_tx_len = 0; s_tx_idx = 0;
	lpi2c1176_slave_config(mp(), &hardware.hw, address);
	attachInterruptVector(hardware.irq, hardware.irq_function);
	NVIC_SET_PRIORITY(hardware.irq, hardware.irq_priority);
	NVIC_ENABLE_IRQ(hardware.irq);
	lpi2c1176_slave_enable(mp());
}

// Runs from ITCM (.fastrun): the slave ISR must refill STDR within the bounded
// SCFGR2.CLKHOLD clock-stretch window, so it can't tolerate flash-XIP fetch stalls.
__attribute__((section(".fastrun")))
void TwoWire::handle_slave_isr() {
	uint32_t ssr = port().SSR;
	if (ssr & (LPI2C1176_SSR_BEF | LPI2C1176_SSR_FEF)) {          // latched slave error -> W1C + re-arm
		port().SSR = (LPI2C1176_SSR_BEF | LPI2C1176_SSR_FEF);    // clear so the slave FIFO recovers
		s_rx_len = 0; s_rx_idx = 0; s_tx_idx = 0; s_tx_len = 0;
	}
	if (ssr & LPI2C1176_SSR_AVF) { volatile uint32_t sasr = port().SASR; (void)sasr; s_rx_len = 0; s_rx_idx = 0; }   // new transfer (read of SASR clears AVF)
	if (ssr & LPI2C1176_SSR_RDF) {                                       // master wrote a byte
		uint8_t d = (uint8_t)port().SRDR;
		if (s_rx_len < BUFFER_LENGTH) s_rx_buf[s_rx_len++] = d;
	}
	if (ssr & LPI2C1176_SSR_TDF) {                                       // master wants a byte (TXDSTALL holds SCL until we write STDR)
		if (s_tx_idx == 0 && on_request) {
			s_tx_len = 0; in_slave_request = true; on_request(); in_slave_request = false;
		}
		port().STDR = (s_tx_idx < s_tx_len) ? s_tx_buf[s_tx_idx++] : 0xFFu;
	}
	if (ssr & LPI2C1176_SSR_SDF) {                                       // STOP -> transfer done
		port().SSR = LPI2C1176_SSR_SDF;                                  // W1C
		if (s_rx_len && on_receive) { s_rx_idx = 0; on_receive(s_rx_len); }
		s_rx_len = 0; s_rx_idx = 0; s_tx_idx = 0; s_tx_len = 0;
	}
}

void TwoWire::setClock(uint32_t freq) {
	clock_hz = freq;
	lpi2c1176_set_clock(mp(), freq);
}

size_t TwoWire::write(uint8_t data) {
	if (in_slave_request) { if (s_tx_len >= BUFFER_LENGTH) return 0; s_tx_buf[s_tx_len++] = data; return 1; }
	if (tx_len >= BUFFER_LENGTH) return 0;
	tx_buf[tx_len++] = data; return 1;
}
size_t TwoWire::write(const uint8_t *data, size_t len) {
	size_t n = 0; while (n < len && write(data[n])) n++; return n;
}

// The ACK/NACK-at-STOP judgement and the post-NACK FIFO flush live in the
// shared core (lpi2c1176_master_write / lpi2c1176_bus_recover).
uint8_t TwoWire::endTransmission(uint8_t sendStop) {
	uint8_t r = (uint8_t)lpi2c1176_master_write(mp(), tx_addr, tx_buf, tx_len, sendStop);
	tx_len = 0;
	return r;
}

uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity, uint8_t sendStop) {
	if (quantity > BUFFER_LENGTH) quantity = BUFFER_LENGTH;
	rx_idx = 0;
	rx_len = (uint8_t)lpi2c1176_master_read(mp(), address, rx_buf, quantity, sendStop);
	return rx_len;
}

int TwoWire::available(void) {
	return is_slave ? (s_rx_len - s_rx_idx) : (rx_len - rx_idx);
}
int TwoWire::read(void) {
	if (is_slave) return (s_rx_idx < s_rx_len) ? s_rx_buf[s_rx_idx++] : -1;
	return (rx_idx < rx_len) ? rx_buf[rx_idx++] : -1;
}
int TwoWire::peek(void) {
	if (is_slave) return (s_rx_idx < s_rx_len) ? s_rx_buf[s_rx_idx] : -1;
	return (rx_idx < rx_len) ? rx_buf[rx_idx] : -1;
}

static void wire_isr();    // forward decls
static void wire1_isr();
static void wire2_isr();

// EVKB Arduino-header I2C = LPI2C1 on GPIO_AD_08 (SCL) / GPIO_AD_09 (SDA),
// ALT1|SION (0x11). Pad 0x1E = ODE|DSE|PUE|PUS (internal pull-up). CLOCK_ROOT37
// (24 MHz) / LPCG98. Values verbatim from the HW-verified Wire_instances.cpp.
const TwoWire::I2C_Hardware_t TwoWire::lpi2c1_hardware = {
	{ /* lpcg */ &CCM_LPCG98_DIRECT,
	  /* clock_root */ &CCM_CLOCK_ROOT37_CONTROL, /* clock_root_val */ 0u,
	  /* scl */ &IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_08, 0x11u, &IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_08,
	  /* sda */ &IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_09, 0x11u, &IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_09,
	  /* scl_select */ &IOMUXC_LPI2C1_SCL_SELECT_INPUT, 0u,
	  /* sda_select */ &IOMUXC_LPI2C1_SDA_SELECT_INPUT, 0u,
	  /* pad_ctl_val */ 0x0000001Eu },
	/* irq */ IRQ_LPI2C1, /* irq_function */ wire_isr, /* irq_priority */ 16u,
};

// LPI2C2: QEMU-loopback slave persona only (no physical EVKB pins). Pin refs
// bind to LPI2C1's IOMUXC regs (inert in QEMU). CLOCK_ROOT38 / LPCG99.
const TwoWire::I2C_Hardware_t TwoWire::lpi2c2_hardware = {
	{ /* lpcg */ &CCM_LPCG99_DIRECT,
	  /* clock_root */ &CCM_CLOCK_ROOT38_CONTROL, /* clock_root_val */ 0u,
	  /* scl */ &IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_08, 0x11u, &IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_08,
	  /* sda */ &IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_09, 0x11u, &IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_09,
	  /* scl_select */ &IOMUXC_LPI2C1_SCL_SELECT_INPUT, 0u,
	  /* sda_select */ &IOMUXC_LPI2C1_SDA_SELECT_INPUT, 0u,
	  /* pad_ctl_val */ 0x0000001Eu },
	/* irq */ IRQ_LPI2C2, /* irq_function */ wire1_isr, /* irq_priority */ 16u,
};

// LPI2C5: onboard eCompass/WM8962 codec bus on GPIO_LPSR_05 (SCL) /
// GPIO_LPSR_04 (SDA), ALT0|SION (0x10). LPSR-domain pad 0x0A. CLOCK_ROOT41
// (mux 1) / LPCG102.
const TwoWire::I2C_Hardware_t TwoWire::lpi2c5_hardware = {
	{ /* lpcg */ &CCM_LPCG102_DIRECT,
	  /* clock_root */ &CCM_CLOCK_ROOT41_CONTROL, /* clock_root_val */ (1u << 8),
	  /* scl */ &IOMUXC_SW_MUX_CTL_PAD_GPIO_LPSR_05, 0x10u, &IOMUXC_SW_PAD_CTL_PAD_GPIO_LPSR_05,
	  /* sda */ &IOMUXC_SW_MUX_CTL_PAD_GPIO_LPSR_04, 0x10u, &IOMUXC_SW_PAD_CTL_PAD_GPIO_LPSR_04,
	  /* scl_select */ &IOMUXC_LPI2C5_SCL_SELECT_INPUT, 0u,
	  /* sda_select */ &IOMUXC_LPI2C5_SDA_SELECT_INPUT, 0u,
	  /* pad_ctl_val */ 0x0000000Au },
	/* irq */ IRQ_LPI2C5, /* irq_function */ wire2_isr, /* irq_priority */ 16u,
};

TwoWire Wire(IMXRT_LPI2C1_ADDRESS, TwoWire::lpi2c1_hardware);
TwoWire Wire1(IMXRT_LPI2C2_ADDRESS, TwoWire::lpi2c2_hardware);
TwoWire Wire2(IMXRT_LPI2C5_ADDRESS, TwoWire::lpi2c5_hardware);

__attribute__((section(".fastrun"))) static void wire_isr()  { Wire.handle_slave_isr(); }
__attribute__((section(".fastrun"))) static void wire1_isr() { Wire1.handle_slave_isr(); }
__attribute__((section(".fastrun"))) static void wire2_isr() { Wire2.handle_slave_isr(); }

#endif // __IMXRT1176__
