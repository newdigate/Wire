/* lpi2c1176.h - shared LPI2C master register/clock core for the NXP MIMXRT1176.
 *
 * This project's HW-verified RT1176 LPI2C bring-up (WireIMXRT1176.cpp, MIT),
 * re-expressed as the single shared C core (Phase 3.3). Consumed by BOTH the
 * CM7 TwoWire class (which passes register addresses from imxrt1176.h) and
 * the bare-metal CM4 gate images (which pass the same addresses as literals)
 * — ending the CM7/CM4 keep-in-sync sequence duplication. Master path only:
 * the slave block (SCR/SSR/NVIC/ISR) is CM7-only and stays in the class.
 * Freestanding C11: compiles under the CM4 image flags (-ffreestanding, no
 * core headers) and inside the C++ library.
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

#ifndef LPI2C1176_H
#define LPI2C1176_H

#include <stdint.h>
#include <stddef.h>

#if defined(__cplusplus)
#define LPI2C1176_ASSERT(c, m) static_assert(c, m)
extern "C" {
#else
#define LPI2C1176_ASSERT(c, m) _Static_assert(c, m)
#endif

/* LPI2C master register-block overlay (offsets per RT1170 RM; equals the
 * master block of the core's IMXRT_LPI2C_t — cross-asserted in
 * WireIMXRT1176.cpp. The slave block stays on IMXRT_LPI2C_t: CM7-only). */
typedef struct {
	volatile uint32_t VERID;        /* 0x00 */
	volatile uint32_t PARAM;        /* 0x04 */
	volatile uint32_t r08, r0C;
	volatile uint32_t MCR;          /* 0x10 */
	volatile uint32_t MSR;          /* 0x14 */
	volatile uint32_t MIER;         /* 0x18 */
	volatile uint32_t MDER;         /* 0x1C */
	volatile uint32_t MCFGR0;       /* 0x20 */
	volatile uint32_t MCFGR1;       /* 0x24 */
	volatile uint32_t MCFGR2;       /* 0x28 */
	volatile uint32_t MCFGR3;       /* 0x2C */
	volatile uint32_t r30[4];
	volatile uint32_t MDMR;         /* 0x40 */
	volatile uint32_t r44;
	volatile uint32_t MCCR0;        /* 0x48 */
	volatile uint32_t r4C;
	volatile uint32_t MCCR1;        /* 0x50 */
	volatile uint32_t r54;
	volatile uint32_t MFCR;         /* 0x58 */
	volatile uint32_t MFSR;         /* 0x5C */
	volatile uint32_t MTDR;         /* 0x60 */
	volatile uint32_t r64[3];
	volatile uint32_t MRDR;         /* 0x70 */
} lpi2c1176_regs_t;

LPI2C1176_ASSERT(offsetof(lpi2c1176_regs_t, MCR)    == 0x10, "LPI2C MCR");
LPI2C1176_ASSERT(offsetof(lpi2c1176_regs_t, MSR)    == 0x14, "LPI2C MSR");
LPI2C1176_ASSERT(offsetof(lpi2c1176_regs_t, MCFGR1) == 0x24, "LPI2C MCFGR1");
LPI2C1176_ASSERT(offsetof(lpi2c1176_regs_t, MCCR0)  == 0x48, "LPI2C MCCR0");
LPI2C1176_ASSERT(offsetof(lpi2c1176_regs_t, MTDR)   == 0x60, "LPI2C MTDR");
LPI2C1176_ASSERT(offsetof(lpi2c1176_regs_t, MRDR)   == 0x70, "LPI2C MRDR");

/* Hardware description: CCM gate/root + SCL/SDA pads. */
typedef struct {
	volatile uint32_t *lpcg;         /* CCM LPCG DIRECT (write 1 to ungate) */
	volatile uint32_t *clock_root;   /* CCM CLOCK_ROOT CONTROL */
	uint32_t clock_root_val;
	volatile uint32_t *scl_mux;  uint32_t scl_mux_val;  volatile uint32_t *scl_pad;
	volatile uint32_t *sda_mux;  uint32_t sda_mux_val;  volatile uint32_t *sda_pad;
	volatile uint32_t *scl_select;  uint32_t scl_select_val;
	volatile uint32_t *sda_select;  uint32_t sda_select_val;
	uint32_t pad_ctl_val;            /* open-drain pad config */
} lpi2c1176_hw_t;

/* MCR */
#define LPI2C1176_MCR_MEN   (1u << 0)
#define LPI2C1176_MCR_RST   (1u << 1)
#define LPI2C1176_MCR_RTF   (1u << 8)
#define LPI2C1176_MCR_RRF   (1u << 9)
/* MSR */
#define LPI2C1176_MSR_TDF   (1u << 0)
#define LPI2C1176_MSR_RDF   (1u << 1)
#define LPI2C1176_MSR_EPF   (1u << 8)
#define LPI2C1176_MSR_SDF   (1u << 9)
#define LPI2C1176_MSR_NDF   (1u << 10)
#define LPI2C1176_MSR_ALF   (1u << 11)
#define LPI2C1176_MSR_FEF   (1u << 12)
/* MTDR commands (data in [7:0], cmd in [10:8]) */
#define LPI2C1176_TX_CMD(cmd, data)  (((uint32_t)(cmd) << 8) | ((data) & 0xFFu))
#define LPI2C1176_CMD_TXD    0u
#define LPI2C1176_CMD_RXD    1u
#define LPI2C1176_CMD_STOP   2u
#define LPI2C1176_CMD_START  4u
#define LPI2C1176_MRDR_RXEMPTY (1u << 14)

#define LPI2C1176_TIMEOUT   100000u

/* Ungate+root the clock and mux the pads (shared by master and slave init). */
void lpi2c1176_clocks_pins(const lpi2c1176_hw_t *hw);
/* clocks_pins + master reset + timing for clock_hz + enable. */
void lpi2c1176_begin(lpi2c1176_regs_t *p, const lpi2c1176_hw_t *hw,
                     uint32_t clock_hz);
void lpi2c1176_end(lpi2c1176_regs_t *p, const lpi2c1176_hw_t *hw);
/* MCFGR1 prescale + MCCR0 timing (~60% low). MEN save/restore. */
void lpi2c1176_set_clock(lpi2c1176_regs_t *p, uint32_t freq);
/* Wait until any bit in mask, or an error bit / timeout. 1 = success; on
 * failure *err: NACK -> 2 (if *err was 0xFF: address) else 3 (data);
 * ALF/FEF -> 4; timeout -> 5. Error flags are W1C'd. */
int lpi2c1176_wait_flag(lpi2c1176_regs_t *p, uint32_t mask,
                        uint32_t error_mask, uint32_t *err);
/* Flush TX/RX FIFOs + W1C latched flags after a NACK/error. */
void lpi2c1176_bus_recover(lpi2c1176_regs_t *p);
/* Polled master write: START+addr(W), per-byte TDF wait, optional STOP with
 * ACK/NACK judged at the SDF wait (watching NDF — TDF leads the ACK bit by a
 * byte-time on silicon). 0 ok / 2 addr-NACK / 3 data-NACK / 4 error / 5 timeout. */
uint32_t lpi2c1176_master_write(lpi2c1176_regs_t *p, uint8_t addr,
                               const uint8_t *data, uint32_t len, int send_stop);
/* Polled master read: (repeated-)START+addr(R), RXD N-1 encoding, per-byte
 * RDF wait, optional STOP. Returns bytes read. */
uint32_t lpi2c1176_master_read(lpi2c1176_regs_t *p, uint8_t addr,
                              uint8_t *dst, uint32_t quantity, int send_stop);

#if defined(__cplusplus)
}
#endif
#endif /* LPI2C1176_H */
