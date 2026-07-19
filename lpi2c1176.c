/* lpi2c1176.c - shared LPI2C master register/clock core for the NXP MIMXRT1176.
 *
 * This project's HW-verified RT1176 LPI2C bring-up (WireIMXRT1176.cpp, MIT),
 * re-expressed as the single shared C core (Phase 3.3): the sequence bodies
 * below are that file's begin()/setClock()/wait_flag()/bus_recover()/
 * endTransmission()/requestFrom() master paths verbatim. Phase 4.2 adds the
 * slave init (slave_config/slave_enable), moved verbatim from
 * TwoWire::begin(uint8_t); the slave ISR bodies stay per-consumer. Consumed
 * by the CM7 TwoWire class and the CM4 gate images.
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

#include "lpi2c1176.h"

void lpi2c1176_clocks_pins(const lpi2c1176_hw_t *hw)
{
	*hw->lpcg = 1u;                              /* ungate LPI2C clock */
	*hw->clock_root = hw->clock_root_val;        /* 24 MHz functional clock */
	*hw->scl_mux = hw->scl_mux_val;  *hw->scl_pad = hw->pad_ctl_val;
	*hw->sda_mux = hw->sda_mux_val;  *hw->sda_pad = hw->pad_ctl_val;
	*hw->scl_select = hw->scl_select_val;
	*hw->sda_select = hw->sda_select_val;
}

void lpi2c1176_set_clock(lpi2c1176_regs_t *p, uint32_t freq)
{
	const uint32_t src = 24000000u;
	uint32_t pre = 0, div = 0;
	for (pre = 0; pre < 8u; pre++) { div = (src >> pre) / freq; if (div <= 120u) break; }
	uint32_t clklo = (div * 6u) / 10u;           /* ~60% low time (I2C tLOW>tHIGH) */
	uint32_t clkhi = (div > clklo) ? (div - clklo) : 1u;
	if (clklo > 63u) clklo = 63u;
	if (clkhi > 63u) clkhi = 63u;
	if (clkhi < 1u) clkhi = 1u;
	uint32_t men = p->MCR & LPI2C1176_MCR_MEN;
	p->MCR = men & ~LPI2C1176_MCR_MEN;           /* MCCR/MCFGR need MEN=0 */
	p->MCFGR1 = (p->MCFGR1 & ~0x7u) | (pre & 0x7u);
	p->MCCR0 = (clklo) | (clkhi << 8) | ((clkhi / 2u) << 16) | ((clkhi / 2u) << 24);
	if (men) p->MCR = LPI2C1176_MCR_MEN;
}

void lpi2c1176_begin(lpi2c1176_regs_t *p, const lpi2c1176_hw_t *hw,
                     uint32_t clock_hz)
{
	lpi2c1176_clocks_pins(hw);
	p->MCR = LPI2C1176_MCR_RST;                  /* reset the master block */
	p->MCR = 0u;
	lpi2c1176_set_clock(p, clock_hz);            /* program MCCR0/MCFGR1 */
	p->MCR = LPI2C1176_MCR_MEN;                  /* enable */
}

void lpi2c1176_end(lpi2c1176_regs_t *p, const lpi2c1176_hw_t *hw)
{
	p->MCR = 0u;
	*hw->lpcg = 0u;
}

int lpi2c1176_wait_flag(lpi2c1176_regs_t *p, uint32_t mask,
                        uint32_t error_mask, uint32_t *err)
{
	for (uint32_t g = 0; g < LPI2C1176_TIMEOUT; g++) {
		uint32_t s = p->MSR;
		if (s & error_mask) {
			if (s & LPI2C1176_MSR_NDF) *err = (*err == 0xFFu) ? 2u : 3u; /* addr vs data NACK */
			else *err = 4u;                       /* ALF/FEF/other */
			p->MSR = s;                           /* W1C the flags */
			return 0;
		}
		if (s & mask) return 1;
	}
	*err = 5u;                                    /* timeout */
	return 0;
}

void lpi2c1176_bus_recover(lpi2c1176_regs_t *p)
{
	p->MCR = LPI2C1176_MCR_MEN | LPI2C1176_MCR_RTF | LPI2C1176_MCR_RRF; /* reset FIFOs, stay enabled */
	p->MCR = LPI2C1176_MCR_MEN;
	p->MSR = p->MSR;                              /* W1C any latched flags */
}

uint32_t lpi2c1176_master_write(lpi2c1176_regs_t *p, uint8_t addr,
                               const uint8_t *data, uint32_t len, int send_stop)
{
	uint32_t err = 0xFFu;                        /* a NACK now is an address NACK (2) */
	p->MSR = p->MSR;                             /* clear stale flags */
	p->MTDR = LPI2C1176_TX_CMD(LPI2C1176_CMD_START, (uint32_t)(addr << 1) | 0u);
	/* IMPORTANT: do NOT treat TDF here as "address ACKed". TDF (TX-FIFO ready)
	 * asserts a byte-time BEFORE the ACK bit is sampled on silicon, so racing
	 * it against NDF makes every address look ACKed (breaks bus scanning).
	 * The ACK/NACK is judged at completion (STOP) below, matching the NXP SDK. */
	for (uint32_t i = 0; i < len; i++) {
		if (!lpi2c1176_wait_flag(p, LPI2C1176_MSR_TDF,
		        LPI2C1176_MSR_NDF | LPI2C1176_MSR_ALF | LPI2C1176_MSR_FEF, &err)) {
			lpi2c1176_bus_recover(p);
			return err;
		}
		err = 0u;                                /* past the address; a NACK now is a data NACK (3) */
		p->MTDR = LPI2C1176_TX_CMD(LPI2C1176_CMD_TXD, data[i]);
	}
	if (send_stop) {
		p->MTDR = LPI2C1176_TX_CMD(LPI2C1176_CMD_STOP, 0);
		/* Completion is the correct point to judge ACK/NACK -- watch NDF here. */
		if (!lpi2c1176_wait_flag(p, LPI2C1176_MSR_SDF,
		        LPI2C1176_MSR_NDF | LPI2C1176_MSR_ALF | LPI2C1176_MSR_FEF, &err)) {
			lpi2c1176_bus_recover(p);
			return err;
		}
		p->MSR = LPI2C1176_MSR_SDF | LPI2C1176_MSR_EPF;
	}
	return 0u;
}

uint32_t lpi2c1176_master_read(lpi2c1176_regs_t *p, uint8_t addr,
                              uint8_t *dst, uint32_t quantity, int send_stop)
{
	uint32_t err = 0xFFu, n = 0;
	p->MSR = p->MSR;
	p->MTDR = LPI2C1176_TX_CMD(LPI2C1176_CMD_START, (uint32_t)(addr << 1) | 1u);
	if (!lpi2c1176_wait_flag(p, LPI2C1176_MSR_TDF,
	        LPI2C1176_MSR_NDF | LPI2C1176_MSR_ALF | LPI2C1176_MSR_FEF, &err)) {
		if (send_stop) p->MTDR = LPI2C1176_TX_CMD(LPI2C1176_CMD_STOP, 0);
		return 0;
	}
	p->MTDR = LPI2C1176_TX_CMD(LPI2C1176_CMD_RXD, (uint8_t)(quantity - 1)); /* N-1 encoding */
	for (uint32_t i = 0; i < quantity; i++) {
		err = 0u;
		if (!lpi2c1176_wait_flag(p, LPI2C1176_MSR_RDF,
		        LPI2C1176_MSR_ALF | LPI2C1176_MSR_FEF, &err)) break;
		uint32_t r = p->MRDR;
		if (r & LPI2C1176_MRDR_RXEMPTY) break;
		dst[n++] = (uint8_t)(r & 0xFFu);
	}
	if (send_stop) {
		p->MTDR = LPI2C1176_TX_CMD(LPI2C1176_CMD_STOP, 0);
		lpi2c1176_wait_flag(p, LPI2C1176_MSR_SDF,
		        LPI2C1176_MSR_ALF | LPI2C1176_MSR_FEF, &err);
		p->MSR = LPI2C1176_MSR_SDF | LPI2C1176_MSR_EPF;
	}
	return n;
}

void lpi2c1176_slave_config(lpi2c1176_regs_t *p, const lpi2c1176_hw_t *hw,
                            uint8_t address)
{
	lpi2c1176_clocks_pins(hw);
	p->SCR = LPI2C1176_SCR_RST;
	p->SCR = 0u;
	p->SAMR = ((uint32_t)address << 1);
	/* SAEN (7-bit address) | RXSTALL (bit1) | TXDSTALL (bit2): clock-stretch until
	 * the ISR drains SRDR / fills STDR, so multi-byte reads/writes stay
	 * byte-correct even when the master clocks faster than the ISR can refill. */
	p->SCFGR1 = (1u << 9) | (1u << 2) | (1u << 1);
	/* SCFGR2.CLKHOLD (bits[3:0]) sets the SCL hold time while stalling — MUST be
	 * non-zero or TXDSTALL/RXSTALL never actually hold the clock, so the ISR
	 * can't refill STDR/drain SRDR in time on multi-byte transfers. Max hold. */
	p->SCFGR2 = 0x0000000Fu;
	/* TDIE is essential: without it only the first read byte (which rides the
	 * AVF interrupt) is served; bytes 2..N need a TDF interrupt each to refill
	 * STDR.  BEIE|FEIE are essential for recovery: a glitch can latch FEF/BEF,
	 * which corrupts the slave FIFO and wedges it into permanent address-NACK;
	 * the ISR W1Cs them so the *next* transfer recovers cleanly. */
	p->SIER = LPI2C1176_SIER_TDIE | LPI2C1176_SIER_RDIE | LPI2C1176_SIER_AVIE
	        | LPI2C1176_SIER_SDIE | LPI2C1176_SIER_BEIE | LPI2C1176_SIER_FEIE;
}

void lpi2c1176_slave_enable(lpi2c1176_regs_t *p)
{
	p->SCR = LPI2C1176_SCR_SEN | LPI2C1176_SCR_FILTEN;
}
