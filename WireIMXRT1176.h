/* WireIMXRT1176.h - TwoWire (I2C) for the NXP MIMXRT1176 (RT1170-EVKB), LPI2C.
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

#ifndef TwoWireIMXRT1176_h
#define TwoWireIMXRT1176_h

#if defined(__IMXRT1176__)

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "imxrt1176.h"
#include "core_pins.h"   // IRQ_NUMBER_t, attachInterruptVector, NVIC_*
#include "Stream.h"
#include "lpi2c1176.h"

#define BUFFER_LENGTH 32
#define WIRE_HAS_END 1
#define WIRE_IMPLEMENT_WIRE
#define WIRE_IMPLEMENT_WIRE1
#define WIRE_IMPLEMENT_WIRE2

class TwoWire : public Stream {
public:
	// Hardware description: the shared C core's desc (clock gate + clock root
	// + SCL/SDA pad config, lpi2c1176.h) plus the CM7-only IRQ binding.
	typedef struct {
		lpi2c1176_hw_t hw;                       // shared C core hardware desc
		IRQ_NUMBER_t irq;
		void (*irq_function)(void);
		uint16_t irq_priority;
	} I2C_Hardware_t;
	static const I2C_Hardware_t lpi2c1_hardware;
	static const I2C_Hardware_t lpi2c2_hardware;
	static const I2C_Hardware_t lpi2c5_hardware;

	TwoWire(uintptr_t myport, const I2C_Hardware_t &myhardware)
		: port_addr(myport), hardware(myhardware) {}

	void begin();
	void begin(uint8_t address);           // slave mode
	void begin(int address) { begin((uint8_t)address); }
	void end();
	void setClock(uint32_t frequency);
	void setSDA(uint8_t pin) {}             // fixed pin per bus; no-op (parity)
	void setSCL(uint8_t pin) {}

	void beginTransmission(uint8_t address) { tx_addr = address; tx_len = 0; }
	void beginTransmission(int address) { beginTransmission((uint8_t)address); }
	uint8_t endTransmission(uint8_t sendStop);
	uint8_t endTransmission(void) { return endTransmission((uint8_t)1); }

	uint8_t requestFrom(uint8_t address, uint8_t quantity, uint8_t sendStop);
	uint8_t requestFrom(uint8_t address, uint8_t quantity, bool sendStop) {
		return requestFrom(address, quantity, (uint8_t)(sendStop ? 1 : 0));
	}
	uint8_t requestFrom(uint8_t address, uint8_t quantity) {
		return requestFrom(address, quantity, (uint8_t)1);
	}
	uint8_t requestFrom(int address, int quantity, int sendStop) {
		return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t)(sendStop ? 1 : 0));
	}
	uint8_t requestFrom(int address, int quantity) {
		return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t)1);
	}
	uint8_t requestFrom(uint8_t addr, uint8_t qty, uint32_t iaddr, uint8_t n, uint8_t stop) {
		if (n > 0) {
			beginTransmission(addr);
			if (n > 3) n = 3;
			while (n-- > 0) write((uint8_t)(iaddr >> (n * 8)));   // internal addr, MSB first
			endTransmission((uint8_t)0);
		}
		return requestFrom(addr, qty, stop);
	}

	virtual size_t write(uint8_t data);
	virtual size_t write(const uint8_t *data, size_t len);
	virtual int available(void);
	virtual int read(void);
	virtual int peek(void);
	virtual void flush(void) {}
	void onReceive(void (*function)(int numBytes)) { on_receive = function; }
	void onRequest(void (*function)(void)) { on_request = function; }
	void handle_slave_isr();               // called by the fastrun trampolines

	// pre-1.0 compatibility shims (parity with WireIMXRT)
	void send(uint8_t b)             { write(b); }
	void send(uint8_t *s, uint8_t n) { write(s, n); }
	void send(int n)                 { write((uint8_t)n); }
	void send(char *s)               { write((uint8_t *)s, strlen(s)); }
	uint8_t receive(void) { int c = read(); return (c < 0) ? 0 : (uint8_t)c; }

	size_t write(unsigned long n) { return write((uint8_t)n); }
	size_t write(long n)          { return write((uint8_t)n); }
	size_t write(unsigned int n)  { return write((uint8_t)n); }
	size_t write(int n)           { return write((uint8_t)n); }
	using Print::write;

	IMXRT_LPI2C_t & port() { return *(IMXRT_LPI2C_t *)port_addr; }

private:
	lpi2c1176_regs_t *mp() { return (lpi2c1176_regs_t *)port_addr; }
	uintptr_t port_addr;
	const I2C_Hardware_t &hardware;

	uint8_t tx_addr = 0;
	uint8_t tx_buf[BUFFER_LENGTH];
	uint8_t tx_len = 0;
	uint8_t rx_buf[BUFFER_LENGTH];
	uint8_t rx_len = 0;
	uint8_t rx_idx = 0;
	uint32_t clock_hz = 100000;

	bool is_slave = false;
	bool in_slave_request = false;
	uint8_t slave_addr = 0;
	void (*on_receive)(int) = nullptr;
	void (*on_request)(void) = nullptr;
	uint8_t s_rx_buf[BUFFER_LENGTH];
	uint8_t s_rx_len = 0;
	uint8_t s_rx_idx = 0;
	uint8_t s_tx_buf[BUFFER_LENGTH];
	uint8_t s_tx_len = 0;
	uint8_t s_tx_idx = 0;

};

extern TwoWire Wire;
extern TwoWire Wire1;
extern TwoWire Wire2;

#endif
#endif
