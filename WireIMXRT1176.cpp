#include "WireIMXRT1176.h"

#if defined(__IMXRT1176__)

#include <string.h>

// MSR flags (imxrt_lpi2c.c contract)
#define MSR_TDF  (1u<<0)
#define MSR_RDF  (1u<<1)
#define MSR_EPF  (1u<<8)
#define MSR_SDF  (1u<<9)
#define MSR_NDF  (1u<<10)
#define MSR_ALF  (1u<<11)
#define MSR_FEF  (1u<<12)
// MCR flags
#define MCR_MEN  (1u<<0)
#define MCR_RST  (1u<<1)
#define MCR_RTF  (1u<<8)
#define MCR_RRF  (1u<<9)
// MTDR commands (data in [7:0], cmd in [10:8])
#define TX_CMD(cmd, data)  (((uint32_t)(cmd) << 8) | ((data) & 0xFF))
#define CMD_TXD    0u
#define CMD_RXD    1u
#define CMD_STOP   2u
#define CMD_START  4u
#define MRDR_RXEMPTY (1u<<14)
// Slave register bits
#define SCR_SEN  (1u<<0)
#define SCR_RST  (1u<<1)
#define SSR_TDF  (1u<<0)
#define SSR_RDF  (1u<<1)
#define SSR_AVF  (1u<<2)
#define SSR_SDF  (1u<<9)
#define SSR_BEF  (1u<<10)
#define SSR_FEF  (1u<<11)
#define SIER_TDIE (1u<<0)
#define SIER_RDIE (1u<<1)
#define SIER_AVIE (1u<<2)
#define SIER_SDIE (1u<<9)
#define SIER_BEIE (1u<<10)
#define SIER_FEIE (1u<<11)

#define WIRE_TIMEOUT 100000u

void TwoWire::begin() {
	hardware.lpcg = 1u;                                    // ungate LPI2C clock
	hardware.clock_root = hardware.clock_root_val;         // 24 MHz functional clock
	hardware.scl_mux = hardware.scl_mux_val;  hardware.scl_pad = hardware.pad_ctl_val;
	hardware.sda_mux = hardware.sda_mux_val;  hardware.sda_pad = hardware.pad_ctl_val;
	hardware.scl_select_input = hardware.scl_select_val;
	hardware.sda_select_input = hardware.sda_select_val;
	port().MCR = MCR_RST;                                  // reset the master block
	port().MCR = 0u;
	setClock(clock_hz);                                    // program MCCR0/MCFGR1
	port().MCR = MCR_MEN;                                  // enable
}

void TwoWire::end() { port().MCR = 0u; hardware.lpcg = 0u; }

void TwoWire::begin(uint8_t address) {
	is_slave = true; slave_addr = address;
	s_rx_len = 0; s_rx_idx = 0; s_tx_len = 0; s_tx_idx = 0;
	hardware.lpcg = 1u; hardware.clock_root = hardware.clock_root_val;
	hardware.scl_mux = hardware.scl_mux_val;  hardware.scl_pad = hardware.pad_ctl_val;
	hardware.sda_mux = hardware.sda_mux_val;  hardware.sda_pad = hardware.pad_ctl_val;
	hardware.scl_select_input = hardware.scl_select_val;
	hardware.sda_select_input = hardware.sda_select_val;
	port().SCR = SCR_RST; port().SCR = 0u;
	port().SAMR = ((uint32_t)address << 1);
	// SAEN (7-bit address) | RXSTALL (bit1) | TXDSTALL (bit2): clock-stretch until the
	// ISR drains SRDR / fills STDR, so multi-byte reads/writes stay byte-correct even
	// when the master clocks faster than the ISR can refill.
	port().SCFGR1 = (1u << 9) | (1u << 2) | (1u << 1);          // SAEN | TXDSTALL | RXSTALL (SDK default)
	// SCFGR2.CLKHOLD (bits[3:0]) sets the SCL hold time while stalling — MUST be
	// non-zero or TXDSTALL/RXSTALL never actually hold the clock, so the ISR can't
	// refill STDR/drain SRDR in time on multi-byte transfers. Max hold; the 996 MHz
	// ISR refills well within it. (Matches the SDK's clockHoldTime default.)
	port().SCFGR2 = 0x0000000Fu;
	// TDIE is essential: without it only the first read byte (which rides the AVF
	// interrupt) is served; bytes 2..N need a TDF interrupt each to refill STDR.
	// BEIE|FEIE are essential for recovery: a multi-byte-read glitch can latch
	// FEF (TX underrun) / BEF, which corrupts the slave FIFO and eventually wedges
	// it into permanent address-NACK. These interrupts fire the ISR to W1C the
	// error so the *next* transfer recovers cleanly (matches the SDK's IRQ handler,
	// which clears BitErr/FifoErr on every interrupt).
	port().SIER = SIER_TDIE | SIER_RDIE | SIER_AVIE | SIER_SDIE | SIER_BEIE | SIER_FEIE;
	attachInterruptVector(hardware.irq, hardware.irq_function);
	NVIC_SET_PRIORITY(hardware.irq, hardware.irq_priority);
	NVIC_ENABLE_IRQ(hardware.irq);
	port().SCR = SCR_SEN | (1u << 4);   // SEN | FILTEN (SDK default)
}

// Runs from ITCM (.fastrun): the slave ISR must refill STDR within the bounded
// SCFGR2.CLKHOLD clock-stretch window, so it can't tolerate flash-XIP fetch stalls.
__attribute__((section(".fastrun")))
void TwoWire::handle_slave_isr() {
	uint32_t ssr = port().SSR;
	if (ssr & (SSR_BEF | SSR_FEF)) {              // latched slave error -> W1C + re-arm
		port().SSR = (SSR_BEF | SSR_FEF);        // clear so the slave FIFO recovers
		s_rx_len = 0; s_rx_idx = 0; s_tx_idx = 0; s_tx_len = 0;
	}
	if (ssr & SSR_AVF) { volatile uint32_t sasr = port().SASR; (void)sasr; s_rx_len = 0; s_rx_idx = 0; }   // new transfer (read of SASR clears AVF)
	if (ssr & SSR_RDF) {                                                 // master wrote a byte
		uint8_t d = (uint8_t)port().SRDR;
		if (s_rx_len < BUFFER_LENGTH) s_rx_buf[s_rx_len++] = d;
	}
	if (ssr & SSR_TDF) {                                                 // master wants a byte (TXDSTALL holds SCL until we write STDR)
		if (s_tx_idx == 0 && on_request) {
			s_tx_len = 0; in_slave_request = true; on_request(); in_slave_request = false;
		}
		port().STDR = (s_tx_idx < s_tx_len) ? s_tx_buf[s_tx_idx++] : 0xFFu;
	}
	if (ssr & SSR_SDF) {                                                 // STOP -> transfer done
		port().SSR = SSR_SDF;                                            // W1C
		if (s_rx_len && on_receive) { s_rx_idx = 0; on_receive(s_rx_len); }
		s_rx_len = 0; s_rx_idx = 0; s_tx_idx = 0; s_tx_len = 0;
	}
}

void TwoWire::setClock(uint32_t freq) {
	clock_hz = freq;
	const uint32_t src = 24000000u;
	uint32_t pre = 0, div = 0;
	for (pre = 0; pre < 8u; pre++) { div = (src >> pre) / freq; if (div <= 120u) break; }
	uint32_t clklo = (div * 6u) / 10u;                // ~60% low time (I2C tLOW>tHIGH)
	uint32_t clkhi = (div > clklo) ? (div - clklo) : 1u;
	if (clklo > 63u) clklo = 63u;
	if (clkhi > 63u) clkhi = 63u; if (clkhi < 1u) clkhi = 1u;
	uint32_t men = port().MCR & MCR_MEN;
	port().MCR = men & ~MCR_MEN;                       // MCCR/MCFGR need MEN=0
	port().MCFGR1 = (port().MCFGR1 & ~0x7u) | (pre & 0x7u);
	port().MCCR0 = (clklo) | (clkhi << 8) | ((clkhi/2u) << 16) | ((clkhi/2u) << 24);
	if (men) port().MCR = MCR_MEN;
}

// Wait until any bit in `mask` is set, or an error bit appears / timeout.
// Returns true on success. On error/timeout sets err to a nonzero Arduino status.
bool TwoWire::wait_flag(uint32_t mask, uint32_t error_mask, uint32_t &err) {
	for (uint32_t g = 0; g < WIRE_TIMEOUT; g++) {
		uint32_t s = port().MSR;
		if (s & error_mask) {
			if (s & MSR_NDF) err = (err == 0xFFu) ? 2u : 3u; // addr vs data NACK
			else err = 4u;                                    // ALF/FEF/other
			port().MSR = s;                                   // W1C the flags
			return false;
		}
		if (s & mask) return true;
	}
	err = 5u;                                                 // timeout
	return false;
}

size_t TwoWire::write(uint8_t data) {
	if (in_slave_request) { if (s_tx_len >= BUFFER_LENGTH) return 0; s_tx_buf[s_tx_len++] = data; return 1; }
	if (tx_len >= BUFFER_LENGTH) return 0;
	tx_buf[tx_len++] = data; return 1;
}
size_t TwoWire::write(const uint8_t *data, size_t len) {
	size_t n = 0; while (n < len && write(data[n])) n++; return n;
}

// After a NACK/error, flush the TX/RX FIFOs so the next transaction starts
// clean (essential when scanning many addresses back-to-back).
void TwoWire::bus_recover() {
	port().MCR = MCR_MEN | MCR_RTF | MCR_RRF;   // reset TX+RX FIFOs, stay enabled
	port().MCR = MCR_MEN;
	port().MSR = port().MSR;                     // W1C any latched flags
}

uint8_t TwoWire::endTransmission(uint8_t sendStop) {
	uint32_t err = 0xFFu;                                     // 0xFF => a NACK now is an address NACK (2)
	port().MSR = port().MSR;                                  // clear stale flags
	port().MTDR = TX_CMD(CMD_START, (tx_addr << 1) | 0u);     // START + addr(W)
	// IMPORTANT: do NOT treat TDF here as "address ACKed". TDF (TX-FIFO ready)
	// asserts a byte-time BEFORE the ACK bit is sampled on silicon, so racing it
	// against NDF makes every address look ACKed (breaks bus scanning). The ACK/
	// NACK is judged at completion (STOP) below, matching the NXP SDK.
	for (uint8_t i = 0; i < tx_len; i++) {
		if (!wait_flag(MSR_TDF, MSR_NDF | MSR_ALF | MSR_FEF, err)) { bus_recover(); tx_len = 0; return (uint8_t)err; }
		err = 0u;                                             // past the address; a NACK now is a data NACK (3)
		port().MTDR = TX_CMD(CMD_TXD, tx_buf[i]);
	}
	if (sendStop) {
		port().MTDR = TX_CMD(CMD_STOP, 0);
		// Completion is the correct point to judge ACK/NACK -- watch NDF here.
		if (!wait_flag(MSR_SDF, MSR_NDF | MSR_ALF | MSR_FEF, err)) { bus_recover(); tx_len = 0; return (uint8_t)err; }
		port().MSR = MSR_SDF | MSR_EPF;
	}
	tx_len = 0;
	return 0u;
}

uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity, uint8_t sendStop) {
	if (quantity > BUFFER_LENGTH) quantity = BUFFER_LENGTH;
	rx_len = 0; rx_idx = 0;
	uint32_t err = 0xFFu;
	port().MSR = port().MSR;
	port().MTDR = TX_CMD(CMD_START, (address << 1) | 1u);     // START + addr(R)
	if (!wait_flag(MSR_TDF, MSR_NDF | MSR_ALF | MSR_FEF, err)) { if (sendStop) port().MTDR = TX_CMD(CMD_STOP,0); return 0; }
	port().MTDR = TX_CMD(CMD_RXD, (uint8_t)(quantity - 1));   // receive `quantity` bytes (N-1 encoding)
	for (uint8_t i = 0; i < quantity; i++) {
		err = 0u;
		if (!wait_flag(MSR_RDF, MSR_ALF | MSR_FEF, err)) break;
		uint32_t r = port().MRDR;
		if (r & MRDR_RXEMPTY) break;
		rx_buf[rx_len++] = (uint8_t)(r & 0xFF);
	}
	if (sendStop) {
		port().MTDR = TX_CMD(CMD_STOP, 0);
		wait_flag(MSR_SDF, MSR_ALF | MSR_FEF, err);
		port().MSR = MSR_SDF | MSR_EPF;
	}
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
	/* lpcg */ CCM_LPCG98_DIRECT,
	/* clock_root */ CCM_CLOCK_ROOT37_CONTROL, /* clock_root_val */ 0u,
	/* scl */ IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_08, 0x11u, IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_08,
	/* sda */ IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_09, 0x11u, IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_09,
	/* scl_select */ IOMUXC_LPI2C1_SCL_SELECT_INPUT, 0u,
	/* sda_select */ IOMUXC_LPI2C1_SDA_SELECT_INPUT, 0u,
	/* pad_ctl_val */ 0x0000001Eu,
	/* irq */ IRQ_LPI2C1, /* irq_function */ wire_isr, /* irq_priority */ 16u,
};

// LPI2C2: QEMU-loopback slave persona only (no physical EVKB pins). Pin refs
// bind to LPI2C1's IOMUXC regs (inert in QEMU). CLOCK_ROOT38 / LPCG99.
const TwoWire::I2C_Hardware_t TwoWire::lpi2c2_hardware = {
	/* lpcg */ CCM_LPCG99_DIRECT,
	/* clock_root */ CCM_CLOCK_ROOT38_CONTROL, /* clock_root_val */ 0u,
	/* scl */ IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_08, 0x11u, IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_08,
	/* sda */ IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_09, 0x11u, IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_09,
	/* scl_select */ IOMUXC_LPI2C1_SCL_SELECT_INPUT, 0u,
	/* sda_select */ IOMUXC_LPI2C1_SDA_SELECT_INPUT, 0u,
	/* pad_ctl_val */ 0x0000001Eu,
	/* irq */ IRQ_LPI2C2, /* irq_function */ wire1_isr, /* irq_priority */ 16u,
};

// LPI2C5: onboard eCompass/WM8962 codec bus on GPIO_LPSR_05 (SCL) /
// GPIO_LPSR_04 (SDA), ALT0|SION (0x10). LPSR-domain pad 0x0A. CLOCK_ROOT41
// (mux 1) / LPCG102.
const TwoWire::I2C_Hardware_t TwoWire::lpi2c5_hardware = {
	/* lpcg */ CCM_LPCG102_DIRECT,
	/* clock_root */ CCM_CLOCK_ROOT41_CONTROL, /* clock_root_val */ (1u << 8),
	/* scl */ IOMUXC_SW_MUX_CTL_PAD_GPIO_LPSR_05, 0x10u, IOMUXC_SW_PAD_CTL_PAD_GPIO_LPSR_05,
	/* sda */ IOMUXC_SW_MUX_CTL_PAD_GPIO_LPSR_04, 0x10u, IOMUXC_SW_PAD_CTL_PAD_GPIO_LPSR_04,
	/* scl_select */ IOMUXC_LPI2C5_SCL_SELECT_INPUT, 0u,
	/* sda_select */ IOMUXC_LPI2C5_SDA_SELECT_INPUT, 0u,
	/* pad_ctl_val */ 0x0000000Au,
	/* irq */ IRQ_LPI2C5, /* irq_function */ wire2_isr, /* irq_priority */ 16u,
};

TwoWire Wire(IMXRT_LPI2C1_ADDRESS, TwoWire::lpi2c1_hardware);
TwoWire Wire1(IMXRT_LPI2C2_ADDRESS, TwoWire::lpi2c2_hardware);
TwoWire Wire2(IMXRT_LPI2C5_ADDRESS, TwoWire::lpi2c5_hardware);

__attribute__((section(".fastrun"))) static void wire_isr()  { Wire.handle_slave_isr(); }
__attribute__((section(".fastrun"))) static void wire1_isr() { Wire1.handle_slave_isr(); }
__attribute__((section(".fastrun"))) static void wire2_isr() { Wire2.handle_slave_isr(); }

#endif // __IMXRT1176__
