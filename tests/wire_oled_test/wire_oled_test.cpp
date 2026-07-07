#include "Arduino.h"
#include "HardwareSerial.h"
#include "Wire.h"

// RT1170 EVKB = I2C SLAVE at 0x42 on the LPI2C1 Arduino header. An Arduino MKR
// Zero acts as MASTER (writes 3 bytes, then reads 4). We mirror what the master
// wrote over Serial1, and answer master reads with 0x11 0x22 0x33 0x44.
#define SLAVE_ADDR 0x42

volatile uint8_t rxbuf[8];
volatile int rxn = 0;
volatile int events = 0;

void onRecv(int n) {
	rxn = n;
	for (int i = 0; i < n && i < 8; i++) rxbuf[i] = Wire.read();
	events++;
}
void onReq(void) {
	Wire.write((uint8_t)0x11); Wire.write((uint8_t)0x22);
	Wire.write((uint8_t)0x33); Wire.write((uint8_t)0x44);
}

void setup() {
	Serial1.begin(115200);
	while (!Serial1) { }
	Wire.begin((uint8_t)SLAVE_ADDR);      // slave mode on LPI2C1 (Arduino header)
	Wire.onReceive(onRecv);
	Wire.onRequest(onReq);
	Serial1.println("RT1170 I2C slave @0x42 ready (MKR Zero = master)");
}

void loop() {
	static int last = -1;
	if (events != last) {
		last = events;
		Serial1.print("event "); Serial1.print(events);
		Serial1.print(" rxn="); Serial1.print(rxn); Serial1.print(" data:");
		for (int i = 0; i < rxn && i < 8; i++) { Serial1.print(" 0x"); Serial1.print(rxbuf[i], HEX); }
		Serial1.println();
	}
	delay(50);
}
