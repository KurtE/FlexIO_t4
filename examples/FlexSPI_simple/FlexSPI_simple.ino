#include <FlexIO_t4.h>
#include <FlexSPI.h>

FlexSPI SPIFLEX(2, 3, 4, 5); // Setup on (int mosiPin, int sckPin, int misoPin, int csPin=-1) :


void setup() {
  pinMode(13, OUTPUT);
  while (!Serial && millis() < 4000);
  Serial.begin(115200);
  delay(500);
  SPIFLEX.begin();

  // See if we can update the speed...
  SPIFLEX.flexIOHandler()->setClockSettings(3, 2, 0);	// clksel(0-3PLL4, Pll3 PFD2 PLL5, *PLL3_sw)

  Serial.printf("Updated Flex IO speed: %u\n", SPIFLEX.flexIOHandler()->computeClockRate());

  Serial.println("End Setup");
}
uint8_t buf[] = "abcdefghijklmnopqrstuvwxyz";
uint16_t ret_buf[256];
uint8_t ch_out = 0;

void loop() {
	SPIFLEX.beginTransaction(FlexSPISettings(20000000, MSBFIRST, SPI_MODE0));
	for (uint8_t ch_out = 0; ch_out < 64; ch_out++) {
		ret_buf[ch_out] = SPIFLEX.transfer(ch_out);
	}
//	Serial.println();
	delay(25);

	uint8_t index = 0;
	for (uint16_t ch_out = 0; ch_out < 500; ch_out+=25) {
	  ret_buf[index++] = SPIFLEX.transfer16(ch_out);	  
	}
	delay(25);
	SPIFLEX.transfer(buf, NULL, sizeof(buf));
	SPIFLEX.endTransaction();
	delay(500);
}