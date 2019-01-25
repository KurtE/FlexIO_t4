#include <FlexIO_t4.h>
#include <FlexSPI.h>

FlexSPI SPIFLEX(2, 3, 4, 5); // Setup on (int mosiPin, int sckPin, int misoPin, int csPin=-1) :


void setup() {
  pinMode(13, OUTPUT);
  while (!Serial && millis() < 4000);
  Serial.begin(115200);
  delay(500);
  SPIFLEX.begin();

  Serial.println("End Setup");
}
uint8_t buf[] = "abcdefghijklmnopqrstuvwxyz";
uint16_t ret_buf[256];
uint8_t ch_out = 0;

void loop() {
	for (uint8_t ch_out = 0; ch_out < 64; ch_out++) {
		ret_buf[ch_out] = SPIFLEX.transfer(ch_out);
	}
//	Serial.println();
	delay(25);

	uint8_t index = 0;
	for (uint16_t ch_out = 0; ch_out < 500; ch_out+=25) {
	  ret_buf[ch_out] = SPIFLEX.transfer16(ch_out);	  
	}
	delay(25);
	SPIFLEX.transfer(buf, NULL, sizeof(buf));
	delay(500);
}
