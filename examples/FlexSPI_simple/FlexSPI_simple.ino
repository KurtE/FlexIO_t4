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

uint8_t ch_out = 0;

void loop() {
  Serial.printf("%02x ", SPIFLEX.transfer(ch_out++));
  if ((ch_out & 0x1f) == 0) Serial.println();
  delay(250);
}
