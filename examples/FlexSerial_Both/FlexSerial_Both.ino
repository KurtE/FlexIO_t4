#include <FlexIO_t4.h>
#include "flexSerial.h"

FlexSerial SerialFlex(-1, 2); // currently setup for pin 2 TX


void setup() {
  pinMode(13, OUTPUT);
  while (!Serial && millis() < 4000);
  Serial.begin(115200);
  Serial1.begin(115200);  // lets start up Serial1, to see if we can receive anything from our FlexSerial
  delay(500);
  SerialFlex.begin(115200);

  Serial.println("End Setup");
}

uint32_t last_output = 0;

void loop() {
  int ch;
  if ((millis() - last_output) > 1000) {
    last_output = millis();
    digitalWrite(13, !digitalRead(13));
    SerialFlex.println("abcdefghijklmnopqrstuvwxyz");
  }
  if (Serial1.available()) {
    while ((ch = Serial1.read()) != -1) {
      SerialFlex.write(ch);
    }
  }

  if (SerialFlex.available()) {
    while ((ch = SerialFlex.read()) != -1) {
      Serial.write(ch);
    }
  }
  delay(500);

}
