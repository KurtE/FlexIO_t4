//============================================================================
// Simple output only test for Flex Serial pin defined on Pin 4
// Idea is to connect pin 38 (Serail3 RX pin) to Pin4 (Flex IO output)
// 
// And the program will hopefully echo the stuff to The serial output
//============================================================================
// Currently setup to test Teensy Micromod, on Machine Learning connet D0 to A1
#include <FlexIO_t4.h>
#include <FlexSerial.h>

FlexSerial SerialFlex(-1, 4); // currently setup for pin 4 to test MicroMod (D0)


void setup() {
  pinMode(13, OUTPUT);
  while (!Serial && millis() < 4000);

  Serial.begin(115200);
  delay(250);
  Serial.println("Start Test Serial Flex"); Serial.flush();

  Serial3.begin(115200);  // lets start up Serial3, to see if we can receive anything from our FlexSerial
  delay(500);
  Serial.println("Before SerialFlex.begin"); Serial.flush();
  SerialFlex.begin(115200);
  Serial.println("After SerialFlex.begin"); Serial.flush();

  // lets enable all of the FlexIO objects to see their parameters.  
  CCM_CCGR5 |= CCM_CCGR5_FLEXIO1(CCM_CCGR_ON);
  CCM_CCGR3 |= CCM_CCGR3_FLEXIO2(CCM_CCGR_ON);
  CCM_CCGR7 |= CCM_CCGR7_FLEXIO3(CCM_CCGR_ON);

  Serial.printf("FLEXIO1_PARAM: %x\n", FLEXIO1_PARAM); Serial.flush();
  Serial.printf("FLEXIO2_PARAM: %x\n", FLEXIO2_PARAM); Serial.flush();
  Serial.printf("FLEXIO3_PARAM: %x\n", FLEXIO3_PARAM); Serial.flush();


  Serial.println("End Setup");
}

uint8_t loop_char = 'a';
void loop() {
  digitalWrite(13, !digitalRead(13));
  SerialFlex.println("abcdefghijklmnopqrstuvwxyz");
  delay(500);

}

void serialEvent3() {
  int ch;
  while ((ch = Serial3.read()) != -1) {
    Serial.write(ch);
  }
}