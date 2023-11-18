// FlexSerial Loopback example
//
// This example tests FlexSerial by passing text through Serial1.  A test
// message is periodically transmitted by FlexSerial.  It is then received
// and retransmitted by Serial1.  Then the Serial1 output is received by
// FlexSerial and printed to USB Serial.  When you see the correct message
// in the Arduino Serial Monitor, you have confirmed FlexSerial is able
// to communicate properly back and forth with Hardware Serial (Serial1).
//
// Required hardware connections:
//
//   Connect pin 0 (RX1) to pin 2 (FlexSerial TX)
//   Connect pin 1 (TX1) to pin 3 (FlexSerial RX)
//
// Pins usable for FlexSerial
//
//   Teensy 4.0: 2-23,26,27,32-33
//   Teensy 4.1: 2-23,26,27,32-41,49,50,52,54
//   MicroMod:   2-23,26,27,32-33,40-45

#include <FlexSerial.h>

FlexSerial MySerial(3, 2); // pin 3 receive, pin 2 transmit

elapsedMillis output_timer;

void setup() {
  const int baud = 115200;
  pinMode(13, OUTPUT);
  while (!Serial && millis() < 4000);
  Serial.begin(baud);
  Serial.println("FlexSerial Loopback Example");
  Serial1.begin(baud);
  // FlexIO slowest baud rate is 58824 with default clock.  Use these for slower baud.
  //MySerial.setClock(9795918);             // 19200 minimum
  //MySerial.setClockUsingVideoPLL(921600); // 1800 minimum
  MySerial.begin(baud);
  Serial.println("End Setup - LED should blink, and if loopback works you should see test message");
  output_timer = 0;
}


void loop() {
  int ch;

  // periodically transmit a message on FlexSerial (pin 2)
  if (output_timer >= 1000) {
    output_timer = 0;
    digitalToggle(13);
    MySerial.println("abcdefghijklmnopqrstuvwxyz");
  }

  // when Serial1 receives the message (pin 0) echo it back (pin 1)
  if (Serial1.available()) {
    ch = Serial1.read();
    Serial1.write(ch);
  }

  // when FlexSerial receives the echoed message (pin 3), send it to USB Serial
  if (MySerial.available()) {
    ch = MySerial.read();
    Serial.write(ch);
  }
}
