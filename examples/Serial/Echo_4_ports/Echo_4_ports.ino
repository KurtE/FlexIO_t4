// FlexSerial Echo_4_ports example
//
// Listen for incoming bytes on USB Serial, two FlexSerial, and Serial1 (pins 0,1)
// When any port receives a byte, print a detailed message to all ports.  To use,
// connect 3 USB-Serial adapters to your computer and open 3 instances of a terminal
// emulator program (eg, CoolTerm, Seyon, etc) and the Arduino Serial Monitor.  When
// you type in any of the terminals, or send with Arduino IDE, all 4 should show the
// the message about the received bytes.
//
// Pins usable for FlexSerial
//
//   Teensy 4.0: 2-23,26,27,32-33
//   Teensy 4.1: 2-23,26,27,32-41,49,50,52,54
//   MicroMod:   2-23,26,27,32-33,40-45

#include <FlexSerial.h>

FlexSerial MySerial1(4, 5);   // pin 4 receive, pin 5 transmit
FlexSerial MySerial2(22, 23); // pin 22 receive, pin 23 transmit 

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  // FlexIO slowest baud rate is 58824 with default clock.  Use these for slower baud.
  //MySerial1.setClock(9795918);             // 19200 minimum
  //MySerial1.setClockUsingVideoPLL(921600); // 1800 minimum
  MySerial1.begin(115200);
  MySerial2.begin(115200);
  while (!Serial) ; // wait for Arduino Serial Monitor
  const char *welcome = "Print every byte to all 4 ports example program";
  Serial.println(welcome);
  Serial1.println(welcome);
  MySerial1.println(welcome);
  MySerial2.println(welcome);
}

void message_to_all(const char *name, int c) {
  Serial.print(name);
  Serial.print(" received byte ");
  Serial.println(c);
  Serial1.print(name);
  Serial1.print(" received byte ");
  Serial1.println(c);
  MySerial1.print(name);
  MySerial1.print(" received byte ");
  MySerial1.println(c);
  MySerial2.print(name);
  MySerial2.print(" received byte ");
  MySerial2.println(c);
}

void loop() {
  int c;
  if (Serial.available()) {
    c = Serial.read();
    message_to_all("Serial", c);
  }
  if (Serial1.available()) {
    c = Serial1.read();
    message_to_all("Serial1", c);
  }
  if (MySerial1.available()) {
    c = MySerial1.read();
    message_to_all("MySerial1", c);
  }
  if (MySerial2.available()) {
    c = MySerial2.read();
    message_to_all("MySerial2", c);
  }
}
