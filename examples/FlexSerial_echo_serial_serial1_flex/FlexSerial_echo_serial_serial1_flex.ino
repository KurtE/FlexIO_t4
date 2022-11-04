#include <FlexIO_t4.h>
#include <FlexSerial.h>

#define MODEM_RX_PIN 9
#define MODEM_TX_PIN 10
#define MODEM_BAUD 115200

FlexSerial modem(MODEM_RX_PIN, MODEM_TX_PIN);

void setup() {

  // waits for Arduino Serial Monitor
  while (!Serial) {}

  Serial1.begin(MODEM_BAUD);
  modem.begin(MODEM_BAUD);
}

void loop() {
  int ch;

  if (Serial.available()) {
    while ((ch = Serial.read()) != -1)
      Serial1.write(ch);
  }

  if (Serial1.available()) {
    while ((ch = Serial1.read()) != -1)
      Serial.write(ch);
  }

  if (modem.available()) {
    for (;;) {
      int ch_peek = modem.peek();
      ch = modem.read();
      if ((ch != ch_peek) && (ch_peek != -1)) {
        Serial.printf("FlexIO Read(%x) != peek(%x)\n", ch, ch_peek);
      }
      if (ch == -1) break;
      modem.write(ch);
    }
  }
}