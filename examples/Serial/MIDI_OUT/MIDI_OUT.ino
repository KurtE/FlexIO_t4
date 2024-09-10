#include <FlexIO_t4.h>
#include <FlexSerial.h>
#include <MIDI.h>

// Demonstrate using FlexSerial for MIDI OUT, inspired by:
// https://forum.pjrc.com/index.php?threads/75818/

FlexSerial myport(-1, 32); // MIDI OUT on pin 32
MIDI_CREATE_INSTANCE(FlexSerial, myport, mymidi);

void setup() {
  myport.setClock(9795918);
  mymidi.begin(MIDI_CHANNEL_OMNI);
}

void loop() {
  mymidi.sendControlChange(9, 65, 1);
  delay(1000);
}

