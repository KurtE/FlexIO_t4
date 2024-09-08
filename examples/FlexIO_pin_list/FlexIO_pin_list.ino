//-----------------------------------------------------------------------------
// FlexIO print out pin lists
//
// Prints out the mapping of IO pins to flexIO pins, first in
// pin number order, than in FlexIO order
//-----------------------------------------------------------------------------

#include <FlexIO_t4.h>

void setup() {
    while (!Serial) {}
    Serial.begin(115200);

#ifdef ARDUINO_TEENSY41
    Serial.print("\nTeensy 4.1 ");
#elif defined(ARDUINO_TEENSY40)
    Serial.print("\nTeensy 4 ");
#elif defined(ARDUINO_TEENSY_MICROMOD)
    Serial.print("\nTeensy Micromod ");
#endif

    if (CrashReport) {
        Serial.print(CrashReport);
        Serial.println("Press Any key to continue");
        while (Serial.read() == -1) {}
    }

    Serial.println("FlexIO pin list");

    uint8_t flex_pin;

    for (uint8_t flexio_index = 0; flexio_index < FlexIOHandler::CNT_FLEX_IO_OBJECT; flexio_index++) {
        FlexIOHandler *pflex = FlexIOHandler::flexIOHandler_list[flexio_index];
        Serial.printf("\nFlexIO%u:\n", flexio_index + 1);

        int output_count = 0;
        Serial.print("\tPin Order: Pin:flex\n\t\t");
        for (uint8_t pin = 0; pin < CORE_NUM_DIGITAL; pin++) {
            output_count = 0;
            flex_pin = pflex->mapIOPinToFlexPin(pin);
            if (flex_pin != 0xff) {
                if (output_count >= 90) {
                    Serial.print("\n\t\t");
                    output_count = 0;
                }
                output_count += Serial.printf(" %u:%u", pin, flex_pin);
            }
        }

        Serial.print("\n\tFlex Pin Order: flex:Pin\n\t\t");
        output_count = 0;
        for (flex_pin = 0; flex_pin < pflex->pinCount(); flex_pin++) {
            uint8_t pin = pflex->mapFlexPinToIOPin(flex_pin);
            if (pin != 0xff) {
                if (output_count >= 90) {
                    Serial.print("\n\t\t");
                    output_count = 0;
                }
                Serial.printf(" %u:%u", flex_pin, pin);
            }
        }
    }

    Serial.println("\nFlex IO Pin ranges:");
    for (uint8_t flexio_index = 0; flexio_index < FlexIOHandler::CNT_FLEX_IO_OBJECT; flexio_index++) {
        FlexIOHandler *pflex = FlexIOHandler::flexIOHandler_list[flexio_index];
        uint8_t flex_pin_range_start = 0xff;
        bool output_comma = false;
        Serial.printf("\tFlexIO%u: ", flexio_index + 1);
        for (flex_pin = 0; flex_pin < pflex->pinCount(); flex_pin++) {
            uint8_t pin = pflex->mapFlexPinToIOPin(flex_pin);
            if (pin != 0xff) {
                if (flex_pin_range_start == 0xff) flex_pin_range_start = flex_pin;
            } else if (flex_pin_range_start != 0xff) {
                // previous range ended
                if (output_comma) Serial.print(", ");
                Serial.print(flex_pin_range_start);
                if (flex_pin_range_start != (flex_pin - 1)) Serial.printf("-%u", flex_pin - 1);
                flex_pin_range_start = 0xff;
                output_comma = true;
            }
        }

        if (flex_pin_range_start != 0xff) {
            // previous range ended
            if (output_comma) Serial.print(", ");
            Serial.print(flex_pin_range_start);
            if (flex_pin_range_start != (flex_pin - 1)) Serial.printf("-%u", flex_pin - 1);
        }
        Serial.println();
    }

    Serial.println("\nBounds check test:");
    Serial.flush();
    for (uint8_t flexio_index = 0; flexio_index < FlexIOHandler::CNT_FLEX_IO_OBJECT; flexio_index++) {
        FlexIOHandler *pflex = FlexIOHandler::flexIOHandler_list[flexio_index];
        Serial.printf("\tFlexIO%u ", flexio_index + 1);
        Serial.flush();

        flex_pin = pflex->mapIOPinToFlexPin(CORE_NUM_DIGITAL);
        Serial.printf(" IOToFlex(%u):%u", CORE_NUM_DIGITAL, flex_pin);
        Serial.flush();
        flex_pin = pflex->mapIOPinToFlexPin(0xff);
        Serial.printf(" IOToFlex(%u):%u", 0xff, flex_pin);
        Serial.flush();

        uint8_t pin_count = pflex->pinCount();
        uint8_t pin = pflex->mapFlexPinToIOPin(pin_count);
        Serial.printf(" FlexToIO(%u):%u", pin_count, pin);
        Serial.flush();
        pin = pflex->mapFlexPinToIOPin(0xff);
        Serial.printf(" FlexToIO(%u):%u\n", 0xff, pin);
        Serial.flush();
    }
}

void loop() {
    // put your main code here, to run repeatedly:
}
