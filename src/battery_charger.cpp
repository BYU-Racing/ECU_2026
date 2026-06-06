#include "Arduino.h"
#include "FlexCAN_T4.h"

#include "util.hpp"
#include "can_serde.hpp"

constexpr uint8_t CURRENT_READING_PIN = 14;

constexpr uint32_t CAN_BAUD_RATE = 250000;
FlexCAN_T4<CAN1, RX_SIZE_256> battery_can;

constexpr uint32_t SERIAL_BAUD_RATE = 115200;

constexpr uint32_t CHARGER_COMMAND_ID = 0x1806E5F4;
constexpr uint32_t CHARGER_RESPONSE_ID = 0x18FF50E5;

/* We add this so we can use SAFETY_ASSERT for debugging. */
void assert_failed_handler(AssertLevel level, LineInfo info, AssertCode error_code) {
    (void)level; /* To get the warnings to go away that says `level` isn't used. */

    while (true) {
        Serial.printf("Assertion failed! In file %s:%d with error code %d\n", info.filename, info.line_no, error_code);
        Serial.printf("File hash %lu\n", (unsigned long) str_hash(info.filename));

        pinMode(13, OUTPUT);
        digitalWrite(13, HIGH);
        delay(500);
        digitalWrite(13, LOW);
        delay(500);
    }
}

void setup()
{
    register_assert_failed_handler(assert_failed_handler);

    Serial.begin(SERIAL_BAUD_RATE);
    battery_can.begin();
    battery_can.setBaudRate(CAN_BAUD_RATE);

    /* Only receive responses from the charger and filter everything else out. */
    battery_can.setFIFOFilter(REJECT_ALL);
    battery_can.setFIFOFilter(0, CHARGER_RESPONSE_ID, EXT);

    Serial.println("Started");
}

void loop()
{
    Serial.print("Time since boot: ");
    Serial.print(static_cast<double>(millis()) / 1000.0);
    Serial.println("s");

    /* Part one of message: max voltage. */
    uint16_t max_voltage_dV = 4450; /* In decivolts. */
    /* Part two of message: max current. */
    uint16_t raw_current_reading = analogRead(CURRENT_READING_PIN);
    uint16_t max_current_dA = map(raw_current_reading, 0, 1023, 0, 250); /* In deciamps. */
    /* Part three of message: charging enabled. */
    bool charging_enabled = true;

    Serial.print("Sending max voltage of ");
    Serial.print(static_cast<double>(max_voltage_dV) / 10.0);
    Serial.println("V");
    Serial.print("Sending max current of ");
    Serial.print(static_cast<double>(max_current_dA) / 10.0);
    Serial.println("A");
    if (charging_enabled) {
        Serial.println("Charging enabled");
    } else {
        Serial.println("Charging disabled");
    }
    Serial.println();

    /* Build and send the message. */
    CAN_message_t charging_msg = {};
    charging_msg.len = 8;
    charging_msg.id = CHARGER_COMMAND_ID;
    write_u16_be(&charging_msg.buf[0], max_voltage_dV);
    write_u16_be(&charging_msg.buf[2], max_current_dA);
    if (charging_enabled) {
        charging_msg.buf[4] = 0;
    } else {
        charging_msg.buf[4] = 1;
    }
    battery_can.write(charging_msg);

    /* Get the latest response from the charger. */
    CAN_message_t charger_response = {};
    bool charger_response_found = false;

    /* Limit to 10 iterations so it doesn't loop forever. */
    for (int i = 0; i < 10; i++) {
        CAN_message_t rmsg;
        if (battery_can.read(rmsg)) {
            /* We filtered out every other message, so this should be the only message type we get .*/
            SAFETY_ASSERT(rmsg.id == CHARGER_RESPONSE_ID, AssertCode::Unknown);
            charger_response = rmsg;
            charger_response_found = true;
        } else {
            break;
        }
    }

    if (charger_response_found) {
        uint16_t output_voltage = read_u16_be(&charger_response.buf[0]);
        uint16_t output_current = read_u16_be(&charger_response.buf[2]);
        uint8_t status_flags = charger_response.buf[4];

        Serial.print("Charger reported voltage of ");
        Serial.print(static_cast<double>(output_voltage) / 10.0);
        Serial.println("V");
        Serial.print("Charger reported current of ");
        Serial.print(static_cast<double>(output_current) / 10.0);
        Serial.println("A");
        Serial.print("Charger reported flags of 0x");
        Serial.println(status_flags, 16);
        Serial.println();
    }

    delay(1000);
}
