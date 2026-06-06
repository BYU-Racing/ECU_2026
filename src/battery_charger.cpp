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

Timer pacing(0, 1000);

void setup()
{
    Serial.begin(SERIAL_BAUD_RATE);
    battery_can.begin();
    battery_can.setBaudRate(CAN_BAUD_RATE);

    Serial.println("Started");
}

/* Check if we've gotten a response from the charger. */
CAN_message_t charger_response = {};

void loop()
{
    uint32_t current_time_ms = millis();

    /* See if we have any messages for us to read. */
    CAN_message_t rmsg;
    if (battery_can.read(rmsg)) {
        /* Only update the stored message if it has the right ID, since other messages may be on this bus. */
        if (rmsg.id == CHARGER_RESPONSE_ID) {
            charger_response = rmsg;
        }
    }

    /* `pacing.shouldFire` makes sure we don't send messages too often. */
    if (pacing.shouldFire(current_time_ms)) {
        Serial.print("Time since boot: ");
        Serial.print(static_cast<double>(current_time_ms) / 1000.0);
        Serial.println("s");

        /* Part one of message: max voltage. */
        uint16_t max_voltage_dV = 4450; /* In decivolts. */
        /* Part two of message: max current. */
        uint16_t raw_current_reading = analogRead(CURRENT_READING_PIN);
        uint16_t max_current_dA = map(raw_current_reading, 0, 1023, 0, 250); /* In deciamps. */
        /* Part three of message: charging enabled. */
        bool charging_enabled = true;

        /* Tell the user what message is about to be sent. */
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

        /* ID will be 0 by default, so once we get a message it'll switch to something else. */
        if (charger_response.id != 0) {
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
    }
}
