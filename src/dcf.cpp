#include "Arduino.h"
#include "FlexCAN_T4.h"

#include "util.hpp"
#include "can_serde.hpp"

constexpr uint8_t THROTTLE_1_PIN = 18;
constexpr uint8_t THROTTLE_2_PIN = 19;
constexpr uint8_t BRAKE_PIN = 20;
constexpr uint8_t SWITCH_PIN = 21;


FlexCAN_T4<CAN1, RX_SIZE_256> motorCAN;
FlexCAN_T4<CAN2, RX_SIZE_256> dataCAN;

constexpr uint32_t CAN_BAUD_RATE = 250000;
constexpr uint32_t SERIAL_BAUD_RATE = 115200;

/* All these timers control how often messages are emitted for each of these sensors. */
Timer throttle1_timer(0, 30);
Timer throttle2_timer(0, 30);
Timer brake_timer(0, 30);
Timer start_switch_timer(0, 30);

void setup()
{
    Serial.begin(SERIAL_BAUD_RATE);
    motorCAN.begin();
    motorCAN.setBaudRate(CAN_BAUD_RATE);
    dataCAN.begin();
    dataCAN.setBaudRate(CAN_BAUD_RATE);
    Serial.println("START");

    /* The throttle and brake are analog inputs, so we don't need to set pinMode for them. */
    pinMode(SWITCH_PIN, INPUT_PULLDOWN);
}

void loop()
{
    uint32_t current_time_ms = millis();

    /* Check if any sensors need to send values down the CAN bus, and
     * if so, send them. */
    if (throttle1_timer.shouldFire(current_time_ms)) {
        uint16_t reading = analogRead(THROTTLE_1_PIN);
        motorCAN.write(create_throttle_one_position(reading));
    }

    if (throttle2_timer.shouldFire(current_time_ms)) {
        uint16_t reading = analogRead(THROTTLE_2_PIN);
        motorCAN.write(create_throttle_two_position(reading));
    }

    if (brake_timer.shouldFire(current_time_ms)) {
        uint16_t reading = analogRead(BRAKE_PIN);
        motorCAN.write(create_brake_pressure(reading));
    }

    if (start_switch_timer.shouldFire(current_time_ms)) {
        bool reading = digitalRead(SWITCH_PIN) != 0;
        motorCAN.write(create_start_switch(reading));
    }
}

