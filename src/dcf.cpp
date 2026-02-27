#include "Arduino.h"
#include "FlexCAN_T4.h"

#include "util.hpp"
#include "can_serde.hpp"

constexpr uint8_t THROTTLE_1_PIN = 18;
constexpr uint8_t THROTTLE_2_PIN = 19;
constexpr uint8_t BRAKE_PIN = 20;
constexpr uint8_t SWITCH_PIN = 21;

/* Results from running sensor_health_check(). */
struct SensorHealthCheck {
    bool throttle1_healthy;
    bool throttle2_healthy;
    bool brake_healthy;
    bool switch_health;
};

FlexCAN_T4<CAN1, RX_SIZE_256> motorCAN;
FlexCAN_T4<CAN2, RX_SIZE_256> dataCAN;

constexpr uint32_t CAN_BAUD_RATE = 250000;
constexpr uint32_t SERIAL_BAUD_RATE = 115200;

/* All these timers control how often messages are emitted for each of these sensors. */
Timer throttle1_timer(0, 100);
Timer throttle2_timer(0, 100);
Timer brake_timer(0, 100);
Timer start_switch_timer(0, 100);

void setup()
{
    Serial.begin(SERIAL_BAUD_RATE);
    motorCAN.begin();
    motorCAN.setBaudRate(CAN_BAUD_RATE);
    dataCAN.begin();
    dataCAN.setBaudRate(CAN_BAUD_RATE);
    Serial.println("START");

    pinMode(THROTTLE_1_PIN, INPUT_PULLDOWN);
    pinMode(THROTTLE_2_PIN, INPUT_PULLDOWN);
    pinMode(BRAKE_PIN, INPUT_PULLDOWN);
    /* SWITCH_PIN is an analog input, so we don't need to set its pin mode. */
}

/* We periodically check that all the sensors are working correctly. */
SensorHealthCheck sensor_health_check();

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

/* This ensures that the pin is in a low impedence state.
 * 
 * Side effects: this will change the pin's pullup/pulldown state,
 * and will end with the pin in pulldown mode. */
bool digital_pin_health_check(int pin) {
    pinMode(pin, INPUT_PULLDOWN);

    /* Impedence check: if we're reading low right now, switch it to
     * a pullup resistor, and make sure it's still reading low
     * (if the pin is high impedence, then the pullup resistor
     * will give a high reading). */
    if (digitalRead(pin) == LOW) {
        pinMode(pin, INPUT_PULLUP);
        if(digitalRead(pin) == LOW) {
            pinMode(pin, INPUT_PULLDOWN);
            return true;
        }
    }

    /* Make sure we end with the pin in PULLDOWN mode. */
    pinMode(pin, INPUT_PULLDOWN);
    return false;
}

SensorHealthCheck sensor_health_check() {
    /* By default, everything is considered not healthy, and we
     * go through each sensor, marking it as healthy if relevant. */
    SensorHealthCheck result = {false, false, false, false};

    if (analogRead(THROTTLE_1_PIN) != 0) result.throttle1_healthy = true;
    if (analogRead(THROTTLE_2_PIN) != 0) result.throttle2_healthy = true;
    if (analogRead(BRAKE_PIN) != 0) result.brake_healthy = true;
    if (digital_pin_health_check(SWITCH_PIN)) result.switch_health = true;

    return result;
}
