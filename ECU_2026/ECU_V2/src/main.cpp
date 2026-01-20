#include "Arduino.h"
#include "FlexCAN_T4.h"

#include "assert.hpp"
#include "can_serde.hpp"

void panic_handler(const char* file, int line, const char* msg) {
    /* Shut everything down. */

    /* Loop forever so we never do anything after the panic. */
    while (true) {
        Serial.printf("Assertion failed! Line %d in file %s with message %s\n", line, file, msg);
        pinMode(13, OUTPUT);
        digitalWrite(13, HIGH);
        delay(500);
        digitalWrite(13, LOW);
        delay(500);
    }
}

constexpr int CAN1_BAUDRATE = 250000;
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
void setup() {
    // First things first, register the panic handler. If something goes
    // wrong during setup, we'll wind everything down.
    register_panic_handler(panic_handler);

    Serial.begin(115200);
    Serial.println("Start");

    can1.begin();
    can1.setBaudRate(CAN1_BAUDRATE);
}

void loop() {
    MotorControlCommand cmd;
    cmd.torque = 5;
    cmd.speed = 5;
    cmd.direction = MotorDirection::Forward;
    cmd.enable_inverter = true;
    cmd.inverter_discharge = false;
    cmd.override_speed = false;
    cmd.torque_limit = 0;

    can1.write(create_motor_control_command(cmd));
    delay(100);
}
