#include <cmath>
#include <optional>

#include "Arduino.h"
#include "FlexCAN_T4.h"

#include "assert.hpp"
#include "can_serde.hpp"
#include "ecu.hpp"

using namespace std;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> MotorCAN;

void panic_handler(const char* file, int line, const char* msg) {
    /* Shut everything down. */
    CAN_message_t shutdown_message = empty_can_message(MessageId::ControlCommand, 8);
    /* `empty_can_message` is guaranteed to generate a message full of zeroes,
     * and since this is panic wind down code, it's best to keep it as simple as
     * possible, so we don't use the normal message creation function. */
    MotorCAN.write(shutdown_message);

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

Ecu ECU = {};

void setup() {
  // First things first, register the panic handler. If something goes
    // wrong during setup, we'll wind everything down.
  register_panic_handler(panic_handler);
  Serial.begin(SERIAL_BAUD_RATE);

  MotorCAN.begin();
  MotorCAN.setBaudRate(CAN_BAUD_RATE);

  Serial.println("============================================");
  Serial.println("==========Motor CAN initialized=============");
  Serial.println("============================================");
}

void loop() {
    uint32_t current_time_ms = millis();

    CAN_message_t rmsg;
    if (MotorCAN.read(rmsg)) {
        ECU.processMessage(current_time_ms, rmsg);
    }

    while (true) {
        std::optional<CAN_message_t> to_send = ECU.emitMessage(current_time_ms);
        if (to_send.has_value()) {
            MotorCAN.write(*to_send);
        } else {
            break;
        }
    }
}

void user_update(int throttle1,
                 int throttle2,
                 int brake_val,
                 bool switch_status,
                 int torque,
                 int system_status)
{
    Serial.println();
    Serial.println("========== SYSTEM STATUS ==========");

    Serial.print("Throttle 1   : ");
    Serial.println(throttle1);

    Serial.print("Throttle 2   : ");
    Serial.println(throttle2);

    Serial.print("Brake        : ");
    Serial.println(brake_val);

    Serial.print("Switch       : ");
    Serial.println(switch_status);

    Serial.print("Torque Cmd   : ");
    Serial.println(torque);

    Serial.print("System State : ");
    Serial.println(system_status);

    Serial.println("==================================");
}
