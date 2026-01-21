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

/* This is used to periodically send state updates with the contents of ECU. */
Trigger state_print_timer = {};

void loop() {
    /* The ECU does not keep track of what time it is, nor does it use `millis`,
     * so we always have to tell it what time it is. */
    uint32_t current_time_ms = millis();

    /* Receive a message and have the ECU process it. */
    CAN_message_t rmsg;
    if (MotorCAN.read(rmsg)) {
        ECU.processMessage(current_time_ms, rmsg);
    }

    /* Generate all outgoing messages and send each. */
    while (true) {
        std::optional<CAN_message_t> to_send = ECU.emitMessage(current_time_ms);
        if (to_send.has_value()) {
            MotorCAN.write(*to_send);
        } else {
            break;
        }
    }

    /* Used to space out state printing messages. */
    if (!state_print_timer.started()) {
        state_print_timer.start(current_time_ms, 100/*ms*/);
    } else if (state_print_timer.triggerReached(current_time_ms)) {
        ECU.printState();
    }
}
