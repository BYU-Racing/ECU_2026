#include <cmath>
#include <optional>
#include <setjmp.h>

#include "Arduino.h"
#include "FlexCAN_T4.h"

#include "assert.hpp"
#include "util.hpp"
#include "can_serde.hpp"
#include "ecu.hpp"

using namespace std;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> MotorCAN;

/* We only use soft resets in debug builds. In production builds they're treated as
 * safety failures. */
#ifdef ENABLE_DEBUGGING
/* If a soft assert fails, this is the timeout before we try continuing to run the system. */
Trigger SOFT_RESET_TRIGGER = {};
/* This lets us jump back to the top of the loop if a soft assert ever fails. */
jmp_buf soft_assert_failed_goto_start_of_loop;
#endif

void safety_assert_failed_handler(const char* file, int line, const char* msg) {
    /* Shut everything down. */
    CAN_message_t shutdown_message = empty_can_message(MessageId::ControlCommand, 8);
    /* `empty_can_message` is guaranteed to generate a message full of zeroes,
    * and since this is panic wind down code, it's best to keep it as simple as
    * possible, so we don't use the normal message creation function. */
    MotorCAN.write(shutdown_message);

    /* Loop forever so we never do anything after the panic. */
    while (true) {
        Serial.printf("Safety assertion failed! In file %s:%d with message %s\n", file, line, msg);
        pinMode(13, OUTPUT);
        digitalWrite(13, HIGH);
        delay(500);
        digitalWrite(13, LOW);
        delay(500);
    }
}

#ifdef ENABLE_DEBUGGING
void soft_assert_failed_handler(const char* file, int line, const char* msg) {
    /* Be sure to let us know if a soft assert failed. */
    Serial.printf("Soft assertion failed! In file %s:%d with message %s\n", file, line, msg);
    /* Start the reset trigger. */
    SOFT_RESET_TRIGGER.start(millis(), SOFT_RESET_LENGTH_MS);
    longjmp(soft_assert_failed_goto_start_of_loop, 0);
}
#endif

void assert_failed_handler(AssertLevel level, const char* file, int line, const char* msg) {
#ifdef ENABLE_DEBUGGING
    switch (level) {
        case AssertLevel::Safety:
            safety_assert_failed_handler(file, line, msg);
            break;
        case AssertLevel::Soft:
            soft_assert_failed_handler(file, line, msg);
            break;
    }
#else
    switch (level) {
        case AssertLevel::Safety:
        case AssertLevel::Soft:
            safety_assert_failed_handler(file, line, msg);
            break;
    }
#endif
}

Ecu ECU = {};

void setup() {
  // First things first, register the panic handler. If something goes
    // wrong during setup, we'll wind everything down.
  register_assert_failed_handler(assert_failed_handler);
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
#ifdef ENABLE_DEBUGGING
    /* Save our current location, so if a soft assert fails, we'll go back to here. */
    setjmp(soft_assert_failed_goto_start_of_loop);
#endif

    /* The ECU does not keep track of what time it is, nor does it use `millis`,
     * so we always have to tell it what time it is. */
    uint32_t current_time_ms = millis();

    /* Receive a message and have the ECU process it. */
    CAN_message_t rmsg;
    if (MotorCAN.read(rmsg)) {
        ECU.processMessage(current_time_ms, rmsg);
    }

#ifdef ENABLE_DEBUGGING
    if (SOFT_RESET_TRIGGER.started()) {
        /* We're currently in a soft reset. We bypass the normal ECU messages
         * and shut off the motor until the soft reset finishes. */

         /* Shut the motor off. */
        CAN_message_t shutdown_message = empty_can_message(MessageId::ControlCommand, 8);
        MotorCAN.write(shutdown_message);
        delay(10); /* Don't oversaturate the CAN bus. */

        /* Make sure to reset the trigger when we're done. */
        SOFT_RESET_TRIGGER.triggerReached(millis());
    }
#else
    if (false) {
    }
#endif
    else {
        /* Generate all outgoing messages and send each. */
        while (true) {
            std::optional<CAN_message_t> to_send = ECU.emitMessage(current_time_ms);
            if (to_send.has_value()) {
                MotorCAN.write(*to_send);
            } else {
                break;
            }
        }
    }

    /* Used to space out state printing messages. */
    if (!state_print_timer.started()) {
        state_print_timer.start(current_time_ms, 500/*ms*/);
    } else if (state_print_timer.triggerReached(current_time_ms)) {
        ECU.printState();
    }
}
