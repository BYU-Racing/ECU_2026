#include <cmath>
#include <optional>

#include "Arduino.h"
#include "FlexCAN_T4.h"

#include "assert.hpp"
#include "can_serde.hpp"
#include "ecu.hpp"

using namespace std;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> MotorCAN;

Ecu ECU = {};

void safety_assert_failed_handler(const char *file, int line, AssertCode error_code)
{
    /* Shut everything down. */
    CAN_message_t shutdown_message = empty_can_message(MessageId::ControlCommand, 8);
    /* `empty_can_message` is guaranteed to generate a message full of zeroes,
     * and since this is panic wind down code, it's best to keep it as simple as
     * possible, so we don't use the normal message creation function. */
    MotorCAN.write(shutdown_message);

    /* Loop forever so we never do anything after the panic. */
    while (true)
    {
        Serial.printf("Safety assertion failed! In file %s:%d with error code %s\n", file, line, error_code);

        CriticalFault fault_msg;
        fault_msg.error_code = error_code;
        fault_msg.assert_failure_line = line;
        // fault_msg.file_name_hash = str_hash(file);

        MotorCAN.write(create_critical_fault_command(fault_msg));

        pinMode(13, OUTPUT);
        digitalWrite(13, HIGH);
        delay(500);
        digitalWrite(13, LOW);
        delay(500);
    }
}

#ifdef ENABLE_DEBUGGING
void soft_assert_failed_handler(const char* file, int line, AssertCode error_code) {
    /* Be sure to let us know if a soft assert failed. */
    Serial.printf("Soft assertion failed! In file %s:%d with error code %s\n", file, line, error_code);
    /* Start the reset trigger. */
    SOFT_RESET_TRIGGER.start(millis(), SOFT_RESET_LENGTH_MS);
    longjmp(soft_assert_failed_goto_start_of_loop, 0);
}
#endif

void assert_failed_handler(AssertLevel level, const char* file, int line, AssertCode error_code) {
#ifdef ENABLE_DEBUGGING
    switch (level) {
        case AssertLevel::Safety:
            safety_assert_failed_handler(file, line, error_code);
            break;
        case AssertLevel::Soft:
            soft_assert_failed_handler(file, line, error_code);
            break;
    }
#else
    switch (level) {
        case AssertLevel::Safety:
        case AssertLevel::Soft:
            safety_assert_failed_handler(file, line, error_code);
            break;
    }
#endif
}

void setup() {
  // First things first, register the panic handler. If something goes
    // wrong during setup, we'll wind everything down.
//   register_assert_failed_handler(assert_failed_handler);
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
        state_print_timer.start(current_time_ms, 500/*ms*/);
    } else if (state_print_timer.triggerReached(current_time_ms)) {
        ECU.printState();
    }
}
