#include <sstream>
#include <iostream>
#include <cstring>
#include <tuple>

#include "unity.h"

#include "constants.hpp"
#include "assert.hpp"
#include "can_serde.hpp"
#include "ecu_logic.hpp"

void setUp(void) {}

void tearDown(void) {}

/* Needs to be volatile since the unity test framework uses `longjmp`. */
volatile bool expecting_safety_violation = false;
volatile bool hit_safety_violation = false;
void safety_assert_failed_handler(AssertLevel level, LineInfo info, AssertCode error_code) {
    UNUSED(level);

    if (expecting_safety_violation) {
        hit_safety_violation = true;
    } else {
        /* If an SAFETY_ASSERT fails somewhere in the code, this will let the testing environment
        * know that we failed the test. */
        std::stringstream fail_msg;
        fail_msg << "Assert failed at " << info.filename << ":" << info.line_no << " with code " << static_cast<uint8_t>(error_code);
        TEST_FAIL_MESSAGE(fail_msg.str().c_str());
    }
}

void test_can_msg_eql(CAN_message_t msg, MessageId id, uint8_t *buf, size_t len) {
    TEST_ASSERT(msg.id == static_cast<uint32_t>(id));
    TEST_ASSERT(memcmp(&msg.buf, buf, len) == 0);
}

std::tuple<Ecu, uint32_t> ecu_after_startup_sequence() {
    uint32_t current_time_ms = 0;

    Ecu ecu = {};

    /* It should not produce any messages at the beginning. */
    TEST_ASSERT(ecu.emitMessage(current_time_ms) == std::nullopt);

    /* Let's let it know that the brake is down and that the switch is on. */
    ecu.processMessage(current_time_ms, create_brake_pressure(BRAKE_CONSIDERED_PRESSED));
    ecu.processMessage(current_time_ms, create_start_switch(true));

    /* However, we still should not emit a motor command, since the car still isn't on. */
    TEST_ASSERT(ecu.emitMessage(current_time_ms) == std::nullopt);

    /* 2 seconds later... */
    current_time_ms += 2000;
    /* Now we should be generating a motor command. */
    /*                                                   reverse vv    vv enabled */
    uint8_t enabled_but_no_torque[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00};
    auto val = ecu.emitMessage(current_time_ms).value(); /* `.value()` will throw if std::nullopt. */
    test_can_msg_eql(val, MessageId::ControlCommand, enabled_but_no_torque, 8);
    /* We shouldn't get anything after the motor command. */
    TEST_ASSERT(ecu.emitMessage(current_time_ms) == std::nullopt);

    return std::make_tuple(ecu, current_time_ms);
}

void test_ecu() {
    std::tuple<Ecu, uint32_t> new_ecu = ecu_after_startup_sequence();
    Ecu ecu = std::get<0>(new_ecu);
    uint32_t current_time_ms = std::get<1>(new_ecu);

    /* We shouldn't emit another message until at least 15 ms has passed since the first message (
     * tested in `ecu_after_startup_sequence`). */
    current_time_ms += 10;
    TEST_ASSERT(ecu.emitMessage(current_time_ms) == std::nullopt);
    current_time_ms += 10;

    /*                                                  backward vv    vv enabled */
    uint8_t enabled_but_no_torque[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00};
    /* We should get the enabled message without torque, since we haven't put down the throttle. */
    test_can_msg_eql(ecu.emitMessage(current_time_ms).value(), MessageId::ControlCommand, enabled_but_no_torque, 8);
    TEST_ASSERT(ecu.emitMessage(current_time_ms) == std::nullopt);

    /* Let's input throttle one. */
    ecu.processMessage(current_time_ms, create_throttle_one_position(50));
    current_time_ms += 20;

    /* The message still shouldn't change, since both throttles need to change. */
    test_can_msg_eql(ecu.emitMessage(current_time_ms).value(), MessageId::ControlCommand, enabled_but_no_torque, 8);
    TEST_ASSERT(ecu.emitMessage(current_time_ms) == std::nullopt);

    /* Now input throttle two. Note: this should cause a safety violation since you're not allowed to have
     * the brake and pedal pressed at the same time. */
    expecting_safety_violation = true;
    ecu.processMessage(current_time_ms, create_throttle_two_position(22));
    expecting_safety_violation = false;
    TEST_ASSERT(hit_safety_violation);

    /* Because we hit a safety violation, `ecu` is in an invalid state, so we need to reinit it. */
    new_ecu = ecu_after_startup_sequence();
    ecu = std::get<0>(new_ecu);
    current_time_ms = std::get<1>(new_ecu);

    /* Lift the brake this time, before putting down the throttle. */
    ecu.processMessage(current_time_ms, create_brake_pressure(BRAKE_PRESSURE_MIN));
    ecu.processMessage(current_time_ms, create_throttle_one_position(64));
    ecu.processMessage(current_time_ms, create_throttle_two_position(175));
    current_time_ms += 20;

    /* Because the ECU has started, and the throttle is down, we should now get a torque command. */
    /* torque amount (lower byte, le) vv            backward vv    vv enabled */
    uint8_t enabled_with_torque[] = {  0, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00};
    auto val = ecu.emitMessage(current_time_ms).value();
    test_can_msg_eql(val, MessageId::ControlCommand, enabled_with_torque, 8);
    TEST_ASSERT(ecu.emitMessage(current_time_ms) == std::nullopt);
}

void test_switch_debounce() {
    /* This test verifies the switch debouncing logic when turning off. */
    std::tuple<Ecu, uint32_t> new_ecu = ecu_after_startup_sequence();
    Ecu ecu = std::get<0>(new_ecu);
    uint32_t current_time_ms = std::get<1>(new_ecu);

    ecu.processMessage(current_time_ms, create_brake_pressure(80));

    current_time_ms += 10;
    /* Lift the brake so the car is fully operational. */
    ecu.processMessage(current_time_ms, create_brake_pressure(BRAKE_PRESSURE_MIN));
    current_time_ms += 20;

    /* At this point, the car should be on and the switch should be on. */
    /* Now turn off the switch. */
    ecu.processMessage(current_time_ms, create_start_switch(false));

    /* The car should still be on immediately after turning off the switch,
     * because of the 1000ms debounce delay. */
    current_time_ms += 20;
    std::optional<CAN_message_t> msg = ecu.emitMessage(current_time_ms);
    TEST_ASSERT(msg != std::nullopt);
    /* Verify the inverter is still enabled. */
    TEST_ASSERT((msg.value().buf[5] & 0x01) == 0x01); /* enable_inverter should be true */

    /* Wait 500ms (halfway through the debounce period). */
    current_time_ms += 500;
    msg = ecu.emitMessage(current_time_ms);
    TEST_ASSERT(msg != std::nullopt);
    /* Car should still be on. */
    TEST_ASSERT((msg.value().buf[5] & 0x01) == 0x01);

    /* Now turn the switch back on before the debounce completes. */
    ecu.processMessage(current_time_ms, create_start_switch(true));
    current_time_ms += 20;

    /* The car should remain on since we canceled the debounce. */
    msg = ecu.emitMessage(current_time_ms);
    TEST_ASSERT(msg != std::nullopt);
    TEST_ASSERT((msg.value().buf[5] & 0x01) == 0x01);

    /* Wait another 1000ms to make sure it stays on. */
    current_time_ms += 1000;
    msg = ecu.emitMessage(current_time_ms);
    TEST_ASSERT(msg != std::nullopt);
    TEST_ASSERT((msg.value().buf[5] & 0x01) == 0x01);

    /* Now turn the switch off again and let the full debounce period elapse. */
    ecu.processMessage(current_time_ms, create_start_switch(false));
    current_time_ms += 20;

    /* Car should still be on. */
    msg = ecu.emitMessage(current_time_ms);
    TEST_ASSERT(msg != std::nullopt);
    TEST_ASSERT((msg.value().buf[5] & 0x01) == 0x01);

    /* Wait for the full 1000ms debounce period to complete. */
    current_time_ms += 1000;
    ecu.processMessage(current_time_ms, create_start_switch(false));

    /* Now the car should be off. */
    current_time_ms += 20;
    msg = ecu.emitMessage(current_time_ms);
    TEST_ASSERT(msg != std::nullopt);
    /* Verify the inverter is now disabled. */
    TEST_ASSERT((msg.value().buf[5] & 0x01) == 0x00); /* enable_inverter should be false */
}

int main() {
    register_assert_failed_handler(safety_assert_failed_handler);

    UNITY_BEGIN();
    RUN_TEST(test_ecu);
    RUN_TEST(test_switch_debounce);
    UNITY_END();
}
