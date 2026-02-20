#include <sstream>
#include <cstring>
#include <tuple>

#include "unity.h"

#include "constants.hpp"
#include "assert.hpp"
#include "can_serde.hpp"
#include "ecu.hpp"

void setUp(void) {}

void tearDown(void) {}

/* Needs to be volatile since the unity test framework uses `longjmp`. */
volatile bool expecting_safety_violation = false;
volatile bool hit_safety_violation = false;
void safety_assert_failed_handler(AssertLevel level, const char *file, int line, AssertCode error_code) {
    if (expecting_safety_violation) {
        hit_safety_violation = true;
    } else {
        /* If an SAFETY_ASSERT fails somewhere in the code, this will let the testing environment
        * know that we failed the test. */
        std::stringstream fail_msg;
        fail_msg << "Assert failed at " << file << ":" << line;
        TEST_FAIL_MESSAGE(fail_msg.str().c_str());
    }
}

void test_can_msg_eql(CAN_message_t msg, MessageId id, uint8_t buf[8]) {
    TEST_ASSERT(msg.id == static_cast<uint32_t>(id));
    TEST_ASSERT(memcmp(&msg.buf, &buf, 8));
}

std::tuple<Ecu, uint32_t> ecu_after_startup_sequence() {
    uint32_t current_time_ms = 0;

    Ecu ecu = {};

    /* It should not produce any messages at the beginning. */
    TEST_ASSERT(ecu.emitMessage(current_time_ms) == std::nullopt);

    /* Let's let it know that the brake is down and that the switch is on. */
    ecu.processMessage(current_time_ms, create_brake_pressure(80));
    ecu.processMessage(current_time_ms, create_start_switch(true));

    /* However, we still should not emit a motor command, since the car still isn't on. */
    TEST_ASSERT(ecu.emitMessage(current_time_ms) == std::nullopt);

    /* 2 seconds later... */
    current_time_ms += 2000;
    /* Now we should be generating a motor command. */
    /*                                       forward vv    vv enabled */
    uint8_t enabled_but_no_torque[] = {0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00};
    test_can_msg_eql(
        ecu.emitMessage(current_time_ms).value(), /* `.value()` will throw if std::nullopt. */
        MessageId::ControlCommand,
        enabled_but_no_torque
    );
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

    /*                                       forward vv    vv enabled */
    uint8_t enabled_but_no_torque[] = {0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00};
    /* We should get the enabled message without torque, since we haven't put down the throttle. */
    test_can_msg_eql(ecu.emitMessage(current_time_ms).value(), MessageId::ControlCommand, enabled_but_no_torque);
    TEST_ASSERT(ecu.emitMessage(current_time_ms) == std::nullopt);

    /* Let's input throttle one. */
    ecu.processMessage(current_time_ms, create_throttle_one_position(THROTTLE1_MIN));
    current_time_ms += 20;

    /* The message still shouldn't change, since both throttles need to change. */
    test_can_msg_eql(ecu.emitMessage(current_time_ms).value(), MessageId::ControlCommand, enabled_but_no_torque);
    TEST_ASSERT(ecu.emitMessage(current_time_ms) == std::nullopt);

    /* Now input throttle two. Note: this should cause a safety violation since you're not allowed to have
     * the brake and pedal pressed at the same time. */
    expecting_safety_violation = true;
    ecu.processMessage(current_time_ms, create_throttle_two_position(THROTTLE2_MIN + 10));
    expecting_safety_violation = false;
    TEST_ASSERT(hit_safety_violation);

    /* Because we hit a safety violation, `ecu` is in an invalid state, so we need to reinit it. */
    new_ecu = ecu_after_startup_sequence();
    ecu = std::get<0>(new_ecu);
    current_time_ms = std::get<1>(new_ecu);

    /* Lift the brake this time, before puttind down the throttle. */
    ecu.processMessage(current_time_ms, create_brake_pressure(BRAKE_PRESSURE_MIN));
    ecu.processMessage(current_time_ms, create_throttle_one_position(THROTTLE1_MIN));
    ecu.processMessage(current_time_ms, create_throttle_two_position(THROTTLE2_MIN));
    current_time_ms += 20;

    /* Because the ECU has started, and the throttle is down, we should now get a torque command. */
    /*  torque amount (lower byte, le) vv              forward vv    vv enabled */
    uint8_t enabled_with_torque[] = {  31, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00};
    test_can_msg_eql(ecu.emitMessage(current_time_ms).value(), MessageId::ControlCommand, enabled_with_torque);
    TEST_ASSERT(ecu.emitMessage(current_time_ms) == std::nullopt);

}

void test_smooth_torque() {
    /* reinit ecu state */
    std::tuple<Ecu, uint32_t> new_ecu = ecu_after_startup_sequence();
    Ecu ecu = std::get<0>(new_ecu);
    uint32_t current_time_ms = std::get<1>(new_ecu);

    /* throttle values diverged too far and should cause a saftey assert */
    expecting_safety_violation = true;
    ecu.processMessage(current_time_ms, create_brake_pressure(BRAKE_PRESSURE_MIN));
    ecu.processMessage(current_time_ms, create_throttle_one_position(THROTTLE1_MAX));
    ecu.processMessage(current_time_ms, create_throttle_two_position(THROTTLE2_MIN));
    current_time_ms += 20;
    expecting_safety_violation = false;
    TEST_ASSERT(hit_safety_violation);

    /* throttle_1 is out of range and should cause a saftey assert */
    expecting_safety_violation = true;
    ecu.processMessage(current_time_ms, create_throttle_one_position(THROTTLE1_MIN - 10));
    ecu.processMessage(current_time_ms, create_throttle_two_position(THROTTLE2_MIN));
    current_time_ms += 20;
    expecting_safety_violation = false;
    TEST_ASSERT(hit_safety_violation);

    /* throttle_2 is out of range and should cause a saftey assert */
    expecting_safety_violation = true;
    ecu.processMessage(current_time_ms, create_throttle_one_position(THROTTLE1_MIN));
    ecu.processMessage(current_time_ms, create_throttle_two_position(THROTTLE2_MIN - 10));
    current_time_ms += 20;
    expecting_safety_violation = false;
    TEST_ASSERT(hit_safety_violation);

    /* TODO test smooth torque */
    /* smooth torque should return zero */
    uint16_t torque = ecu.smooth_torque(current_time_ms, 0);
    TEST_ASSERT(torque == 0);

    /* should send same torque message for 20 ms */
    /* reset clock */
    current_time_ms += 20;
    /* average out torque should be 100 */
    torque = ecu.smooth_torque(current_time_ms, 100);
    current_time_ms += 20;
    torque = ecu.smooth_torque(current_time_ms, 100);
    current_time_ms += 20;
    torque = ecu.smooth_torque(current_time_ms, 100);
    current_time_ms += 20;
    torque = ecu.smooth_torque(current_time_ms, 100);
    TEST_ASSERT(torque == 100);

    Serial.begin(SERIAL_BAUD_RATE);
    Serial.println(torque);

}

int main(int argc, char **argv) {
    register_assert_failed_handler(safety_assert_failed_handler);

    UNITY_BEGIN();
    RUN_TEST(test_ecu);
    RUN_TEST(test_smooth_torque);
    UNITY_END();
}
