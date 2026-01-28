/* We can't normally use these libraries, but since we're running on the host computer we can use normal C++ stuff. */
#include <fstream>
#include <sstream>
#include <string>
#include <iostream>

#include "unity.h"

#include "assert.hpp"
#include "can_serde.hpp"

void setUp(void) {
    // set stuff up here
}

void tearDown(void) {
    // clean stuff up here
}

void panic_handler(const char *file, int line, const char *msg) {
    /* If an SAFETY_ASSERT fails somewhere in the code, this will let the testing environment
     * know that we failed the test. */
    std::stringstream fail_msg;
    fail_msg << "Assert failed at " << file << ":" << line;
    TEST_FAIL_MESSAGE(fail_msg.str().c_str());
}

void test_start_switch() {
    TEST_ASSERT(true);
    CAN_message_t msg = empty_can_message((MessageId)0, 1);
    msg.buf[0] = 1;
    TEST_ASSERT(parse_start_switch(msg) == true);
}

void test_safety_violation() {
    CAN_message_t msg = empty_can_message(MessageId::Rvc, 5);
    msg.buf[0] = 8; /* Out of range for RVC type. */
    parse_rvc(msg);
}

void test_can_dump_parsing() {
    /* This test reads in an .trc file that has been used in the past
     * so we can test the CAN code against actual usage. */
    std::ifstream file;
    file.open("test/test_can_serde/can-dump-1.trc");
    if (!file) {
        TEST_FAIL_MESSAGE("Could not open can-dump-1.trc - check the path");
        return;
    }

    std::string line_contents;

    while (std::getline(file, line_contents)) {
        /* Skip empty and comment lines. */
        if (line_contents.empty() || line_contents[0] == ';') continue;

        /* Load in the message. */
        std::istringstream iss(line_contents);
        int msg_num;
        double time_offset;
        std::string type, id_hex, direction;
        int len;

        iss >> msg_num >> time_offset >> type >> id_hex >> direction >> len;

        /* Construct the message that we'll be using. */
        CAN_message_t msg = {0};
        msg.id = std::stoul(id_hex, nullptr, 16); /* Hex -> number. */
        msg.len = len;

        for (int i = 0; i < len && i < 8; i++) {
            std::string byte_hex;
            iss >> byte_hex;
            msg.buf[i] = std::stoul(byte_hex, nullptr, 16);
        }

        switch (static_cast<MessageId>(msg.id)) {
            case MessageId::StartSwitch:
                parse_start_switch(msg);
                break;
            case MessageId::ThrottleOnePosition:
                parse_throttle_one_position(msg);
                break;
            case MessageId::ThrottleTwoPosition:
                parse_throttle_two_position(msg);
                break;
            case MessageId::BrakePressure:
                parse_brake_pressure(msg);
                break;
            case MessageId::TemperaturesOne:
                parse_motor_temperatures_one(msg);
                break;
            case MessageId::TemperaturesTwo:
                parse_motor_temperatures_two(msg);
                break;
            case MessageId::TemperaturesThree:
                parse_motor_temperatures_three(msg);
                break;
            case MessageId::AnalogInputVoltages:
                parse_motor_analog_input_voltages(msg);
                break;
            case MessageId::DigitalInputStatus:
                parse_motor_digital_input_status(msg);
                break;
            case MessageId::PositionInfo:
                parse_motor_position_info(msg);
                break;
            case MessageId::CurrentInfo:
                parse_motor_current_info(msg);
                break;
            case MessageId::VoltageInfo:
                parse_motor_voltage_info(msg);
                break;
            case MessageId::FluxInfo:
                parse_motor_flux_info(msg);
                break;
            case MessageId::InternalVoltages:
                parse_motor_internal_voltages(msg);
                break;
            case MessageId::InternalStates:
                parse_motor_internal_states(msg);
                break;
            case MessageId::FaultCodes:
                parse_motor_fault_codes(msg);
                break;
            case MessageId::TorqueAndTimerInfo:
                parse_motor_torque_and_timer_info(msg);
                break;
            case MessageId::ModIndexAndFlux:
                parse_motor_mod_index_and_flux(msg);
                break;
            case MessageId::FirmwareInfo:
                /* Motor internal message. */
                break;
            case MessageId::TorqueCapability:
                parse_motor_torque_capability(msg);
                break;
            case MessageId::ControlCommand:
                parse_motor_control_command(msg);
                break;
            default:
                /* Right now I don't know what some of the message IDs are, so this is a switch
                 * of IDs I'm specifically ignoring. Otherwise it'll throw an error so I can make
                 * sure I handle every message type. */
                switch (msg.id) {
                    case 55:
                    case 56:
                    case 128:
                    case 1713:
                    case 1744:
                    case 403105268:
                    case 403105780:
                    case 403106292:
                    case 406385536:
                    case 406451072:
                    case 418316160:
                        // FIXME these are mystery messages.
                        break;
                    default:
                        std::stringstream fail_msg;
                        fail_msg << "Did not account for message id " << msg.id;
                        TEST_FAIL_MESSAGE(fail_msg.str().c_str());
                        break;
                }
                break;
        }
    }
}

void test_inverter_creation_message() {
    MotorControlCommand cmd;
    cmd.torque = 0;
    cmd.speed = 0;
    cmd.direction = MotorDirection::Forward;
    cmd.enable_inverter = true;
    cmd.inverter_discharge = false;
    cmd.override_speed = false;
    cmd.torque_limit = 0;

    CAN_message_t created = create_motor_control_command(cmd);

    TEST_ASSERT(created.buf[0] == 0x00);
    TEST_ASSERT(created.buf[1] == 0x00);
    TEST_ASSERT(created.buf[2] == 0x00);
    TEST_ASSERT(created.buf[3] == 0x00);
    TEST_ASSERT(created.buf[4] == 0x01);
    TEST_ASSERT(created.buf[5] == 0x01);
    TEST_ASSERT(created.buf[6] == 0x00);
    TEST_ASSERT(created.buf[7] == 0x00);
}

int main(int argc, char **argv) {
    register_panic_handler(panic_handler);

    UNITY_BEGIN();
    RUN_TEST(test_start_switch);
    RUN_TEST(test_can_dump_parsing);
    RUN_TEST(test_inverter_creation_message);
    // RUN_TEST(test_safety_violation);
    UNITY_END();
}