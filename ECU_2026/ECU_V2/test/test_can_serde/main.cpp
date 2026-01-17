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

void test_start_switch() {
    TEST_ASSERT(true);
    CAN_message_t msg = empty_can_message((MessageId)0, 1);
    msg.buf[0] = 1;
    TEST_ASSERT(parse_start_switch(msg) == true);
}

void test_can_dump_parsing() {
    /* This test reads in an .trc file that has been used in the past
     * so we can test the CAN code against actual usage. */
    std::ifstream file;
    file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
    file.open("test/test_can_serde/can-dump-1.trc");
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
                break;
            default:
                std::stringstream fail_msg;
                fail_msg << "Did not account for message id " << msg.id;
                TEST_FAIL_MESSAGE(fail_msg.str().c_str());
                break;
        }
    }
}

void panic_handler(const char *file, int line, const char *msg) {
    /* If an SAFETY_ASSERT fails somewhere in the code, this will let the testing environment
     * know that we failed the test. */
    TEST_ASSERT(false);
}

int main(int argc, char **argv) {
    register_panic_handler(panic_handler);

    UNITY_BEGIN();
    RUN_TEST(test_start_switch);
    RUN_TEST(test_can_dump_parsing);
    UNITY_END();
}