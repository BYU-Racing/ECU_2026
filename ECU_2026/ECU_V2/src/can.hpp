#pragma once

#include "can_serde.hpp"

#include "FlexCAN_T4.h"

/* You can ignore everything in-between template<>, this is just so we can work with the FlexCAN library. */
template<CAN_DEV_TABLE _bus, FLEXCAN_RXQUEUE_TABLE _rxSize, FLEXCAN_TXQUEUE_TABLE _txSize>
class CanController {
    private:
        /* This is the can bus we'll be using for sending and receiving messages. */
        FlexCAN_T4<_bus, _rxSize, _txSize> can;

    public:
        void init(FlexCAN_T4<_bus, _rxSize, _txSize> can_to_use, uint32_t baud_rate) {
        can = can_to_use;

        can.begin();
        can.setBaudRate(baud_rate);
    }

    CAN_message_t poll_message() {
        /* Read the message into a temporary variable, then return the message
            * on the stack so it's easier to work with. */
        CAN_message_t msg;
        can.read(&msg);
        return msg;
    }

    void send_start_switch(bool value) {
        can.write(create_start_switch(value));
    }

    void send_throttle_one_position(uint16_t value) {
        can.write(create_throttle_one_position(value));
    }

    void send_throttle_two_position(uint16_t value) {
        can.write(create_throttle_two_position(value));
    }

    void send_throttle_break_pressure(uint16_t value) {
        can.write(create_throttle_brake_pressure(value));
    }

    void send_rvc(RvcMessage value) {
        can.write(create_rvc(value));
    }

    void send_tire_rpm(TireRpmMessage value) {
        can.write(create_tire_rpm(value));
    }

    void send_tire_temperature(TireTemperatureMessage value) {
        can.write(create_tire_temperature(value));
    }

    void send_lap(LapMessage value) {
        can.write(create_lap(value));
    }
};