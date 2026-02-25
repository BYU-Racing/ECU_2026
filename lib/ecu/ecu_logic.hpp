#pragma once

#include <cstdint>
#include <optional>

#include "constants.hpp"
#include "CAN_message_t.hpp"
#include "util.hpp"

class Ecu {
public:
    void processMessage(uint32_t current_time_ms, CAN_message_t msg);

    /* The caller of this should continously call this until it returns
     * `std::nullopt`. For each message emitted, the caller should send
     * along the CAN bus. */
    std::optional<CAN_message_t> emitMessage(uint32_t current_time_ms);

    /* smooth torque */
    int16_t smoothTorque(uint32_t current_time);

    void handleStartupSequence(uint32_t current_time_ms);

    /* Uses PRINTF for all printing. */
    void printState();

private:
    /* What is the purpose of `std::optional`? It lets us signal that this value may not have a value.
    * Since at the beginning of the program, we don't know the throttle's position, we initialize
    * both of them to `std::nullopt`. Whenever we receive a throttle position for the first time,
    * we change its value from std::nullopt to its new numerical value.
    * 
    * When we finally get both throttle positions, we pass both of them into `throttle_map`,
    * which calculates the torque. Afterwards, we set both of them back to std::nullopt so
    * we know we're waiting for new values. */

    /* Last reading of the  first throttle sensor. Averaged with `throttle2_pos`. */
    std::optional<uint16_t> throttle1_pos = std::nullopt;

    /* Last reading of the second throttle sensor. Averaged with `throttle1_pos`. */
    std::optional<uint16_t> throttle2_pos = std::nullopt;

    /* Whether the car is fully on (not starting up). */
    bool car_fully_on = false;

    /* Whether the car is switched on or not. */
    bool start_switch_on = false;

    /* FIXME this is a hack to work around the start switch sensor. */
    bool last_start_switch_value = false;
    Trigger turn_off_timeout;

    /* How far the brake is pressed down. */
    uint16_t brake_pressure = BRAKE_PRESSURE_MIN;

    /* The current mapped torque, pre-smoothing. */
    int16_t mapped_torque = 0;

    /* Calculated torque is sent directly to the motor. */
    int16_t calculated_torque = 0;

    /* When the brake is pressed and the enable switch is flipped, we still
     * wait 2 seconds before enabling the motor. This is that startup countdown. */
    Trigger startup_countdown;

    /* Timer to pace how often we send motor control commands. */
    Timer motor_control_pacing = Timer(0, 15);

    /* throttle mapping memory */
    uint16_t torque_memory[4] = {0, 0, 0, 0};
    /* We only shift to the next value in torque_memory every 20ms. */
    Timer torque_memory_pacing = Timer(0, 20);

};
