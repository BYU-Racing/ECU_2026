#pragma once

#include <cstdint>
#include <optional>

#include "constants.hpp"
#include "CAN_message_t.hpp"
#include "util.hpp"
#include "assert.hpp"

/* This section deals with processing the values from the pedal sensors,
 * currently the two throttle sensors and the brake sensor. It also deals with
 * maintaining safety features of the sensors, like value timeouts and out of
 * range failures. */

/* If an implausibility occurs (as defined in the 2026 rules, section T.4.2.4),
 * we want to save the details of the implausibility. That way, if the
 * implausibility lasts too long, we can report what implausibility it was and
 * where it came from. */
struct ImplausibilityDetails {
    LineInfo line_info;
    uint32_t happened_at_ms;
    AssertCode code;
};

class Pedals {
  public:
    void poll(uint32_t current_time_ms);
    void inputThrottleOnePosition(uint32_t current_time_ms, uint16_t value);
    void inputThrottleTwoPosition(uint32_t current_time_ms, uint16_t value);
    void inputBrakePosition(uint32_t current_time_ms, uint16_t position);
    bool isThrottlePressed();
    uint8_t getThrottleValue();
    bool isBrakePressed();
    int16_t getCurrentTorqueAmount();

  private:
    std::optional<ImplausibilityDetails> implausibility = std::nullopt;

    /* These two triggers are used to handle message timeouts (if it's been too long
     * since we received a throttle message, we'll panic). */
    Trigger too_long_since_throttle_1;
    Trigger too_long_since_throttle_2;
    Trigger too_long_since_brake;

    /* What is the purpose of `std::optional`? It lets us signal that this value
    * may not have a value. Since at the beginning of the program, we don't know
    * the throttle's position, we initialize both of them to `std::nullopt`.
    * Whenever we receive a throttle position for the first time, we change its
    * value from std::nullopt to its new numerical value.
    *
    * When we finally get both throttle positions, they get passed into
    * `maybeRecomputeMappedThrottle`, which maps the throttle positions to a
    * percentage in [0, 100]. Afterwards, we set both positions back to
    * std::nullopt so we know we're waiting for new values. */

    /* Last reading of the first throttle sensor. Averaged with `throttle_2_pos`. */
    std::optional<uint16_t> throttle_1_pos = std::nullopt;
    /* Last reading of the second throttle sensor. Averaged with `throttle_1_pos`. */
    std::optional<uint16_t> throttle_2_pos = std::nullopt;
    /* Mapped throttle value. This is an intermediate result after mapping
     * the two throttle sensors to [0, 100] and averaging them. */
    uint8_t mapped_throttle = 0;

    /* Current brake position. Raw sensor reading. */
    uint16_t brake_pos = 0;

    /* Throttle smoothing memory. */
    uint8_t torque_memory[4] = {0, 0, 0, 0};
    /* We only shift to the next value in torque_memory every SMOOTH_PERIOD_MS ms. */
    Timer torque_memory_pacing = Timer(0, SMOOTH_PERIOD_MS);
    /* The result from smoothing the throttle. */
    uint8_t smoothed_throttle = 0;

    /* If we've received both a new throttle 1 value, and a new throttle 2 value, then
     * this will calculate the new mapped throttle value (as a percentage in range [0, 100]).*/
    void maybeRecomputeMappedThrottle(uint32_t current_time_ms);

    /* Internal helper function used to smooth the throttle through time. */
    void smoothThrottle(uint32_t current_time_ms);

    /* We don't immediately panic when an implausibility occurs, as the rules
     * allow us to tolerate an implausibility for up to 100 ms. See the 2026
     * rules, section T.4.2.5. */
    void noteImplausibility(uint32_t current_time_ms, LineInfo line_info, AssertCode code);
};

struct GpioState {
    bool brake_light_on;
};

class Ecu {
public:
    void processMessage(uint32_t current_time_ms, CAN_message_t msg);

    /* The caller of this should continously call this until it returns
     * `std::nullopt`. For each message emitted, the caller should send
     * along the CAN bus. */
    std::optional<CAN_message_t> poll(uint32_t current_time_ms);

    GpioState Ecu::getGpioState(uint32_t current_time_ms);

    /* smooth torque */
    int16_t smoothThrottle(uint32_t current_time);

    void handleStartupSequence(uint32_t current_time_ms);

private:
    Pedals pedals;

    /* Whether the car is fully on (not starting up). */
    bool car_fully_on = false;

    /* Whether the car is switched on or not. */
    bool start_switch_on = false;

    /* Brake and throttle cannot be pressed at the same time. See the 2026 rules, EV.4.7. */
    bool throttle_and_brake_pressed = false;

    /* FIXME this is a hack to work around the start switch sensor. */
    bool last_start_switch_value = false;
    Trigger turn_off_timeout;

    /* When the brake is pressed and the enable switch is flipped, we still
     * wait 2 seconds before enabling the motor. This is that startup countdown. */
    Trigger startup_countdown;

    /* Timer to pace how often we send motor control commands. */
    Timer motor_control_pacing = Timer(0, 15);
};
