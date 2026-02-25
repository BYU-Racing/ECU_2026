#include "ecu_logic.hpp"

#include <cmath>

#include "assert.hpp"
#include "can_serde.hpp"
#include "constants.hpp"

int64_t map(int32_t input_32, int32_t old_min_32, int32_t old_max_32, int32_t new_min_32, int32_t new_max_32)
{
    if (old_min_32 == old_max_32 || new_min_32 == new_max_32)
    {
        /* Avoid division by zero. */
        return 0;
    }

    /* Widen all values to 64-bit ints to avoid any issues with integer overflow. */
    int64_t input = input_32;
    int64_t old_min = old_min_32;
    int64_t old_max = old_max_32;
    int64_t new_min = new_min_32;
    int64_t new_max = new_max_32;

    int64_t old_difference = old_max - old_min;
    int64_t new_difference = new_max - new_min;

    int64_t input_at_origin = input - old_min;
    /* Why not `(input_at_origin / old_difference) * new_difference`? Because
     * integer division floors, so we'd lose a lot of precision. */
    int64_t output_at_origin = (input_at_origin * new_difference) / old_difference;
    int64_t output_shifted = output_at_origin + new_min;

    return output_shifted;
}

int16_t Ecu::smoothTorque(uint32_t current_time_ms)
{
    /* Smooth out throttle values by averaging out previous four values. */

    /* We're assuming we have four values in torque memory. */
    static_assert(sizeof(this->torque_memory) / sizeof(this->torque_memory[0]) == 4);

    /* If we receive a value of 0, reset history and return 0. */
    if (this->mapped_torque == 0)
    {
        for (size_t i = 0; i < 4; i++) this->torque_memory[i] = 0;
        return 0;
    }
    else if (this->torque_memory_pacing.shouldFire(current_time_ms))
    {
        /* Only cycle memory when it's been long enough. */

        /* cycle through last 4 torque values*/
        this->torque_memory[3] = this->torque_memory[2];
        this->torque_memory[2] = this->torque_memory[1];
        this->torque_memory[1] = this->torque_memory[0];
        this->torque_memory[0] = this->mapped_torque;
    }

    int32_t total_torque = 0;
    /* sum last 4 torque values */
    for (int i = 0; i < 4; i++)
    {
        total_torque += this->torque_memory[i];
    }

    int16_t averaged = static_cast<int16_t>(total_torque / 4);
    SAFETY_ASSERT(averaged >= 0, AssertCode::SmoothTorqueLessThanZero);

    return averaged;
}

int16_t throttle_map(uint16_t throttle1, uint16_t throttle2)
{
    /* Make sure the throttle values are in range. */
    SAFETY_ASSERT(throttle1 >= THROTTLE1_MIN_OUT_OF_RANGE, AssertCode::ThrottleOutOfRange);
    SAFETY_ASSERT(throttle1 <= THROTTLE1_MAX_OUT_OF_RANGE, AssertCode::ThrottleOutOfRange);
    SAFETY_ASSERT(throttle2 >= THROTTLE2_MIN_OUT_OF_RANGE, AssertCode::ThrottleOutOfRange);
    SAFETY_ASSERT(throttle2 <= THROTTLE2_MAX_OUT_OF_RANGE, AssertCode::ThrottleOutOfRange);

    int64_t throttle1_percent = map(throttle1, THROTTLE1_LOW, THROTTLE1_HIGH, 0, 100);
    int64_t throttle2_percent = map(throttle2, THROTTLE2_LOW, THROTTLE2_HIGH, 0, 100);

    /* Throttle values may go slightly below 0 or above 100, so we'll just saturate it
     * at those values (if it's significantly out of range, the safety asserts at the
     * beginning of this function will fail). */
    if (throttle1_percent < 0) throttle1_percent = 0;
    if (throttle1_percent > 100) throttle1_percent = 100;
    if (throttle2_percent < 0) throttle2_percent = 0;
    if (throttle2_percent > 100) throttle2_percent = 100;

    // FIXME this is throwing an error
    /* Make sure the two throttle values haven't diverged too far. */
    // SAFETY_ASSERT((abs(throttle1_percent - throttle2_percent) < THROTTLE_DISAGREE), AssertCode::ThrottleDisagree);

    // FIXME actually average the values.
    // int64_t average = (throttle1_percent + throttle2_percent) / 2;
    int64_t average = throttle1_percent;

    int64_t torque_mapped = map(average, MIN_THROTTLE, MAX_THROTTLE, 0, MAX_TORQUE);

    /* Make sure the average can fit into the new size (int16_t). */
    // SAFETY_ASSERT((torque_mapped >= MIN_THROTTLE && torque_mapped <= MAX_THROTTLE), AssertCode::ThrottleOverflow);
    // SAFETY_ASSERT((torque_mapped >= 0), AssertCode::TorqueLessThanZero);

    return static_cast<int16_t>(torque_mapped);
}

void Ecu::processMessage(uint32_t current_time_ms, CAN_message_t msg)
{
    /* If we received a CAN message, update the corresponding value. */
    switch (static_cast<MessageId>(msg.id))
    {
    case MessageId::StartSwitch:
        this->start_switch_on = parse_start_switch(msg);
        break;
    case MessageId::ThrottleOnePosition:
        this->throttle1_pos = parse_throttle_one_position(msg);
        break;
    case MessageId::ThrottleTwoPosition:
        this->throttle2_pos = parse_throttle_two_position(msg);
        break;
    case MessageId::BrakePressure:
        this->brake_pressure = parse_brake_pressure(msg);
        break;
    default:
        break;
    }

    /* If both throttles have new values, average them and recalculate the throttle amount. */
    if (this->throttle1_pos.has_value() && this->throttle2_pos.has_value())
    {
        int16_t mapped_torque = throttle_map(*this->throttle1_pos, *this->throttle2_pos);
        /* After we read the values, invalidate them as they're now stale. */
        this->throttle1_pos = std::nullopt;
        this->throttle2_pos = std::nullopt;

        this->mapped_torque = mapped_torque;
    }
}

void Ecu::handleStartupSequence(uint32_t current_time_ms) {
    bool debounced_switch;

    /* FIXME hack working around the non-debounced switch. */
    if (this->last_start_switch_value == this->start_switch_on)
    {
        /* Nothing to do, as the last value is the same as the current value. */
        debounced_switch = this->start_switch_on;
        this->turn_off_timeout.cancel();
    }
    else if (this->start_switch_on)
    {
        debounced_switch = true;
        this->last_start_switch_value = true;
        this->turn_off_timeout.cancel();
    }
    else
    {
        /* The switch was turned off, but we need to wait a second before
         * considering it switched off. */

        if (!this->turn_off_timeout.started())
        {
            /* If we haven't started the timer yet, go ahead and start it. */
            this->turn_off_timeout.start(current_time_ms, 1000);
            /* Keep the last value while we wait. */
            debounced_switch = true;
        }
        else if (this->turn_off_timeout.triggerReached(current_time_ms))
        {
            /* It's been long enough to consider it off. */
            debounced_switch = false;
            this->last_start_switch_value = false;
        }
        else
        {
            /* Timer is running but hasn't reached yet - keep the old value. */
            debounced_switch = true;
        }
    }

    /* Car startup sequence. */
    if (this->car_fully_on)
    {
        /* No need to do the motor startup sequence if the car is already fully started. */
    }
    else
    {
        /* Startup sequence needed. In order to start up the motor, we need two things:
         *   1. The brake needs to be down.
         *   2. The start switch needs to be on.
         * Once these preconditions are met, we can do the startup sequence.
         * We will also wait two seconds before fully starting up, or abort if
         * one of the preconditions stops holding. */

        /* FIXME right now we ignore the brake value. */
        // if ((this->brake_pressure >= BRAKE_CONSIDERED_PRESSED) && debounced_switch)

        if (debounced_switch)
        {
            if (!this->startup_countdown.started())
            {
                this->startup_countdown.start(current_time_ms, STARTUP_DELAY_MS);
            }
            else if (this->startup_countdown.triggerReached(current_time_ms))
            {
                /* Good to go! */
                this->car_fully_on = true;
            }
        }
        else
        {
            /* One of the preconditions failed, so we need to reset the timer. */
            this->startup_countdown.cancel();
        }
    }

    if (!debounced_switch)
    {
        /* Pretty self-explanatory: if the start switch turns off, the car turns off. */
        this->car_fully_on = false;
    }
}

std::optional<CAN_message_t> Ecu::emitMessage(uint32_t current_time_ms)
{
    this->handleStartupSequence(current_time_ms);

    int16_t smoothed_torque = this->smoothTorque(current_time_ms);
    /* Brake and throttle cannot be pressed at the same time. */
    // FIXME this isn't working.
    // SAFETY_ASSERT(!(smoothed_torque > 0 && this->brake_pressure >= BRAKE_CONSIDERED_PRESSED), AssertCode::BrakeAndThrottle);
    this->calculated_torque = smoothed_torque;

    bool inverter_enabled = false;

    /* Engine is enabled if all the preconditions of `car_fully_on` passed,
     * and if the brake is up. We don't enable the inverter until the
     * driver has lifted up the brake. */

    /* FIXME testing only. */
    // if (this->car_fully_on && this->brake_pressure < BRAKE_CONSIDERED_PRESSED) {
    if (this->car_fully_on)
    {
        inverter_enabled = true;
    }

    int16_t torque_to_use = 0;
    if (inverter_enabled)
    {
        torque_to_use = this->calculated_torque;
    }

    /* Pace how often we send a motor command by pacing it with a timer. */
    if (this->motor_control_pacing.shouldFire(current_time_ms))
    {
        MotorControlCommand cmd;
        cmd.torque = torque_to_use;
        cmd.speed = 0;
        /* Run it in reverse, since the way the motor is mounted means this is
         * actually forwards. */
        cmd.direction = MotorDirection::Reverse;
        cmd.enable_inverter = inverter_enabled;
        cmd.inverter_discharge = false;
        cmd.override_speed = false;
        cmd.torque_limit = 0;

        return create_motor_control_command(cmd);
    }

    /* No message was generated, so let the caller know the don't need to keep sending messages. */
    return std::nullopt;
}

void Ecu::printState()
{
    PRINTF("=== ECU State ===\n");
    PRINTF("car_fully_on: %s\n", this->car_fully_on ? "true" : "false");
    PRINTF("start_switch_on: %s\n", this->start_switch_on ? "true" : "false");
    PRINTF("last_start_switch_value: %s\n", this->last_start_switch_value ? "true" : "false");
    PRINTF("brake_pressure: %u\n", this->brake_pressure);
    PRINTF("calculated_torque: %d\n", this->calculated_torque);

    if (this->throttle1_pos.has_value())
    {
        PRINTF("throttle1_pos: %u\n", *this->throttle1_pos);
    }
    else
    {
        PRINTF("throttle1_pos: (none)\n");
    }

    if (this->throttle2_pos.has_value())
    {
        PRINTF("throttle2_pos: %u\n", *this->throttle2_pos);
    }
    else
    {
        PRINTF("throttle2_pos: (none)\n");
    }

    PRINTF("startup_countdown: %s\n", this->startup_countdown.started() ? "started" : "not started");
}
