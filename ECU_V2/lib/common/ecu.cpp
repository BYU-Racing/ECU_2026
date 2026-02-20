#include "ecu.hpp"

#include <cmath>

#include "assert.hpp"
#include "can_serde.hpp"
#include "constants.hpp"


int64_t map(int32_t input_32, int32_t old_min_32, int32_t old_max_32, int32_t new_min_32, int32_t new_max_32) {
    if (old_min_32 == old_max_32 || new_min_32 == new_max_32) {
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

int16_t Ecu::smooth_torque(uint16_t current_time_ms, uint16_t torque) {
    /* smooth out throttle values by averaging out previous four values */

    /* If recieves a value of 0 return 0 and set all values to 0 ;) */
    if (torque == 0) {
        for (int i = 0; i < 4; i++)
        {
            this->torque_memory[i] = 0;
        }
        this->last_output_torque = 0;
        return 0;
    }

    /* Continues to send same torque message until 20ms has passed */
    if ((uint16_t)(current_time_ms - last_smooth_update_ms) < SMOOTH_PERIOD_MS)
    {
        return this->last_output_torque;
    }
    /* Starts timer for next cycle when 20ms has passed */
    this->last_smooth_update_ms = current_time_ms;

    uint16_t total_torque = 0;
    /* cycle through last 4 torque values*/
    this->torque_memory[3] = this->torque_memory[2];
    this->torque_memory[2] = this->torque_memory[1];
    this->torque_memory[1] = this->torque_memory[0];
    this->torque_memory[0] = torque;

    /* sum last 4 torque values */
    for (int i = 0; i < 4; i++) {
        total_torque += this->torque_memory[i];
    }

    /* average out torque and check for errors */
    total_torque = total_torque / 4;
    SAFETY_ASSERT_CODE((total_torque >= 0), AssertCode::SmoothTorqueLessThanZero);
    this->last_output_torque = total_torque;
    return total_torque;
}


int16_t throttle_map(uint16_t throttle1, uint16_t throttle2) {
    /* Make sure the throttle values are in range. */
    SAFETY_ASSERT_CODE((throttle1 >= THROTTLE1_MIN && throttle1 <= THROTTLE1_MAX), AssertCode::ThrottleOutOfRange);
    SAFETY_ASSERT_CODE((throttle2 >= THROTTLE2_MIN && throttle2 <= THROTTLE2_MAX), AssertCode::ThrottleOutOfRange);

    int64_t throttle1_percent = map(throttle1, THROTTLE1_MIN, THROTTLE1_MAX, 0, 100);
    /* DEBUG ONLY!!!! Throttle 2 set to throttle 1 :) */
    // int64_t throttle2_percent = map(throttle1, THROTTLE1_MIN, THROTTLE1_MAX, 0, 100);
    int64_t throttle2_percent = map(throttle2, THROTTLE2_MIN, THROTTLE2_MAX, 0, 100);

    // FIXME this is throwing an error
    /* Make sure the two throttle values haven't diverged too far. */
    SAFETY_ASSERT_CODE((abs(throttle1_percent - throttle2_percent) < THROTTLE_DISAGREE), AssertCode::ThrottleDisagree);

    int64_t average = (throttle1_percent + throttle2_percent) / 2;

    // FIXME this throwing an error
    /* Make sure the average can fit into the new size (int16_t). */
    SAFETY_ASSERT_CODE((average >= MIN_THROTTLE && average <= MAX_THROTTLE), AssertCode::ThrottleOverflow);

    int16_t torque_mapped = map(average, MIN_THROTTLE, MAX_THROTTLE, 0, MAX_TORQUE);
    SAFETY_ASSERT_CODE((torque_mapped >= 0), AssertCode::TorqueLessThanZero);
    /* TODO look into this */
    if (torque_mapped < 0) {
        torque_mapped = 0;
    }

    return torque_mapped;
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

        /* mapped_torque must be higher than TORQUE_FLOOR to start the motor. */
        if (mapped_torque < TORQUE_FLOOR)
        {
            mapped_torque = 0;
        }

        // FIXME this isn't working
        /* Brake and throttle cannot be pressed at the same time. */
        SAFETY_ASSERT_CODE(!(mapped_torque > 0 && this->brake_pressure >= BRAKE_THRESHOLD), AssertCode::BrakeAndThrottle);
        int16_t smoothed_torque = smooth_torque(current_time_ms, mapped_torque);
        this->calculated_torque = smoothed_torque;
    }

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

        /* DEBUG ONLY */
        // FIXME this needs to be looked over 
        // if ((this->brake_pressure >= BRAKE_THRESHOLD) && this->start_switch_on) {
        if (this->start_switch_on) {
            if (!this->startup_countdown.started())
            {
                this->startup_countdown.start(current_time_ms, STARTUP_DELAY_MS);
            } else if (this->startup_countdown.triggerReached(current_time_ms)) {
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
    bool inverter_enabled = false;

    /* Engine is enabled if all the preconditions of `car_fully_on` passed,
     * and if the brake is up. We don't enable the inverter until the
     * driver has lifted up the brake.  */
    // if (this->car_fully_on && this->brake_pressure < BRAKE_THRESHOLD) {
    if (this->car_fully_on)
    {
        inverter_enabled = true;
    }

    /* The engine needs to be enabled, and the calculated_torque needs to be
     * high enough to actually engage the motor. */
    int16_t torque_to_use = 0;
    if (inverter_enabled && this->calculated_torque > TORQUE_FLOOR)
    {
        torque_to_use = this->calculated_torque;
    }

    /* We need to pace how often motor control messages are passed along the CAN bus.
     * We do this by using a trigger. To use the trigger, we first start it. Once
     * enough time has passed since we started it, `triggerReached` will return true,
     * and mark the trigger as not started. Next time, we'll see that the trigger is
     * marked as not started, and we'll start it again. This loops forever. */
    if (!this->motor_control_message_pacing.started())
    {
        this->motor_control_message_pacing.start(current_time_ms, 15 /*ms*/);
    }
    else if (this->motor_control_message_pacing.triggerReached(current_time_ms))
    {
        MotorControlCommand cmd;
        cmd.torque = torque_to_use;
        cmd.speed = 0;
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
    PRINTF("motor_control_message_pacing: %s\n", this->motor_control_message_pacing.started() ? "started" : "not started");
}
