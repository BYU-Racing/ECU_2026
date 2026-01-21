#include "ecu.hpp"

#include "assert.hpp"
#include "can_serde.hpp"
#include "constants.hpp"

/* FIXME this does not properly account for integer overflow. */
int32_t map(int32_t input, int32_t min, int32_t max) {
    if (input < min) {
        input = min;
    } if (input > max) {
        input = max;
    }

    if (max == min) {
        return 0;
    }

    return ((input - min) * 100) / (max - min);
}

int16_t throttle_map(uint16_t throttle1, uint16_t throttle2) {
    int32_t throttleA = map(static_cast<int32_t>(throttle1), THROTTLE1_MIN, THROTTLE1_MAX);
    int32_t throttleB = map(static_cast<int32_t>(throttle2), THROTTLE2_MIN, THROTTLE2_MAX);

    SAFETY_ASSERT(abs(throttleA - throttleB) < THROTTLE_DISAGREE);

    int32_t average = (throttleA + throttleB) / 2;

    /* Make sure the average can fit into the new size (int16_t). */
    SAFETY_ASSERT(average >= INT16_MIN && average <= INT16_MAX);
    int16_t torque_percentage = static_cast<int16_t>(average);

    int16_t torque_mapped = map(torque_percentage, MIN_THROTTLE, MAX_THROTTLE);
    SAFETY_ASSERT(torque_mapped >= 0);

    return torque_mapped;
}

void Ecu::processMessage(uint32_t current_time_ms, CAN_message_t msg) {
    /* If we received a CAN message, update the corresponding value. */
    switch (static_cast<MessageId>(msg.id)) {
        case MessageId::StartSwitch:
            start_switch_on = parse_start_switch(msg);
            break;
        case MessageId::ThrottleOnePosition:
            throttle1_pos = parse_throttle_one_position(msg);
            break;
        case MessageId::ThrottleTwoPosition:
            throttle2_pos = parse_throttle_two_position(msg);
            break;
        case MessageId::BrakePressure:
            brake_pressure = parse_brake_pressure(msg);
            SAFETY_ASSERT(brake_pressure > BRAKE_PRESSURE_MIN);
            break;
        default:
            break;
    }

    /* If both throttles have new values, average them and recalculate the throttle amount. */
    if (throttle1_pos.has_value() && throttle2_pos.has_value()) {
        calculated_torque = throttle_map(*throttle1_pos, *throttle2_pos);
        /* After we read the values, invalidate them as they're now stale. */
        throttle1_pos = std::nullopt;
        throttle2_pos = std::nullopt;
    }

    /* Car startup sequence. */
    if (car_fully_on) {
        /* No need to do the motor startup sequence if the motor is already enabled. */
    } else {
        /* Startup sequence needed. In order to start up the motor, we need two things:
         *   1. The brake needs to be down.
         *   2. The start switch needs to be on.
         * Once these preconditions are met, we can do the startup sequence. 
         * We will wait two seconds before fully starting up, or abort if
         * one of the preconditions stops holding. */
        if ((brake_pressure >= ACTIVATE_BRAKE_THRESHOLD) && start_switch_on) {
            if (!startup_countdown.started()) {
                startup_countdown.start(current_time_ms, STARTUP_DELAY_MS);
            } else if (startup_countdown.triggerReached(current_time_ms)) {
                /* Good to go! */
                car_fully_on = true;
            }
        } else {
            /* One of the preconditions failed, so we need to reset the timer. */
            startup_countdown.cancel();
        }
    }

    if (!start_switch_on) {
        /* Pretty self-explanatory: if the start switch turns off, the car turns off. */
        car_fully_on = false;
    }
}


std::optional<CAN_message_t> Ecu::emitMessage(uint32_t current_time_ms) {
    /* Engine is enabled if all the preconditions of `car_fully_on` passed. */
    engine_enabled = car_fully_on;

    SAFETY_ASSERT(brake_pressure < BRAKE_THRESHOLD);

    if (engine_enabled) {
        /* We pace how often motor control messages are passed along the CAN bus.
        * We do this by using a trigger, which triggers after a certain amount
        * of time, sends the message, restarts itself, and that continues for
        * infinity. */
        if (!motor_control_message_pacing.started()) {
            motor_control_message_pacing.start(current_time_ms, 15/*ms*/);
        } else if (motor_control_message_pacing.triggerReached(current_time_ms)) {
            MotorControlCommand cmd;
            cmd.torque = 0;
            cmd.speed = 0;
            cmd.direction = MotorDirection::Forward;
            cmd.enable_inverter = false;
            cmd.inverter_discharge = false;
            cmd.override_speed = false;
            cmd.torque_limit = 0;

            return create_motor_control_command(cmd);
        }
    } else {
        motor_control_message_pacing.cancel();
    }
}

