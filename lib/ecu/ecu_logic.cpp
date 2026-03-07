#include "ecu_logic.hpp"

#include <cmath>
#include <string.h>

#include "assert.hpp"
#include "can_serde.hpp"
#include "constants.hpp"

/* Currently a stub. Implement the torque mapping here. */
int16_t map_throttle_to_torque(uint16_t throttle_percent) {
    int64_t result = map(throttle_percent, 0, 100, 0, MAX_TORQUE);
    /* Make sure we can narrow the integer. */
    SAFETY_ASSERT(result >= INT16_MIN && result <= INT16_MAX, AssertCode::IntegerOverflow);
    return static_cast<int16_t>(result);
}

void Pedals::poll(uint32_t current_time_ms) {
    /* Save the current implausibility value, before we clear
        * `this->implausibility`. This lets us see if the implausibility is
        * still happening. This works because if we clear the implausibility,
        * and get a new implausibility, it means the implausibility hasn't
        * stopped. */
    auto last_implausibility = this->implausibility;
    this->implausibility = std::nullopt;

    /* According to the 2026 rules (section T.4.2.11(b)), if a message
        * hasn't been received since a certain amount of time, we need to
        * panic. The following code handles this timeout scenario. */

    /* We proactively start these triggers, but cancel them when we receive
     * a message (see `inputThrottleOnePosition` for example). */
    this->too_long_since_throttle_1.startIfStopped(current_time_ms, PEDAL_TIMEOUT_MS);
    this->too_long_since_throttle_2.startIfStopped(current_time_ms, PEDAL_TIMEOUT_MS);
    this->too_long_since_brake.startIfStopped(current_time_ms, PEDAL_TIMEOUT_MS);

    if (this->too_long_since_throttle_1.triggerReached(current_time_ms)) {
        SAFETY_ASSERT(false, AssertCode::PedalTimeout);
    }
    if (this->too_long_since_throttle_2.triggerReached(current_time_ms)) {
        SAFETY_ASSERT(false, AssertCode::PedalTimeout);
    }
    if (this->too_long_since_brake.triggerReached(current_time_ms)) {
        SAFETY_ASSERT(false, AssertCode::PedalTimeout);
    }

    if (this->implausibility.has_value()) {
        ImplausibilityDetails details = *this->implausibility;

        if (last_implausibility.has_value()) {
            /* Restore the original implausibility information, so we have
                * both the time and information that the first implausibility
                * happened at. */
            this->implausibility = *last_implausibility;
        }

        /* Use this to panic if an implausibility has occured for too long. */
        if (current_time_ms - details.happened_at_ms > ALLOWED_IMPLAUSILIBTY_LENGTH_MS) {
            /* Because we tracked the original file and error number, we report
                * those details, instead of the current line. */
            assert_failed(AssertLevel::Safety, details.line_info, details.code);
        }
    }

    this->smoothThrottle(current_time_ms);
}

void Pedals::inputThrottleOnePosition(uint32_t current_time_ms, uint16_t value) {
    this->throttle_1_pos = value;
    /* Reset the timeout for receiving a message. */
    this->too_long_since_throttle_1.cancel();
    this->maybeRecomputeMappedThrottle(current_time_ms);
}

void Pedals::inputThrottleTwoPosition(uint32_t current_time_ms, uint16_t value) {
    this->throttle_2_pos = value;
    this->too_long_since_throttle_2.cancel();
    this->maybeRecomputeMappedThrottle(current_time_ms);
}

void Pedals::inputBrakePosition(uint32_t current_time_ms, uint16_t position) {
    /* This argument isn't strictly necessary, but we may use it in the future
     * (the (void) just keeps the compiler from complaining). */
    (void)current_time_ms;

    this->brake_pos = position;
    this->too_long_since_brake.cancel();
}

bool Pedals::isThrottlePressed() {
    return this->smoothed_throttle > 0;
}

bool Pedals::isBrakePressed() {
    return this->brake_pos >= BRAKE_CONSIDERED_PRESSED;
}

int16_t Pedals::getCurrentTorqueAmount() {
    return map_throttle_to_torque(this->smoothed_throttle);
}

void Pedals::maybeRecomputeMappedThrottle(uint32_t current_time_ms) {
    if (this->throttle_1_pos.has_value() && this->throttle_2_pos.has_value()) {
        uint16_t throttle1 = *this->throttle_1_pos;
        uint16_t throttle2 = *this->throttle_2_pos;

        /* Make sure the throttle values are in range. */
        // FIXME these are throwing errors right now.
#ifndef FIXME_HACKS_TO_GET_THINGS_WORKING
        if (this->throttle_1_pos < THROTTLE1_MIN_OUT_OF_RANGE) {
            /* This is a garbage value, so we really shouldn't use it.
            * Hence, we'll return early so the rest of the code in
            * this function doesn't run. */
            this->noteImplausibility(current_time_ms, CAPTURE_LINE_INFO(), AssertCode::ThrottleOutOfRange);
            return;
        }
        if (this->throttle_1_pos > THROTTLE1_MAX_OUT_OF_RANGE) {
            this->noteImplausibility(current_time_ms, CAPTURE_LINE_INFO(), AssertCode::ThrottleOutOfRange);
            return;
        }
        if (this->throttle_2_pos < THROTTLE2_MIN_OUT_OF_RANGE) {
            this->noteImplausibility(current_time_ms, CAPTURE_LINE_INFO(), AssertCode::ThrottleOutOfRange);
            return;
        }
        if (this->throttle_2_pos > THROTTLE2_MAX_OUT_OF_RANGE) {
            this->noteImplausibility(current_time_ms, CAPTURE_LINE_INFO(), AssertCode::ThrottleOutOfRange);
            return;
        }
#endif

        int64_t throttle_1_percent = map(throttle1, THROTTLE1_LOW, THROTTLE1_HIGH, 0, 100);
        int64_t throttle_2_percent = map(throttle2, THROTTLE2_LOW, THROTTLE2_HIGH, 0, 100);

        /* Throttle values may go slightly below 0 or above 100, so we'll
            * just saturate at those values (if it's significantly out of
            * range, the preconditions of this block will return early). */
        if (throttle_1_percent < 0) throttle_1_percent = 0;
        if (throttle_1_percent > 100) throttle_1_percent = 100;
        if (throttle_2_percent < 0) throttle_2_percent = 0;
        if (throttle_2_percent > 100) throttle_2_percent = 100;

        // FIXME this is throwing an error.
        /* Make sure the two throttle values haven't diverged too far. */
#ifndef FIXME_HACKS_TO_GET_THINGS_WORKING
        if (abs(throttle_1_percent - throttle_2_percent) >= THROTTLE_DISAGREE) {
            this->noteImplausibility(current_time_ms, CAPTURE_LINE_INFO(), AssertCode::ThrottleDisagree);
            return;
        }
#endif

        // FIXME actually average the values.
#ifdef FIXME_HACKS_TO_GET_THINGS_WORKING
        int64_t average = throttle_1_percent;
#else
        int64_t average = (throttle_1_percent + throttle_2_percent) / 2;
#endif

        this->mapped_throttle = average;
    }
}

void Pedals::smoothThrottle(uint32_t current_time_ms) {
    /* Smooth out throttle values by averaging out previous values. */

    constexpr size_t torque_memory_len = sizeof(this->torque_memory) / sizeof(this->torque_memory[0]);

    /* If we receive a value of 0, reset history and return 0. */
    if (this->mapped_throttle == 0) {
        memset(&this->torque_memory, 0, sizeof(this->torque_memory));
        this->smoothed_throttle = 0;
    }

    if (this->torque_memory_pacing.shouldFire(current_time_ms)) {
        /* Only cycle memory when it's been long enough. */

        /* Cycle through last torque values. */
        for (size_t i = torque_memory_len - 1; i >= 1; i--) {
            this->torque_memory[i] = this->torque_memory[i - 1];
        }

        this->torque_memory[0] = this->mapped_throttle;
    }

    int32_t total_torque = 0;
    /* Sum up history. */
    for (size_t i = 0; i < torque_memory_len; i++) {
        total_torque += torque_memory[i];
    }

    int16_t averaged = static_cast<int16_t>(total_torque / torque_memory_len);
    SAFETY_ASSERT(averaged >= 0, AssertCode::SmoothTorqueLessThanZero);

    this->smoothed_throttle = averaged;
}

/* This records that an implausibility happened. We don't immediately panic when
 * an implausibility occurs, as the rules allow us to tolerate an implausibility
 * for up to 100 ms (2026 rules, section T.4.2.5). */
void Pedals::noteImplausibility(uint32_t current_time_ms, LineInfo line_info, AssertCode code) {
    ImplausibilityDetails details;
    details.happened_at_ms = current_time_ms;
    details.line_info = line_info;
    details.code = code;
    this->implausibility = details;
}

/* Ecu implementation. */
void Ecu::handleStartupSequence(uint32_t current_time_ms) {
    bool debounced_switch;

    /* FIXME hack working around the non-debounced switch. */
    if (this->last_start_switch_value == this->start_switch_on) {
        /* Nothing to do, as the last value is the same as the current value. */
        debounced_switch = this->start_switch_on;
        this->turn_off_timeout.cancel();
    } else if (this->start_switch_on) {
        debounced_switch = true;
        this->last_start_switch_value = true;
        this->turn_off_timeout.cancel();
    } else {
        /* The switch was turned off, but we need to wait a second before
         * considering it switched off. */

        if (!this->turn_off_timeout.started()) {
            /* If we haven't started the timer yet, go ahead and start it. */
            this->turn_off_timeout.start(current_time_ms, 1000);
            /* Keep the last value while we wait. */
            debounced_switch = true;
        } else if (this->turn_off_timeout.triggerReached(current_time_ms)) {
            /* It's been long enough to consider it off. */
            debounced_switch = false;
            this->last_start_switch_value = false;
        } else {
            /* Timer is running but hasn't reached yet - keep the old value. */
            debounced_switch = true;
        }
    }

    /* Car startup sequence. */
    if (this->car_fully_on) {
        /* No need to do the motor startup sequence if the car is already fully started. */
    } else {
        /* Startup sequence needed. In order to start up the motor, we need two things:
         *   1. The brake needs to be down.
         *   2. The start switch needs to be on.
         * Once these preconditions are met, we can do the startup sequence.
         * We will also wait two seconds before fully starting up, or abort if
         * one of the preconditions stops holding. */

        /* FIXME right now we ignore the brake value. */
#ifdef FIXME_HACKS_TO_GET_THINGS_WORKING
        if (debounced_switch) {
#else
        if (this->pedals.isBrakePressed() && debounced_switch) {
#endif
            if (!this->startup_countdown.started()) {
                this->startup_countdown.start(current_time_ms, STARTUP_DELAY_MS);
            } else if (this->startup_countdown.triggerReached(current_time_ms)) {
                /* Good to go! */
                this->car_fully_on = true;
            }
        } else {
            /* One of the preconditions failed, so we need to reset the timer. */
            this->startup_countdown.cancel();
        }
    }

    if (!debounced_switch) {
        /* Pretty self-explanatory: if the start switch turns off, the car turns off. */
        this->car_fully_on = false;
    }
}

void Ecu::processMessage(uint32_t current_time_ms, CAN_message_t msg) {
    /* If we received a CAN message, update the corresponding value. */
    switch (static_cast<MessageId>(msg.id)) {
        case MessageId::StartSwitch:
            this->start_switch_on = parse_start_switch(msg);
            break;
        case MessageId::ThrottleOnePosition:
            this->pedals.inputThrottleOnePosition(current_time_ms, parse_throttle_one_position(msg));
            break;
        case MessageId::ThrottleTwoPosition:
            this->pedals.inputThrottleTwoPosition(current_time_ms, parse_throttle_two_position(msg));
            break;
        case MessageId::BrakePressure:
            this->pedals.inputBrakePosition(current_time_ms, parse_brake_pressure(msg));
            break;
        default:
            break;
    }
};

std::optional<CAN_message_t> Ecu::poll(uint32_t current_time_ms) {
    this->handleStartupSequence(current_time_ms);
    this->pedals.poll(current_time_ms);

    /* Engine is enabled if all the preconditions of `car_fully_on` passed,
     * and if the brake is up. We don't enable the inverter until the
     * driver has lifted up the brake. */

    bool inverter_enabled = false;
    int16_t torque_to_use = 0;

    if (this->car_fully_on) {
        inverter_enabled = true;
        torque_to_use = this->pedals.getCurrentTorqueAmount();
    }

    /* Brake and throttle cannot be pressed at the same time. */
    // FIXME this isn't working.
#ifndef FIXME_HACKS_TO_GET_THINGS_WORKING
    SAFETY_ASSERT(!(this->pedals.isThrottlePressed() && this->pedals.isBrakePressed()), AssertCode::BrakeAndThrottle);
#endif

    /* Pace how often we send a motor command by using a timer. Note, we still
     * send these messages, even when the inverter is off, so that the motor
     * gets shutdown messages. */
    if (this->motor_control_pacing.shouldFire(current_time_ms)) {
        MotorControlCommand cmd;
        cmd.torque = torque_to_use;
        cmd.speed = 0;
        cmd.direction = MotorDirection::Forward;
        cmd.enable_inverter = inverter_enabled;
        cmd.inverter_discharge = false;
        cmd.override_speed = false;
        cmd.torque_limit = 0;

        return create_motor_control_command(cmd);
    }

    /* No message was generated, so let the caller know the don't need to keep sending messages. */
    return std::nullopt;
}
