#include "util.hpp"

void Trigger::start(uint32_t current_time_ms, uint32_t duration) {
    started_at = current_time_ms;
    target_duration = duration;
}

void Trigger::startIfStopped(uint32_t current_time_ms, uint32_t duration) {
    if (!this->started()) {
        this->start(current_time_ms, duration);
    }
}

bool Trigger::started() {
    return started_at.has_value();
}

/* Makes it so the next time Trigger::triggerReached is called, it returns true. */
void Trigger::primeTrigger(uint32_t current_time_ms) {
    started_at = current_time_ms;
    target_duration = 0;
}

bool Trigger::triggerReached(uint32_t current_time_ms) {
    if (!started_at.has_value()) return false;

    if (current_time_ms >= (*started_at + target_duration)) {
        /* Reset the trigger, so that Trigger::started returns false. */
        started_at = std::nullopt;
        return true;
    } else {
        return false;
    }
}

void Trigger::cancel() {
    started_at = std::nullopt;
}


/* Timer */
Timer::Timer(uint32_t current_time, uint32_t duration_ms) {
    this->last_fired_at = current_time;
    this->duration_ms = duration_ms;
}

/* Important: this will reset the timer when this returns true. */
bool Timer::shouldFire(uint32_t current_time) {
    if (current_time - this->last_fired_at >= this->duration_ms) {
        this->last_fired_at = current_time;
        return true;
    } else {
        return false;
    }
}

/* Tells us how long until this timer will fire again, or std::nullopt if
 * it's due to fire. */
std::optional<uint32_t> Timer::timeUntilNextFiring(uint32_t current_time_ms) {
    if (current_time_ms - this->last_fired_at < this->duration_ms) {
        return this->duration_ms - (current_time_ms - this->last_fired_at);
    } else {
        return std::nullopt;
    }
}

/* Pid */
Pid::Pid(uint32_t current_time_ms, double p_term, double i_term, double d_term, double windup_cap) {
    this->last_time_ms = current_time_ms;
    this->p_term = p_term;
    this->i_term = i_term;
    this->d_term = d_term;
    this->windup_cap = windup_cap;
}

double Pid::nextValue(uint32_t current_time_ms, double target, double reading) {
    /* Convert the difference (in milliseconds) to seconds. */
    double delta_time = (double)(current_time_ms - this->last_time_ms) / 1000.0;
    double error = target - reading;

    double p = error;

    this->integral += error * delta_time;
    /* Prevent the integral from winding up too much. */
    if (this->integral > this->windup_cap) this->integral = this->windup_cap;
    if (this->integral < -this->windup_cap) this->integral = -this->windup_cap;
    double i = this->integral;

    double d;
    if (delta_time == 0) {
        /* Avoid dividing by 0. */
        d = 0.0;
    } else {
        d = (error - this->last_error) / delta_time;
    }
    this->last_error = error;

    this->last_time_ms = current_time_ms;

    return p * this->p_term + i * this->i_term + d * this->d_term;
}

/* Used to condense the file name into something short enough we can send
 * along the CAN bus in the case of a fault. */
uint32_t str_hash(const char *str) {
    /* djb2 hash. */
    uint32_t hash = 5381;
    uint32_t c;

    while ((c = *str++))
        hash = ((hash << 5) + hash) + c; /* hash * 33 + c */

    return hash;
}

/* Generic function to remap one numerical range to another. */
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
