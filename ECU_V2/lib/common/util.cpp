#include "util.hpp"

void Trigger::start(uint32_t current_time_ms, uint32_t duration) {
    started_at = current_time_ms;
    target_duration = duration;
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

/* Used to condense the file name into something short enough we can send
 * along the CAN bus in the case of a fault. */
uint32_t str_hash(const char *str) {
    /* djb2 hash. */
    uint32_t hash = 5381;
    uint32_t c;

    while (c = *str++)
        hash = ((hash << 5) + hash) + c; /* hash * 33 + c */

    return hash;
}
