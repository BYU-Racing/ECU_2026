#pragma once

#include <cstdint>
#include <optional>

class Trigger {
public:
    void start(uint32_t current_time_ms, uint32_t target_duration);
    void startIfStopped(uint32_t current_time_ms, uint32_t target_duration);
    bool started();
    void primeTrigger(uint32_t current_time_ms);
    bool triggerReached(uint32_t current_time_ms);
    void cancel();

private:
    std::optional<uint32_t> started_at = std::nullopt;
    uint32_t target_duration = 0;
};

class Timer {
public:
    Timer(uint32_t current_time, uint32_t frequency);
    bool shouldFire(uint32_t current_time);
    std::optional<uint32_t> timeUntilNextFiring(uint32_t current_time_ms);
private:
    uint32_t last_fired_at = 0;
    uint32_t duration_ms = 0;
};

class Pid {
  public:
    /* A classic PID implementation, with an additional windup cap. The windup
     * cap prevents both positive and negative windup. The cap itself should be positive. */
    Pid(uint32_t current_time_ms, double p_term, double i_term, double d_term, double windup_cap);
    double nextValue(uint32_t current_time_ms, double target, double reading);
  private:
    double p_term;
    double i_term;
    double d_term;
    double windup_cap;

    double last_error = 0.0;
    double integral = 0.0;
    uint32_t last_time_ms;
};

#ifdef BUILD_MODE_TEST
#include <cstdio>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#include "Arduino.h"
#define PRINTF(...) Serial.printf(__VA_ARGS__)
#endif

#define UNUSED(x) ((void)x)

uint32_t str_hash(const char *str);
int64_t map(int32_t input_32, int32_t old_min_32, int32_t old_max_32, int32_t new_min_32, int32_t new_max_32);
