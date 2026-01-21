#include <cstdint>
#include <optional>

class Trigger {
public:
    void start(uint32_t current_time_ms, uint32_t target_duration);
    bool started();
    void primeTrigger(uint32_t current_time_ms);
    bool triggerReached(uint32_t current_time_ms);
    void cancel();

private:
    std::optional<uint32_t> started_at = std::nullopt;
    uint32_t target_duration = 0;
};

#ifdef BUILD_MODE_TEST
#include <cstdio>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#include "Arduino.h"
#define PRINTF(...) Serial.printf(__VA_ARGS__)
#endif
