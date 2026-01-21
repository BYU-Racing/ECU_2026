#include <cstdint>

constexpr int16_t THROTTLE1_MIN = 0;
constexpr int16_t THROTTLE1_MAX = 160;

constexpr int16_t THROTTLE2_MIN = 0;
constexpr int16_t THROTTLE2_MAX = 70;

/* Maximum allowed difference between the two throttles' values,
 * after those values have been mapped to [THROTTLEX_MIN, THROTTLEX_MAX]. */
constexpr uint16_t THROTTLE_DISAGREE = 10;

constexpr int16_t MIN_THROTTLE = 0; /* In CASCADIA format, 1 = 0.1Nm */
constexpr int16_t MAX_THROTTLE = 100; /* In CASCADIA format, 1 = 0.1Nm */

constexpr int16_t ACTIVATE_BRAKE_THRESHOLD = 20;  /* percent */
constexpr uint32_t STARTUP_DELAY_MS = 2000;  /* 2 seconds */

constexpr uint16_t BRAKE_PRESSURE_MIN = 10;

constexpr uint16_t BRAKE_THRESHOLD = 100;
