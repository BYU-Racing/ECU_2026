#pragma once

#include <cstdint>

constexpr int HORN_PIN = 19;
constexpr int BRAKE_LIGHT_PIN = 13;

/* ASSERTS */
constexpr uint32_t SOFT_RESET_LENGTH_MS = 2000;

/* COMMUNICATIONS */
constexpr long SERIAL_BAUD_RATE = 115200;
constexpr uint32_t CAN_BAUD_RATE = 250000;

/* Interval between broadcasting the system build info. */
constexpr uint32_t BROADCAST_INFO_INTERVAL_MS = 1000;

/* Throttle constants */

/* How long we tolerate between receiving pedal values until we panic. */
constexpr uint32_t PEDAL_TIMEOUT_MS = 200;
/* How long an implausibility is allowed to last before panicking.
 * Why 60 ms, and not 100 ms, like the rules say? Because the DCF
 * only generates messages every 30 ms, so we could receive data
 * that's 30 ms old. 30 ms + 60 ms = 90 ms, which gives us another
 * 10 ms of wiggle room for the code to run. See the 2026 rules,
 * section T.4.2.5 */
constexpr uint32_t ALLOWED_IMPLAUSILIBTY_LENGTH_MS = 60;

/* THROTTLE1_LOW and THROTTLE1_HIGH can be reversed. */
constexpr int16_t THROTTLE1_LOW = 660;
constexpr int16_t THROTTLE1_HIGH = 950;
/* If the throttle 1 values is lower or higher than this range, we panic.
 * Used to comply with T.4.2.10, 2026 rules. */
constexpr int16_t THROTTLE1_MIN_OUT_OF_RANGE = 620;
constexpr int16_t THROTTLE1_MAX_OUT_OF_RANGE = 970;

/* THROTTLE2_LOW and THROTTLE2_HIGH can be reversed. If outside this range, the
 * throttle will clamp to 0%-100%. */
constexpr int16_t THROTTLE2_LOW = 430;
constexpr int16_t THROTTLE2_HIGH = 650;
/* If the throttle 2 value is lower or higher than this range, we panic.
 * Used to comply with T.4.2.10, 2026 rules. */
constexpr int16_t THROTTLE2_MIN_OUT_OF_RANGE = 350;
constexpr int16_t THROTTLE2_MAX_OUT_OF_RANGE = 725;

/* Maximum allowed difference between the two throttles' values,
 * after those values have been mapped to [THROTTLE[X]_MIN, THROTTLE[X]_MAX]. */
constexpr uint16_t THROTTLE_DISAGREE = 10;

constexpr int16_t MIN_THROTTLE = 0; /* In CASCADIA format, 1 = 0.1Nm */
constexpr int16_t MAX_THROTTLE = 100; /* In CASCADIA format, 1 = 0.1Nm */
constexpr uint32_t SMOOTH_PERIOD_MS = 30; /* How often the input is smoothed */
constexpr uint32_t PID_PERIOD_MS = 10;    /* How often the PID is updated */


/* BRAKES */
/* The reported brake level should never be lower than this, or it is considered
 * an unrecoverable fault. */
constexpr uint16_t BRAKE_PRESSURE_MIN = 10;
constexpr uint16_t BRAKE_PRESSURE_MAX = 100;

/* The amount that the brake needs to be pressed down before it's considered
 * pressed by the system. */
constexpr int16_t BRAKE_CONSIDERED_PRESSED = 20;


constexpr uint32_t STARTUP_DELAY_MS = 2000;  /* 2 seconds */

constexpr int16_t TORQUE_FLOOR = 10;

/* MAX TORQUE in CASCADIA format, 1 = 0.1Nm */
constexpr int16_t MAX_TORQUE = 1000;
 
/* Drive Modes */
constexpr int8_t DRIVE_MODE = 0;
