#pragma once

#include <cstdint>

/* ASSERTS */
constexpr uint32_t SOFT_RESET_LENGTH_MS = 2000;

/* COMMUNICATIONS */
constexpr long SERIAL_BAUD_RATE = 115200;
constexpr uint32_t CAN_BAUD_RATE = 250000;

/* THROTTLE1_LOW and THROTTLE1_HIGH can be reversed. */
constexpr int16_t THROTTLE1_LOW = 250;
constexpr int16_t THROTTLE1_HIGH = 30;
constexpr int16_t THROTTLE1_MIN_OUT_OF_RANGE = 5;
constexpr int16_t THROTTLE1_MAX_OUT_OF_RANGE = 290;

/* THROTTLE2_LOW and THROTTLE2_HIGH can be reversed. */
constexpr int16_t THROTTLE2_LOW = 160;
constexpr int16_t THROTTLE2_HIGH = 177;
constexpr int16_t THROTTLE2_MIN_OUT_OF_RANGE = 150;
constexpr int16_t THROTTLE2_MAX_OUT_OF_RANGE = 190;

/* Maximum allowed difference between the two throttles' values,
 * after those values have been mapped to [THROTTLE[X]_MIN, THROTTLE[X]_MAX]. */
constexpr uint16_t THROTTLE_DISAGREE = 10;

constexpr int16_t MIN_THROTTLE = 0; /* In CASCADIA format, 1 = 0.1Nm */
constexpr int16_t MAX_THROTTLE = 100; /* In CASCADIA format, 1 = 0.1Nm */
constexpr uint32_t SMOOTH_PERIOD_MS = 10; /* How often the input is smoothed */


/* BRAKES */
/* The reported brake level should never be lower than this, or it is considered
 * an unrecoverable fault. */
constexpr uint16_t BRAKE_PRESSURE_MIN = 10;

/* The amount that the brake needs to be pressed down before it's considered
 * pressed by the system. */
constexpr int16_t BRAKE_CONSIDERED_PRESSED = 20;


constexpr uint32_t STARTUP_DELAY_MS = 2000;  /* 2 seconds */

constexpr int16_t TORQUE_FLOOR =10;

/* MAX TORQUE */
/* N * 0.1 */
constexpr int16_t MAX_TORQUE = 200;
 
/* Drive Modes */
constexpr int8_t DRIVE_MODE = 0;
