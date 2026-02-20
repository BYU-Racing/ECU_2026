#pragma once

#include <cstdint>

/* Assertion and panic handling code. This allows any part of the code to use assertions.
 * What's an assertion? Let's look at an example.
 * Say there's a temperature sensor that should never exceed 90°C.
 * You could write code like
 * 
 *   uint16_t temperature = read_temperature();
 *   SAFETY_ASSERT(temperature <= 90);
 * 
 * If the temperature ever reaches 91°C, it will call `assert_failed`, which runs
 * whatever code the user has for handling failed asserts. This makes it really
 * easy to check if values are out of range, or certain preconditions must always
 * be met.
 * 
 * There's one other important thing to know: there are multiple different levels
 * of asserts, each of which the user can choose what to do if it fails. Here are
 * some of them:
 *  - `AssertLevel::Soft` is for minor state checks. If a soft assert fails, it means
 *    that nothing dangerous has happened, just some minor check failed. This will do
 *    a soft reset of the system, where it shuts off the engine for a few seconds
 *    before resuming as normal.
 *  - `AssertLevel::Safety` is for safety-critical checks. If a safety assert fails
 *    the car goes into a unrecoverable shutdown, and the whole thing will need to
 *    be reset.
 *  */


/* Error codes for safety asserts */
enum class AssertCode: uint8_t {
    Unknown = 0,
    BrakeAndThrottle = 1,
    ThrottleOutOfRange = 2,
    ThrottleDisagree = 3,
    TorqueLessThanZero = 4,
    SmoothTorqueLessThanZero = 5,
    ThrottleOverflow = 6,
    BadMessage = 7,
};

enum class AssertLevel {
    Soft,
    Safety,
};

/* This is the function type for the panic handler. */
typedef void (*assert_failed_handler_t)(AssertLevel level, const char* file, int line, AssertCode error_code);

/* Call this to set what the code should do if an assertion fails. */
void register_assert_failed_handler(assert_failed_handler_t handler);

/* Calls the registered assert failure handler. */
void assert_failed(AssertLevel level, const char* file, int line, AssertCode error_code);

#define GENERIC_ASSERT(level, condition, code) \
    do { \
        if (!(condition)) { \
            assert_failed((level), __FILE__, __LINE__, (code)); \
        } \
    } while (0)

#define SOFT_ASSERT(condition, code)   GENERIC_ASSERT(AssertLevel::Soft, (condition), (code))
#define SAFETY_ASSERT(condition, code) GENERIC_ASSERT(AssertLevel::Safety, (condition), (code))
