/* Assertion and panic handling code. This allows any part of the code to use assertions.
 * What's an assertion? Let's look at an example.
 * Say there's a temperature sensor that should never exceed 90 C.
 * You could write code like
 *   uint16_t temperature = read_temperature();
 *   SAFETY_ASSERT(temperature <= 90);
 * This makes it really easy to check if values are out of range. If the assert
 * fails, it will run the registered panic function to properly wind things down. */

/* This is the function type for the panic handler. */
typedef void (*panic_handler_t)(const char* file, int line, const char* msg);

/* Call this to set what the code should do if an assertion fails. */
void register_panic_handler(panic_handler_t handler);

/* Calls the registered panic handler. */
void safety_panic(const char* file, int line, const char* msg);

#define SAFETY_ASSERT(condition) \
    do { \
        if (!(condition)) { \
            safety_panic(__FILE__, __LINE__, ""); \
        } \
    } while (0)
