#include "Arduino.h"

#include "util.hpp"

void panic_handler(const char* file, int line, const char* msg) {
    // Shut everything down.

    // Write to serial so we get some debug info.
    Serial.printf("Assertion failed! Line %d in file %s with message %s\n", line, file, msg);
}

void setup() {
    // First things first, register the panic handler. If something goes
    // wrong during setup, we'll wind everything down.
    register_panic_handler(panic_handler);
}

void loop() {
    
}