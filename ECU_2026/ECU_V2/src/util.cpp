#include "util.hpp"

// `static` makes this variable private to this file, so nobody
// can directly access `current_handler`.
static panic_handler_t current_handler = nullptr;

void register_panic_handler(panic_handler_t handler) {
    current_handler = handler;
}

void safety_panic(const char* file, int line, const char* msg) {
    if (current_handler != nullptr) {
        current_handler(file, line, msg);
    }
}
