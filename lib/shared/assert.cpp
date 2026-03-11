#include "assert.hpp"

// `static` makes this variable private to this file, so nobody
// can directly access `current_handler`.
static assert_failed_handler_t current_handler = nullptr;

void register_assert_failed_handler(assert_failed_handler_t handler) {
    current_handler = handler;
}

void assert_failed(AssertLevel level, LineInfo line_info, AssertCode error_code) {
    if (current_handler != nullptr) {
        current_handler(level, line_info, error_code);
    }
}
