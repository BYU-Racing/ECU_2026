#include <exception>

class AssertionError : public std::exception {
    public:
        char* message;
};

void assert(bool assertion, char* message) {
    if (!assertion) {
        AssertionError err;
        err.message = message;
        throw err;
    }
}