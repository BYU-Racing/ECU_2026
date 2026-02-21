#include <iostream>
#include <cstring>
#include <vector>
#include <algorithm>
#include <filesystem>
#include <sstream>

#include "unity.h"

#include "assert.hpp"
#include "util.hpp"

namespace fs = std::filesystem;

void setUp(void) {}

void tearDown(void) {}

int main() {
    std::vector<fs::path> target_dirs = {"lib", "src"};
    std::stringstream csv_entries;

    for (const auto& target_dir : target_dirs) {
        for (const auto& dir_entry : fs::recursive_directory_iterator(target_dir)) {
            if (!fs::is_regular_file(dir_entry)) continue;

            auto path = dir_entry.path();
            auto extension = path.extension();
            if (extension == ".hpp" || extension == ".cpp" || extension == ".c" || extension == ".h") {
                /* This is a code file, so we should generate its hash name. */

                /* Because some developers are on windows, we create a mapping for both forward and
                 * backward slashes. */
                std::string forward = path.generic_string();
                uint32_t forward_hash = str_hash(forward.c_str());
                std::string backward = path.generic_string();
                std::replace(backward.begin(), backward.end(), '/', '\\');
                uint32_t backward_hash = str_hash(backward.c_str());

                std::cerr << forward_hash << ",\"" << forward << "\"\n";
                std::cerr << backward_hash << ",\"" << backward << "\"\n";
            }
        }
    }
}
