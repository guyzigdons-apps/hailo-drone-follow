// gst-hailo-cache — test-only CLI for the cache_keys Python parity test.
//
// Plan 5 Task 3: Python's tests/integration/test_cache_keys_python_parity.py
// feeds this binary 100 random (x, y, w, h, q) tuples on stdin and
// asserts the canonical 4-tuple it prints matches
// hailo_tiling.cache.hashing.canonicalize_crop byte-for-byte.
//
// stdin: one tuple per line, space-separated: "x y w h q"
// stdout: one canonical tuple per line, space-separated: "cx cy cw ch"
//
// q == 0 is accepted as a synonym for Python's `quantise=None`
// (identity) so the parity test can pass that case through cleanly.
//
// NOT INSTALLED — see meson `install : false`.

#include "cache_keys.hpp"

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>

int main(int /*argc*/, char** /*argv*/) {
    std::string line;
    while (std::getline(std::cin, line)) {
        if (line.empty()) continue;
        std::istringstream iss(line);
        long long x = 0, y = 0, w = 0, h = 0;
        int q = 0;
        if (!(iss >> x >> y >> w >> h >> q)) {
            std::fprintf(stderr, "cache_keys_cli: parse error on line: %s\n",
                         line.c_str());
            return 2;
        }
        auto out = hailo_cache::canonicalize_crop(
            static_cast<std::int32_t>(x),
            static_cast<std::int32_t>(y),
            static_cast<std::int32_t>(w),
            static_cast<std::int32_t>(h),
            q);
        std::printf("%d %d %d %d\n", out.x, out.y, out.w, out.h);
    }
    return 0;
}
