// gst-hailo-cache — test-only CLI for the bit-exact Python round-trip.
//
// Plan 5 Task 2: Python's `tests/integration/test_cache_cpp_round_trip.py`
// writes 10 rows through THIS binary, then re-opens the resulting SQLite
// via hailo_tiling.cache.SqliteCacheStore and asserts the dets_json comes
// back byte-identical.
//
// Usage:
//   cache_db_cli <sqlite-path>
//
// Stdin must be JSON-lines, one row per line, with keys:
//   frame_idx, crop_x, crop_y, crop_w, crop_h, ppv, dets_json, ts_epoch
//
// We deliberately do NOT pull in a JSON library — we parse the small
// fixed schema by hand. dets_json is the field that needs byte-perfect
// preservation, so the parser reads it as a literal string between the
// outer quotes with backslash-escape handling that mirrors json.dumps's
// default output (`json.dumps(..., separators=(",",":"))`).
//
// NOT INSTALLED — see meson `install : false`.

#include "tile_cache_db.hpp"

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

// Minimal JSON-ish extractor for a specific key in a flat JSON object.
// Returns the raw value string (no outer quotes for string types).
//
// Crude but adequate: matches `"key":` and reads until the next comma
// or end-of-object at the top level. For dets_json (which is a string
// containing escaped quotes), we go through extract_string_value().
std::string find_key_(const std::string& line, const std::string& key) {
    std::string needle = "\"" + key + "\":";
    size_t pos = line.find(needle);
    if (pos == std::string::npos) {
        throw std::runtime_error("missing key: " + key);
    }
    size_t i = pos + needle.size();
    // Skip optional whitespace after the colon (json.dumps default
    // emits `"k": v`; compact emits `"k":v` — accept both).
    while (i < line.size() && (line[i] == ' ' || line[i] == '\t')) ++i;
    return line.substr(i);
}

// Read an int from a position-1 substring. Stops at comma/brace/space.
long long extract_int_(const std::string& s) {
    size_t end = 0;
    while (end < s.size() && (std::isdigit((unsigned char)s[end]) || s[end] == '-' || s[end] == '+')) {
        ++end;
    }
    return std::stoll(s.substr(0, end));
}

double extract_double_(const std::string& s) {
    size_t end = 0;
    while (end < s.size() && (std::isdigit((unsigned char)s[end]) ||
                              s[end] == '-' || s[end] == '+' ||
                              s[end] == '.' || s[end] == 'e' || s[end] == 'E')) {
        ++end;
    }
    return std::stod(s.substr(0, end));
}

// Extract a JSON string value (with proper backslash-escape handling)
// starting at the first `"` in `s`. Returns the DECODED string (so the
// stored dets_json is byte-identical to the input — same as Python).
std::string extract_string_(const std::string& s) {
    size_t i = s.find('"');
    if (i == std::string::npos) throw std::runtime_error("expected string");
    ++i;
    std::string out;
    while (i < s.size()) {
        char c = s[i];
        if (c == '\\') {
            if (i + 1 >= s.size()) throw std::runtime_error("bad escape");
            char n = s[i + 1];
            switch (n) {
                case '"':  out.push_back('"');  break;
                case '\\': out.push_back('\\'); break;
                case '/':  out.push_back('/');  break;
                case 'b':  out.push_back('\b'); break;
                case 'f':  out.push_back('\f'); break;
                case 'n':  out.push_back('\n'); break;
                case 'r':  out.push_back('\r'); break;
                case 't':  out.push_back('\t'); break;
                default:
                    // We don't handle \uXXXX here — the Python test driver
                    // only emits ASCII dets_json, which is enough for
                    // round-trip parity.
                    throw std::runtime_error(std::string("unknown escape \\") + n);
            }
            i += 2;
        } else if (c == '"') {
            return out;
        } else {
            out.push_back(c);
            ++i;
        }
    }
    throw std::runtime_error("unterminated string");
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::fprintf(stderr, "usage: %s <sqlite-path>\n", argv[0]);
        return 2;
    }
    const std::string out_path = argv[1];

    std::vector<hailo_cache::Row> rows;
    std::string line;
    while (std::getline(std::cin, line)) {
        if (line.empty()) continue;
        try {
            hailo_cache::Row r;
            r.frame_idx = extract_int_(find_key_(line, "frame_idx"));
            r.crop_x    = static_cast<std::int32_t>(extract_int_(find_key_(line, "crop_x")));
            r.crop_y    = static_cast<std::int32_t>(extract_int_(find_key_(line, "crop_y")));
            r.crop_w    = static_cast<std::int32_t>(extract_int_(find_key_(line, "crop_w")));
            r.crop_h    = static_cast<std::int32_t>(extract_int_(find_key_(line, "crop_h")));
            r.ppv       = static_cast<std::int32_t>(extract_int_(find_key_(line, "ppv")));
            r.dets_json = extract_string_(find_key_(line, "dets_json"));
            r.ts_epoch  = extract_double_(find_key_(line, "ts_epoch"));
            rows.push_back(std::move(r));
        } catch (const std::exception& e) {
            std::fprintf(stderr, "parse error: %s\nline: %s\n", e.what(), line.c_str());
            return 3;
        }
    }

    try {
        hailo_cache::TileCacheDb db;
        db.open(out_path);
        db.put_many(rows);
        db.close();
    } catch (const std::exception& e) {
        std::fprintf(stderr, "TileCacheDb error: %s\n", e.what());
        return 4;
    }
    std::fprintf(stderr, "cache_db_cli: wrote %zu rows to %s\n",
                 rows.size(), out_path.c_str());
    return 0;
}
