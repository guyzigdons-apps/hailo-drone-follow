// gst-hailo-cache — C++ tile-cache DB helper.
//
// Plan 5, Task 2 (docs/superpowers/plans/2026-05-28-gst-cache-plugins.md).
//
// This is the C++ counterpart to hailo_tiling/cache/store.py
// (SqliteCacheStore). Both must produce byte-identical SQLite files —
// schema is taken LINE-FOR-LINE from hailo_tiling/cache/schema.sql.
//
// Lifetime / threading:
//   - One TileCacheDb instance owns one sqlite3* connection. SQLite
//     connections are not safe to share across threads; the caller is
//     expected to own the lifecycle (typically the GStreamer writer
//     thread for hailocachewriter).
//   - put_many() runs the whole batch in a single explicit
//     BEGIN/COMMIT transaction. On any error mid-batch it ROLLBACKs
//     and throws std::runtime_error.
//   - get/get_many do point lookups against the composite PRIMARY KEY.

#pragma once

#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

struct sqlite3;
struct sqlite3_stmt;

namespace hailo_cache {

// Schema version. Must match hailo_tiling/cache/schema.sql (PRAGMA user_version).
constexpr int kSchemaVersion = 1;

// One detection-cache row. Mirrors the `detections` table layout
// from hailo_tiling/cache/schema.sql.
struct Row {
    std::int64_t frame_idx{0};
    std::int32_t crop_x{0};
    std::int32_t crop_y{0};
    std::int32_t crop_w{0};
    std::int32_t crop_h{0};
    std::int32_t ppv{0};
    std::string  dets_json;
    double       ts_epoch{0.0};
};

// One full-frame result row (spec §7.8 / §7.13). Mirrors the
// `frame_results` table layout. Written by hailocachewriter
// mode=full_frame AFTER hailodetiler — `dets_json` carries the
// aggregated detections in source-frame normalized coords; `tiles_json`
// carries the tile layout used for the frame as `[{x,y,w,h,mode}, ...]`.
struct FrameResultRow {
    std::int64_t frame_idx{0};
    std::int32_t ppv{0};
    std::string  dets_json;
    std::string  tiles_json;
    double       ts_epoch{0.0};
};

class TileCacheDb {
public:
    TileCacheDb() = default;
    ~TileCacheDb();

    // Non-copyable, movable.
    TileCacheDb(const TileCacheDb&)            = delete;
    TileCacheDb& operator=(const TileCacheDb&) = delete;
    TileCacheDb(TileCacheDb&& other) noexcept;
    TileCacheDb& operator=(TileCacheDb&& other) noexcept;

    // Open (or create) the SQLite at `path`.
    //
    // Applies:
    //   PRAGMA journal_mode = WAL
    //   PRAGMA synchronous  = NORMAL
    //   PRAGMA user_version = kSchemaVersion (when creating)
    //
    // Schema (CREATE TABLE IF NOT EXISTS) is applied either on
    // create OR when the existing file has user_version == 0
    // (matches SqliteCacheStore.open semantics).
    //
    // Throws std::runtime_error on:
    //   - sqlite3_open_v2 failure
    //   - parent directory missing AND !create_if_missing
    //   - user_version mismatch (file != kSchemaVersion AND != 0)
    void open(const std::string& path, bool create_if_missing = true);

    // Close the underlying connection. Safe to call multiple times.
    // Best-effort: failures are silently swallowed (we're closing).
    void close();

    bool is_open() const { return con_ != nullptr; }

    // Read a value from the `meta` table. Returns nullopt on miss.
    std::optional<std::string> meta_get(const std::string& key);

    // Upsert (insert-or-update) into the `meta` table.
    // Throws std::runtime_error on SQLite error.
    void meta_put(const std::string& key, const std::string& value);

    // Insert `rows` in a single transaction.
    //
    // Empty input: no-op (matches Python). On any insert failure
    // (e.g. duplicate PRIMARY KEY), the transaction is rolled back
    // and std::runtime_error is thrown — same contract as
    // hailo_tiling.cache.store.SqliteCacheStore.put_many.
    void put_many(const std::vector<Row>& rows);

    // Insert `rows` into the `frame_results` table in a single
    // transaction. Plan 5 Task 6 (`hailocachewriter mode=full_frame`).
    //
    // Same empty-input + rollback-on-failure contract as `put_many`.
    // Both schemas (`detections` and `frame_results`) live in the same
    // .so / DB file, but a writer typically targets only ONE table per
    // output-file (see schema.sql commentary).
    void put_frame_results(const std::vector<FrameResultRow>& rows);

    // Single-row lookup. Returns nullopt on cache miss.
    std::optional<Row> get(std::int64_t frame_idx,
                           std::int32_t crop_x,
                           std::int32_t crop_y,
                           std::int32_t crop_w,
                           std::int32_t crop_h,
                           std::int32_t ppv);

    // Batched lookup. Order-preserving: result[i] corresponds to
    // input[i]; nullopt for cache misses. Same semantics as
    // SqliteCacheStore.get_many.
    //
    // The `crops` vector is interpreted as a flat list of
    // (crop_x, crop_y, crop_w, crop_h) tuples. Use the Row-shaped
    // overload below when convenient.
    struct CropKey {
        std::int32_t x;
        std::int32_t y;
        std::int32_t w;
        std::int32_t h;
    };
    std::vector<std::optional<Row>>
    get_many(std::int64_t frame_idx,
             const std::vector<CropKey>& crops,
             std::int32_t ppv);

    // Path the DB was opened with (informational; "" if not open).
    const std::string& path() const { return path_; }

private:
    void apply_schema_();
    void check_user_version_(bool was_new);
    void exec_(const char* sql);
    static std::runtime_error make_error_(sqlite3* con, const char* what);

    sqlite3*    con_{nullptr};
    std::string path_;
};

}  // namespace hailo_cache
