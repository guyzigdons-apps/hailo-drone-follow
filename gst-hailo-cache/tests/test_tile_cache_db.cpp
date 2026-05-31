// gst-hailo-cache — unit tests for TileCacheDb.
//
// Plan 5 Task 2 + Task 6. Eight tests:
//   1. open-empty → schema persists; reopen idempotent
//   2. meta_put upsert (insert then update same key)
//   3. put_many is one transaction (duplicate PK → rollback the WHOLE batch)
//   4. get / get_many order preservation
//   5. mismatched user_version raises
//   6. open creates the frame_results table too (Task 6)
//   7. put_frame_results inserts + duplicate-PK rolls back the batch (Task 6)
//   8. detections + frame_results coexist in one DB (Task 6)

#include "tile_cache_db.hpp"

#include <gtest/gtest.h>
#include <sqlite3.h>

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

namespace fs = std::filesystem;

class TmpFile {
public:
    TmpFile() {
        auto dir = fs::temp_directory_path() / "gsthailocache_tests";
        fs::create_directories(dir);
        // Mix in pid + a counter for parallel-test safety.
        static int counter = 0;
        path_ = dir / ("cache_" + std::to_string(::getpid()) + "_" +
                       std::to_string(++counter) + ".sqlite3");
        // Ensure clean slate.
        std::error_code ec;
        fs::remove(path_, ec);
        // Also remove the WAL/SHM sidecars from a stale prior run.
        fs::remove(fs::path(path_).string() + "-wal", ec);
        fs::remove(fs::path(path_).string() + "-shm", ec);
    }
    ~TmpFile() {
        std::error_code ec;
        fs::remove(path_, ec);
        fs::remove(fs::path(path_).string() + "-wal", ec);
        fs::remove(fs::path(path_).string() + "-shm", ec);
    }
    const std::string& str() const { return path_str_cache_(); }
private:
    const std::string& path_str_cache_() const {
        if (cached_.empty()) cached_ = path_.string();
        return cached_;
    }
    fs::path             path_;
    mutable std::string  cached_;
};

hailo_cache::Row make_row(std::int64_t f, std::int32_t x, std::int32_t y,
                          std::int32_t w, std::int32_t h, std::int32_t ppv,
                          std::string dets = "[]", double ts = 1700000000.0) {
    hailo_cache::Row r;
    r.frame_idx = f;
    r.crop_x = x;
    r.crop_y = y;
    r.crop_w = w;
    r.crop_h = h;
    r.ppv = ppv;
    r.dets_json = std::move(dets);
    r.ts_epoch = ts;
    return r;
}

// Helper: read PRAGMA user_version directly via libsqlite3 (independent
// path from TileCacheDb so the assertions actually verify on-disk state).
int read_user_version(const std::string& path) {
    sqlite3* con = nullptr;
    int rc = sqlite3_open_v2(path.c_str(), &con, SQLITE_OPEN_READONLY, nullptr);
    if (rc != SQLITE_OK) { if (con) sqlite3_close(con); return -1; }
    sqlite3_stmt* st = nullptr;
    sqlite3_prepare_v2(con, "PRAGMA user_version", -1, &st, nullptr);
    int uv = -1;
    if (sqlite3_step(st) == SQLITE_ROW) uv = sqlite3_column_int(st, 0);
    sqlite3_finalize(st);
    sqlite3_close(con);
    return uv;
}

bool table_exists(const std::string& path, const char* name) {
    sqlite3* con = nullptr;
    if (sqlite3_open_v2(path.c_str(), &con, SQLITE_OPEN_READONLY, nullptr) != SQLITE_OK) {
        if (con) sqlite3_close(con);
        return false;
    }
    sqlite3_stmt* st = nullptr;
    sqlite3_prepare_v2(con,
        "SELECT name FROM sqlite_master WHERE type='table' AND name=?",
        -1, &st, nullptr);
    sqlite3_bind_text(st, 1, name, -1, SQLITE_TRANSIENT);
    bool found = sqlite3_step(st) == SQLITE_ROW;
    sqlite3_finalize(st);
    sqlite3_close(con);
    return found;
}

}  // namespace

// ---------------------------------------------------------------------------
// Test 1 — open-empty creates schema; reopen is idempotent.
// ---------------------------------------------------------------------------
TEST(TileCacheDb, OpenEmptyCreatesSchemaAndReopenIsIdempotent) {
    TmpFile tmp;

    {
        hailo_cache::TileCacheDb db;
        db.open(tmp.str());
        EXPECT_TRUE(db.is_open());
        db.close();
        EXPECT_FALSE(db.is_open());
    }

    EXPECT_EQ(read_user_version(tmp.str()), hailo_cache::kSchemaVersion);
    EXPECT_TRUE(table_exists(tmp.str(), "detections"));
    EXPECT_TRUE(table_exists(tmp.str(), "meta"));

    // Reopen — must succeed and not double-apply / reset anything.
    {
        hailo_cache::TileCacheDb db;
        db.open(tmp.str());
        // Insert a row, close, reopen and confirm the row's still there
        // (i.e. reopen didn't drop the table).
        db.put_many({make_row(0, 10, 20, 100, 200, 1, "[]")});
        db.close();
    }
    {
        hailo_cache::TileCacheDb db;
        db.open(tmp.str());
        auto r = db.get(0, 10, 20, 100, 200, 1);
        ASSERT_TRUE(r.has_value());
        EXPECT_EQ(r->dets_json, "[]");
        db.close();
    }
}

// ---------------------------------------------------------------------------
// Test 2 — meta_put upserts (insert then update same key).
// ---------------------------------------------------------------------------
TEST(TileCacheDb, MetaPutInsertsThenUpdates) {
    TmpFile tmp;
    hailo_cache::TileCacheDb db;
    db.open(tmp.str());

    // Insert.
    db.meta_put("video_sha", "abc123");
    auto v1 = db.meta_get("video_sha");
    ASSERT_TRUE(v1.has_value());
    EXPECT_EQ(*v1, "abc123");

    // Upsert with the SAME key — must update, not duplicate-PK fail.
    db.meta_put("video_sha", "def456");
    auto v2 = db.meta_get("video_sha");
    ASSERT_TRUE(v2.has_value());
    EXPECT_EQ(*v2, "def456");

    // Unknown key returns nullopt.
    EXPECT_FALSE(db.meta_get("does_not_exist").has_value());

    db.close();
}

// ---------------------------------------------------------------------------
// Test 3 — put_many is ONE transaction: a mid-batch failure rolls back
//          ALL inserts in the batch (matches Python
//          test_put_many_is_one_transaction).
//
// We trigger the failure by including two rows with the same primary
// key in a single batch — the second INSERT fails with SQLITE_CONSTRAINT.
// ---------------------------------------------------------------------------
TEST(TileCacheDb, PutManyIsOneTransaction) {
    TmpFile tmp;
    hailo_cache::TileCacheDb db;
    db.open(tmp.str());

    // Sanity: first put a valid row OUTSIDE the failing batch.
    db.put_many({make_row(42, 1, 2, 3, 4, 1, "[\"existing\"]")});
    ASSERT_TRUE(db.get(42, 1, 2, 3, 4, 1).has_value());

    // Now attempt a batch that contains a duplicate primary key
    // between rows[0] and rows[1]. rows[2] is a UNIQUE good row that
    // we want to confirm did NOT land — proving the rollback.
    std::vector<hailo_cache::Row> rows = {
        make_row(100, 0, 0, 640, 480, 1, "[\"a\"]"),
        make_row(100, 0, 0, 640, 480, 1, "[\"b\"]"),  // dup PK → fail
        make_row(101, 0, 0, 640, 480, 1, "[\"c\"]"),  // never reached
    };
    EXPECT_THROW(db.put_many(rows), std::runtime_error);

    // None of the batch's three rows should be visible. The
    // pre-existing row 42 must still be there.
    EXPECT_FALSE(db.get(100, 0, 0, 640, 480, 1).has_value());
    EXPECT_FALSE(db.get(101, 0, 0, 640, 480, 1).has_value());
    auto pre = db.get(42, 1, 2, 3, 4, 1);
    ASSERT_TRUE(pre.has_value());
    EXPECT_EQ(pre->dets_json, "[\"existing\"]");

    db.close();
}

// ---------------------------------------------------------------------------
// Test 4 — get / get_many preserve input order; nullopt on miss.
// ---------------------------------------------------------------------------
TEST(TileCacheDb, GetManyPreservesOrder) {
    TmpFile tmp;
    hailo_cache::TileCacheDb db;
    db.open(tmp.str());

    // Insert a few rows out of "request order".
    db.put_many({
        make_row(7, 100, 100, 200, 200, 1, "[\"alpha\"]"),
        make_row(7, 300, 300, 200, 200, 1, "[\"beta\"]"),
        make_row(7, 500, 500, 200, 200, 1, "[\"gamma\"]"),
    });

    // Build a lookup vector with deliberate (hit, miss, hit, miss, hit)
    // ordering — verifies position-preserving nullopts.
    std::vector<hailo_cache::TileCacheDb::CropKey> crops = {
        {500, 500, 200, 200},   // hit  → gamma
        {999, 999, 200, 200},   // miss
        {100, 100, 200, 200},   // hit  → alpha
        {  0,   0, 200, 200},   // miss
        {300, 300, 200, 200},   // hit  → beta
    };
    auto results = db.get_many(7, crops, 1);
    ASSERT_EQ(results.size(), crops.size());

    ASSERT_TRUE(results[0].has_value());
    EXPECT_EQ(results[0]->dets_json, "[\"gamma\"]");

    EXPECT_FALSE(results[1].has_value());

    ASSERT_TRUE(results[2].has_value());
    EXPECT_EQ(results[2]->dets_json, "[\"alpha\"]");

    EXPECT_FALSE(results[3].has_value());

    ASSERT_TRUE(results[4].has_value());
    EXPECT_EQ(results[4]->dets_json, "[\"beta\"]");

    // single-row get sanity-check
    auto g = db.get(7, 100, 100, 200, 200, 1);
    ASSERT_TRUE(g.has_value());
    EXPECT_EQ(g->dets_json, "[\"alpha\"]");
    EXPECT_EQ(g->frame_idx, 7);
    EXPECT_EQ(g->crop_x, 100);
    EXPECT_EQ(g->ppv, 1);

    db.close();
}

// ---------------------------------------------------------------------------
// Test 5 — mismatched user_version raises on open.
// ---------------------------------------------------------------------------
TEST(TileCacheDb, MismatchedUserVersionRaises) {
    TmpFile tmp;

    // Create a SQLite with PRAGMA user_version = 99 (bogus future schema).
    {
        sqlite3* con = nullptr;
        ASSERT_EQ(sqlite3_open_v2(tmp.str().c_str(), &con,
                                  SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE,
                                  nullptr),
                  SQLITE_OK);
        ASSERT_EQ(sqlite3_exec(con, "PRAGMA user_version = 99",
                               nullptr, nullptr, nullptr),
                  SQLITE_OK);
        sqlite3_close(con);
    }

    hailo_cache::TileCacheDb db;
    try {
        db.open(tmp.str());
        FAIL() << "Expected std::runtime_error for mismatched user_version";
    } catch (const std::runtime_error& e) {
        std::string msg = e.what();
        EXPECT_NE(msg.find("schema_version mismatch"), std::string::npos)
            << "msg = " << msg;
        EXPECT_NE(msg.find("file=99"), std::string::npos) << "msg = " << msg;
        EXPECT_NE(msg.find("expected=1"), std::string::npos) << "msg = " << msg;
    }

    // After failed open, the object must not be open.
    EXPECT_FALSE(db.is_open());
}

// ---------------------------------------------------------------------------
// Test 2b (Task 3) — provenance envelope meta keys round-trip.
//
// Locks the typed round-trip for the canonical envelope keys that the
// pixel-provenance writer (Plan: gst-cache-source-pixel-provenance, Task 4)
// stamps into `meta`: video_w/video_h (source pixels), resize_mode, dst_w/
// dst_h (network input dims), interpolation, hef_sha. See schema.sql for the
// shared key spec. meta_get/meta_put already exist — this just guards them.
// ---------------------------------------------------------------------------
TEST(TileCacheDbMeta, EnvelopeRoundTrip) {
    TmpFile tmp;
    hailo_cache::TileCacheDb db;
    db.open(tmp.str(), /*create_if_missing=*/true);

    db.meta_put("video_w", "3840");
    db.meta_put("video_h", "2160");
    db.meta_put("resize_mode", "stretch");
    db.meta_put("dst_w", "640");
    db.meta_put("dst_h", "640");
    db.meta_put("interpolation", "linear");
    db.meta_put("hef_sha", "deadbeef");

    EXPECT_EQ(db.meta_get("video_w").value(), "3840");
    EXPECT_EQ(db.meta_get("video_h").value(), "2160");
    EXPECT_EQ(db.meta_get("resize_mode").value(), "stretch");
    EXPECT_EQ(db.meta_get("dst_w").value(), "640");
    EXPECT_EQ(db.meta_get("dst_h").value(), "640");
    EXPECT_EQ(db.meta_get("interpolation").value(), "linear");
    EXPECT_EQ(db.meta_get("hef_sha").value(), "deadbeef");

    EXPECT_FALSE(db.meta_get("missing_key").has_value());

    db.close();
}

// ---------------------------------------------------------------------------
// Test 6 (Task 6) — opening a DB creates the frame_results table too.
// ---------------------------------------------------------------------------
TEST(TileCacheDb, OpenCreatesFrameResultsTable) {
    TmpFile tmp;
    {
        hailo_cache::TileCacheDb db;
        db.open(tmp.str());
        db.close();
    }
    // Both schemas (detections + meta + frame_results) are applied on
    // every open(); the file is one logical artifact whether the writer
    // ever populates frame_results or not.
    EXPECT_TRUE(table_exists(tmp.str(), "detections"));
    EXPECT_TRUE(table_exists(tmp.str(), "meta"));
    EXPECT_TRUE(table_exists(tmp.str(), "frame_results"));
}

// ---------------------------------------------------------------------------
// Test 7 (Task 6) — put_frame_results round-trip + duplicate-PK rollback.
// ---------------------------------------------------------------------------
TEST(TileCacheDb, PutFrameResultsRoundTripAndRollback) {
    TmpFile tmp;
    hailo_cache::TileCacheDb db;
    db.open(tmp.str());

    auto make_fr = [](std::int64_t f, std::int32_t ppv, std::string dets,
                      std::string tiles, double ts = 1700000000.0) {
        hailo_cache::FrameResultRow r;
        r.frame_idx  = f;
        r.ppv        = ppv;
        r.dets_json  = std::move(dets);
        r.tiles_json = std::move(tiles);
        r.ts_epoch   = ts;
        return r;
    };

    // Successful insert.
    db.put_frame_results({
        make_fr(0, 1, "[]",   "[]"),
        make_fr(1, 1, "[{\"cls\":0}]", "[{\"x\":0,\"y\":0,\"w\":640,\"h\":480,\"mode\":\"m\"}]"),
    });

    // Read back via raw SQLite to keep this test independent of any
    // future TileCacheDb::get_frame_results helper.
    sqlite3* con = nullptr;
    ASSERT_EQ(sqlite3_open_v2(tmp.str().c_str(), &con,
                              SQLITE_OPEN_READONLY, nullptr), SQLITE_OK);
    sqlite3_stmt* st = nullptr;
    ASSERT_EQ(sqlite3_prepare_v2(
        con,
        "SELECT frame_idx, ppv, dets_json, tiles_json, ts_epoch "
        "FROM frame_results ORDER BY frame_idx",
        -1, &st, nullptr), SQLITE_OK);
    int seen = 0;
    while (sqlite3_step(st) == SQLITE_ROW) {
        if (seen == 0) {
            EXPECT_EQ(sqlite3_column_int64(st, 0), 0);
            EXPECT_EQ(sqlite3_column_int  (st, 1), 1);
            EXPECT_STREQ(reinterpret_cast<const char*>(sqlite3_column_text(st, 2)), "[]");
            EXPECT_STREQ(reinterpret_cast<const char*>(sqlite3_column_text(st, 3)), "[]");
        } else if (seen == 1) {
            EXPECT_EQ(sqlite3_column_int64(st, 0), 1);
            EXPECT_STREQ(reinterpret_cast<const char*>(sqlite3_column_text(st, 2)),
                         "[{\"cls\":0}]");
        }
        ++seen;
    }
    EXPECT_EQ(seen, 2);
    sqlite3_finalize(st);
    sqlite3_close(con);

    // Now attempt a batch with a duplicate (frame_idx=0, ppv=1) — the
    // primary key — and one good row that should NOT land due to rollback.
    std::vector<hailo_cache::FrameResultRow> bad = {
        make_fr(0, 1, "[]", "[]"),         // dup PK → fail
        make_fr(2, 1, "[]", "[]"),         // would-be-good
    };
    EXPECT_THROW(db.put_frame_results(bad), std::runtime_error);

    // Confirm frame_idx=2 didn't slip through (rolled-back batch).
    ASSERT_EQ(sqlite3_open_v2(tmp.str().c_str(), &con,
                              SQLITE_OPEN_READONLY, nullptr), SQLITE_OK);
    ASSERT_EQ(sqlite3_prepare_v2(
        con, "SELECT COUNT(*) FROM frame_results WHERE frame_idx=2",
        -1, &st, nullptr), SQLITE_OK);
    ASSERT_EQ(sqlite3_step(st), SQLITE_ROW);
    EXPECT_EQ(sqlite3_column_int(st, 0), 0) << "frame_idx=2 row should not have landed";
    sqlite3_finalize(st);
    sqlite3_close(con);

    db.close();
}

// ---------------------------------------------------------------------------
// Test 8 (Task 6) — detections + frame_results live in the same DB.
// ---------------------------------------------------------------------------
TEST(TileCacheDb, DetectionsAndFrameResultsCoexist) {
    TmpFile tmp;
    hailo_cache::TileCacheDb db;
    db.open(tmp.str());

    db.put_many({make_row(0, 0, 0, 640, 480, 1, "[]")});

    hailo_cache::FrameResultRow fr;
    fr.frame_idx = 0;
    fr.ppv = 1;
    fr.dets_json = "[]";
    fr.tiles_json = "[]";
    fr.ts_epoch = 1700000000.0;
    db.put_frame_results({fr});

    // Round-trip the detections row.
    auto got = db.get(0, 0, 0, 640, 480, 1);
    ASSERT_TRUE(got.has_value());

    // Verify the frame_results row exists via raw sqlite.
    sqlite3* con = nullptr;
    ASSERT_EQ(sqlite3_open_v2(tmp.str().c_str(), &con,
                              SQLITE_OPEN_READONLY, nullptr), SQLITE_OK);
    sqlite3_stmt* st = nullptr;
    ASSERT_EQ(sqlite3_prepare_v2(
        con, "SELECT COUNT(*) FROM frame_results", -1, &st, nullptr),
        SQLITE_OK);
    ASSERT_EQ(sqlite3_step(st), SQLITE_ROW);
    EXPECT_EQ(sqlite3_column_int(st, 0), 1);
    sqlite3_finalize(st);
    sqlite3_close(con);

    db.close();
}
