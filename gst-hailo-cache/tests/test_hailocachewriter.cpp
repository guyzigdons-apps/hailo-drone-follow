// gst-hailo-cache — gtest cases for the hailocachewriter element.
//
// Plan 5 Tasks 4 + 5 + 6. Verifies:
//   * The element class is registered after the plugin loads.
//   * Every spec §7.8 property is present with the right type & default.
//   * `passthrough` is TRUE on a freshly constructed instance.
//   * Task 5: A minimal pipeline produces a SQLite file with one row per
//     buffer (single full-frame crop fallback).
//   * Task 5: `record-empty=false` skips empty rows.
//   * Task 5: `record-cache-hits` property is preserved (semantics
//     verified end-to-end in Task 9 once the reader registers the meta).
//   * Task 6: mode=full_frame produces a `frame_results` table with the
//     correct schema and one row per buffer.
//   * Task 6: tile_cache + full_frame writers coexist in one run (two
//     distinct output files, two distinct schemas).
//   * Task 6: record-cache-hits=true is silently ignored under
//     mode=full_frame (pipeline runs cleanly).

#include <gst/gst.h>
#include <gst/base/gstbasetransform.h>
#include <gtest/gtest.h>

#include <sqlite3.h>

#include <sys/stat.h>
#include <unistd.h>

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <string>

namespace {

class GstEnv : public ::testing::Environment {
public:
    void SetUp() override {
        gst_init(nullptr, nullptr);

        // Load the freshly built plugin from the build tree so the
        // test exercises exactly the .so we're shipping (not whatever
        // ships in the system gstreamer-1.0 plugins dir, which may be
        // stale or absent on a fresh checkout).
        //
        // Path relative to the test binary: tests/<exe>; plugin lives
        // in src/libgsthailocache.so under the same build root.
        const char* plugin_path_env = std::getenv("GST_HAILOCACHE_PLUGIN_PATH");
        std::string plugin_path = plugin_path_env
            ? plugin_path_env
            : std::string("../src/libgsthailocache.so");

        GError* err = nullptr;
        GstPlugin* plugin = gst_plugin_load_file(plugin_path.c_str(), &err);
        if (!plugin) {
            FAIL() << "Failed to load plugin at " << plugin_path
                   << ": " << (err ? err->message : "(unknown)");
        }
        gst_object_unref(plugin);
    }
};

static ::testing::Environment* const kGstEnv =
    ::testing::AddGlobalTestEnvironment(new GstEnv);

// -- Helpers ----------------------------------------------------------------

static bool file_exists(const std::string& path) {
    struct stat st;
    return ::stat(path.c_str(), &st) == 0;
}

static GParamSpec* find_pspec(GObjectClass* klass, const char* name) {
    return g_object_class_find_property(klass, name);
}

// Build a unique tmp DB path per (test tag, PID, monotonic counter) so
// parallel test runs (`meson test --repeat N` or `--num-processes >1`)
// don't collide on a shared `/tmp/...sqlite3` filename. Mirrors the
// pattern used in test_hailocachereader.cpp's `tmp_db_path`.
static std::string tmp_db_path(const char* tag) {
    static std::atomic<unsigned> counter{0};
    const unsigned n = counter.fetch_add(1, std::memory_order_relaxed);
    std::string path = std::string("/tmp/gst_hailocachewriter_task5_") +
                       tag + "_" + std::to_string(::getpid()) +
                       "_" + std::to_string(n) + ".sqlite3";
    // Ensure a clean slate even if a previous failed run left a file.
    ::unlink(path.c_str());
    ::unlink((path + "-wal").c_str());
    ::unlink((path + "-shm").c_str());
    return path;
}

// -- Tests ------------------------------------------------------------------

TEST(ElementExists, FactoryIsResolvable) {
    GstElementFactory* f = gst_element_factory_find("hailocachewriter");
    ASSERT_NE(f, nullptr) << "hailocachewriter factory not found";
    gst_object_unref(f);
}

TEST(AllPropertiesPresent, MatchesSpec) {
    GstElement* el = gst_element_factory_make("hailocachewriter", nullptr);
    ASSERT_NE(el, nullptr);
    GObjectClass* klass = G_OBJECT_GET_CLASS(el);

    // mode — enum, default = TILE_CACHE.
    {
        GParamSpec* pspec = find_pspec(klass, "mode");
        ASSERT_NE(pspec, nullptr);
        EXPECT_EQ(G_TYPE_FUNDAMENTAL(pspec->value_type), G_TYPE_ENUM);
        GValue v = G_VALUE_INIT;
        g_value_init(&v, pspec->value_type);
        g_object_get_property(G_OBJECT(el), "mode", &v);
        // 0 == GST_HAILOCACHEWRITER_MODE_TILE_CACHE.
        EXPECT_EQ(g_value_get_enum(&v), 0);
        g_value_unset(&v);
    }

    // output-file — string, default NULL.
    {
        GParamSpec* pspec = find_pspec(klass, "output-file");
        ASSERT_NE(pspec, nullptr);
        EXPECT_EQ(pspec->value_type, G_TYPE_STRING);
        gchar* s = nullptr;
        g_object_get(el, "output-file", &s, NULL);
        EXPECT_EQ(s, nullptr) << "output-file default should be unset (NULL)";
        g_free(s);
    }

    // flush-interval-ms — uint, default 100.
    {
        GParamSpec* pspec = find_pspec(klass, "flush-interval-ms");
        ASSERT_NE(pspec, nullptr);
        EXPECT_EQ(pspec->value_type, G_TYPE_UINT);
        guint v = 0;
        g_object_get(el, "flush-interval-ms", &v, NULL);
        EXPECT_EQ(v, 100u);
    }

    // batch-size — uint, default 64.
    {
        GParamSpec* pspec = find_pspec(klass, "batch-size");
        ASSERT_NE(pspec, nullptr);
        EXPECT_EQ(pspec->value_type, G_TYPE_UINT);
        guint v = 0;
        g_object_get(el, "batch-size", &v, NULL);
        EXPECT_EQ(v, 64u);
    }

    // frame-id-source — enum, default COUNTER (0).
    {
        GParamSpec* pspec = find_pspec(klass, "frame-id-source");
        ASSERT_NE(pspec, nullptr);
        EXPECT_EQ(G_TYPE_FUNDAMENTAL(pspec->value_type), G_TYPE_ENUM);
        GValue v = G_VALUE_INIT;
        g_value_init(&v, pspec->value_type);
        g_object_get_property(G_OBJECT(el), "frame-id-source", &v);
        EXPECT_EQ(g_value_get_enum(&v), 0);
        g_value_unset(&v);
    }

    // record-empty — bool, default TRUE.
    {
        GParamSpec* pspec = find_pspec(klass, "record-empty");
        ASSERT_NE(pspec, nullptr);
        EXPECT_EQ(pspec->value_type, G_TYPE_BOOLEAN);
        gboolean v = FALSE;
        g_object_get(el, "record-empty", &v, NULL);
        EXPECT_TRUE(v);
    }

    // record-cache-hits — bool, default FALSE.
    {
        GParamSpec* pspec = find_pspec(klass, "record-cache-hits");
        ASSERT_NE(pspec, nullptr);
        EXPECT_EQ(pspec->value_type, G_TYPE_BOOLEAN);
        gboolean v = TRUE;
        g_object_get(el, "record-cache-hits", &v, NULL);
        EXPECT_FALSE(v);
    }

    // hef-id-meta-key — string, default "hailo-hef-sha".
    {
        GParamSpec* pspec = find_pspec(klass, "hef-id-meta-key");
        ASSERT_NE(pspec, nullptr);
        EXPECT_EQ(pspec->value_type, G_TYPE_STRING);
        gchar* s = nullptr;
        g_object_get(el, "hef-id-meta-key", &s, NULL);
        ASSERT_NE(s, nullptr);
        EXPECT_STREQ(s, "hailo-hef-sha");
        g_free(s);
    }

    gst_object_unref(el);
}

TEST(PassthroughMode, IsTrueOnFreshElement) {
    GstElement* el = gst_element_factory_make("hailocachewriter", nullptr);
    ASSERT_NE(el, nullptr);
    EXPECT_TRUE(gst_base_transform_is_passthrough(GST_BASE_TRANSFORM(el)));
    gst_object_unref(el);
}

// -- Task 5 helpers ---------------------------------------------------------

// Run the pipeline string to EOS; fail the test on bus errors / timeouts.
// Returns true if the pipeline ran cleanly to EOS.
static bool run_pipeline_to_eos(const std::string& pipeline_str, int timeout_s = 10) {
    GError* err = nullptr;
    GstElement* pipeline = gst_parse_launch(pipeline_str.c_str(), &err);
    if (!pipeline) {
        ADD_FAILURE() << "parse_launch failed: " << (err ? err->message : "(none)");
        if (err) g_error_free(err);
        return false;
    }
    if (err) { g_error_free(err); err = nullptr; }

    GstStateChangeReturn sc = gst_element_set_state(pipeline, GST_STATE_PLAYING);
    if (sc == GST_STATE_CHANGE_FAILURE) {
        ADD_FAILURE() << "set_state(PLAYING) failed";
        gst_object_unref(pipeline);
        return false;
    }

    GstBus* bus = gst_element_get_bus(pipeline);
    GstMessage* msg = gst_bus_timed_pop_filtered(
        bus, timeout_s * GST_SECOND,
        (GstMessageType)(GST_MESSAGE_EOS | GST_MESSAGE_ERROR));

    bool ok = false;
    if (!msg) {
        ADD_FAILURE() << "Pipeline timed out (no EOS / ERROR)";
    } else if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_EOS) {
        ok = true;
    } else {
        GError* gerr = nullptr;
        gchar* dbg = nullptr;
        gst_message_parse_error(msg, &gerr, &dbg);
        ADD_FAILURE() << "Pipeline error: " << (gerr ? gerr->message : "?")
                      << " | debug: " << (dbg ? dbg : "");
        if (gerr) g_error_free(gerr);
        g_free(dbg);
    }
    if (msg) gst_message_unref(msg);
    gst_object_unref(bus);

    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
    return ok;
}

// Count rows in the `detections` table. Returns -1 on SQLite error.
static int count_detections(const std::string& path) {
    sqlite3* db = nullptr;
    if (sqlite3_open_v2(path.c_str(), &db, SQLITE_OPEN_READONLY, nullptr) != SQLITE_OK) {
        if (db) sqlite3_close(db);
        return -1;
    }
    sqlite3_stmt* stmt = nullptr;
    int n = -1;
    if (sqlite3_prepare_v2(db, "SELECT COUNT(*) FROM detections", -1, &stmt, nullptr) == SQLITE_OK) {
        if (sqlite3_step(stmt) == SQLITE_ROW) {
            n = sqlite3_column_int(stmt, 0);
        }
        sqlite3_finalize(stmt);
    }
    sqlite3_close(db);
    return n;
}

// -- Task 5 tests -----------------------------------------------------------

TEST(WriterCreatesFileOnEos, ProducesSqliteWithNRows) {
    const std::string out = tmp_db_path("create");

    const int N = 30;
    const std::string pipeline_str =
        std::string("videotestsrc num-buffers=") + std::to_string(N) + " ! "
      + "identity ! "
      + "hailocachewriter mode=tile_cache output-file=" + out + " ! "
      + "fakesink";

    ASSERT_TRUE(run_pipeline_to_eos(pipeline_str));

    // The file should exist after EOS (writer-thread synchronous drain).
    ASSERT_TRUE(file_exists(out)) << "Expected " << out << " to exist after EOS";

    const int rows = count_detections(out);
    EXPECT_GE(rows, N)
        << "Expected at least " << N << " rows; got " << rows;

    ::unlink(out.c_str());
    // Also clear the WAL/SHM sidecar files SQLite creates.
    ::unlink((out + "-wal").c_str());
    ::unlink((out + "-shm").c_str());
}

TEST(WriterRespectsRecordEmpty, NoRowsWhenEmptyAndDisabled) {
    // Task 5 emits dets_json="[]" for every frame (no detection
    // extraction yet). With record-empty=false, that means NO rows
    // should land.
    const std::string out = tmp_db_path("empty");

    const std::string pipeline_str =
        std::string("videotestsrc num-buffers=10 ! ")
      + "hailocachewriter mode=tile_cache record-empty=false "
      + "output-file=" + out + " ! fakesink";

    ASSERT_TRUE(run_pipeline_to_eos(pipeline_str));

    // The DB file is created (the writer opens it on start), but it
    // should contain zero rows.
    ASSERT_TRUE(file_exists(out));
    EXPECT_EQ(count_detections(out), 0)
        << "record-empty=false should drop dets_json='[]' rows";

    ::unlink(out.c_str());
    ::unlink((out + "-wal").c_str());
    ::unlink((out + "-shm").c_str());
}

TEST(WriterRespectsCacheHitsFlag, PropertyPersists) {
    // The semantic check (skipping buffers carrying `hailo-cache-hit`
    // meta) is end-to-end-verifiable once Task 9 registers that meta
    // type from the reader. For Task 5, we just confirm the property
    // surface accepts and persists the value.
    GstElement* el = gst_element_factory_make("hailocachewriter", nullptr);
    ASSERT_NE(el, nullptr);

    // Default is FALSE.
    gboolean v = TRUE;
    g_object_get(el, "record-cache-hits", &v, NULL);
    EXPECT_FALSE(v);

    // Set TRUE; read back.
    g_object_set(el, "record-cache-hits", TRUE, NULL);
    g_object_get(el, "record-cache-hits", &v, NULL);
    EXPECT_TRUE(v);

    // Set FALSE; read back.
    g_object_set(el, "record-cache-hits", FALSE, NULL);
    g_object_get(el, "record-cache-hits", &v, NULL);
    EXPECT_FALSE(v);

    gst_object_unref(el);
}

TEST(WriterExposesDroppedRows, ReadableProperty) {
    GstElement* el = gst_element_factory_make("hailocachewriter", nullptr);
    ASSERT_NE(el, nullptr);
    GObjectClass* klass = G_OBJECT_GET_CLASS(el);
    GParamSpec* pspec = find_pspec(klass, "dropped-rows");
    ASSERT_NE(pspec, nullptr);
    EXPECT_EQ(pspec->value_type, G_TYPE_UINT);
    // Read-only — flags should NOT include G_PARAM_WRITABLE.
    EXPECT_FALSE(pspec->flags & G_PARAM_WRITABLE);
    guint n = 999u;
    g_object_get(el, "dropped-rows", &n, NULL);
    EXPECT_EQ(n, 0u);
    gst_object_unref(el);
}

// -- Task 6 helpers ---------------------------------------------------------

// Count rows in the `frame_results` table. Returns -1 on SQLite error
// (e.g. the table doesn't exist).
static int count_frame_results(const std::string& path) {
    sqlite3* db = nullptr;
    if (sqlite3_open_v2(path.c_str(), &db, SQLITE_OPEN_READONLY, nullptr) != SQLITE_OK) {
        if (db) sqlite3_close(db);
        return -1;
    }
    sqlite3_stmt* stmt = nullptr;
    int n = -1;
    if (sqlite3_prepare_v2(db, "SELECT COUNT(*) FROM frame_results", -1, &stmt, nullptr) == SQLITE_OK) {
        if (sqlite3_step(stmt) == SQLITE_ROW) {
            n = sqlite3_column_int(stmt, 0);
        }
        sqlite3_finalize(stmt);
    }
    sqlite3_close(db);
    return n;
}

// Verify the `frame_results` table exists AND its columns match the
// spec §7.8 schema EXACTLY (5 columns, the documented names + SQLite
// types, no extras). Asserts pass-by-pass via gtest macros so any
// mismatch points to the offending column.
static void verify_frame_results_schema(const std::string& path) {
    sqlite3* db = nullptr;
    ASSERT_EQ(sqlite3_open_v2(path.c_str(), &db, SQLITE_OPEN_READONLY, nullptr),
              SQLITE_OK);

    // PRAGMA table_info(frame_results) returns one row per column:
    //   cid, name, type, notnull, dflt_value, pk
    sqlite3_stmt* stmt = nullptr;
    ASSERT_EQ(sqlite3_prepare_v2(db, "PRAGMA table_info(frame_results)",
                                 -1, &stmt, nullptr),
              SQLITE_OK);

    struct ColExpect {
        const char* name;
        const char* type;
        int         notnull;
        int         pk;   // primary-key ordinal (1-based) or 0
    };
    static const ColExpect kExpected[] = {
        // PRIMARY KEY (frame_idx, ppv) ordered → pk=1, pk=2.
        { "frame_idx",  "INTEGER", 1, 1 },
        { "ppv",        "INTEGER", 1, 2 },
        { "dets_json",  "TEXT",    1, 0 },
        { "tiles_json", "TEXT",    1, 0 },
        { "ts_epoch",   "REAL",    1, 0 },
    };
    constexpr int kExpectedColCount = sizeof(kExpected) / sizeof(kExpected[0]);

    int seen = 0;
    while (sqlite3_step(stmt) == SQLITE_ROW) {
        ASSERT_LT(seen, kExpectedColCount)
            << "frame_results has MORE columns than spec §7.8 documents";
        const ColExpect& want = kExpected[seen];
        const unsigned char* name = sqlite3_column_text(stmt, 1);
        const unsigned char* type = sqlite3_column_text(stmt, 2);
        int notnull = sqlite3_column_int(stmt, 3);
        int pk      = sqlite3_column_int(stmt, 5);
        EXPECT_STREQ(reinterpret_cast<const char*>(name), want.name)
            << "column " << seen << " name mismatch";
        EXPECT_STREQ(reinterpret_cast<const char*>(type), want.type)
            << "column " << seen << " (" << want.name << ") type mismatch";
        EXPECT_EQ(notnull, want.notnull)
            << "column " << seen << " (" << want.name << ") NOT NULL mismatch";
        EXPECT_EQ(pk, want.pk)
            << "column " << seen << " (" << want.name << ") PK ordinal mismatch";
        ++seen;
    }
    EXPECT_EQ(seen, kExpectedColCount)
        << "frame_results has FEWER columns than spec §7.8 documents";

    sqlite3_finalize(stmt);
    sqlite3_close(db);
}

// -- Task 6 tests -----------------------------------------------------------

TEST(FullFrameSchema, FrameResultsTableHasCorrectShape) {
    const std::string out = tmp_db_path("ff_schema");

    const int N = 10;
    const std::string pipeline_str =
        std::string("videotestsrc num-buffers=") + std::to_string(N) + " ! "
      + "identity ! "
      + "hailocachewriter mode=full_frame output-file=" + out + " ! "
      + "fakesink";

    ASSERT_TRUE(run_pipeline_to_eos(pipeline_str));

    ASSERT_TRUE(file_exists(out)) << "Expected " << out << " to exist after EOS";

    // The frame_results table must exist and match spec §7.8 byte-for-byte.
    verify_frame_results_schema(out);

    // And we should have ONE row per buffer (record-empty default = TRUE).
    EXPECT_EQ(count_frame_results(out), N)
        << "Expected " << N << " frame_results rows; got "
        << count_frame_results(out);

    // Sanity: the `detections` table is present (schema.sql + apply_schema_
    // create both tables on open) but should be EMPTY in full_frame mode
    // — we should NOT have leaked tile_cache rows into this file.
    EXPECT_EQ(count_detections(out), 0)
        << "full_frame writer should NOT populate the detections table";

    ::unlink(out.c_str());
    ::unlink((out + "-wal").c_str());
    ::unlink((out + "-shm").c_str());
}

TEST(FullFrameAndTileCacheCoexist, TwoFilesInOneRun) {
    const std::string out_tile  = tmp_db_path("coexist_tile");
    const std::string out_frame = tmp_db_path("coexist_frame");

    const int N = 7;
    // Chain a tile_cache writer feeding a full_frame writer in the same
    // pipeline (passthrough, so both see every buffer). This mirrors the
    // production pipeline placement from spec §7.8:
    //   ... ! hailocachewriter mode=tile_cache ! hailocachewriter mode=full_frame ! ...
    // (In production the hailodetiler element sits between them; for the
    // no-chip test we elide it — the writer doesn't read detection metadata
    // yet, so the placement is observationally identical.)
    const std::string pipeline_str =
        std::string("videotestsrc num-buffers=") + std::to_string(N) + " ! "
      + "hailocachewriter mode=tile_cache  output-file=" + out_tile  + " ! "
      + "hailocachewriter mode=full_frame  output-file=" + out_frame + " ! "
      + "fakesink";

    ASSERT_TRUE(run_pipeline_to_eos(pipeline_str));

    ASSERT_TRUE(file_exists(out_tile));
    ASSERT_TRUE(file_exists(out_frame));

    // Tile-cache file: detections rows present, frame_results empty.
    EXPECT_GE(count_detections(out_tile), N);
    EXPECT_EQ(count_frame_results(out_tile), 0);

    // Full-frame file: frame_results rows present, detections empty.
    EXPECT_EQ(count_frame_results(out_frame), N);
    EXPECT_EQ(count_detections(out_frame), 0);

    // Schema of frame_results in the full-frame file is correct.
    verify_frame_results_schema(out_frame);

    ::unlink(out_tile.c_str());
    ::unlink((out_tile + "-wal").c_str());
    ::unlink((out_tile + "-shm").c_str());
    ::unlink(out_frame.c_str());
    ::unlink((out_frame + "-wal").c_str());
    ::unlink((out_frame + "-shm").c_str());
}

TEST(RecordCacheHitsIgnoredInFullFrame, PipelineRunsCleanly) {
    // Spec §7.8: `record-cache-hits` is ignored in full_frame mode. The
    // writer logs a GST_INFO once at start, then accepts buffers as
    // normal. We verify the pipeline runs cleanly and produces the
    // expected row count.
    const std::string out = tmp_db_path("ff_recordhits");

    const int N = 5;
    const std::string pipeline_str =
        std::string("videotestsrc num-buffers=") + std::to_string(N) + " ! "
      + "hailocachewriter mode=full_frame record-cache-hits=true "
      + "output-file=" + out + " ! fakesink";

    ASSERT_TRUE(run_pipeline_to_eos(pipeline_str));

    ASSERT_TRUE(file_exists(out));
    EXPECT_EQ(count_frame_results(out), N)
        << "record-cache-hits=true must NOT change the row count under "
           "mode=full_frame (the flag is ignored, per spec §7.8)";

    ::unlink(out.c_str());
    ::unlink((out + "-wal").c_str());
    ::unlink((out + "-shm").c_str());
}

}  // namespace
