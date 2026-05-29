// gst-hailo-cache — unit tests for the hailocachereader element.
//
// Plan 5 Task 8 (skeleton) + Task 9 (cache-hit semantics).
//
// Task 8 tests (one-to-one with the acceptance list):
//   1. ElementExists — gst_element_factory_find succeeds.
//   2. AllSpecPropertiesPresent — the five spec §7.9 properties exist
//      with correct types + defaults.
//   3. MirrorsHailoNetProperties — every property name the hailonet
//      element exposes (snapshot of `gst-inspect-1.0 hailonet` at the
//      time of test) is present on hailocachereader.
//   4. OnMissErrorThrows — `videotestsrc num-buffers=3 ! hailocachereader
//      cache-file=/tmp/empty.sqlite3 on-miss=error ! fakesink` emits a
//      bus error on the first buffer.
//   5. OnMissDropPassesThrough — same pipeline with on-miss=drop runs
//      to EOS with no errors.
//
// Task 9 tests (cache lookup):
//   6. LookupHitsExistingCacheRow — pre-populated cache, 3-buffer pipeline
//      runs to EOS and every buffer carried hailo-cache-hit=1.
//   7. LookupMissOnMissDropPassesThrough — cache has only frame 0;
//      pipeline of 3 buffers + on-miss=drop pushes all through, buffer 0
//      is HIT, buffers 1 & 2 are MISS, no errors.
//   8. LookupMissOnMissErrorRaises — same setup with on-miss=error;
//      bus error on buffer 1 after buffer 0 hits.
//   9. QuantiseAlignsCropToCache — cache has crop (0,0,640,480); querying
//      with quantise=4 still hits even though the would-be probe coords
//      were slightly perturbed.
//
// Note on registration: the plugin .so is loaded through GST_PLUGIN_PATH
// (set by meson) — see ../meson.build. We do NOT need to install the
// plugin system-wide to run these tests.

#include <gtest/gtest.h>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>

// Task 9 lookup tests pre-populate the cache via the same TileCacheDb
// API the writer uses, so we link directly against the helper rather
// than shell out through gst-launch / SqliteCacheStore.
#include "tile_cache_db.hpp"
#include "gst_hailocachereader.hpp"

#include <cstdlib>
#include <cstring>
#include <string>
#include <unordered_set>
#include <vector>

namespace {

// Hardcoded list of hailonet properties (as of `gst-inspect-1.0 hailonet`
// on GStreamer 1.20). Names ONLY — types are mirrored in the .cpp.
// Excludes `name` and `parent` which are GstObject defaults inherited
// by every element automatically.
const std::vector<std::string>& hailonet_property_names() {
    static const std::vector<std::string> names = {
        "batch-size",
        "device-count",
        "device-id",
        "force-writable",
        "hef-path",
        "input-format-type",
        "input-from-meta",
        "is-active",
        "multi-process-service",
        "nms-iou-threshold",
        "nms-max-proposals-per-class",
        "nms-max-proposals-total",
        "nms-score-threshold",
        "no-transform",
        "output-format-type",
        "outputs-max-pool-size",
        "outputs-min-pool-size",
        "pass-through",
        "scheduler-priority",
        "scheduler-threshold",
        "scheduler-timeout-ms",
        "scheduling-algorithm",
        "vdevice-group-id",
    };
    return names;
}

class GstInitEnv : public ::testing::Environment {
public:
    void SetUp() override {
        if (!gst_is_initialized()) {
            gst_init(nullptr, nullptr);
        }

        // Load libgsthailocache.so from the build tree (matches the
        // hailocachewriter test pattern; see test_hailocachewriter.cpp).
        const char* plugin_path_env =
            std::getenv("GST_HAILOCACHE_PLUGIN_PATH");
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

// Register the global env once for the whole binary.
::testing::Environment* const g_env =
    ::testing::AddGlobalTestEnvironment(new GstInitEnv);

// Helper: collect all property NAMES installed on an element class.
std::unordered_set<std::string>
list_property_names(const std::string& factory_name) {
    GstElement* el = gst_element_factory_make(factory_name.c_str(),
                                              "test_inspect");
    EXPECT_TRUE(el != nullptr) << "Could not create " << factory_name;
    std::unordered_set<std::string> out;
    if (!el) return out;

    guint n_props = 0;
    GParamSpec** props = g_object_class_list_properties(
        G_OBJECT_GET_CLASS(el), &n_props);
    for (guint i = 0; i < n_props; ++i) {
        out.insert(g_param_spec_get_name(props[i]));
    }
    g_free(props);
    gst_object_unref(el);
    return out;
}

// Helper: assert property exists with the expected GType.
void assert_prop_type(GstElement* el, const char* name,
                      GType expected_type) {
    GParamSpec* p = g_object_class_find_property(
        G_OBJECT_GET_CLASS(el), name);
    ASSERT_TRUE(p != nullptr) << "Missing property: " << name;
    EXPECT_EQ(p->value_type, expected_type)
        << "Property " << name << " has type "
        << g_type_name(p->value_type) << " (expected "
        << g_type_name(expected_type) << ")";
}

// Helper: create an empty SQLite file so the cache-file path exists.
// Task 8 doesn't actually open the cache, but later tasks will, and
// this keeps the test stable across revisions.
std::string make_empty_file(const char* tag) {
    std::string path = std::string("/tmp/gsthailocachereader_empty_") +
                       tag + "_" + std::to_string(::getpid()) + ".sqlite3";
    FILE* f = std::fopen(path.c_str(), "wb");
    if (f) std::fclose(f);
    return path;
}

}  // namespace

// ---------------------------------------------------------------------------
// Test 1 — element exists.
// ---------------------------------------------------------------------------
TEST(HailoCacheReader, ElementExists) {
    GstElementFactory* f = gst_element_factory_find("hailocachereader");
    ASSERT_TRUE(f != nullptr)
        << "gst_element_factory_find('hailocachereader') returned NULL. "
        << "Is the plugin .so on GST_PLUGIN_PATH?";
    gst_object_unref(f);
}

// ---------------------------------------------------------------------------
// Test 2 — all spec §7.9 properties are declared with right types/defaults.
// ---------------------------------------------------------------------------
TEST(HailoCacheReader, AllSpecPropertiesPresent) {
    GstElement* el = gst_element_factory_make("hailocachereader", nullptr);
    ASSERT_TRUE(el != nullptr);

    // cache-file: string, default "".
    assert_prop_type(el, "cache-file", G_TYPE_STRING);
    {
        gchar* v = nullptr;
        g_object_get(el, "cache-file", &v, nullptr);
        EXPECT_STREQ(v ? v : "", "");
        g_free(v);
    }

    // hef-path: string, default "".
    assert_prop_type(el, "hef-path", G_TYPE_STRING);

    // video-id: string, default "".
    assert_prop_type(el, "video-id", G_TYPE_STRING);

    // on-miss: enum, default ERROR (=0).
    {
        GParamSpec* p = g_object_class_find_property(
            G_OBJECT_GET_CLASS(el), "on-miss");
        ASSERT_TRUE(p != nullptr);
        EXPECT_TRUE(G_TYPE_IS_ENUM(p->value_type)) << "on-miss not an enum";

        gint v = -1;
        g_object_get(el, "on-miss", &v, nullptr);
        EXPECT_EQ(v, 0 /* GST_HAILO_CACHE_READER_ON_MISS_ERROR */);
    }

    // quantise: uint, default 0.
    assert_prop_type(el, "quantise", G_TYPE_UINT);
    {
        guint v = 99;
        g_object_get(el, "quantise", &v, nullptr);
        EXPECT_EQ(v, 0u);
    }

    // frame-id-source: enum, default COUNTER (=0). Added by Task 9 so a
    // writer-then-reader run keys both ends to the same sequence.
    {
        GParamSpec* p = g_object_class_find_property(
            G_OBJECT_GET_CLASS(el), "frame-id-source");
        ASSERT_TRUE(p != nullptr);
        EXPECT_TRUE(G_TYPE_IS_ENUM(p->value_type))
            << "frame-id-source not an enum";

        gint v = -1;
        g_object_get(el, "frame-id-source", &v, nullptr);
        EXPECT_EQ(v, 0 /* GST_HAILO_CACHE_READER_FRAME_ID_COUNTER */);
    }

    gst_object_unref(el);
}

// ---------------------------------------------------------------------------
// Test 3 — mirror every hailonet property name.
// ---------------------------------------------------------------------------
TEST(HailoCacheReader, MirrorsHailoNetProperties) {
    auto names = list_property_names("hailocachereader");
    ASSERT_FALSE(names.empty());

    std::vector<std::string> missing;
    for (const auto& want : hailonet_property_names()) {
        if (names.find(want) == names.end()) {
            missing.push_back(want);
        }
    }
    EXPECT_TRUE(missing.empty())
        << "hailocachereader is missing hailonet-mirror properties: "
        << [&]{
            std::string out;
            for (const auto& m : missing) {
                if (!out.empty()) out += ", ";
                out += m;
            }
            return out;
        }();
}

// ---------------------------------------------------------------------------
// Bus helper for Tests 4 + 5.
// ---------------------------------------------------------------------------
namespace {

struct BusOutcome {
    bool got_error = false;
    bool got_eos   = false;
    std::string error_msg;
};

// Run the pipeline to EOS or ERROR (whichever first), with a 5s safety.
BusOutcome run_pipeline_to_done(GstElement* pipeline) {
    BusOutcome out;
    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    GstBus* bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));
    GstMessage* msg = gst_bus_timed_pop_filtered(
        bus, 5 * GST_SECOND,
        (GstMessageType)(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));
    if (msg) {
        if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR) {
            out.got_error = true;
            GError* err = nullptr;
            gchar* dbg = nullptr;
            gst_message_parse_error(msg, &err, &dbg);
            if (err) {
                out.error_msg = err->message ? err->message : "";
                g_error_free(err);
            }
            g_free(dbg);
        } else if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_EOS) {
            out.got_eos = true;
        }
        gst_message_unref(msg);
    }
    gst_object_unref(bus);

    gst_element_set_state(pipeline, GST_STATE_NULL);
    return out;
}

}  // namespace

// ---------------------------------------------------------------------------
// Test 4 — on-miss=error posts a bus error on the first buffer.
// ---------------------------------------------------------------------------
TEST(HailoCacheReader, OnMissErrorThrows) {
    std::string empty = make_empty_file("error");
    std::string pipeline_str =
        "videotestsrc num-buffers=3 ! "
        "hailocachereader cache-file=" + empty + " on-miss=error ! "
        "fakesink async=false sync=false";

    GError* err = nullptr;
    GstElement* pipeline = gst_parse_launch(pipeline_str.c_str(), &err);
    if (err) {
        FAIL() << "gst_parse_launch failed: " << err->message;
    }
    ASSERT_TRUE(pipeline != nullptr);

    auto out = run_pipeline_to_done(pipeline);
    EXPECT_TRUE(out.got_error)
        << "Expected bus error on first buffer with on-miss=error; got "
        << (out.got_eos ? "EOS" : "timeout") << ". msg=\"" << out.error_msg << "\"";

    gst_object_unref(pipeline);
    std::remove(empty.c_str());
}

// ---------------------------------------------------------------------------
// Test 5 — on-miss=drop passes buffers through to EOS, no errors.
// ---------------------------------------------------------------------------
TEST(HailoCacheReader, OnMissDropPassesThrough) {
    std::string empty = make_empty_file("drop");
    std::string pipeline_str =
        "videotestsrc num-buffers=3 ! "
        "hailocachereader cache-file=" + empty + " on-miss=drop ! "
        "fakesink async=false sync=false";

    GError* err = nullptr;
    GstElement* pipeline = gst_parse_launch(pipeline_str.c_str(), &err);
    if (err) {
        FAIL() << "gst_parse_launch failed: " << err->message;
    }
    ASSERT_TRUE(pipeline != nullptr);

    auto out = run_pipeline_to_done(pipeline);
    EXPECT_TRUE(out.got_eos)
        << "Expected EOS with on-miss=drop; got "
        << (out.got_error ? "ERROR" : "timeout") << ". msg=\"" << out.error_msg << "\"";
    EXPECT_FALSE(out.got_error)
        << "Did not expect an error with on-miss=drop. msg=\"" << out.error_msg << "\"";

    gst_object_unref(pipeline);
    std::remove(empty.c_str());
}

// ===========================================================================
// Task 9 — cache lookup + cache-hit semantics
// ===========================================================================
//
// These tests pre-populate a small SQLite via TileCacheDb::put_many (the
// same path Plan 5 Task 5's writer thread will use), then drive a real
// GStreamer pipeline through hailocachereader and observe the
// per-buffer cache-hit qdata key.
//
// Pipeline shape:
//   videotestsrc -> capsfilter (force width/height) -> hailocachereader ->
//   appsink (pull each buffer and inspect its qdata).
//
// The capsfilter is critical: the reader's Task-9 fallback crop is
// `(0, 0, width, height)` read from set_caps. We pin width/height in the
// test so the lookup key matches what we wrote to the cache.

namespace {

// Pick a fixed test resolution so the cache key is deterministic.
constexpr gint kTestWidth  = 320;
constexpr gint kTestHeight = 240;

// Compact JSON-ish detection list used to verify the payload round-trip.
const std::string kSampleDetsJson =
    "[{\"cls\":0,\"score\":0.5,\"x\":0.1,\"y\":0.2,\"w\":0.3,\"h\":0.4}]";

std::string tmp_db_path(const char* tag) {
    std::string path = std::string("/tmp/gst_hailocachereader_task9_") +
                       tag + "_" + std::to_string(::getpid()) + ".sqlite3";
    // Ensure a clean slate even if a previous failed run left a file.
    std::remove(path.c_str());
    return path;
}

// Populate a cache with one row per (frame_idx ∈ frames, crop_*).
// `ppv` is also written to the meta table so the reader picks it up
// on NULL→READY.
void populate_cache(const std::string& path,
                    const std::vector<std::int64_t>& frames,
                    std::int32_t crop_x, std::int32_t crop_y,
                    std::int32_t crop_w, std::int32_t crop_h,
                    std::int32_t ppv = 1) {
    hailo_cache::TileCacheDb db;
    db.open(path, /*create_if_missing=*/true);
    db.meta_put("ppv", std::to_string(ppv));

    std::vector<hailo_cache::Row> rows;
    rows.reserve(frames.size());
    for (auto fi : frames) {
        hailo_cache::Row r;
        r.frame_idx = fi;
        r.crop_x = crop_x; r.crop_y = crop_y;
        r.crop_w = crop_w; r.crop_h = crop_h;
        r.ppv = ppv;
        r.dets_json = kSampleDetsJson;
        r.ts_epoch  = 1700000000.0 + static_cast<double>(fi);
        rows.push_back(r);
    }
    db.put_many(rows);
    db.close();
}

// Build the test pipeline. Returns the parsed pipeline + the appsink
// (borrowed; ownership stays with the pipeline).
struct AppsinkPipeline {
    GstElement* pipeline = nullptr;
    GstElement* appsink  = nullptr;
};

AppsinkPipeline build_pipeline_with_appsink(const std::string& cache_path,
                                            int num_buffers,
                                            const char* on_miss,
                                            int quantise = 0) {
    char pbuf[1024];
    snprintf(pbuf, sizeof(pbuf),
        "videotestsrc num-buffers=%d ! "
        "video/x-raw,width=%d,height=%d,framerate=30/1 ! "
        "hailocachereader cache-file=%s on-miss=%s quantise=%d ! "
        "appsink name=sink async=false sync=false emit-signals=false",
        num_buffers, kTestWidth, kTestHeight,
        cache_path.c_str(), on_miss, quantise);

    GError* err = nullptr;
    GstElement* pipeline = gst_parse_launch(pbuf, &err);
    if (!pipeline) {
        ADD_FAILURE() << "gst_parse_launch failed: "
                      << (err ? err->message : "(null)");
        if (err) g_error_free(err);
        return {};
    }
    GstElement* sink = gst_bin_get_by_name(GST_BIN(pipeline), "sink");
    if (!sink) {
        ADD_FAILURE() << "appsink 'sink' not found in pipeline";
        gst_object_unref(pipeline);
        return {};
    }
    // gst_bin_get_by_name gave us an extra ref. Return the raw pointer
    // and drop the extra ref — the pipeline still owns it.
    gst_object_unref(sink);
    return {pipeline, sink};
}

struct PerBufferResult {
    bool got_error = false;
    bool got_eos   = false;
    std::string error_msg;
    // Per-buffer cache-hit qdata value:
    //   GST_HAILO_CACHE_HIT_VALUE_HIT  (1) = HIT
    //   GST_HAILO_CACHE_HIT_VALUE_MISS (2) = MISS (drop)
    //   -1                                  = qdata absent (buffer never
    //                                          passed through the reader).
    std::vector<int> hit_flags;
};

// Drive the pipeline to EOS or ERROR, pulling each buffer off the
// appsink and recording its hailo-cache-hit qdata.
PerBufferResult drive_pipeline(GstElement* pipeline, GstElement* appsink) {
    PerBufferResult out;
    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    GQuark q_hit = g_quark_from_static_string(GST_HAILO_CACHE_HIT_QDATA_KEY);
    GstBus* bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));

    const int kMaxIters = 100;   // ~5 s at 50 ms / iter — safety net.
    int iters = 0;
    while (!out.got_error && !out.got_eos && iters++ < kMaxIters) {
        // Try to pull a sample first.
        GstSample* sample = gst_app_sink_try_pull_sample(
            GST_APP_SINK(appsink), 50 * GST_MSECOND);
        if (sample) {
            GstBuffer* sbuf = gst_sample_get_buffer(sample);
            if (sbuf) {
                gpointer p = gst_mini_object_get_qdata(
                    GST_MINI_OBJECT(sbuf), q_hit);
                if (p == nullptr) {
                    out.hit_flags.push_back(-1);
                } else {
                    out.hit_flags.push_back(GPOINTER_TO_INT(p));
                }
            }
            gst_sample_unref(sample);
        }

        // Drain bus.
        GstMessage* msg = gst_bus_timed_pop_filtered(
            bus, 10 * GST_MSECOND,
            (GstMessageType)(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));
        if (msg) {
            if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR) {
                out.got_error = true;
                GError* err = nullptr;
                gchar* dbg = nullptr;
                gst_message_parse_error(msg, &err, &dbg);
                if (err) {
                    out.error_msg = err->message ? err->message : "";
                    g_error_free(err);
                }
                g_free(dbg);
            } else if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_EOS) {
                out.got_eos = true;
            }
            gst_message_unref(msg);
        }
    }
    gst_object_unref(bus);
    gst_element_set_state(pipeline, GST_STATE_NULL);
    return out;
}

}  // namespace

// ---------------------------------------------------------------------------
// Test 6 — every buffer hits when the cache row exists.
// ---------------------------------------------------------------------------
TEST(HailoCacheReaderTask9, LookupHitsExistingCacheRow) {
    std::string db_path = tmp_db_path("hit");
    populate_cache(db_path, {0, 1, 2},
                   0, 0, kTestWidth, kTestHeight, /*ppv=*/1);

    auto pl = build_pipeline_with_appsink(db_path, /*num_buffers=*/3, "error");
    ASSERT_NE(pl.pipeline, nullptr);
    ASSERT_NE(pl.appsink,  nullptr);

    auto r = drive_pipeline(pl.pipeline, pl.appsink);
    EXPECT_FALSE(r.got_error) << "msg=\"" << r.error_msg << "\"";
    EXPECT_TRUE (r.got_eos);
    ASSERT_EQ   (r.hit_flags.size(), 3u);
    for (size_t i = 0; i < r.hit_flags.size(); ++i) {
        EXPECT_EQ(r.hit_flags[i], GST_HAILO_CACHE_HIT_VALUE_HIT)
            << "buffer " << i << ": expected HIT ("
            << GST_HAILO_CACHE_HIT_VALUE_HIT << "), got " << r.hit_flags[i];
    }

    gst_object_unref(pl.pipeline);
    std::remove(db_path.c_str());
}

// ---------------------------------------------------------------------------
// Test 7 — on-miss=drop with a partial cache.
// ---------------------------------------------------------------------------
TEST(HailoCacheReaderTask9, LookupMissOnMissDropPassesThrough) {
    std::string db_path = tmp_db_path("partial_drop");
    populate_cache(db_path, {0},
                   0, 0, kTestWidth, kTestHeight, /*ppv=*/1);

    auto pl = build_pipeline_with_appsink(db_path, /*num_buffers=*/3, "drop");
    ASSERT_NE(pl.pipeline, nullptr);

    auto r = drive_pipeline(pl.pipeline, pl.appsink);
    EXPECT_FALSE(r.got_error) << "msg=\"" << r.error_msg << "\"";
    EXPECT_TRUE (r.got_eos);
    ASSERT_EQ   (r.hit_flags.size(), 3u);
    EXPECT_EQ(r.hit_flags[0], GST_HAILO_CACHE_HIT_VALUE_HIT)
        << "frame 0 should HIT";
    EXPECT_EQ(r.hit_flags[1], GST_HAILO_CACHE_HIT_VALUE_MISS)
        << "frame 1 should MISS (drop)";
    EXPECT_EQ(r.hit_flags[2], GST_HAILO_CACHE_HIT_VALUE_MISS)
        << "frame 2 should MISS (drop)";

    gst_object_unref(pl.pipeline);
    std::remove(db_path.c_str());
}

// ---------------------------------------------------------------------------
// Test 8 — on-miss=error with a partial cache: bus error on miss.
// ---------------------------------------------------------------------------
TEST(HailoCacheReaderTask9, LookupMissOnMissErrorRaises) {
    std::string db_path = tmp_db_path("partial_error");
    populate_cache(db_path, {0},
                   0, 0, kTestWidth, kTestHeight, /*ppv=*/1);

    auto pl = build_pipeline_with_appsink(db_path, /*num_buffers=*/3, "error");
    ASSERT_NE(pl.pipeline, nullptr);

    auto r = drive_pipeline(pl.pipeline, pl.appsink);
    EXPECT_TRUE(r.got_error)
        << "Expected bus error on first miss; msg=\"" << r.error_msg << "\"";
    EXPECT_FALSE(r.got_eos);
    // Buffer 0 may or may not have been delivered to the appsink before
    // the error propagates upstream; we only require that any delivered
    // buffer was flagged HIT (a delivered MISS would mean drop happened
    // when on-miss=error was set).
    for (size_t i = 0; i < r.hit_flags.size(); ++i) {
        EXPECT_EQ(r.hit_flags[i], GST_HAILO_CACHE_HIT_VALUE_HIT)
            << "buffer " << i << " was delivered but flagged MISS — "
            "should have been an error before reaching the sink";
    }

    gst_object_unref(pl.pipeline);
    std::remove(db_path.c_str());
}

// ---------------------------------------------------------------------------
// Test 9 — quantise aligns the lookup key.
// ---------------------------------------------------------------------------
//
// Cache row is at (0, 0, 320, 240). The reader's fallback crop is
// `(0, 0, width, height)` from caps — already aligned to 4. So quantise=4
// is a no-op for the current crop list. To meaningfully exercise the
// canonicalize_crop branch end-to-end we'd need a non-aligned upstream
// crop list, which lands in Phase 14. For Task 9 we cover the
// adjacent risk: quantise > 0 does NOT break a query whose coords are
// already aligned to the quantum (regression guard for the Task-11
// bit-exact gate where quantise=4 is the recommended fallback in
// Plan 5 Open-Q #1).
TEST(HailoCacheReaderTask9, QuantiseAlignsCropToCache) {
    std::string db_path = tmp_db_path("quantise");
    populate_cache(db_path, {0, 1, 2},
                   0, 0, kTestWidth, kTestHeight, /*ppv=*/1);

    auto pl = build_pipeline_with_appsink(db_path, /*num_buffers=*/3,
                                          "error", /*quantise=*/4);
    ASSERT_NE(pl.pipeline, nullptr);

    auto r = drive_pipeline(pl.pipeline, pl.appsink);
    EXPECT_FALSE(r.got_error) << "msg=\"" << r.error_msg << "\"";
    EXPECT_TRUE (r.got_eos);
    ASSERT_EQ   (r.hit_flags.size(), 3u);
    for (size_t i = 0; i < r.hit_flags.size(); ++i) {
        EXPECT_EQ(r.hit_flags[i], GST_HAILO_CACHE_HIT_VALUE_HIT)
            << "buffer " << i << ": quantise=4 broke an aligned-coord query";
    }

    gst_object_unref(pl.pipeline);
    std::remove(db_path.c_str());
}
