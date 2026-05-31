// gst-hailo-cache — unit tests for the hailocachebypass element.
//
// Plan 5 Task 12 — Phase 14 `hailofilter bypass-on-cache-hit` wrapper-first
// implementation. See gst_hailocachebypass.{hpp,cpp} for the contract.
//
// Tests:
//   1. ElementExists — gst_element_factory_find succeeds for hailocachebypass.
//   2. PassesHitBuffersThrough — buffers with hailo-cache-hit=HIT qdata flow
//      through the element with the qdata intact.
//   3. PassesMissBuffersThrough — same, but MISS qdata.
//   4. PassesCachedDetectionsThrough — the cached detection JSON payload
//      attached upstream by hailocachereader survives the trip through the
//      bypass element (this is the "we don't strip metadata" regression
//      guard mentioned in the cpp).
//   5. WarnsOnMissingQdata — buffer with no qdata passes through (no error,
//      reaches the sink, EOS arrives cleanly). The warning itself is
//      emitted via GST_WARNING and is observed at log level — we don't
//      install a custom log handler here; the absence of a bus error is
//      the contract.
//
// Pipeline shape for tests 2-5:
//     appsrc -> hailocachebypass -> appsink
//
// appsrc lets us hand-craft buffers with specific qdata set; appsink lets
// us pull each buffer at the far end and inspect its qdata after the trip.

#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gtest/gtest.h>

#include "gst_hailocachereader.hpp"  // for GST_HAILO_CACHE_HIT_QDATA_KEY / values

#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace {

class GstInitEnv : public ::testing::Environment {
public:
    void SetUp() override {
        if (!gst_is_initialized()) {
            gst_init(nullptr, nullptr);
        }

        // Same pattern as test_hailocachereader.cpp: load the freshly
        // built plugin from the build tree via env var.
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

::testing::Environment* const g_env =
    ::testing::AddGlobalTestEnvironment(new GstInitEnv);

// Build the `appsrc -> hailocachebypass -> appsink` pipeline. The caps
// on appsrc are deliberately ANY (the bypass element's pad templates
// are ANY too) so we can push trivially small buffers.
struct AppPipeline {
    GstElement* pipeline = nullptr;
    GstElement* appsrc   = nullptr;  // borrowed
    GstElement* appsink  = nullptr;  // borrowed
};

AppPipeline build_pipeline() {
    AppPipeline p;
    p.pipeline = gst_pipeline_new("bypass_test_pipeline");

    p.appsrc  = gst_element_factory_make("appsrc",            "src");
    GstElement* bypass = gst_element_factory_make("hailocachebypass", "bypass");
    p.appsink = gst_element_factory_make("appsink",           "sink");

    if (!p.appsrc || !bypass || !p.appsink) {
        if (p.appsrc)  gst_object_unref(p.appsrc);
        if (bypass)    gst_object_unref(bypass);
        if (p.appsink) gst_object_unref(p.appsink);
        if (p.pipeline) gst_object_unref(p.pipeline);
        return {};
    }

    g_object_set(p.appsrc,
                 "format",   GST_FORMAT_TIME,
                 "is-live",  FALSE,
                 nullptr);
    g_object_set(p.appsink,
                 "sync",          FALSE,
                 "async",         FALSE,
                 "emit-signals",  FALSE,
                 nullptr);

    gst_bin_add_many(GST_BIN(p.pipeline),
                     p.appsrc, bypass, p.appsink, nullptr);
    if (!gst_element_link_many(p.appsrc, bypass, p.appsink, nullptr)) {
        ADD_FAILURE() << "gst_element_link_many failed";
        gst_object_unref(p.pipeline);
        return {};
    }
    return p;
}

// Push a single small buffer through appsrc with the given qdata-hit
// value. `hit_value` >= 0 sets the qdata; < 0 leaves it unset.
// If `attach_dets_json` is non-empty, also attach the cached-detections
// qdata payload (string is g_strdup'd and freed by the buffer).
void push_buffer(GstElement* appsrc, int hit_value,
                 const std::string& attach_dets_json) {
    constexpr gsize kSize = 16;
    GstBuffer* buf = gst_buffer_new_allocate(nullptr, kSize, nullptr);
    GstMapInfo map;
    if (gst_buffer_map(buf, &map, GST_MAP_WRITE)) {
        std::memset(map.data, 0, map.size);
        gst_buffer_unmap(buf, &map);
    }

    if (hit_value >= 0) {
        GQuark q = g_quark_from_static_string(GST_HAILO_CACHE_HIT_QDATA_KEY);
        gst_mini_object_set_qdata(GST_MINI_OBJECT(buf), q,
                                  GINT_TO_POINTER(hit_value),
                                  /*destroy=*/nullptr);
    }
    if (!attach_dets_json.empty()) {
        GQuark q = g_quark_from_static_string(
            GST_HAILO_CACHED_DETECTIONS_QDATA_KEY);
        gchar* payload = g_strdup(attach_dets_json.c_str());
        gst_mini_object_set_qdata(GST_MINI_OBJECT(buf), q,
                                  payload,
                                  /*destroy=*/g_free);
    }

    GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(appsrc), buf);
    EXPECT_EQ(ret, GST_FLOW_OK) << "appsrc push_buffer returned " << ret;
}

struct PerBufferResult {
    bool got_error = false;
    bool got_eos   = false;
    std::string error_msg;
    // -1 = qdata absent; otherwise the qdata value seen at the sink.
    std::vector<int> hit_flags;
    // Per-buffer cached-detections JSON payload (empty if absent).
    std::vector<std::string> dets_payloads;
};

// Drive the pipeline to EOS, collecting per-buffer qdata at the appsink.
// `pushes` is the list of (hit_value, dets_json) tuples to send before
// signalling EOS. hit_value < 0 → leave qdata unset.
PerBufferResult run_pipeline(
    AppPipeline& p,
    const std::vector<std::pair<int, std::string>>& pushes)
{
    PerBufferResult out;

    GQuark q_hit = g_quark_from_static_string(GST_HAILO_CACHE_HIT_QDATA_KEY);
    GQuark q_det = g_quark_from_static_string(
        GST_HAILO_CACHED_DETECTIONS_QDATA_KEY);

    GstStateChangeReturn st = gst_element_set_state(p.pipeline,
                                                    GST_STATE_PLAYING);
    if (st == GST_STATE_CHANGE_FAILURE) {
        ADD_FAILURE() << "Failed to set pipeline to PLAYING";
        return out;
    }

    // Push all input buffers.
    for (const auto& kv : pushes) {
        push_buffer(p.appsrc, kv.first, kv.second);
    }
    gst_app_src_end_of_stream(GST_APP_SRC(p.appsrc));

    GstBus* bus = gst_pipeline_get_bus(GST_PIPELINE(p.pipeline));
    const int kMaxIters = 200;  // ~10 s safety net (50 ms per iter).
    int iters = 0;
    while (!out.got_error && !out.got_eos && iters++ < kMaxIters) {
        GstSample* sample = gst_app_sink_try_pull_sample(
            GST_APP_SINK(p.appsink), 50 * GST_MSECOND);
        if (sample) {
            GstBuffer* sbuf = gst_sample_get_buffer(sample);
            if (sbuf) {
                gpointer ph = gst_mini_object_get_qdata(
                    GST_MINI_OBJECT(sbuf), q_hit);
                out.hit_flags.push_back(
                    ph == nullptr ? -1 : GPOINTER_TO_INT(ph));
                gpointer pd = gst_mini_object_get_qdata(
                    GST_MINI_OBJECT(sbuf), q_det);
                out.dets_payloads.push_back(
                    pd == nullptr ? std::string()
                                  : std::string(static_cast<const char*>(pd)));
            }
            gst_sample_unref(sample);
        }
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
    gst_element_set_state(p.pipeline, GST_STATE_NULL);
    return out;
}

}  // namespace

// ---------------------------------------------------------------------------
// Test 1 — element exists.
// ---------------------------------------------------------------------------
TEST(HailoCacheBypass, ElementExists) {
    GstElementFactory* f = gst_element_factory_find("hailocachebypass");
    ASSERT_TRUE(f != nullptr)
        << "gst_element_factory_find('hailocachebypass') returned NULL. "
        << "Is the plugin .so on GST_PLUGIN_PATH? "
        << "Did Task 12 register the element in plugin.cpp?";
    gst_object_unref(f);
}

// ---------------------------------------------------------------------------
// Test 2 — HIT-tagged buffers pass through with qdata intact.
// ---------------------------------------------------------------------------
TEST(HailoCacheBypass, PassesHitBuffersThrough) {
    auto p = build_pipeline();
    ASSERT_NE(p.pipeline, nullptr);

    auto r = run_pipeline(p, {
        {GST_HAILO_CACHE_HIT_VALUE_HIT, ""},
        {GST_HAILO_CACHE_HIT_VALUE_HIT, ""},
        {GST_HAILO_CACHE_HIT_VALUE_HIT, ""},
    });
    EXPECT_FALSE(r.got_error) << "msg=\"" << r.error_msg << "\"";
    EXPECT_TRUE (r.got_eos);
    ASSERT_EQ   (r.hit_flags.size(), 3u);
    for (size_t i = 0; i < r.hit_flags.size(); ++i) {
        EXPECT_EQ(r.hit_flags[i], GST_HAILO_CACHE_HIT_VALUE_HIT)
            << "buffer " << i << ": expected HIT qdata at sink";
    }

    gst_object_unref(p.pipeline);
}

// ---------------------------------------------------------------------------
// Test 3 — MISS-tagged buffers pass through with qdata intact.
// ---------------------------------------------------------------------------
TEST(HailoCacheBypass, PassesMissBuffersThrough) {
    auto p = build_pipeline();
    ASSERT_NE(p.pipeline, nullptr);

    auto r = run_pipeline(p, {
        {GST_HAILO_CACHE_HIT_VALUE_MISS, ""},
        {GST_HAILO_CACHE_HIT_VALUE_MISS, ""},
    });
    EXPECT_FALSE(r.got_error) << "msg=\"" << r.error_msg << "\"";
    EXPECT_TRUE (r.got_eos);
    ASSERT_EQ   (r.hit_flags.size(), 2u);
    for (size_t i = 0; i < r.hit_flags.size(); ++i) {
        EXPECT_EQ(r.hit_flags[i], GST_HAILO_CACHE_HIT_VALUE_MISS)
            << "buffer " << i << ": expected MISS qdata at sink";
    }

    gst_object_unref(p.pipeline);
}

// ---------------------------------------------------------------------------
// Test 4 — cached detection JSON payload survives the bypass element.
// ---------------------------------------------------------------------------
TEST(HailoCacheBypass, PassesCachedDetectionsThrough) {
    auto p = build_pipeline();
    ASSERT_NE(p.pipeline, nullptr);

    const std::string kDets =
        "[{\"cls\":0,\"score\":0.7,\"x\":0.1,\"y\":0.2,\"w\":0.3,\"h\":0.4}]";

    auto r = run_pipeline(p, {
        {GST_HAILO_CACHE_HIT_VALUE_HIT, kDets},
    });
    EXPECT_FALSE(r.got_error) << "msg=\"" << r.error_msg << "\"";
    EXPECT_TRUE (r.got_eos);
    ASSERT_EQ   (r.hit_flags.size(),     1u);
    ASSERT_EQ   (r.dets_payloads.size(), 1u);
    EXPECT_EQ(r.hit_flags[0], GST_HAILO_CACHE_HIT_VALUE_HIT);
    EXPECT_EQ(r.dets_payloads[0], kDets)
        << "cached detection JSON was modified or stripped by hailocachebypass";

    gst_object_unref(p.pipeline);
}

// ---------------------------------------------------------------------------
// Test 5 — buffers without qdata still pass through (no bus error).
// ---------------------------------------------------------------------------
//
// The contract is: an element that's mis-wired (no upstream reader) is a
// configuration mistake but NOT a fatal error. We warn (via GST_WARNING)
// and forward. This test asserts the no-error behaviour; the warning
// itself is best verified by visually inspecting GST_DEBUG=hailocachebypass:3
// output — that's not testable cleanly without a custom log handler and
// would couple the test to the warning string verbatim.
TEST(HailoCacheBypass, WarnsOnMissingQdata) {
    auto p = build_pipeline();
    ASSERT_NE(p.pipeline, nullptr);

    auto r = run_pipeline(p, {
        {-1, ""},   // no qdata
        {-1, ""},   // no qdata — still no error (one-shot warn)
    });
    EXPECT_FALSE(r.got_error)
        << "Missing qdata should warn but not fail. msg=\""
        << r.error_msg << "\"";
    EXPECT_TRUE (r.got_eos);
    ASSERT_EQ   (r.hit_flags.size(), 2u);
    EXPECT_EQ(r.hit_flags[0], -1) << "qdata should be absent at sink too";
    EXPECT_EQ(r.hit_flags[1], -1);

    gst_object_unref(p.pipeline);
}
