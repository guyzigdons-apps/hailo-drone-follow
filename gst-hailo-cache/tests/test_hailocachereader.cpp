// gst-hailo-cache — unit tests for the hailocachereader element (Task 8).
//
// Five tests, one-to-one with the Plan 5 Task 8 acceptance list:
//
//   1. ElementExists — gst_element_factory_find succeeds.
//   2. AllSpecPropertiesPresent — the five spec §7.9 properties exist
//      with correct types + defaults.
//   3. MirrorsHailoNetProperties — every property name the hailonet
//      element exposes (snapshot of `gst-inspect-1.0 hailonet` at the
//      time of test) is present on hailocachereader. We hard-code the
//      list so the test doesn't depend on a runtime gst-inspect (and
//      so it documents the contract).
//   4. OnMissErrorThrows — `videotestsrc num-buffers=3 ! hailocachereader
//      cache-file=/tmp/empty.sqlite3 on-miss=error ! fakesink` emits a
//      bus error on the first buffer.
//   5. OnMissDropPassesThrough — same pipeline with on-miss=drop runs
//      to EOS with no errors.
//
// Note on registration: the plugin .so is loaded through GST_PLUGIN_PATH
// (set by meson) — see ../meson.build. We do NOT need to install the
// plugin system-wide to run these tests.

#include <gtest/gtest.h>

#include <gst/gst.h>

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
