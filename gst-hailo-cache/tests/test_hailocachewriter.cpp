// gst-hailo-cache — gtest cases for the hailocachewriter element.
//
// Plan 5 Task 4. Verifies:
//   * The element class is registered after the plugin loads.
//   * Every spec §7.8 property is present with the right type & default.
//   * `passthrough` is TRUE on a freshly constructed instance.
//   * A minimal pipeline (videotestsrc ! hailocachewriter ! fakesink)
//     runs to EOS with no bus errors AND no file is created on disk
//     (writes land in Task 5).

#include <gst/gst.h>
#include <gst/base/gstbasetransform.h>
#include <gtest/gtest.h>

#include <sys/stat.h>
#include <unistd.h>

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

TEST(PipelineRunsToEos, NoErrorsNoFile) {
    // Use a path we'll explicitly remove first so we can assert "no file
    // created" reliably even if a previous failed run left a stale file.
    const std::string out = "/tmp/gst_hailocachewriter_task4.sqlite3";
    ::unlink(out.c_str());

    const std::string pipeline_str =
        std::string("videotestsrc num-buffers=10 ! ")
      + "hailocachewriter mode=tile_cache output-file=" + out + " ! "
      + "fakesink";

    GError* err = nullptr;
    GstElement* pipeline = gst_parse_launch(pipeline_str.c_str(), &err);
    ASSERT_NE(pipeline, nullptr)
        << "parse_launch failed: " << (err ? err->message : "(none)");
    if (err) { g_error_free(err); err = nullptr; }

    ASSERT_EQ(gst_element_set_state(pipeline, GST_STATE_PLAYING),
              GST_STATE_CHANGE_ASYNC);

    GstBus* bus = gst_element_get_bus(pipeline);
    ASSERT_NE(bus, nullptr);

    // Wait up to 5 s for EOS or ERROR.
    GstMessage* msg = gst_bus_timed_pop_filtered(
        bus, 5 * GST_SECOND,
        (GstMessageType)(GST_MESSAGE_EOS | GST_MESSAGE_ERROR));
    ASSERT_NE(msg, nullptr) << "Pipeline timed out (no EOS / ERROR)";
    EXPECT_EQ(GST_MESSAGE_TYPE(msg), GST_MESSAGE_EOS)
        << "Got error/non-EOS on bus";
    if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR) {
        GError* gerr = nullptr;
        gchar* dbg = nullptr;
        gst_message_parse_error(msg, &gerr, &dbg);
        ADD_FAILURE() << "Pipeline error: " << (gerr ? gerr->message : "?")
                      << " | debug: " << (dbg ? dbg : "");
        if (gerr) g_error_free(gerr);
        g_free(dbg);
    }
    gst_message_unref(msg);
    gst_object_unref(bus);

    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);

    // Task 4 contract: NO file is created. (Task 5 wires the SQLite
    // writer thread; until that lands, output-file is property-only.)
    EXPECT_FALSE(file_exists(out))
        << "Task 4 must NOT create the output file. Found: " << out;
    ::unlink(out.c_str());
}

}  // namespace
