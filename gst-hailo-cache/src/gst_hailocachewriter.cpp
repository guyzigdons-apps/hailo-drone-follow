// gst-hailo-cache — hailocachewriter element (impl).
//
// Plan 5, Task 4 — passthrough skeleton + spec §7.8 property surface.
//
// transform_ip is intentionally trivial in this task: it increments a
// buffer counter and logs at GST_LOG level. The SPSC ring buffer +
// background writer thread land in Task 5; the SQLite writes land
// alongside it. The point of this task is to ship the plumbing so
// pipelines compose and `gst-inspect-1.0` shows the full property
// surface before any real work happens on the streaming thread.

#include "gst_hailocachewriter.hpp"

#include <gst/gst.h>

GST_DEBUG_CATEGORY_STATIC(gst_hailocachewriter_debug);
#define GST_CAT_DEFAULT gst_hailocachewriter_debug

// Default values mirror spec §7.8 exactly. The "RECORD_EMPTY" default is
// TRUE — the spec is explicit that the no-opt-out invariant exists so
// that a missing row means "never inferred", and a row with dets='[]'
// means "looked, nothing found".
#define DEFAULT_MODE              GST_HAILOCACHEWRITER_MODE_TILE_CACHE
#define DEFAULT_FLUSH_INTERVAL_MS 100
#define DEFAULT_BATCH_SIZE        64
#define DEFAULT_FRAME_ID_SOURCE   GST_HAILOCACHEWRITER_FRAME_ID_COUNTER
#define DEFAULT_RECORD_EMPTY      TRUE
#define DEFAULT_RECORD_CACHE_HITS FALSE
#define DEFAULT_HEF_ID_META_KEY   "hailo-hef-sha"

enum {
    PROP_0,
    PROP_MODE,
    PROP_OUTPUT_FILE,
    PROP_FLUSH_INTERVAL_MS,
    PROP_BATCH_SIZE,
    PROP_FRAME_ID_SOURCE,
    PROP_RECORD_EMPTY,
    PROP_RECORD_CACHE_HITS,
    PROP_HEF_ID_META_KEY,
};

// -- GEnum type registrations -----------------------------------------------

static GType
gst_hailocachewriter_mode_get_type(void)
{
    static gsize id = 0;
    if (g_once_init_enter(&id)) {
        static const GEnumValue values[] = {
            { GST_HAILOCACHEWRITER_MODE_TILE_CACHE, "Per-crop tile-local detections (Section 7.2 schema)", "tile_cache" },
            { GST_HAILOCACHEWRITER_MODE_FULL_FRAME, "Per-frame source-coord detections + tiles_json",      "full_frame" },
            { 0, NULL, NULL },
        };
        GType t = g_enum_register_static("GstHailoCacheWriterMode", values);
        g_once_init_leave(&id, t);
    }
    return (GType)id;
}

static GType
gst_hailocachewriter_frame_id_source_get_type(void)
{
    static gsize id = 0;
    if (g_once_init_enter(&id)) {
        static const GEnumValue values[] = {
            { GST_HAILOCACHEWRITER_FRAME_ID_COUNTER, "Per-instance monotonic counter (starts at 0)", "counter" },
            { GST_HAILOCACHEWRITER_FRAME_ID_PTS,     "GST_BUFFER_PTS / GST_NSECOND",                 "pts"     },
            { 0, NULL, NULL },
        };
        GType t = g_enum_register_static("GstHailoCacheWriterFrameIdSource", values);
        g_once_init_leave(&id, t);
    }
    return (GType)id;
}

// -- Static pad templates ---------------------------------------------------
//
// Task 4: keep caps permissive — passthrough, no negotiation constraints
// beyond what GstBaseTransform requires. Task 5/6 may tighten this if
// the upstream tilecropper imposes specific raw video constraints, but
// for now ANY caps work.
static GstStaticPadTemplate sink_template =
    GST_STATIC_PAD_TEMPLATE("sink",
                            GST_PAD_SINK,
                            GST_PAD_ALWAYS,
                            GST_STATIC_CAPS_ANY);

static GstStaticPadTemplate src_template =
    GST_STATIC_PAD_TEMPLATE("src",
                            GST_PAD_SRC,
                            GST_PAD_ALWAYS,
                            GST_STATIC_CAPS_ANY);

// -- Prototypes -------------------------------------------------------------

static void         gst_hailocachewriter_set_property(GObject* object, guint property_id, const GValue* value, GParamSpec* pspec);
static void         gst_hailocachewriter_get_property(GObject* object, guint property_id, GValue* value, GParamSpec* pspec);
static void         gst_hailocachewriter_finalize    (GObject* object);
static GstFlowReturn gst_hailocachewriter_transform_ip(GstBaseTransform* trans, GstBuffer* buffer);

// -- Class boilerplate ------------------------------------------------------

G_DEFINE_TYPE_WITH_CODE(GstHailoCacheWriter, gst_hailocachewriter,
                        GST_TYPE_BASE_TRANSFORM,
                        GST_DEBUG_CATEGORY_INIT(gst_hailocachewriter_debug,
                                                "hailocachewriter", 0,
                                                "Hailo tile-cache writer element"));

static void
gst_hailocachewriter_class_init(GstHailoCacheWriterClass* klass)
{
    GObjectClass* gobject_class = G_OBJECT_CLASS(klass);
    GstBaseTransformClass* base_transform_class = GST_BASE_TRANSFORM_CLASS(klass);

    gst_element_class_add_static_pad_template(GST_ELEMENT_CLASS(klass), &sink_template);
    gst_element_class_add_static_pad_template(GST_ELEMENT_CLASS(klass), &src_template);

    gst_element_class_set_static_metadata(GST_ELEMENT_CLASS(klass),
                                          "Hailo cache writer",
                                          "Filter/Recorder",
                                          "Records per-crop or per-frame Hailo detections to a SQLite cache",
                                          "hailo.ai <contact@hailo.ai>");

    gobject_class->set_property = gst_hailocachewriter_set_property;
    gobject_class->get_property = gst_hailocachewriter_get_property;
    gobject_class->finalize     = gst_hailocachewriter_finalize;

    base_transform_class->transform_ip = GST_DEBUG_FUNCPTR(gst_hailocachewriter_transform_ip);

    g_object_class_install_property(gobject_class, PROP_MODE,
        g_param_spec_enum("mode", "mode",
                          "Output schema / pipeline placement (tile_cache | full_frame)",
                          gst_hailocachewriter_mode_get_type(),
                          DEFAULT_MODE,
                          (GParamFlags)(G_PARAM_READWRITE | GST_PARAM_MUTABLE_READY | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_OUTPUT_FILE,
        g_param_spec_string("output-file", "output-file",
                            "Path to the SQLite file to append to (or create). Required.",
                            NULL,
                            (GParamFlags)(G_PARAM_READWRITE | GST_PARAM_MUTABLE_READY | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_FLUSH_INTERVAL_MS,
        g_param_spec_uint("flush-interval-ms", "flush-interval-ms",
                          "Flush queued rows every N ms (background writer thread)",
                          0, G_MAXUINT, DEFAULT_FLUSH_INTERVAL_MS,
                          (GParamFlags)(G_PARAM_READWRITE | GST_PARAM_MUTABLE_READY | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_BATCH_SIZE,
        g_param_spec_uint("batch-size", "batch-size",
                          "Flush after N rows queued (whichever hits first vs flush-interval-ms)",
                          1, G_MAXUINT, DEFAULT_BATCH_SIZE,
                          (GParamFlags)(G_PARAM_READWRITE | GST_PARAM_MUTABLE_READY | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_FRAME_ID_SOURCE,
        g_param_spec_enum("frame-id-source", "frame-id-source",
                          "Frame indexing strategy (counter | pts)",
                          gst_hailocachewriter_frame_id_source_get_type(),
                          DEFAULT_FRAME_ID_SOURCE,
                          (GParamFlags)(G_PARAM_READWRITE | GST_PARAM_MUTABLE_READY | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_RECORD_EMPTY,
        g_param_spec_boolean("record-empty", "record-empty",
                             "Record rows even when detections list is empty (dets_json='[]'). "
                             "No opt-out planned — absence of a row means 'never inferred'.",
                             DEFAULT_RECORD_EMPTY,
                             (GParamFlags)(G_PARAM_READWRITE | GST_PARAM_MUTABLE_READY | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_RECORD_CACHE_HITS,
        g_param_spec_boolean("record-cache-hits", "record-cache-hits",
                             "Record buffers carrying hailo-cache-hit=true (tile_cache mode only; "
                             "ignored in full_frame).",
                             DEFAULT_RECORD_CACHE_HITS,
                             (GParamFlags)(G_PARAM_READWRITE | GST_PARAM_MUTABLE_READY | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_HEF_ID_META_KEY,
        g_param_spec_string("hef-id-meta-key", "hef-id-meta-key",
                            "Buffer-meta key the upstream pipeline sets; stored into the cache meta table.",
                            DEFAULT_HEF_ID_META_KEY,
                            (GParamFlags)(G_PARAM_READWRITE | GST_PARAM_MUTABLE_READY | G_PARAM_STATIC_STRINGS)));
}

static void
gst_hailocachewriter_init(GstHailoCacheWriter* self)
{
    self->mode              = DEFAULT_MODE;
    self->output_file       = NULL;
    self->flush_interval_ms = DEFAULT_FLUSH_INTERVAL_MS;
    self->batch_size        = DEFAULT_BATCH_SIZE;
    self->frame_id_source   = DEFAULT_FRAME_ID_SOURCE;
    self->record_empty      = DEFAULT_RECORD_EMPTY;
    self->record_cache_hits = DEFAULT_RECORD_CACHE_HITS;
    self->hef_id_meta_key   = g_strdup(DEFAULT_HEF_ID_META_KEY);
    self->buffer_count      = 0;

    // Task 4: passthrough on the data path. Task 5 may flip this off if
    // it needs to consult buffer metadata that GstBaseTransform skips
    // when passthrough=TRUE; the current plan is to keep it ON because
    // the writer is documented as a "passthrough recorder" (spec §7.8).
    gst_base_transform_set_passthrough(GST_BASE_TRANSFORM(self), TRUE);
}

static void
gst_hailocachewriter_set_property(GObject* object, guint property_id,
                                  const GValue* value, GParamSpec* pspec)
{
    GstHailoCacheWriter* self = GST_HAILOCACHEWRITER(object);

    switch (property_id) {
    case PROP_MODE:
        self->mode = (GstHailoCacheWriterMode)g_value_get_enum(value);
        break;
    case PROP_OUTPUT_FILE:
        g_free(self->output_file);
        self->output_file = g_value_dup_string(value);
        break;
    case PROP_FLUSH_INTERVAL_MS:
        self->flush_interval_ms = g_value_get_uint(value);
        break;
    case PROP_BATCH_SIZE:
        self->batch_size = g_value_get_uint(value);
        break;
    case PROP_FRAME_ID_SOURCE:
        self->frame_id_source = (GstHailoCacheWriterFrameIdSource)g_value_get_enum(value);
        break;
    case PROP_RECORD_EMPTY:
        self->record_empty = g_value_get_boolean(value);
        break;
    case PROP_RECORD_CACHE_HITS:
        self->record_cache_hits = g_value_get_boolean(value);
        break;
    case PROP_HEF_ID_META_KEY:
        g_free(self->hef_id_meta_key);
        self->hef_id_meta_key = g_value_dup_string(value);
        break;
    default:
        G_OBJECT_WARN_INVALID_PROPERTY_ID(object, property_id, pspec);
        break;
    }
}

static void
gst_hailocachewriter_get_property(GObject* object, guint property_id,
                                  GValue* value, GParamSpec* pspec)
{
    GstHailoCacheWriter* self = GST_HAILOCACHEWRITER(object);

    switch (property_id) {
    case PROP_MODE:
        g_value_set_enum(value, self->mode);
        break;
    case PROP_OUTPUT_FILE:
        g_value_set_string(value, self->output_file);
        break;
    case PROP_FLUSH_INTERVAL_MS:
        g_value_set_uint(value, self->flush_interval_ms);
        break;
    case PROP_BATCH_SIZE:
        g_value_set_uint(value, self->batch_size);
        break;
    case PROP_FRAME_ID_SOURCE:
        g_value_set_enum(value, self->frame_id_source);
        break;
    case PROP_RECORD_EMPTY:
        g_value_set_boolean(value, self->record_empty);
        break;
    case PROP_RECORD_CACHE_HITS:
        g_value_set_boolean(value, self->record_cache_hits);
        break;
    case PROP_HEF_ID_META_KEY:
        g_value_set_string(value, self->hef_id_meta_key);
        break;
    default:
        G_OBJECT_WARN_INVALID_PROPERTY_ID(object, property_id, pspec);
        break;
    }
}

static void
gst_hailocachewriter_finalize(GObject* object)
{
    GstHailoCacheWriter* self = GST_HAILOCACHEWRITER(object);
    g_free(self->output_file);
    g_free(self->hef_id_meta_key);
    G_OBJECT_CLASS(gst_hailocachewriter_parent_class)->finalize(object);
}

static GstFlowReturn
gst_hailocachewriter_transform_ip(GstBaseTransform* trans, GstBuffer* /*buffer*/)
{
    GstHailoCacheWriter* self = GST_HAILOCACHEWRITER(trans);

    // Task 4: just count. Task 5 will read frame id + crop list +
    // detections off the buffer and push to the SPSC ring.
    self->buffer_count++;
    GST_LOG_OBJECT(self,
        "passthrough buffer #%" G_GUINT64_FORMAT
        " (mode=%d output-file=%s batch-size=%u)",
        self->buffer_count,
        (int)self->mode,
        self->output_file ? self->output_file : "(unset)",
        self->batch_size);

    return GST_FLOW_OK;
}
