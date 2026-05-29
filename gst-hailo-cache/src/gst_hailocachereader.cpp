// gst-hailo-cache — hailocachereader element (Plan 5 Task 8 skeleton).
//
// See gst_hailocachereader.hpp for the contract. This file:
//
//   1. Registers all five spec §7.9 properties:
//        cache-file, hef-path, video-id, on-miss, quantise.
//   2. Mirrors every public property of `hailonet` (extracted from
//      `gst-inspect-1.0 hailonet` on GStreamer 1.20). Properties whose
//      meaning is hailonet-internal (batch-size, device-id, …) are
//      declared with matching name + type + default but are NO-OPS:
//      reading them returns the stored value (or the default), writing
//      stores the value without changing element behaviour. The point
//      is property-name/caps compatibility so a pipeline can do
//      `s/hailonet/hailocachereader/` with zero other edits.
//   3. Implements `transform_ip` with the Task-8 placeholder semantics:
//      - on-miss=error  → post GST_ERROR + EOS on the first buffer (we
//                         have no cache file open yet, so every buffer
//                         is by definition a miss).
//      - on-miss=drop   → pass through unmodified.
//      Task 9 swaps this for a real DB lookup.

#include "gst_hailocachereader.hpp"

#include <cstring>
#include <string>

GST_DEBUG_CATEGORY_STATIC(gst_hailo_cache_reader_debug);
#define GST_CAT_DEFAULT gst_hailo_cache_reader_debug

// ---------------------------------------------------------------------------
// Enum registrations
// ---------------------------------------------------------------------------

GType gst_hailo_cache_reader_on_miss_get_type(void) {
    static GType t = 0;
    if (t == 0) {
        static const GEnumValue values[] = {
            { GST_HAILO_CACHE_READER_ON_MISS_ERROR, "Post GST_ERROR + EOS", "error" },
            { GST_HAILO_CACHE_READER_ON_MISS_DROP,  "Drop detections (pass buffer through with empty results)", "drop"  },
            { 0, nullptr, nullptr }
        };
        t = g_enum_register_static("GstHailoCacheReaderOnMiss", values);
    }
    return t;
}

GType gst_hailo_cache_reader_format_type_get_type(void) {
    static GType t = 0;
    if (t == 0) {
        // Names match hailonet's GstHailoFormatType so external tooling
        // that inspects the enum names sees identical values.
        static const GEnumValue values[] = {
            { GST_HAILO_CACHE_READER_FORMAT_AUTO,    "auto",    "HAILO_FORMAT_TYPE_AUTO"    },
            { GST_HAILO_CACHE_READER_FORMAT_UINT8,   "uint8",   "HAILO_FORMAT_TYPE_UINT8"   },
            { GST_HAILO_CACHE_READER_FORMAT_UINT16,  "uint16",  "HAILO_FORMAT_TYPE_UINT16"  },
            { GST_HAILO_CACHE_READER_FORMAT_FLOAT32, "float32", "HAILO_FORMAT_TYPE_FLOAT32" },
            { 0, nullptr, nullptr }
        };
        t = g_enum_register_static("GstHailoCacheReaderFormatType", values);
    }
    return t;
}

GType gst_hailo_cache_reader_scheduling_get_type(void) {
    static GType t = 0;
    if (t == 0) {
        static const GEnumValue values[] = {
            { GST_HAILO_CACHE_READER_SCHED_NONE,        "Scheduler not active", "HAILO_SCHEDULING_ALGORITHM_NONE"        },
            { GST_HAILO_CACHE_READER_SCHED_ROUND_ROBIN, "Round robin",          "HAILO_SCHEDULING_ALGORITHM_ROUND_ROBIN" },
            { 0, nullptr, nullptr }
        };
        t = g_enum_register_static("GstHailoCacheReaderSchedulingAlgorithms", values);
    }
    return t;
}

// ---------------------------------------------------------------------------
// Property IDs
// ---------------------------------------------------------------------------

// Spec §7.9 properties — these are the real ones we'll wire up in Task 9.
// Hailonet-mirror properties follow; treated as no-op stubs for now.
enum {
    PROP_0 = 0,

    // ---- Spec §7.9 properties (REAL, behavior wired in Task 9) ----
    PROP_CACHE_FILE,
    PROP_HEF_PATH,
    PROP_VIDEO_ID,
    PROP_ON_MISS,
    PROP_QUANTISE,

    // ---- hailonet mirror (NO-OP stubs) ----
    // Listed in the order `gst-inspect-1.0 hailonet` reports them so a
    // future diff against a new hailonet release is mechanical.
    PROP_BATCH_SIZE,
    PROP_DEVICE_COUNT,
    PROP_DEVICE_ID,
    PROP_FORCE_WRITABLE,
    // PROP_HEF_PATH is already declared above (spec §7.9).
    PROP_INPUT_FORMAT_TYPE,
    PROP_INPUT_FROM_META,
    PROP_IS_ACTIVE,
    PROP_MULTI_PROCESS_SERVICE,
    PROP_NMS_IOU_THRESHOLD,
    PROP_NMS_MAX_PROPOSALS_PER_CLASS,
    PROP_NMS_MAX_PROPOSALS_TOTAL,
    PROP_NMS_SCORE_THRESHOLD,
    PROP_NO_TRANSFORM,
    PROP_OUTPUT_FORMAT_TYPE,
    PROP_OUTPUTS_MAX_POOL_SIZE,
    PROP_OUTPUTS_MIN_POOL_SIZE,
    PROP_PASS_THROUGH,
    PROP_SCHEDULER_PRIORITY,
    PROP_SCHEDULER_THRESHOLD,
    PROP_SCHEDULER_TIMEOUT_MS,
    PROP_SCHEDULING_ALGORITHM,
    PROP_VDEVICE_GROUP_ID,
};

// ---------------------------------------------------------------------------
// Instance struct
// ---------------------------------------------------------------------------

struct _GstHailoCacheReader {
    GstBaseTransform parent;

    // Spec §7.9 properties.
    gchar*    cache_file;    // required at PAUSED in Task 9
    gchar*    hef_path;
    gchar*    video_id;
    GstHailoCacheReaderOnMiss on_miss;
    guint     quantise;

    // hailonet-mirror property storage (no-op; we keep the value so
    // get_property returns what was last set).
    guint     m_batch_size;
    guint     m_device_count;
    gchar*    m_device_id;
    gboolean  m_force_writable;
    GstHailoCacheReaderFormatType m_input_format_type;
    gboolean  m_input_from_meta;
    gboolean  m_is_active;
    gboolean  m_multi_process_service;
    gfloat    m_nms_iou_threshold;
    guint     m_nms_max_proposals_per_class;
    guint     m_nms_max_proposals_total;
    gfloat    m_nms_score_threshold;
    gboolean  m_no_transform;
    GstHailoCacheReaderFormatType m_output_format_type;
    guint     m_outputs_max_pool_size;
    guint     m_outputs_min_pool_size;
    gboolean  m_pass_through;
    guint     m_scheduler_priority;
    guint     m_scheduler_threshold;
    guint     m_scheduler_timeout_ms;
    GstHailoCacheReaderScheduling m_scheduling_algorithm;
    gchar*    m_vdevice_group_id;

    // Internal state.
    gboolean  miss_error_posted;  // ensures we only push one error+EOS.
};

G_DEFINE_TYPE_WITH_CODE(
    GstHailoCacheReader,
    gst_hailo_cache_reader,
    GST_TYPE_BASE_TRANSFORM,
    GST_DEBUG_CATEGORY_INIT(gst_hailo_cache_reader_debug,
                            "hailocachereader", 0,
                            "Hailo cache replay reader (Task 8 skeleton)");
)

// ---------------------------------------------------------------------------
// Pad templates (Task 8: ANY/ANY; tighten in Task 9 per spec).
// ---------------------------------------------------------------------------

static GstStaticPadTemplate sink_template = GST_STATIC_PAD_TEMPLATE(
    "sink",
    GST_PAD_SINK,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS_ANY);

static GstStaticPadTemplate src_template = GST_STATIC_PAD_TEMPLATE(
    "src",
    GST_PAD_SRC,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS_ANY);

// ---------------------------------------------------------------------------
// Property accessors
// ---------------------------------------------------------------------------

static void
gst_hailo_cache_reader_set_property(GObject* object, guint prop_id,
                                    const GValue* value, GParamSpec* pspec)
{
    GstHailoCacheReader* self = GST_HAILO_CACHE_READER(object);
    switch (prop_id) {
        // ---- spec §7.9 ----
        case PROP_CACHE_FILE:
            g_free(self->cache_file);
            self->cache_file = g_value_dup_string(value);
            break;
        case PROP_HEF_PATH:
            g_free(self->hef_path);
            self->hef_path = g_value_dup_string(value);
            break;
        case PROP_VIDEO_ID:
            g_free(self->video_id);
            self->video_id = g_value_dup_string(value);
            break;
        case PROP_ON_MISS:
            self->on_miss = (GstHailoCacheReaderOnMiss)g_value_get_enum(value);
            break;
        case PROP_QUANTISE:
            self->quantise = g_value_get_uint(value);
            break;

        // ---- hailonet mirror (no-op storage) ----
        case PROP_BATCH_SIZE:
            self->m_batch_size = g_value_get_uint(value); break;
        case PROP_DEVICE_COUNT:
            self->m_device_count = g_value_get_uint(value); break;
        case PROP_DEVICE_ID:
            g_free(self->m_device_id);
            self->m_device_id = g_value_dup_string(value);
            break;
        case PROP_FORCE_WRITABLE:
            self->m_force_writable = g_value_get_boolean(value); break;
        case PROP_INPUT_FORMAT_TYPE:
            self->m_input_format_type = (GstHailoCacheReaderFormatType)g_value_get_enum(value);
            break;
        case PROP_INPUT_FROM_META:
            self->m_input_from_meta = g_value_get_boolean(value); break;
        case PROP_IS_ACTIVE:
            self->m_is_active = g_value_get_boolean(value); break;
        case PROP_MULTI_PROCESS_SERVICE:
            self->m_multi_process_service = g_value_get_boolean(value); break;
        case PROP_NMS_IOU_THRESHOLD:
            self->m_nms_iou_threshold = g_value_get_float(value); break;
        case PROP_NMS_MAX_PROPOSALS_PER_CLASS:
            self->m_nms_max_proposals_per_class = g_value_get_uint(value); break;
        case PROP_NMS_MAX_PROPOSALS_TOTAL:
            self->m_nms_max_proposals_total = g_value_get_uint(value); break;
        case PROP_NMS_SCORE_THRESHOLD:
            self->m_nms_score_threshold = g_value_get_float(value); break;
        case PROP_NO_TRANSFORM:
            self->m_no_transform = g_value_get_boolean(value); break;
        case PROP_OUTPUT_FORMAT_TYPE:
            self->m_output_format_type = (GstHailoCacheReaderFormatType)g_value_get_enum(value);
            break;
        case PROP_OUTPUTS_MAX_POOL_SIZE:
            self->m_outputs_max_pool_size = g_value_get_uint(value); break;
        case PROP_OUTPUTS_MIN_POOL_SIZE:
            self->m_outputs_min_pool_size = g_value_get_uint(value); break;
        case PROP_PASS_THROUGH:
            self->m_pass_through = g_value_get_boolean(value); break;
        case PROP_SCHEDULER_PRIORITY:
            self->m_scheduler_priority = g_value_get_uint(value); break;
        case PROP_SCHEDULER_THRESHOLD:
            self->m_scheduler_threshold = g_value_get_uint(value); break;
        case PROP_SCHEDULER_TIMEOUT_MS:
            self->m_scheduler_timeout_ms = g_value_get_uint(value); break;
        case PROP_SCHEDULING_ALGORITHM:
            self->m_scheduling_algorithm = (GstHailoCacheReaderScheduling)g_value_get_enum(value);
            break;
        case PROP_VDEVICE_GROUP_ID:
            g_free(self->m_vdevice_group_id);
            self->m_vdevice_group_id = g_value_dup_string(value);
            break;

        default:
            G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
            break;
    }
}

static void
gst_hailo_cache_reader_get_property(GObject* object, guint prop_id,
                                    GValue* value, GParamSpec* pspec)
{
    GstHailoCacheReader* self = GST_HAILO_CACHE_READER(object);
    switch (prop_id) {
        // ---- spec §7.9 ----
        case PROP_CACHE_FILE: g_value_set_string(value, self->cache_file); break;
        case PROP_HEF_PATH:   g_value_set_string(value, self->hef_path);   break;
        case PROP_VIDEO_ID:   g_value_set_string(value, self->video_id);   break;
        case PROP_ON_MISS:    g_value_set_enum  (value, self->on_miss);    break;
        case PROP_QUANTISE:   g_value_set_uint  (value, self->quantise);   break;

        // ---- hailonet mirror ----
        case PROP_BATCH_SIZE:                   g_value_set_uint   (value, self->m_batch_size); break;
        case PROP_DEVICE_COUNT:                 g_value_set_uint   (value, self->m_device_count); break;
        case PROP_DEVICE_ID:                    g_value_set_string (value, self->m_device_id); break;
        case PROP_FORCE_WRITABLE:               g_value_set_boolean(value, self->m_force_writable); break;
        case PROP_INPUT_FORMAT_TYPE:            g_value_set_enum   (value, self->m_input_format_type); break;
        case PROP_INPUT_FROM_META:              g_value_set_boolean(value, self->m_input_from_meta); break;
        case PROP_IS_ACTIVE:                    g_value_set_boolean(value, self->m_is_active); break;
        case PROP_MULTI_PROCESS_SERVICE:        g_value_set_boolean(value, self->m_multi_process_service); break;
        case PROP_NMS_IOU_THRESHOLD:            g_value_set_float  (value, self->m_nms_iou_threshold); break;
        case PROP_NMS_MAX_PROPOSALS_PER_CLASS:  g_value_set_uint   (value, self->m_nms_max_proposals_per_class); break;
        case PROP_NMS_MAX_PROPOSALS_TOTAL:      g_value_set_uint   (value, self->m_nms_max_proposals_total); break;
        case PROP_NMS_SCORE_THRESHOLD:          g_value_set_float  (value, self->m_nms_score_threshold); break;
        case PROP_NO_TRANSFORM:                 g_value_set_boolean(value, self->m_no_transform); break;
        case PROP_OUTPUT_FORMAT_TYPE:           g_value_set_enum   (value, self->m_output_format_type); break;
        case PROP_OUTPUTS_MAX_POOL_SIZE:        g_value_set_uint   (value, self->m_outputs_max_pool_size); break;
        case PROP_OUTPUTS_MIN_POOL_SIZE:        g_value_set_uint   (value, self->m_outputs_min_pool_size); break;
        case PROP_PASS_THROUGH:                 g_value_set_boolean(value, self->m_pass_through); break;
        case PROP_SCHEDULER_PRIORITY:           g_value_set_uint   (value, self->m_scheduler_priority); break;
        case PROP_SCHEDULER_THRESHOLD:          g_value_set_uint   (value, self->m_scheduler_threshold); break;
        case PROP_SCHEDULER_TIMEOUT_MS:         g_value_set_uint   (value, self->m_scheduler_timeout_ms); break;
        case PROP_SCHEDULING_ALGORITHM:         g_value_set_enum   (value, self->m_scheduling_algorithm); break;
        case PROP_VDEVICE_GROUP_ID:             g_value_set_string (value, self->m_vdevice_group_id); break;
        default:
            G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
            break;
    }
}

// ---------------------------------------------------------------------------
// finalize
// ---------------------------------------------------------------------------

static void
gst_hailo_cache_reader_finalize(GObject* object)
{
    GstHailoCacheReader* self = GST_HAILO_CACHE_READER(object);
    g_clear_pointer(&self->cache_file, g_free);
    g_clear_pointer(&self->hef_path,   g_free);
    g_clear_pointer(&self->video_id,   g_free);
    g_clear_pointer(&self->m_device_id,        g_free);
    g_clear_pointer(&self->m_vdevice_group_id, g_free);
    G_OBJECT_CLASS(gst_hailo_cache_reader_parent_class)->finalize(object);
}

// ---------------------------------------------------------------------------
// transform_ip — Task 8 placeholder
// ---------------------------------------------------------------------------

static GstFlowReturn
gst_hailo_cache_reader_transform_ip(GstBaseTransform* trans, GstBuffer* /*buf*/)
{
    GstHailoCacheReader* self = GST_HAILO_CACHE_READER(trans);

    // TODO(Task 9): real DB lookup. For Task 8 every buffer is a miss
    // because we haven't opened the cache.
    switch (self->on_miss) {
        case GST_HAILO_CACHE_READER_ON_MISS_ERROR: {
            if (!self->miss_error_posted) {
                self->miss_error_posted = TRUE;
                GST_ELEMENT_ERROR(self, RESOURCE, NOT_FOUND,
                    ("Cache miss with on-miss=error (Task 8 skeleton: no cache file opened)"),
                    ("hailocachereader Task 8 always misses; set on-miss=drop "
                     "to suppress this error. cache-file=\"%s\"",
                     self->cache_file ? self->cache_file : ""));
                // Post EOS so downstream stops cleanly.
                gst_pad_push_event(GST_BASE_TRANSFORM_SRC_PAD(trans),
                                   gst_event_new_eos());
            }
            return GST_FLOW_ERROR;
        }
        case GST_HAILO_CACHE_READER_ON_MISS_DROP:
        default:
            // Pass through unmodified.
            return GST_FLOW_OK;
    }
}

// ---------------------------------------------------------------------------
// class_init / init
// ---------------------------------------------------------------------------

static void
gst_hailo_cache_reader_class_init(GstHailoCacheReaderClass* klass)
{
    GObjectClass*          gobject_class = G_OBJECT_CLASS(klass);
    GstElementClass*       element_class = GST_ELEMENT_CLASS(klass);
    GstBaseTransformClass* trans_class   = GST_BASE_TRANSFORM_CLASS(klass);

    gobject_class->set_property = gst_hailo_cache_reader_set_property;
    gobject_class->get_property = gst_hailo_cache_reader_get_property;
    gobject_class->finalize     = gst_hailo_cache_reader_finalize;

    // --- Spec §7.9 properties (REAL) ---
    g_object_class_install_property(gobject_class, PROP_CACHE_FILE,
        g_param_spec_string("cache-file", "Cache file",
            "Path to the SQLite cache file (required)",
            "",
            (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_HEF_PATH,
        g_param_spec_string("hef-path", "HEF path",
            "Location of the HEF file to read (compat with hailonet; "
            "informational under hailocachereader)",
            "",
            (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_VIDEO_ID,
        g_param_spec_string("video-id", "Video id",
            "Logical video identifier (informational; used by upstream tools)",
            "",
            (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_ON_MISS,
        g_param_spec_enum("on-miss", "On miss",
            "Policy when a buffer's crop is not in the cache",
            GST_TYPE_HAILO_CACHE_READER_ON_MISS,
            GST_HAILO_CACHE_READER_ON_MISS_ERROR,
            (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_QUANTISE,
        g_param_spec_uint("quantise", "Quantise",
            "Round crop coordinates DOWN to multiples of this value before "
            "lookup; 0 means no quantisation",
            0, G_MAXUINT, 0,
            (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    // --- hailonet mirror (NO-OP stubs). Types + defaults match the
    //     hailonet `gst-inspect-1.0` output so naive copy-paste of a
    //     pipeline keeps working. ---

    g_object_class_install_property(gobject_class, PROP_BATCH_SIZE,
        g_param_spec_uint("batch-size", "Batch size",
            "(stub) How many frames to send in one batch — no-op in hailocachereader",
            0, 16, 0,
            (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_DEVICE_COUNT,
        g_param_spec_uint("device-count", "Device count",
            "(stub) Number of physical devices — no-op in hailocachereader",
            1, 65535, 1,
            (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_DEVICE_ID,
        g_param_spec_string("device-id", "Device id",
            "(stub) PCIe device id — no-op in hailocachereader",
            "",
            (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_FORCE_WRITABLE,
        g_param_spec_boolean("force-writable", "Force writable",
            "(stub) Force input buffer writable — no-op in hailocachereader",
            FALSE,
            (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_INPUT_FORMAT_TYPE,
        g_param_spec_enum("input-format-type", "Input format type",
            "(stub) Input format type — no-op in hailocachereader",
            GST_TYPE_HAILO_CACHE_READER_FORMAT_TYPE,
            GST_HAILO_CACHE_READER_FORMAT_AUTO,
            (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_INPUT_FROM_META,
        g_param_spec_boolean("input-from-meta", "Input from meta",
            "(stub) Take input from metadata — no-op in hailocachereader",
            FALSE,
            (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_IS_ACTIVE,
        g_param_spec_boolean("is-active", "Is active",
            "(stub) Whether the network is active — no-op in hailocachereader",
            FALSE,
            (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_MULTI_PROCESS_SERVICE,
        g_param_spec_boolean("multi-process-service", "Multi-process service",
            "(stub) Run HailoRT over its service — no-op in hailocachereader",
            FALSE,
            (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_NMS_IOU_THRESHOLD,
        g_param_spec_float("nms-iou-threshold", "NMS IoU threshold",
            "(stub) NMS IoU threshold — no-op in hailocachereader",
            0.0f, 1.0f, 0.0f,
            (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_NMS_MAX_PROPOSALS_PER_CLASS,
        g_param_spec_uint("nms-max-proposals-per-class", "NMS max proposals per class",
            "(stub) NMS max proposals per class — no-op in hailocachereader",
            0, G_MAXUINT, 0,
            (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_NMS_MAX_PROPOSALS_TOTAL,
        g_param_spec_uint("nms-max-proposals-total", "NMS max proposals total",
            "(stub) NMS max proposals total — no-op in hailocachereader",
            0, G_MAXUINT, 0,
            (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_NMS_SCORE_THRESHOLD,
        g_param_spec_float("nms-score-threshold", "NMS score threshold",
            "(stub) NMS score threshold — no-op in hailocachereader",
            0.0f, 1.0f, 0.0f,
            (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_NO_TRANSFORM,
        g_param_spec_boolean("no-transform", "No transform",
            "(stub) Keep HW format — no-op in hailocachereader",
            FALSE,
            (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_OUTPUT_FORMAT_TYPE,
        g_param_spec_enum("output-format-type", "Output format type",
            "(stub) Output format type — no-op in hailocachereader",
            GST_TYPE_HAILO_CACHE_READER_FORMAT_TYPE,
            GST_HAILO_CACHE_READER_FORMAT_AUTO,
            (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_OUTPUTS_MAX_POOL_SIZE,
        g_param_spec_uint("outputs-max-pool-size", "Outputs max pool size",
            "(stub) Outputs max pool size — no-op in hailocachereader",
            0, G_MAXUINT, 64,
            (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_OUTPUTS_MIN_POOL_SIZE,
        g_param_spec_uint("outputs-min-pool-size", "Outputs min pool size",
            "(stub) Outputs min pool size — no-op in hailocachereader",
            0, G_MAXUINT, 16,
            (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_PASS_THROUGH,
        g_param_spec_boolean("pass-through", "Pass through",
            "(stub) Pass buffers through without inference — no-op in hailocachereader",
            FALSE,
            (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_SCHEDULER_PRIORITY,
        g_param_spec_uint("scheduler-priority", "Scheduler priority",
            "(stub) Scheduler priority — no-op in hailocachereader",
            0, 31, 16,
            (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_SCHEDULER_THRESHOLD,
        g_param_spec_uint("scheduler-threshold", "Scheduler threshold",
            "(stub) Scheduler threshold — no-op in hailocachereader",
            0, G_MAXUINT, 0,
            (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_SCHEDULER_TIMEOUT_MS,
        g_param_spec_uint("scheduler-timeout-ms", "Scheduler timeout ms",
            "(stub) Scheduler timeout ms — no-op in hailocachereader",
            0, G_MAXUINT, 0,
            (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_SCHEDULING_ALGORITHM,
        g_param_spec_enum("scheduling-algorithm", "Scheduling algorithm",
            "(stub) Scheduling algorithm — no-op in hailocachereader",
            GST_TYPE_HAILO_CACHE_READER_SCHEDULING,
            GST_HAILO_CACHE_READER_SCHED_ROUND_ROBIN,
            (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_VDEVICE_GROUP_ID,
        g_param_spec_string("vdevice-group-id", "VDevice group id",
            "(stub) VDevice group id — no-op in hailocachereader",
            "",
            (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    // --- Pad templates + element metadata ---
    gst_element_class_add_static_pad_template(element_class, &src_template);
    gst_element_class_add_static_pad_template(element_class, &sink_template);

    gst_element_class_set_static_metadata(element_class,
        "Hailo cache reader",
        "Hailo/Cache",
        "Drop-in replacement for hailonet on the cache-replay path "
        "(Task 8 skeleton — no DB lookup yet)",
        "Hailo drone-follow contributors");

    // --- BaseTransform config ---
    trans_class->transform_ip = gst_hailo_cache_reader_transform_ip;
    // passthrough=FALSE per Task 8 spec (the element must own the
    // transform_ip callback so on-miss=error can post errors). The
    // passthrough flip lives in _init() because GstBaseTransform reads
    // it per-instance, not per-class.
    (void)trans_class;
}

static void
gst_hailo_cache_reader_init(GstHailoCacheReader* self)
{
    // Spec §7.9 defaults.
    self->cache_file = g_strdup("");
    self->hef_path   = g_strdup("");
    self->video_id   = g_strdup("");
    self->on_miss    = GST_HAILO_CACHE_READER_ON_MISS_ERROR;
    self->quantise   = 0;

    // hailonet-mirror defaults (match the values printed by
    // gst-inspect-1.0 hailonet on GStreamer 1.20).
    self->m_batch_size = 0;
    self->m_device_count = 1;
    self->m_device_id = g_strdup("");
    self->m_force_writable = FALSE;
    self->m_input_format_type = GST_HAILO_CACHE_READER_FORMAT_AUTO;
    self->m_input_from_meta = FALSE;
    self->m_is_active = FALSE;
    self->m_multi_process_service = FALSE;
    self->m_nms_iou_threshold = 0.0f;
    self->m_nms_max_proposals_per_class = 0;
    self->m_nms_max_proposals_total = 0;
    self->m_nms_score_threshold = 0.0f;
    self->m_no_transform = FALSE;
    self->m_output_format_type = GST_HAILO_CACHE_READER_FORMAT_AUTO;
    self->m_outputs_max_pool_size = 64;
    self->m_outputs_min_pool_size = 16;
    self->m_pass_through = FALSE;
    self->m_scheduler_priority = 16;
    self->m_scheduler_threshold = 0;
    self->m_scheduler_timeout_ms = 0;
    self->m_scheduling_algorithm = GST_HAILO_CACHE_READER_SCHED_ROUND_ROBIN;
    self->m_vdevice_group_id = g_strdup("");

    self->miss_error_posted = FALSE;

    gst_base_transform_set_passthrough(GST_BASE_TRANSFORM(self), FALSE);
    // We modify metadata in Task 9; for Task 8 we DO want transform_ip
    // to be called even though caps are ANY. set_in_place=TRUE keeps
    // BaseTransform from allocating a new output buffer.
    gst_base_transform_set_in_place(GST_BASE_TRANSFORM(self), TRUE);
}

// ---------------------------------------------------------------------------
// Plugin-init registration helper
// ---------------------------------------------------------------------------

gboolean
gst_hailo_cache_reader_plugin_init(GstPlugin* plugin)
{
    return gst_element_register(plugin, "hailocachereader",
                                GST_RANK_NONE,
                                GST_TYPE_HAILO_CACHE_READER);
}
