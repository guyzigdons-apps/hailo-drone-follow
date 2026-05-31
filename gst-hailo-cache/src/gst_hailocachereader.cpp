// gst-hailo-cache — hailocachereader element (Plan 5 Task 9).
//
// See gst_hailocachereader.hpp for the contract. This file:
//
//   1. Registers all five spec §7.9 properties:
//        cache-file, hef-path, video-id, on-miss, quantise.
//   2. Registers `frame-id-source` (Task 9 addition; mirrors the writer's
//      enum so a writer-then-reader run keys both ends to the same
//      sequence of frame_idx values).
//   3. Mirrors every public property of `hailonet` (extracted from
//      `gst-inspect-1.0 hailonet` on GStreamer 1.20). Properties whose
//      meaning is hailonet-internal (batch-size, device-id, …) are
//      declared with matching name + type + default but are NO-OPS:
//      reading them returns the stored value (or the default), writing
//      stores the value without changing element behaviour. The point
//      is property-name/caps compatibility so a pipeline can do
//      `s/hailonet/hailocachereader/` with zero other edits.
//   4. Implements `transform_ip` with the §7.9 cache-hit semantics:
//      - Open the cache lazily on NULL→READY (cache `meta` table `ppv`
//        is cached on first open).
//      - Per buffer: derive `frame_idx` per `frame-id-source`, take the
//        upstream crop list (fallback: single full-frame crop;
//        TODO Phase 14 — read GstHailoBaseCropperDyn provenance meta),
//        apply `quantise` via `cache_keys::canonicalize_crop`, then
//        `tile_cache_db::get` each crop.
//      - On HIT: attach cached detections as a JSON payload under
//        `GST_HAILO_CACHED_DETECTIONS_QDATA_KEY`, mark
//        `GST_HAILO_CACHE_HIT_QDATA_KEY` = 1. Emit ZERO HailoTensor metas.
//        (TODO Phase 14 — switch to native TAPPAS HailoROI / HailoDetection
//        once `hailo-apps-core` ships the public API path; see Plan 5
//        Open-Q #2.)
//      - On MISS, `on-miss=error`: post GST_ELEMENT_ERROR + return
//        GST_FLOW_ERROR (Task-8 fix at commit 0a2c763 — let
//        GstBaseTransform handle EOS; pushing an explicit EOS here races
//        the error handler).
//      - On MISS, `on-miss=drop`: mark `GST_HAILO_CACHE_HIT_QDATA_KEY` =
//        0, attach no detections, push buffer through.

#include "gst_hailocachereader.hpp"

#include "cache_keys.hpp"
#include "tile_cache_db.hpp"

// Crop provenance (Task 5): when built against TAPPAS core, read each
// tile-crop's HailoROI bbox and derive the SAME source-pixel crop key the
// writer (Task 4) recorded, so replay HITs. Guarded so the lib still
// builds on a no-Hailo box (the reader then keeps the full-frame caps
// fallback — mirrors the writer's HAVE_GSTHAILOMETA guard).
#if defined(HAVE_GSTHAILOMETA)
#include <gst_hailo_meta.hpp>   // get_hailo_main_roi
#include <hailo_objects.hpp>    // HailoROI, HailoBBox
#endif

#include <cstdint>
#include <cstring>
#include <exception>
#include <memory>
#include <string>
#include <vector>

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

GType gst_hailo_cache_reader_frame_id_source_get_type(void) {
    static GType t = 0;
    if (t == 0) {
        // Nicks match the writer's enum exactly so the same property
        // values can be passed verbatim on either element.
        static const GEnumValue values[] = {
            { GST_HAILO_CACHE_READER_FRAME_ID_COUNTER, "Per-instance monotonic counter (starts at 0)", "counter" },
            { GST_HAILO_CACHE_READER_FRAME_ID_PTS,     "GST_BUFFER_PTS / GST_NSECOND",                 "pts"     },
            { 0, nullptr, nullptr }
        };
        t = g_enum_register_static("GstHailoCacheReaderFrameIdSource", values);
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
    PROP_FRAME_ID_SOURCE,

    // ---- Source-pixel provenance (Task 5) ----
    // Mirror the writer's source-width/source-height so the reader derives
    // the SAME source-pixel crop key from each tile's HailoROI bbox.
    PROP_SOURCE_WIDTH,
    PROP_SOURCE_HEIGHT,

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
    GstHailoCacheReaderFrameIdSource frame_id_source;

    // Source-pixel provenance (Task 5). 0 = "not set" → fall back to the
    // negotiated caps dims (preserves the pre-Task-5 full-frame behaviour).
    guint     source_width;
    guint     source_height;

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
    gboolean  miss_error_posted;     // ensures we only post one error.
    std::unique_ptr<hailo_cache::TileCacheDb>* db;   // heap-allocated owner.
    std::int64_t frame_counter;      // counter_state for cache_keys.
    std::int32_t cached_ppv;         // value from `meta` table; -1 = unset.
    gint      cached_width;          // upstream caps width  (for fallback crop).
    gint      cached_height;         // upstream caps height (for fallback crop).
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
        case PROP_FRAME_ID_SOURCE:
            self->frame_id_source =
                (GstHailoCacheReaderFrameIdSource)g_value_get_enum(value);
            break;
        case PROP_SOURCE_WIDTH:
            self->source_width = g_value_get_uint(value);
            break;
        case PROP_SOURCE_HEIGHT:
            self->source_height = g_value_get_uint(value);
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
        case PROP_FRAME_ID_SOURCE:
            g_value_set_enum(value, self->frame_id_source);
            break;
        case PROP_SOURCE_WIDTH:  g_value_set_uint(value, self->source_width);  break;
        case PROP_SOURCE_HEIGHT: g_value_set_uint(value, self->source_height); break;

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

    // Tear down the cache handle if state-change cleanup didn't run
    // (e.g. element disposed before ever going to READY).
    if (self->db) {
        delete self->db;
        self->db = nullptr;
    }

    G_OBJECT_CLASS(gst_hailo_cache_reader_parent_class)->finalize(object);
}

// ---------------------------------------------------------------------------
// Cache lifecycle helpers
// ---------------------------------------------------------------------------

// Open the SQLite cache file referenced by `cache-file` and cache the
// `ppv` value from the meta table. Called on NULL→READY. Posts a bus
// error and returns FALSE on any failure.
static gboolean
gst_hailo_cache_reader_open_cache(GstHailoCacheReader* self)
{
    if (self->db && (*self->db)->is_open()) {
        return TRUE;
    }
    if (!self->cache_file || self->cache_file[0] == '\0') {
        GST_ELEMENT_ERROR(self, RESOURCE, NOT_FOUND,
            ("hailocachereader: cache-file property is unset"),
            ("Set the cache-file property to a SQLite cache produced by "
             "hailocachewriter (or SqliteCacheStore on the Python side)."));
        return FALSE;
    }

    if (!self->db) {
        self->db = new std::unique_ptr<hailo_cache::TileCacheDb>(
            new hailo_cache::TileCacheDb());
    }
    try {
        // create_if_missing=false: read path expects a populated cache.
        // Letting it create a fresh empty DB silently would mask the
        // "stale-cache or wrong-path" failure mode this element is
        // specifically designed to catch (cf. spec §7.9 "loud errors").
        (*self->db)->open(self->cache_file, /*create_if_missing=*/false);
    } catch (const std::exception& ex) {
        GST_ELEMENT_ERROR(self, RESOURCE, OPEN_READ,
            ("hailocachereader: failed to open cache-file '%s'",
             self->cache_file),
            ("%s", ex.what()));
        delete self->db;
        self->db = nullptr;
        return FALSE;
    }

    // Pull `ppv` from the cache's `meta` table; spec §7.9 says the
    // reader reads it from the meta table exclusively (see plan §
    // "Open questions / risks" #3). Fall back to 1 if absent — matches
    // Plan 5 Task 5's writer default for caches written before this
    // contract was finalised.
    try {
        auto v = (*self->db)->meta_get("ppv");
        if (v.has_value() && !v->empty()) {
            self->cached_ppv = static_cast<std::int32_t>(std::stol(*v));
        } else {
            self->cached_ppv = 1;
        }
    } catch (...) {
        self->cached_ppv = 1;
    }

    GST_INFO_OBJECT(self,
        "opened cache '%s' (ppv=%d, on-miss=%s, quantise=%u)",
        self->cache_file,
        (int)self->cached_ppv,
        self->on_miss == GST_HAILO_CACHE_READER_ON_MISS_ERROR ? "error" : "drop",
        self->quantise);

    return TRUE;
}

static void
gst_hailo_cache_reader_close_cache(GstHailoCacheReader* self)
{
    if (self->db) {
        if ((*self->db)->is_open()) {
            (*self->db)->close();
        }
        delete self->db;
        self->db = nullptr;
    }
    self->cached_ppv = -1;
    self->frame_counter = 0;
    self->miss_error_posted = FALSE;
}

// Sniff width/height from upstream caps; used to construct the
// fallback full-frame crop (TODO Phase 14 — read the real crop list
// from GstHailoBaseCropperDyn provenance metadata once the upstream
// pad probe lands).
static gboolean
gst_hailo_cache_reader_set_caps(GstBaseTransform* trans,
                                GstCaps* incaps,
                                GstCaps* /*outcaps*/)
{
    GstHailoCacheReader* self = GST_HAILO_CACHE_READER(trans);
    self->cached_width  = 0;
    self->cached_height = 0;

    if (!incaps) return TRUE;
    GstStructure* s = gst_caps_get_structure(incaps, 0);
    if (!s) return TRUE;
    gst_structure_get_int(s, "width",  &self->cached_width);
    gst_structure_get_int(s, "height", &self->cached_height);
    GST_DEBUG_OBJECT(self, "set_caps: width=%d height=%d",
                     self->cached_width, self->cached_height);
    return TRUE;
}

static GstStateChangeReturn
gst_hailo_cache_reader_change_state(GstElement* element,
                                    GstStateChange transition)
{
    GstHailoCacheReader* self = GST_HAILO_CACHE_READER(element);

    if (transition == GST_STATE_CHANGE_NULL_TO_READY) {
        if (!gst_hailo_cache_reader_open_cache(self)) {
            return GST_STATE_CHANGE_FAILURE;
        }
    }

    GstStateChangeReturn ret = GST_ELEMENT_CLASS(
        gst_hailo_cache_reader_parent_class)->change_state(element, transition);

    if (transition == GST_STATE_CHANGE_READY_TO_NULL) {
        gst_hailo_cache_reader_close_cache(self);
    }
    return ret;
}

// ---------------------------------------------------------------------------
// transform_ip — Task 9: real cache lookup
// ---------------------------------------------------------------------------

// Mark a buffer as a HIT or MISS. Uses gst_mini_object_set_qdata
// because GstBuffer is a GstMiniObject, not a GObject — g_object_set_data
// won't work. This is the Task-9 fallback for the proper GstMeta that
// Phase 14 will ship; see the TODO in the header.
//
// Encoding is HIT=1 / MISS=2 (not 1/0) because qdata stores a gpointer
// and `nullptr` is the "key absent" sentinel — GINT_TO_POINTER(0) would
// collide with that.
static void
mark_buffer_cache_hit(GstBuffer* buf, gboolean hit)
{
    GQuark q = g_quark_from_static_string(GST_HAILO_CACHE_HIT_QDATA_KEY);
    int v = hit ? GST_HAILO_CACHE_HIT_VALUE_HIT
                : GST_HAILO_CACHE_HIT_VALUE_MISS;
    gst_mini_object_set_qdata(GST_MINI_OBJECT(buf), q,
                              GINT_TO_POINTER(v),
                              /*destroy=*/nullptr);
}

// Attach the cached detection JSON payload to the buffer. Owned by the
// buffer; freed via g_free when the buffer is unreffed.
//
// TODO Phase 14 — switch to the native TAPPAS `HailoROI::add_object()` API
// (via `<hailo/tappas/hailo_objects.hpp>`) once we're comfortable taking
// on the TAPPAS ABI risk (Plan 5 Open-Q #2). Until then, downstream
// elements read the JSON via `gst_mini_object_get_qdata`.
static void
attach_cached_dets_json(GstBuffer* buf, const std::string& dets_json)
{
    GQuark q = g_quark_from_static_string(GST_HAILO_CACHED_DETECTIONS_QDATA_KEY);
    gchar* payload = g_strdup(dets_json.c_str());
    gst_mini_object_set_qdata(GST_MINI_OBJECT(buf), q, payload,
                              /*destroy=*/g_free);
}

static GstFlowReturn
gst_hailo_cache_reader_transform_ip(GstBaseTransform* trans, GstBuffer* buf)
{
    GstHailoCacheReader* self = GST_HAILO_CACHE_READER(trans);

    // The cache should already be open via change_state(NULL→READY);
    // re-open lazily as a defensive belt-and-braces for test paths that
    // skip the formal state machine.
    if (!self->db || !(*self->db)->is_open()) {
        if (!gst_hailo_cache_reader_open_cache(self)) {
            return GST_FLOW_ERROR;
        }
    }

    // (1) Frame id (counter or PTS — same rule as the writer; see
    //     cache_keys::frame_id_from_buffer in cache_keys.cpp).
    hailo_cache::FrameIdSource src =
        (self->frame_id_source == GST_HAILO_CACHE_READER_FRAME_ID_PTS)
            ? hailo_cache::FrameIdSource::PTS
            : hailo_cache::FrameIdSource::COUNTER;
    std::int64_t frame_idx =
        hailo_cache::frame_id_from_buffer(buf, src, self->frame_counter);
    if (frame_idx < 0) {
        GST_ELEMENT_ERROR(self, STREAM, FORMAT,
            ("hailocachereader: buffer has no valid PTS and frame-id-source=pts"),
            ("Switch frame-id-source to 'counter' or ensure upstream sets PTS."));
        return GST_FLOW_ERROR;
    }

    // (2) Crop list.
    //
    //     When the buffer carries a per-tile HailoROI (the cropped branch,
    //     after hailotilecropper_dynamic) AND source-width/height are set,
    //     derive the SAME source-pixel crop key the writer (Task 4)
    //     recorded: normalized bbox * source dims, using the shared
    //     truncate-then-clamp helper (cache_keys::tile_crop_to_source_px).
    //     This is what makes writer-then-reader replay HIT.
    //
    //     Otherwise fall back to a single full-frame crop using the
    //     negotiated caps dims — preserves the pre-Task-5 behaviour the
    //     existing reader tests (videotestsrc, no ROI) rely on.
    //
    // Conversion dims: source props win (so the lookup key lands in
    // source-video pixels, matching the writer); else fall back to caps —
    // byte-identical to the pre-Task-5 reader. (Same pick_dim rule as the
    // writer's transform_ip.)
    const std::int32_t conv_w =
        hailo_cache::pick_dim(self->source_width,  self->cached_width);
    const std::int32_t conv_h =
        hailo_cache::pick_dim(self->source_height, self->cached_height);

    std::int32_t fw = conv_w > 0 ? conv_w : 0;
    std::int32_t fh = conv_h > 0 ? conv_h : 0;
    std::vector<hailo_cache::TileCacheDb::CropKey> crops;

#if defined(HAVE_GSTHAILOMETA)
    bool used_tile_roi = false;
    if (conv_w > 0 && conv_h > 0) {
        HailoROIPtr roi = get_hailo_main_roi(buf, /*create_if_missing=*/false);
        if (roi) {
            HailoBBox bb = roi->get_bbox();
            const float xmin = bb.xmin();
            const float ymin = bb.ymin();
            const float w    = bb.width();
            const float h    = bb.height();
            // A whole-frame ROI (0,0,1,1) carries no per-tile provenance —
            // treat it like "no tile ROI" so behaviour matches the writer's
            // full-frame fallback for non-tiled pipelines.
            if (!(xmin == 0.0f && ymin == 0.0f && w == 1.0f && h == 1.0f)) {
                hailo_cache::CanonicalCrop r =
                    hailo_cache::tile_crop_to_source_px(
                        (double)xmin, (double)ymin, (double)w, (double)h,
                        conv_w, conv_h);
                crops.push_back({r.x, r.y, r.w, r.h});
                used_tile_roi = true;
            }
        }
    }
    if (!used_tile_roi) {
        crops.push_back({0, 0, fw, fh});
    }
#else
    crops.push_back({0, 0, fw, fh});
#endif

    // (3) Per-crop lookup.
    bool all_hit = true;
    // Accumulate detection JSON across crops. For the single-crop
    // fallback this is just the one row's payload; the multi-crop
    // path is wired in Phase 14 once we have a real upstream crop list.
    std::string accumulated_dets;

    for (const auto& c : crops) {
        // Quantise per spec §7.3 before lookup.
        hailo_cache::CanonicalCrop k{c.x, c.y, c.w, c.h};
        if (self->quantise > 0) {
            k = hailo_cache::canonicalize_crop(
                c.x, c.y, c.w, c.h, (int)self->quantise);
        }

        std::optional<hailo_cache::Row> row;
        try {
            row = (*self->db)->get(frame_idx, k.x, k.y, k.w, k.h,
                                   self->cached_ppv);
        } catch (const std::exception& ex) {
            GST_ELEMENT_ERROR(self, RESOURCE, READ,
                ("hailocachereader: lookup failed for frame_idx=%ld",
                 (long)frame_idx),
                ("%s", ex.what()));
            return GST_FLOW_ERROR;
        }

        if (!row.has_value()) {
            all_hit = false;
            GST_DEBUG_OBJECT(self,
                "MISS frame=%ld crop=(%d,%d,%d,%d) ppv=%d",
                (long)frame_idx, (int)k.x, (int)k.y, (int)k.w, (int)k.h,
                (int)self->cached_ppv);
            // We could break early on the first miss; collect all of
            // them in DEBUG so log review shows the full pattern.
            break;
        }

        GST_LOG_OBJECT(self,
            "HIT frame=%ld crop=(%d,%d,%d,%d) ppv=%d dets_json.size=%zu",
            (long)frame_idx, (int)k.x, (int)k.y, (int)k.w, (int)k.h,
            (int)self->cached_ppv, row->dets_json.size());

        // Single-crop fallback: just use the only payload. Multi-crop
        // accumulation lands with the real upstream crop list.
        accumulated_dets = row->dets_json;
    }

    if (all_hit) {
        mark_buffer_cache_hit(buf, TRUE);
        if (!accumulated_dets.empty()) {
            attach_cached_dets_json(buf, accumulated_dets);
        }
        return GST_FLOW_OK;
    }

    // ---- MISS path ----
    switch (self->on_miss) {
        case GST_HAILO_CACHE_READER_ON_MISS_ERROR: {
            if (!self->miss_error_posted) {
                self->miss_error_posted = TRUE;
                GST_ELEMENT_ERROR(self, RESOURCE, NOT_FOUND,
                    ("hailocachereader: cache miss with on-miss=error"),
                    ("frame_idx=%ld not in cache '%s'. "
                     "Re-warm the cache or set on-miss=drop.",
                     (long)frame_idx,
                     self->cache_file ? self->cache_file : ""));
            }
            return GST_FLOW_ERROR;
        }
        case GST_HAILO_CACHE_READER_ON_MISS_DROP:
        default:
            mark_buffer_cache_hit(buf, FALSE);
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

    // Task 9 addition: mirror the writer's frame-id-source enum so a
    // writer-then-reader run keys both ends to the same sequence.
    g_object_class_install_property(gobject_class, PROP_FRAME_ID_SOURCE,
        g_param_spec_enum("frame-id-source", "frame-id-source",
            "Frame indexing strategy (counter | pts) — must match the writer "
            "that produced the cache",
            GST_TYPE_HAILO_CACHE_READER_FRAME_ID_SOURCE,
            GST_HAILO_CACHE_READER_FRAME_ID_COUNTER,
            (GParamFlags)(G_PARAM_READWRITE | GST_PARAM_MUTABLE_READY | G_PARAM_STATIC_STRINGS)));

    // --- Source-pixel provenance (Task 5). Mirror the writer's
    //     source-width/source-height so writer-then-reader replay HITs:
    //     the reader scales each tile's normalized HailoROI bbox by these
    //     dims (when >0) to derive the SAME source-pixel crop key the
    //     writer recorded. 0 = fall back to caps dims (full-frame). ---
    g_object_class_install_property(gobject_class, PROP_SOURCE_WIDTH,
        g_param_spec_uint("source-width", "source-width",
            "Source video width in pixels. When >0, the tile crop lookup "
            "key is computed in source-pixel space (bbox * source-width) to "
            "match a writer run with the same source-width. 0 = caps width.",
            0, G_MAXUINT, 0,
            (GParamFlags)(G_PARAM_READWRITE | GST_PARAM_MUTABLE_READY | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_SOURCE_HEIGHT,
        g_param_spec_uint("source-height", "source-height",
            "Source video height in pixels. When >0, the tile crop lookup "
            "key is computed in source-pixel space (bbox * source-height) to "
            "match a writer run with the same source-height. 0 = caps height.",
            0, G_MAXUINT, 0,
            (GParamFlags)(G_PARAM_READWRITE | GST_PARAM_MUTABLE_READY | G_PARAM_STATIC_STRINGS)));

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
        "Filter/Cache",
        "Drop-in replacement for hailonet on the cache-replay path "
        "(Task 8 skeleton — no DB lookup yet)",
        "hailo.ai <contact@hailo.ai>");

    // --- BaseTransform config ---
    trans_class->transform_ip = gst_hailo_cache_reader_transform_ip;
    trans_class->set_caps     = gst_hailo_cache_reader_set_caps;
    // passthrough=FALSE per Task 8 spec (the element must own the
    // transform_ip callback so on-miss=error can post errors). The
    // passthrough flip lives in _init() because GstBaseTransform reads
    // it per-instance, not per-class.

    // --- Element-level state-change override (Task 9 cache lifecycle).
    element_class->change_state = gst_hailo_cache_reader_change_state;
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
    self->frame_id_source = GST_HAILO_CACHE_READER_FRAME_ID_COUNTER;
    self->source_width  = 0;
    self->source_height = 0;

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
    self->db                = nullptr;
    self->frame_counter     = 0;
    self->cached_ppv        = -1;
    self->cached_width      = 0;
    self->cached_height     = 0;

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
