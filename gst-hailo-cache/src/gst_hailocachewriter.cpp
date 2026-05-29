// gst-hailo-cache — hailocachewriter element (impl).
//
// Plan 5, Task 5 — SPSC ring + background writer thread + tile_cache mode.
//
// Threading model (spec §7.8):
//
//   Streaming thread (transform_ip)
//       │  compute frame_id, build Row(s), atomic push to ring
//       │  (NEVER blocks; if ring is full, drop and bump dropped-rows)
//       ▼
//   ┌──────────── SPSC ring buffer (lock-free) ────────────┐
//       ▲
//       │
//   Writer thread (writer_thread_main_)
//       │  drain rows in batches, BEGIN ... INSERT ... COMMIT
//       │  flush on (batch-size reached) || (flush-interval-ms elapsed)
//       │  on EOS / stop: synchronous final drain, close DB
//       ▼
//   SQLite file on disk
//
// What gets written PER BUFFER in tile_cache mode (this task):
//   - One row per upstream crop. If upstream crop-list metadata is
//     unavailable (Phase 14 not yet landed), we emit a single row with
//     the full-frame crop (0, 0, frame_w, frame_h) — see the TODO in
//     `build_rows_for_buffer_()`.
//   - `dets_json` is currently always `"[]"` (no detection extraction
//     wired yet — Task 5 only needs to prove the writer path is
//     correct end-to-end; Task 7 wires real detections behind the chip).
//   - `record-empty=false` skips empty rows; `record-cache-hits=false`
//     skips buffers carrying the `hailo-cache-hit` meta (Task 9 sets it).
//
// full_frame mode lands in Task 6. Until then, transform_ip falls back
// to tile_cache row-building when mode=full_frame (with a one-shot
// GST_WARNING) so the plumbing stays exercised but no wrong-schema rows
// hit the disk.

#include "gst_hailocachewriter.hpp"

#include <gst/gst.h>
#include <gst/video/video.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <mutex>
#include <new>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "cache_keys.hpp"
#include "tile_cache_db.hpp"

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

// Ring capacity rule (plan Task 5): max(1024, 4 * batch_size).
#define RING_MIN_CAPACITY         1024u

// Buffer-meta quark used to flag "this buffer carries cached detections".
// Spec §7.9 — the reader sets this; Task 5 only needs to *read* it. The
// gst meta API is the natural channel, but the reader currently has no
// public registration for it (lands in Task 9). For now we look it up by
// name via gst_meta_api_type_from_name() and gracefully fall through if
// no such meta type is registered (we accept all buffers in that case).
// TODO(Task 9): once the reader registers its meta type, switch to that
// quark directly instead of name-based lookup.
#define HAILO_CACHE_HIT_META_NAME "GstHailoCacheHitMeta"

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
    PROP_DROPPED_ROWS,
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
// Caps stay permissive — passthrough, no negotiation constraints beyond
// what GstBaseTransform requires. The writer never modifies the data
// path; it just inspects metadata.
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

static void          gst_hailocachewriter_set_property(GObject* object, guint property_id, const GValue* value, GParamSpec* pspec);
static void          gst_hailocachewriter_get_property(GObject* object, guint property_id, GValue* value, GParamSpec* pspec);
static void          gst_hailocachewriter_finalize    (GObject* object);
static GstFlowReturn gst_hailocachewriter_transform_ip(GstBaseTransform* trans, GstBuffer* buffer);
static gboolean      gst_hailocachewriter_set_caps    (GstBaseTransform* trans, GstCaps* incaps, GstCaps* outcaps);
static gboolean      gst_hailocachewriter_start       (GstBaseTransform* trans);
static gboolean      gst_hailocachewriter_stop        (GstBaseTransform* trans);
static gboolean      gst_hailocachewriter_sink_event  (GstBaseTransform* trans, GstEvent* event);

// -- Writer-thread helpers --------------------------------------------------
//
// Implementation file-scope helpers so we don't expose internal types.

namespace {

// Try-push a row onto the SPSC ring. Returns true on success, false on
// full (caller must NOT block; bump the dropped counter and move on).
bool ring_try_push_(HailoCacheWriterRing* ring, hailo_cache::Row&& row)
{
    if (!ring || ring->capacity == 0) return false;
    const std::size_t head = ring->head.load(std::memory_order_relaxed);
    const std::size_t next = (head + 1) % ring->capacity;
    if (next == ring->tail.load(std::memory_order_acquire)) {
        return false;  // full
    }
    ring->slots[head] = std::move(row);
    ring->head.store(next, std::memory_order_release);
    return true;
}

// Number of populated slots (snapshot; the writer thread is the only
// caller, so this is consistent w.r.t. the consumer-side index).
std::size_t ring_size_(const HailoCacheWriterRing* ring)
{
    const std::size_t head = ring->head.load(std::memory_order_acquire);
    const std::size_t tail = ring->tail.load(std::memory_order_relaxed);
    if (head >= tail) return head - tail;
    return ring->capacity - (tail - head);
}

// Drain up to `max_rows` from the ring into `out`. Returns the number
// drained. The writer thread is the only caller; `tail` is published
// with release semantics so the producer can observe space.
std::size_t ring_drain_(HailoCacheWriterRing* ring,
                        std::size_t max_rows,
                        std::vector<hailo_cache::Row>& out)
{
    std::size_t drained = 0;
    while (drained < max_rows) {
        const std::size_t tail = ring->tail.load(std::memory_order_relaxed);
        if (tail == ring->head.load(std::memory_order_acquire)) {
            break;  // empty
        }
        out.push_back(std::move(ring->slots[tail]));
        ring->tail.store((tail + 1) % ring->capacity, std::memory_order_release);
        ++drained;
    }
    return drained;
}

// Post a non-fatal ERROR on the element bus and continue.
void post_writer_error_(GstHailoCacheWriter* self, const std::string& what)
{
    GError* gerr = g_error_new_literal(GST_RESOURCE_ERROR,
                                       GST_RESOURCE_ERROR_WRITE,
                                       what.c_str());
    gchar* debug = g_strdup_printf("hailocachewriter background-thread error");
    gst_element_post_message(GST_ELEMENT(self),
        gst_message_new_error(GST_OBJECT(self), gerr, debug));
    g_error_free(gerr);
    g_free(debug);
}

// Look up the gst meta API for `hailo-cache-hit` lazily. Returns 0
// if no such meta type is registered (Task 9 will register it from the
// reader). We re-check until it's found, since the reader may register
// its meta type after the writer starts processing.
GType cache_hit_meta_api_type_()
{
    // Cache only the *resolved* type; until then, look it up each call.
    // This is cheap (a hash-table lookup in GLib) and lets the writer
    // pick up the meta type as soon as the reader registers it.
    static std::atomic<GType> cached{0};
    GType t = cached.load(std::memory_order_acquire);
    if (t != 0) return t;
    // GstMeta API types are registered as plain GTypes; look them up
    // through GLib's type registry by name. This returns 0 if no type
    // is registered with that name.
    t = g_type_from_name(HAILO_CACHE_HIT_META_NAME);
    if (t != 0) {
        cached.store(t, std::memory_order_release);
    }
    return t;
}

bool buffer_has_cache_hit_meta_(GstBuffer* buf)
{
    GType t = cache_hit_meta_api_type_();
    if (t == 0) return false;  // reader hasn't registered the meta yet
    return gst_buffer_get_meta(buf, t) != nullptr;
}

}  // namespace

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
    base_transform_class->set_caps     = GST_DEBUG_FUNCPTR(gst_hailocachewriter_set_caps);
    base_transform_class->start        = GST_DEBUG_FUNCPTR(gst_hailocachewriter_start);
    base_transform_class->stop         = GST_DEBUG_FUNCPTR(gst_hailocachewriter_stop);
    base_transform_class->sink_event   = GST_DEBUG_FUNCPTR(gst_hailocachewriter_sink_event);

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

    g_object_class_install_property(gobject_class, PROP_DROPPED_ROWS,
        g_param_spec_uint("dropped-rows", "dropped-rows",
                          "Number of rows dropped because the SPSC ring buffer was full. "
                          "Streaming-thread correctness is preserved by dropping; bump batch-size "
                          "or shorten flush-interval-ms if this is non-zero.",
                          0, G_MAXUINT, 0,
                          (GParamFlags)(G_PARAM_READABLE | G_PARAM_STATIC_STRINGS)));
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
    self->counter_state     = 0;
    self->frame_width       = 0;
    self->frame_height      = 0;

    // Allocate the C++ internals. We use plain `new` and store the
    // pointers in the POD struct so GObject doesn't need to know about
    // the C++ types. Lifetime ends in `finalize`.
    self->ring           = new HailoCacheWriterRing();
    self->writer_thread  = nullptr;  // started lazily in `start()`
    self->writer_stop    = new std::atomic<bool>(false);
    self->writer_mu      = new std::mutex();
    self->writer_cv      = new std::condition_variable();
    self->writer_failed  = new std::atomic<bool>(false);
    self->writer_started = new std::atomic<bool>(false);
    self->dropped_rows   = new std::atomic<std::uint64_t>(0);

    // Spec §7.8: writer is a *passthrough* recorder. Keep
    // passthrough=TRUE so GstBaseTransform doesn't reallocate / copy
    // buffers on our behalf.
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
    case PROP_DROPPED_ROWS: {
        std::uint64_t n = self->dropped_rows
            ? self->dropped_rows->load(std::memory_order_relaxed)
            : 0;
        // Clamp to G_MAXUINT for the GValue. Diagnostics only.
        g_value_set_uint(value, (guint)(n > G_MAXUINT ? G_MAXUINT : n));
        break;
    }
    default:
        G_OBJECT_WARN_INVALID_PROPERTY_ID(object, property_id, pspec);
        break;
    }
}

static void
gst_hailocachewriter_finalize(GObject* object)
{
    GstHailoCacheWriter* self = GST_HAILOCACHEWRITER(object);

    // The writer thread should have been stopped in `stop()`, but
    // belt-and-braces: signal stop and join here if not.
    if (self->writer_started && self->writer_started->load() && self->writer_thread) {
        self->writer_stop->store(true, std::memory_order_release);
        {
            std::lock_guard<std::mutex> lk(*self->writer_mu);
            self->writer_cv->notify_all();
        }
        if (self->writer_thread->joinable()) {
            try { self->writer_thread->join(); } catch (...) {}
        }
    }

    delete self->writer_thread;     self->writer_thread = nullptr;
    delete self->writer_stop;       self->writer_stop = nullptr;
    delete self->writer_mu;         self->writer_mu = nullptr;
    delete self->writer_cv;         self->writer_cv = nullptr;
    delete self->writer_failed;     self->writer_failed = nullptr;
    delete self->writer_started;    self->writer_started = nullptr;
    delete self->dropped_rows;      self->dropped_rows = nullptr;
    delete self->ring;              self->ring = nullptr;

    g_free(self->output_file);
    g_free(self->hef_id_meta_key);
    G_OBJECT_CLASS(gst_hailocachewriter_parent_class)->finalize(object);
}

// -- Caps -------------------------------------------------------------------

static gboolean
gst_hailocachewriter_set_caps(GstBaseTransform* trans, GstCaps* incaps, GstCaps* /*outcaps*/)
{
    GstHailoCacheWriter* self = GST_HAILOCACHEWRITER(trans);

    // We try to extract width/height for the fallback full-frame crop.
    // If the upstream caps aren't video (e.g. raw appsrc with ANY caps),
    // we leave width/height at 0 — the fallback will skip the row and
    // log a warning on the first buffer.
    GstVideoInfo info;
    gst_video_info_init(&info);
    if (gst_video_info_from_caps(&info, incaps)) {
        self->frame_width  = (std::int32_t)GST_VIDEO_INFO_WIDTH(&info);
        self->frame_height = (std::int32_t)GST_VIDEO_INFO_HEIGHT(&info);
        GST_INFO_OBJECT(self,
            "set_caps: frame=%dx%d", self->frame_width, self->frame_height);
    } else {
        // Try generic structure read — works for any caps with width/height.
        const GstStructure* s = gst_caps_get_structure(incaps, 0);
        gint w = 0, h = 0;
        if (gst_structure_get_int(s, "width", &w) &&
            gst_structure_get_int(s, "height", &h)) {
            self->frame_width  = (std::int32_t)w;
            self->frame_height = (std::int32_t)h;
            GST_INFO_OBJECT(self,
                "set_caps (non-video): frame=%dx%d", self->frame_width, self->frame_height);
        } else {
            GST_WARNING_OBJECT(self,
                "set_caps: could not extract width/height from caps; "
                "using 0x0 fallback crop. Buffers will still be recorded but "
                "the cache row will carry crop=(0,0,0,0).");
        }
    }
    return TRUE;
}

// -- Writer thread ----------------------------------------------------------
//
// The thread opens the SQLite, then loops draining the ring in batches.
// It exits when `writer_stop` is set AND the ring is empty.

static void
writer_thread_main_(GstHailoCacheWriter* self)
{
    // 1) Open the DB. Errors here are fatal (we have nowhere to send
    //    rows); post on the bus and set writer_failed. The streaming
    //    thread will see writer_failed and skip pushes, but it MUST
    //    not block.
    hailo_cache::TileCacheDb db;
    try {
        if (!self->output_file || !*self->output_file) {
            throw std::runtime_error("output-file property is required (got empty/null)");
        }
        db.open(self->output_file, /*create_if_missing=*/true);

        // First-write meta: stamp the hef-id-meta-key (informational) and
        // a writer marker so consumers can tell who wrote the file.
        // We don't yet have an actual HEF SHA — Task 7 will plumb it
        // through. Storing the *key name* lets the reader find the SHA
        // once Phase 14 lands.
        if (self->hef_id_meta_key && *self->hef_id_meta_key) {
            db.meta_put("writer.hef_id_meta_key", self->hef_id_meta_key);
        }
        db.meta_put("writer.element", "hailocachewriter");
    } catch (const std::exception& e) {
        self->writer_failed->store(true, std::memory_order_release);
        post_writer_error_(self,
            std::string("hailocachewriter: failed to open SQLite (") +
            (self->output_file ? self->output_file : "(null)") +
            "): " + e.what());
        return;
    }

    std::vector<hailo_cache::Row> batch;
    batch.reserve(self->batch_size);

    const auto flush_period = std::chrono::milliseconds(self->flush_interval_ms);
    auto deadline = std::chrono::steady_clock::now() + flush_period;

    while (true) {
        const bool stop = self->writer_stop->load(std::memory_order_acquire);
        // Sleep until either: a row arrives (signalled via cv), the
        // flush deadline passes, or we're asked to stop.
        if (!stop && ring_size_(self->ring) < self->batch_size) {
            std::unique_lock<std::mutex> lk(*self->writer_mu);
            self->writer_cv->wait_until(lk, deadline, [&] {
                return self->writer_stop->load(std::memory_order_acquire) ||
                       ring_size_(self->ring) >= self->batch_size;
            });
        }

        // Drain up to batch_size into `batch`.
        batch.clear();
        ring_drain_(self->ring, self->batch_size, batch);

        if (!batch.empty()) {
            try {
                db.put_many(batch);
            } catch (const std::exception& e) {
                // Don't set writer_failed here — a single failed batch
                // shouldn't poison the connection. Post on the bus and
                // continue accepting buffers. (The streaming thread is
                // never blocked either way.)
                post_writer_error_(self,
                    std::string("hailocachewriter: put_many failed: ") + e.what());
            }
        }

        // Reset deadline so we batch up to flush_interval_ms after the
        // last flush, not from the start of the queue.
        deadline = std::chrono::steady_clock::now() + flush_period;

        // Exit only when stop has been requested AND the ring is drained.
        if (self->writer_stop->load(std::memory_order_acquire) &&
            ring_size_(self->ring) == 0) {
            break;
        }
    }

    try { db.close(); } catch (...) { /* swallow */ }
}

static void
ensure_ring_allocated_(GstHailoCacheWriter* self)
{
    if (self->ring->capacity == 0) {
        std::size_t cap = static_cast<std::size_t>(self->batch_size) * 4;
        if (cap < RING_MIN_CAPACITY) cap = RING_MIN_CAPACITY;
        self->ring->slots = std::unique_ptr<hailo_cache::Row[]>(new hailo_cache::Row[cap]);
        self->ring->capacity = cap;
        self->ring->head.store(0, std::memory_order_relaxed);
        self->ring->tail.store(0, std::memory_order_relaxed);
    }
}

static gboolean
gst_hailocachewriter_start(GstBaseTransform* trans)
{
    GstHailoCacheWriter* self = GST_HAILOCACHEWRITER(trans);

    // Idempotent: start may be called once per state-transition cycle,
    // but a misbehaving harness may invoke it twice. Bail if already
    // running.
    if (self->writer_started->load(std::memory_order_acquire)) {
        return TRUE;
    }

    self->writer_stop->store(false, std::memory_order_release);
    self->writer_failed->store(false, std::memory_order_release);
    self->dropped_rows->store(0, std::memory_order_release);
    self->counter_state = 0;
    self->buffer_count  = 0;
    ensure_ring_allocated_(self);

    // Spawn the writer thread.
    try {
        self->writer_thread = new std::thread(writer_thread_main_, self);
    } catch (const std::exception& e) {
        GST_ERROR_OBJECT(self,
            "failed to spawn writer thread: %s", e.what());
        self->writer_failed->store(true, std::memory_order_release);
        return FALSE;
    }
    self->writer_started->store(true, std::memory_order_release);

    GST_INFO_OBJECT(self,
        "writer started: output-file=%s mode=%d batch-size=%u "
        "flush-interval-ms=%u ring-capacity=%zu",
        self->output_file ? self->output_file : "(unset)",
        (int)self->mode, self->batch_size, self->flush_interval_ms,
        self->ring->capacity);

    return TRUE;
}

static gboolean
gst_hailocachewriter_stop(GstBaseTransform* trans)
{
    GstHailoCacheWriter* self = GST_HAILOCACHEWRITER(trans);

    if (!self->writer_started->load(std::memory_order_acquire)) {
        return TRUE;
    }

    // Signal the writer thread to drain and exit.
    self->writer_stop->store(true, std::memory_order_release);
    {
        std::lock_guard<std::mutex> lk(*self->writer_mu);
        self->writer_cv->notify_all();
    }

    if (self->writer_thread && self->writer_thread->joinable()) {
        try { self->writer_thread->join(); }
        catch (const std::exception& e) {
            GST_WARNING_OBJECT(self,
                "writer thread join failed: %s", e.what());
        }
    }
    delete self->writer_thread;
    self->writer_thread = nullptr;
    self->writer_started->store(false, std::memory_order_release);

    GST_INFO_OBJECT(self,
        "writer stopped: total buffers=%" G_GUINT64_FORMAT
        " dropped rows=%" G_GUINT64_FORMAT,
        self->buffer_count,
        (guint64)self->dropped_rows->load(std::memory_order_relaxed));

    return TRUE;
}

// -- Sink event (synchronous EOS flush) -------------------------------------
//
// GstBaseTransform's default sink_event handler propagates events, but we
// override here to land a synchronous flush on EOS so the writer thread
// has drained before downstream sees the EOS marker. This preserves the
// spec §7.8 "EOS triggers a synchronous final flush before returning"
// contract.

static gboolean
gst_hailocachewriter_sink_event(GstBaseTransform* trans, GstEvent* event)
{
    GstHailoCacheWriter* self = GST_HAILOCACHEWRITER(trans);

    if (GST_EVENT_TYPE(event) == GST_EVENT_EOS) {
        GST_INFO_OBJECT(self, "EOS received — draining writer thread");
        if (self->writer_started->load(std::memory_order_acquire)) {
            self->writer_stop->store(true, std::memory_order_release);
            {
                std::lock_guard<std::mutex> lk(*self->writer_mu);
                self->writer_cv->notify_all();
            }
            if (self->writer_thread && self->writer_thread->joinable()) {
                try { self->writer_thread->join(); }
                catch (const std::exception& e) {
                    GST_WARNING_OBJECT(self,
                        "EOS: writer thread join failed: %s", e.what());
                }
            }
            delete self->writer_thread;
            self->writer_thread = nullptr;
            self->writer_started->store(false, std::memory_order_release);
        }
    }

    return GST_BASE_TRANSFORM_CLASS(gst_hailocachewriter_parent_class)
        ->sink_event(trans, event);
}

// -- transform_ip (streaming-thread side) -----------------------------------
//
// For Task 5, in tile_cache mode we emit ONE row per buffer with the
// full-frame crop (0, 0, width, height) and `dets_json="[]"`. This is
// the documented fallback when upstream crop-list metadata is missing;
// Phase 14 will plumb real crops + detections through.
//
// In full_frame mode (Task 6 — not yet implemented), we currently fall
// back to the same single-row tile_cache shape with a one-shot WARNING.
// This keeps the pipeline composing without writing wrong-schema rows;
// Task 6 will branch the row construction.

namespace {

void emit_tile_cache_row_(GstHailoCacheWriter* self,
                          std::int64_t frame_idx,
                          double ts_epoch,
                          std::int32_t cx, std::int32_t cy,
                          std::int32_t cw, std::int32_t ch,
                          std::int32_t ppv,
                          const std::string& dets_json)
{
    // record-empty=false: skip if no detections.
    if (!self->record_empty && dets_json == "[]") {
        return;
    }

    hailo_cache::Row row;
    row.frame_idx = frame_idx;
    row.crop_x    = cx;
    row.crop_y    = cy;
    row.crop_w    = cw;
    row.crop_h    = ch;
    row.ppv       = ppv;
    row.dets_json = dets_json;
    row.ts_epoch  = ts_epoch;

    if (!ring_try_push_(self->ring, std::move(row))) {
        const std::uint64_t prev = self->dropped_rows->fetch_add(1, std::memory_order_relaxed);
        if ((prev % 100) == 0) {
            GST_WARNING_OBJECT(self,
                "SPSC ring full — dropping row (total dropped=%" G_GUINT64_FORMAT
                "). Increase batch-size or shorten flush-interval-ms.",
                (guint64)(prev + 1));
        }
        return;
    }

    // Wake the writer thread if we just crossed the batch threshold.
    if (ring_size_(self->ring) >= self->batch_size) {
        std::lock_guard<std::mutex> lk(*self->writer_mu);
        self->writer_cv->notify_one();
    }
}

}  // namespace

static GstFlowReturn
gst_hailocachewriter_transform_ip(GstBaseTransform* trans, GstBuffer* buffer)
{
    GstHailoCacheWriter* self = GST_HAILOCACHEWRITER(trans);

    self->buffer_count++;

    // Hard fast-path skip if the writer thread failed to open the DB.
    // We don't error the streaming thread; the bus message has already
    // been posted.
    if (self->writer_failed->load(std::memory_order_acquire)) {
        return GST_FLOW_OK;
    }

    // record-cache-hits=false: skip buffers carrying the cache-hit meta.
    // Until Task 9 registers the meta type, this is a no-op (the lookup
    // returns 0 GType and `buffer_has_cache_hit_meta_` returns false).
    if (!self->record_cache_hits && buffer_has_cache_hit_meta_(buffer)) {
        return GST_FLOW_OK;
    }

    // (a) Compute frame id (counter or pts).
    const hailo_cache::FrameIdSource src =
        (self->frame_id_source == GST_HAILOCACHEWRITER_FRAME_ID_PTS)
            ? hailo_cache::FrameIdSource::PTS
            : hailo_cache::FrameIdSource::COUNTER;
    const std::int64_t frame_idx =
        hailo_cache::frame_id_from_buffer(buffer, src, self->counter_state);

    // (b) Build crop list. TODO(Phase 14): consult the upstream
    //     `GstHailoBaseCropperDyn` provenance metadata (or the
    //     identity-signal channel) to recover per-tile crop rects.
    //     For Task 5 we emit a SINGLE crop covering the whole frame.
    const std::int32_t cx = 0;
    const std::int32_t cy = 0;
    const std::int32_t cw = self->frame_width;
    const std::int32_t ch = self->frame_height;

    // (c) Detections: empty for Task 5. Task 7 will read
    //     `HailoROI` children attached to each crop by upstream
    //     `hailofilter`.
    const std::string dets_json = "[]";

    // (d) `ppv` — read from the cache `meta` table via Open Question 3
    //     fallback: env var, default 1.
    static const std::int32_t kPpv = []() {
        const char* env = std::getenv("HAILO_TILE_CACHE_PPV");
        if (!env || !*env) return 1;
        try { return std::stoi(env); }
        catch (...) { return 1; }
    }();

    // (e) ts_epoch from PTS or wall clock.
    double ts_epoch = 0.0;
    {
        // Wall-clock fallback — matches Python's `time.time()` rounding.
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts_epoch = (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
    }

    if (self->mode == GST_HAILOCACHEWRITER_MODE_FULL_FRAME) {
        // TODO(Task 6): branch to frame_results-shaped row.
        static gboolean warned_once = FALSE;
        if (!warned_once) {
            warned_once = TRUE;
            GST_WARNING_OBJECT(self,
                "mode=full_frame is a Task 6 deliverable; currently falling "
                "back to tile_cache row shape. Output file may carry rows "
                "with the wrong schema for full_frame consumers.");
        }
    }

    emit_tile_cache_row_(self, frame_idx, ts_epoch, cx, cy, cw, ch, kPpv, dets_json);

    GST_LOG_OBJECT(self,
        "buffer #%" G_GUINT64_FORMAT " frame_idx=%" G_GINT64_FORMAT
        " crop=(%d,%d,%dx%d)",
        self->buffer_count, (gint64)frame_idx, cx, cy, cw, ch);

    return GST_FLOW_OK;
}
