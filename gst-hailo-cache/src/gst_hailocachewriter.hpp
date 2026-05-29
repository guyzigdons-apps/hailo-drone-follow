// gst-hailo-cache — hailocachewriter element header.
//
// Plan 5, Task 5 (docs/superpowers/plans/2026-05-28-gst-cache-plugins.md):
//   SPSC ring + background writer thread + tile_cache DB writes.
//
// The streaming-thread side (`transform_ip`) builds a Row from each
// buffer and pushes it onto a lock-free single-producer / single-consumer
// ring buffer. A background writer thread (`writer_thread_main_`) drains
// the ring in batches and writes them to SQLite through `TileCacheDb`.
//
// Properties (spec §7.8):
//   mode               enum  {tile_cache, full_frame}, default tile_cache
//   output-file        string (required)
//   flush-interval-ms  uint, default 100
//   batch-size         uint, default 64
//   frame-id-source    enum  {counter, pts}, default counter
//   record-empty       bool, default TRUE
//   record-cache-hits  bool, default FALSE
//   hef-id-meta-key    string, default "hailo-hef-sha"
//   dropped-rows       uint (read-only) — count of rows dropped because
//                      the SPSC ring was full. Task 5 surfaces it for
//                      diagnostics; the streaming thread never blocks.

#pragma once

#include <gst/base/gstbasetransform.h>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "tile_cache_db.hpp"

G_BEGIN_DECLS

#define GST_TYPE_HAILOCACHEWRITER (gst_hailocachewriter_get_type())
#define GST_HAILOCACHEWRITER(obj) \
    (G_TYPE_CHECK_INSTANCE_CAST((obj), GST_TYPE_HAILOCACHEWRITER, GstHailoCacheWriter))
#define GST_HAILOCACHEWRITER_CLASS(klass) \
    (G_TYPE_CHECK_CLASS_CAST((klass), GST_TYPE_HAILOCACHEWRITER, GstHailoCacheWriterClass))
#define GST_IS_HAILOCACHEWRITER(obj) \
    (G_TYPE_CHECK_INSTANCE_TYPE((obj), GST_TYPE_HAILOCACHEWRITER))
#define GST_IS_HAILOCACHEWRITER_CLASS(klass) \
    (G_TYPE_CHECK_CLASS_TYPE((klass), GST_TYPE_HAILOCACHEWRITER))

typedef struct _GstHailoCacheWriter      GstHailoCacheWriter;
typedef struct _GstHailoCacheWriterClass GstHailoCacheWriterClass;

// Enum: mode property values. Numeric values are part of the GEnum
// type registration — keep STABLE (don't reorder) so saved pipeline
// strings using mode=0/1 stay valid.
typedef enum {
    GST_HAILOCACHEWRITER_MODE_TILE_CACHE = 0,
    GST_HAILOCACHEWRITER_MODE_FULL_FRAME = 1,
} GstHailoCacheWriterMode;

// Enum: frame-id-source property values. Mirrors the
// hailo_cache::FrameIdSource C++ enum the cache_keys helper uses;
// kept as a separate GLib enum because GObject introspection wants
// its own type for property registration.
typedef enum {
    GST_HAILOCACHEWRITER_FRAME_ID_COUNTER = 0,
    GST_HAILOCACHEWRITER_FRAME_ID_PTS     = 1,
} GstHailoCacheWriterFrameIdSource;

G_END_DECLS

// -- C++-only internals -----------------------------------------------------
//
// The ring buffer + writer-thread plumbing lives in C++ types embedded in
// the GstHailoCacheWriter struct. GObject doesn't care about the layout
// as long as G_DEFINE_TYPE can compute sizeof at compile time, so we keep
// these as POD pointers + a couple of `std::atomic` fields. The C++
// destructor work happens in the GObject finalize hook (manual placement
// new/delete is *not* used; we just construct/destruct via the GObject
// init/finalize seams).

// Fixed-size single-producer/single-consumer ring of pending rows.
// Capacity is set lazily on first push (it depends on the `batch-size`
// property, which may be tuned before the element transitions to
// PLAYING). We pick `max(1024, 4*batch_size)` per the plan.
//
// Layout note: head_ is mutated only by the streaming thread; tail_ only
// by the writer thread. Both publish their respective indices via release
// stores; the reader uses acquire loads. This is a textbook SPSC ring.
struct HailoCacheWriterRing {
    std::unique_ptr<hailo_cache::Row[]> slots;
    std::size_t                          capacity{0};
    std::atomic<std::size_t>             head{0};   // producer: streaming thread
    std::atomic<std::size_t>             tail{0};   // consumer: writer thread
};

struct _GstHailoCacheWriter {
    GstBaseTransform base;

    // Properties
    GstHailoCacheWriterMode          mode;
    gchar*                           output_file;
    guint                            flush_interval_ms;
    guint                            batch_size;
    GstHailoCacheWriterFrameIdSource frame_id_source;
    gboolean                         record_empty;
    gboolean                         record_cache_hits;
    gchar*                           hef_id_meta_key;

    // Diagnostics (read-only property)
    std::atomic<std::uint64_t>*      dropped_rows;

    // Internal state — Task 5 wires the writer thread.
    guint64                          buffer_count;
    std::int64_t                     counter_state;   // frame-id COUNTER source

    // Frame dimensions captured from sink caps. Used as the fallback crop
    // when no upstream crop-list metadata is available (Phase 14 wiring
    // not landed yet — see Task 5 description in the plan).
    std::int32_t                     frame_width;
    std::int32_t                     frame_height;

    // Cached state for the writer thread.
    HailoCacheWriterRing*            ring;
    std::thread*                     writer_thread;
    std::atomic<bool>*               writer_stop;
    std::mutex*                      writer_mu;
    std::condition_variable*         writer_cv;
    std::atomic<bool>*               writer_failed;
    std::atomic<bool>*               writer_started;
};

struct _GstHailoCacheWriterClass {
    GstBaseTransformClass base_class;
};

G_BEGIN_DECLS

GType gst_hailocachewriter_get_type(void);

G_END_DECLS
