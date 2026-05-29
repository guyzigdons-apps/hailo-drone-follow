// gst-hailo-cache — hailocachewriter element header.
//
// Plan 5, Task 4 (docs/superpowers/plans/2026-05-28-gst-cache-plugins.md):
//   passthrough skeleton + spec §7.8 property surface only.
//
// Subclasses GstBaseTransform with passthrough=TRUE so the element does
// nothing to the streaming data path beyond logging the buffer count.
// DB writes land in Task 5.
//
// Properties (spec §7.8):
//   mode               enum  {tile_cache, full_frame}, default tile_cache
//   output-file        string (required; not validated in Task 4)
//   flush-interval-ms  uint, default 100
//   batch-size         uint, default 64
//   frame-id-source    enum  {counter, pts}, default counter
//   record-empty       bool, default TRUE
//   record-cache-hits  bool, default FALSE
//   hef-id-meta-key    string, default "hailo-hef-sha"

#pragma once

#include <gst/base/gstbasetransform.h>

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

    // Internal state — Task 4 only uses buffer_count.
    guint64                          buffer_count;
};

struct _GstHailoCacheWriterClass {
    GstBaseTransformClass base_class;
};

GType gst_hailocachewriter_get_type(void);

G_END_DECLS
