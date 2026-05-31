// gst-hailo-cache — GStreamer plugin entry point.
//
// Task 1 of Plan 5 (docs/superpowers/plans/2026-05-28-gst-cache-plugins.md):
//   scaffold only — no element classes registered yet.
// Task 4 of Plan 5:
//   register `hailocachewriter` (passthrough skeleton + property surface).
// Task 8 of Plan 5 (concurrent):
//   register `hailocachereader` alongside the writer. Both registrations
//   sit side-by-side here so the two tasks can land without disturbing
//   each other; if Task 8 lands first, its `gst_element_register(...)`
//   call will already be present — just leave it alone and add the
//   writer line next to it.

#include <gst/gst.h>

#include "gst_hailocachewriter.hpp"
#include "gst_hailocachereader.hpp"
#include "gst_hailocachebypass.hpp"

#ifndef PACKAGE
#define PACKAGE "hailo-drone-follow"
#endif

static gboolean
plugin_init(GstPlugin* plugin)
{
    // WRITER — Task 4.
    if (!gst_element_register(plugin, "hailocachewriter",
                              GST_RANK_NONE,
                              GST_TYPE_HAILOCACHEWRITER)) {
        return FALSE;
    }

    // READER — Task 8. Element class lives in gst_hailocachereader.cpp;
    // gst_hailo_cache_reader_plugin_init wraps the gst_element_register
    // call so the reader's GType + enum types stay private to its TU.
    if (!gst_hailo_cache_reader_plugin_init(plugin)) {
        return FALSE;
    }

    // BYPASS — Task 12 (Phase 14 `bypass-on-cache-hit` wrapper-first
    // implementation). Stands in for `hailofilter` in the cache-replay
    // pipeline; forwards HIT/MISS-tagged buffers from `hailocachereader`
    // without running the postprocess .so.
    if (!gst_hailo_cache_bypass_plugin_init(plugin)) {
        return FALSE;
    }

    return TRUE;
}

GST_PLUGIN_DEFINE(
    GST_VERSION_MAJOR,
    GST_VERSION_MINOR,
    hailocache,
    "Hailo tile-cache GStreamer plugins",
    plugin_init,
    "0.1.0",
    "LGPL",
    PACKAGE,
    "https://github.com/hailo-ai/hailo-drone-follow"
)
