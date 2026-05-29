// gst-hailo-cache — GStreamer plugin entry point.
//
// Task 1 of Plan 5 (docs/superpowers/plans/2026-05-28-gst-cache-plugins.md):
//   scaffold only — no element classes registered yet.
//
// Element classes (hailocachewriter, hailocachereader) are registered in
// later tasks (Task 4 and Task 8 respectively). For now this file only
// brings up the GST_PLUGIN_DEFINE plumbing so the install + registry
// hand-off can be exercised end-to-end before any real code lands.

#include <gst/gst.h>

#ifndef PACKAGE
#define PACKAGE "hailo-drone-follow"
#endif

static gboolean
plugin_init(GstPlugin* /*plugin*/)
{
    // Intentionally empty: element classes register in later tasks.
    // Returning TRUE means "plugin loaded; the (empty) element set was
    // installed successfully", which is what gst-inspect expects for a
    // scaffold drop.
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
