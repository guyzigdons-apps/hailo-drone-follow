// gst-hailo-cache — hailocachebypass element (Plan 5 Task 12, option 1).
//
// See gst_hailocachebypass.hpp for the full design contract. This is the
// wrapper-first implementation of spec §7.9's cache-hit bypass: a thin
// GstBaseTransform passthrough that stands in for the postprocess
// `hailofilter .so` in the cache-replay pipeline.
//
// Behaviour summary (see header for full rationale):
//
//   HIT     → forward buffer untouched (qdata + cached detection JSON
//             already attached upstream by hailocachereader).
//   MISS    → forward buffer untouched.
//   absent  → forward buffer + emit GST_WARNING once.
//
// There are no public properties: behaviour is fully determined by the
// upstream `hailocachereader` qdata. (If spec §7.9 evolves to need a
// `record-cache-hits` style toggle here, add it as a property on this
// element rather than reaching across to `hailocachewriter`.)

#include "gst_hailocachebypass.hpp"

// We READ the qdata keys that `hailocachereader` writes, so we share its
// public header. We do NOT depend on any of the reader's internal C++
// types — only the macro constants under `GST_HAILO_CACHE_HIT_*`.
#include "gst_hailocachereader.hpp"

GST_DEBUG_CATEGORY_STATIC(gst_hailo_cache_bypass_debug);
#define GST_CAT_DEFAULT gst_hailo_cache_bypass_debug

// ---------------------------------------------------------------------------
// Instance struct
// ---------------------------------------------------------------------------

struct _GstHailoCacheBypass {
    GstBaseTransform parent;

    // One-shot "you ran me without a hailocachereader upstream" warning.
    // GstBaseTransform calls transform_ip on every buffer; without this
    // gate the log explodes on misconfigured pipelines. The flag is reset
    // on state-change READY→NULL so a paused/resumed run can re-warn.
    gboolean warned_qdata_absent;
};

G_DEFINE_TYPE_WITH_CODE(
    GstHailoCacheBypass,
    gst_hailo_cache_bypass,
    GST_TYPE_BASE_TRANSFORM,
    GST_DEBUG_CATEGORY_INIT(gst_hailo_cache_bypass_debug,
                            "hailocachebypass", 0,
                            "Hailo cache-hit hailofilter bypass (Task 12)");
)

// ---------------------------------------------------------------------------
// Pad templates — ANY/ANY (same as hailocachereader; we don't touch caps).
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
// transform_ip — inspect the qdata, log, forward.
// ---------------------------------------------------------------------------

static GstFlowReturn
gst_hailo_cache_bypass_transform_ip(GstBaseTransform* trans, GstBuffer* buf)
{
    GstHailoCacheBypass* self = GST_HAILO_CACHE_BYPASS(trans);

    GQuark q_hit = g_quark_from_static_string(GST_HAILO_CACHE_HIT_QDATA_KEY);
    gpointer p   = gst_mini_object_get_qdata(GST_MINI_OBJECT(buf), q_hit);

    if (p == nullptr) {
        // No upstream reader marked this buffer. Forward it but warn —
        // exactly once per state-cycle — so the operator knows their
        // pipeline is mis-wired (e.g. they pointed at a live `hailonet`
        // path and accidentally inserted hailocachebypass).
        if (!self->warned_qdata_absent) {
            self->warned_qdata_absent = TRUE;
            GST_WARNING_OBJECT(self,
                "buffer has no hailo-cache-hit qdata — "
                "hailocachebypass is only meaningful downstream of "
                "hailocachereader. Forwarding buffer unchanged. "
                "(This warning is emitted once per state cycle.)");
        }
        return GST_FLOW_OK;
    }

    int v = GPOINTER_TO_INT(p);
    switch (v) {
        case GST_HAILO_CACHE_HIT_VALUE_HIT:
            // The cached detection JSON is already attached upstream.
            // We're explicitly NOT invoking any hailofilter postprocess
            // — that's the entire point of this element. The qdata and
            // any GST_HAILO_CACHED_DETECTIONS_QDATA_KEY payload pass
            // through untouched because we don't strip them.
            GST_LOG_OBJECT(self, "HIT: bypassing hailofilter postprocess");
            break;

        case GST_HAILO_CACHE_HIT_VALUE_MISS:
            // The upstream reader was in on-miss=drop mode and pushed
            // the buffer through without detections. We could run the
            // postprocess here, but `hailocachebypass` is a STAND-IN
            // for hailofilter in this pipeline; we don't have access to
            // network output tensors anyway (the reader emits zero
            // HailoTensor metas). Forward and let downstream cope.
            GST_DEBUG_OBJECT(self,
                "MISS: forwarding buffer without postprocess "
                "(reader on-miss=drop semantics; no tensors available).");
            break;

        default:
            // Unknown qdata value. Treat as "absent" and forward — but
            // log a one-time warning so future Plan 5 / Phase 14 changes
            // to the reader's encoding are caught loudly.
            if (!self->warned_qdata_absent) {
                self->warned_qdata_absent = TRUE;
                GST_WARNING_OBJECT(self,
                    "unknown hailo-cache-hit qdata value %d — "
                    "forwarding buffer unchanged. "
                    "(Update hailocachebypass to handle this encoding.)",
                    v);
            }
            break;
    }

    return GST_FLOW_OK;
}

// ---------------------------------------------------------------------------
// State-change override — reset the one-shot warn gate on PAUSED→READY
// so a paused/resumed run re-warns rather than silently swallowing the
// configuration mistake on second startup.
// ---------------------------------------------------------------------------

static GstStateChangeReturn
gst_hailo_cache_bypass_change_state(GstElement* element,
                                    GstStateChange transition)
{
    GstHailoCacheBypass* self = GST_HAILO_CACHE_BYPASS(element);

    GstStateChangeReturn ret = GST_ELEMENT_CLASS(
        gst_hailo_cache_bypass_parent_class)->change_state(element, transition);

    if (transition == GST_STATE_CHANGE_PAUSED_TO_READY) {
        self->warned_qdata_absent = FALSE;
    }
    return ret;
}

// ---------------------------------------------------------------------------
// class_init / init
// ---------------------------------------------------------------------------

static void
gst_hailo_cache_bypass_class_init(GstHailoCacheBypassClass* klass)
{
    GstElementClass*       element_class = GST_ELEMENT_CLASS(klass);
    GstBaseTransformClass* trans_class   = GST_BASE_TRANSFORM_CLASS(klass);

    gst_element_class_add_static_pad_template(element_class, &src_template);
    gst_element_class_add_static_pad_template(element_class, &sink_template);

    gst_element_class_set_static_metadata(element_class,
        "Hailo cache-hit bypass",
        "Filter/Cache",
        "Stand-in for hailofilter in the cache-replay pipeline. Forwards "
        "buffers tagged with hailo-cache-hit qdata (by hailocachereader) "
        "without running the postprocess .so — Phase 14 `bypass-on-cache-hit` "
        "wrapper-first implementation (Plan 5 Task 12).",
        "hailo.ai <contact@hailo.ai>");

    trans_class->transform_ip = gst_hailo_cache_bypass_transform_ip;
    element_class->change_state = gst_hailo_cache_bypass_change_state;
}

static void
gst_hailo_cache_bypass_init(GstHailoCacheBypass* self)
{
    self->warned_qdata_absent = FALSE;

    // Passthrough=TRUE so GstBaseTransform doesn't allocate a copy of
    // the buffer. transform_ip is still called because we explicitly
    // assign it (BaseTransform calls the IP fn even in passthrough mode
    // when set_in_place is also TRUE).
    gst_base_transform_set_passthrough(GST_BASE_TRANSFORM(self), TRUE);
    gst_base_transform_set_in_place(GST_BASE_TRANSFORM(self), TRUE);
}

// ---------------------------------------------------------------------------
// Plugin-init registration helper
// ---------------------------------------------------------------------------

gboolean
gst_hailo_cache_bypass_plugin_init(GstPlugin* plugin)
{
    return gst_element_register(plugin, "hailocachebypass",
                                GST_RANK_NONE,
                                GST_TYPE_HAILO_CACHE_BYPASS);
}
