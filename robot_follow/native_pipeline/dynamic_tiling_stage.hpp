#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "hailo_analytics/pipeline/core/buffer.hpp"
#include "hailo_analytics/pipeline/core/stage.hpp"
#include "hailo_analytics/pipeline/cropping/dsp_cropping.hpp"
#include "hailo_postprocess_tools/objects/hailo_objects.hpp"

namespace drone_follow_app
{

// Drop-in replacement for hailo_analytics::cropping::TilingCropStage that
// supports hot-swapping the tile geometry at runtime via ``set_bbox_tiles``.
//
// Why this can be done safely without a pipeline restart:
//   * DspBaseCropStage::process() is serial per-frame on a single thread —
//     prepare_crops, the DSP multi-crop call, and the per-subframe
//     get_crop_bbox() loop all run in one continuous call. So the snapshot
//     pinned in prepare_crops stays valid through get_crop_bbox().
//   * After get_crop_bbox(i), the bbox is baked into each subframe's ROI
//     via ``cropped_buffer_ptr->get_roi()->set_scaling_bbox(...)``
//     (dsp_cropping.cpp:250). Downstream stages (detection NPU, aggregator)
//     read from the buffer's own ROI, not from this stage's cache. So
//     mutating the cache between process() calls cannot misattribute
//     in-flight detections.
//
// Tile-count invariant:
//   The aggregator stage is constructed with set_static_subframes_opt(N)
//   in our builder, so N is fixed for the lifetime of the pipeline. We
//   enforce this on ``set_bbox_tiles`` by rejecting any new vector whose
//   size doesn't match the constructor's initial_tiles.size(). Tile
//   GEOMETRIES are fully dynamic; only the COUNT is locked.
class DynamicTilingCropStage : public hailo_analytics::pipeline::cropping::DspBaseCropStage
{
public:
    DynamicTilingCropStage(
        std::string name,
        int output_pool_size,
        int input_width, int input_height,
        int output_width, int output_height,
        std::string main_sub_name, std::string sub_sub_name,
        const std::vector<HailoBBox> &initial_tiles,
        size_t queue_size,
        bool leaky = false,
        bool trace_processing_operations = true,
        hailo_analytics::pipeline::StagePoolMode pool_mode =
            hailo_analytics::pipeline::StagePoolMode::FAIL_ON_EMPTY_POOL,
        size_t crop_every_x_frames = 1);

    // DspBaseCropStage hooks. All operate on the per-frame snapshot
    // (m_current_tile_rois) populated by prepare_crops.
    hailo_analytics::pipeline::AppStatus init() override;
    void prepare_crops(
        hailo_analytics::pipeline::BufferPtr input_buffer,
        std::vector<dsp_crop_api_t> &crop_resize_dims) override;
    HailoBBox get_crop_bbox(int index) override;
    void pre_crop(hailo_analytics::pipeline::BufferPtr /*input_buffer*/) override {}
    void post_crop(hailo_analytics::pipeline::BufferPtr /*input_buffer*/) override {}
    HailoROIPtr get_crop_roi(int /*index*/) override { return nullptr; }

    // Atomic hot-swap of the tile geometry. Safe to call from any thread;
    // the new tiles take effect on the NEXT frame's prepare_crops call.
    // ``new_tiles.size()`` MUST equal the count fixed at construction or
    // this throws std::invalid_argument.
    void set_bbox_tiles(std::vector<HailoBBox> new_tiles);

    // Read-only access for diagnostics / round-tripping the current
    // geometry back to the orchestrator. Returns a snapshot (copy by
    // value of the underlying shared_ptr's payload).
    std::vector<HailoBBox> get_bbox_tiles() const;

    size_t tile_count() const { return m_tile_count; }

    // Monotonic counters for latency measurement. ``set_epoch`` is bumped
    // by set_bbox_tiles (i.e., when the *application* decided to swap);
    // ``observed_epoch`` is bumped by prepare_crops the first time it
    // sees the new snapshot pointer. A toggler thread can measure the
    // wall-clock delta between bumping set_epoch and observing
    // observed_epoch catch up — that's the "API call → next frame uses
    // new tiles" latency.
    uint64_t set_epoch() const { return m_set_epoch.load(std::memory_order_acquire); }
    uint64_t observed_epoch() const { return m_observed_epoch.load(std::memory_order_acquire); }

private:
    // Mutex-protected swap point. Lock held only for the duration of a
    // shared_ptr copy/swap (a few instructions). std::atomic<shared_ptr<T>>
    // would be ideal but libstdc++ in this Yocto SDK (GCC 11) hasn't
    // landed the C++20 specialization yet; the static_assert in <atomic>
    // rejects it. The mutex is uncontended in steady state (one writer
    // per /api/tiles POST, one reader per frame).
    mutable std::mutex m_tiles_mutex;
    std::shared_ptr<const std::vector<HailoBBox>> m_tiles_current;
    const size_t m_tile_count;

    // Per-frame ROI cache. Populated in prepare_crops from the snapshot,
    // consumed in the same process() call by get_crop_bbox. Not shared
    // across frames or threads — DspBaseCropStage::process() runs serially
    // per stage, so no synchronization needed.
    std::vector<HailoTileROIPtr> m_current_tile_rois;
    // The shared_ptr address of the last snapshot consumed by
    // prepare_crops. Used to detect new geometries without comparing
    // each bbox — a pointer-equality check is enough since
    // set_bbox_tiles always allocates a fresh shared_ptr.
    const std::vector<HailoBBox> *m_last_consumed_snapshot_ptr = nullptr;

    // Latency-measurement counters. See accessors above for semantics.
    std::atomic<uint64_t> m_set_epoch{0};
    std::atomic<uint64_t> m_observed_epoch{0};
};

} // namespace drone_follow_app
