#include "dynamic_tiling_stage.hpp"

#include <hailodsp.h>
#include <media_library/buffer_pool.hpp>
#include <media_library/dsp_utils.hpp>
#include <media_library/media_library_buffer.hpp>
#include <media_library/media_library_types.hpp>
#include <stdexcept>
#include <utility>

#include "hailo_analytics/logger/hailo_analytics_logger.hpp"

namespace drone_follow_app
{

namespace cropping = hailo_analytics::pipeline::cropping;
namespace pipeline = hailo_analytics::pipeline;

DynamicTilingCropStage::DynamicTilingCropStage(
    std::string name,
    int output_pool_size,
    int input_width, int input_height,
    int output_width, int output_height,
    std::string main_sub_name, std::string sub_sub_name,
    const std::vector<HailoBBox> &initial_tiles,
    size_t queue_size,
    bool leaky,
    bool trace_processing_operations,
    pipeline::StagePoolMode pool_mode,
    size_t crop_every_x_frames)
    : DspBaseCropStage(std::move(name), output_pool_size, input_width, input_height,
                       output_width, output_height, std::move(main_sub_name),
                       std::move(sub_sub_name), queue_size, leaky,
                       trace_processing_operations, pool_mode, crop_every_x_frames),
      m_tile_count(initial_tiles.size())
{
    if (m_tile_count < 2) {
        // Mirrors the constraint in main.cpp:parse_tiles_spec — the framework's
        // tiling path needs ≥2 tiles to drive the DSP cropper, single-tile
        // leaves the AI stage with zero buffers.
        throw std::invalid_argument(
            "DynamicTilingCropStage: needs ≥2 initial tiles, got " +
            std::to_string(m_tile_count));
    }
    m_tiles_current =
        std::make_shared<const std::vector<HailoBBox>>(initial_tiles);
}

pipeline::AppStatus DynamicTilingCropStage::init()
{
    // Mirror TilingCropStage::init exactly: DSP device acquire (base), then
    // a buffer pool sized for the output crop dimensions, plus the pool
    // notification hook for BLOCKING pool mode.
    pipeline::AppStatus base_status = DspBaseCropStage::init();
    if (base_status != pipeline::AppStatus::SUCCESS) {
        return base_status;
    }

    auto bytes_per_line = dsp_utils::get_dsp_desired_stride_from_width(m_output_width);
    m_buffer_pool = std::make_shared<MediaLibraryBufferPool>(
        m_output_width, m_output_height, HAILO_FORMAT_NV12,
        m_output_pool_size, HAILO_MEMORY_TYPE_DMABUF, bytes_per_line,
        "dynamic_tiling_buffer_pool");
    if (m_buffer_pool->init() != MEDIA_LIBRARY_SUCCESS) {
        return pipeline::AppStatus::DSP_OPERATION_ERROR;
    }

    setup_pool_notification();

    // Seed the ROI cache from the initial tile snapshot. prepare_crops
    // will refresh this from m_tiles_current each frame; this just gives
    // us a non-empty cache before the first frame arrives.
    std::shared_ptr<const std::vector<HailoBBox>> initial;
    {
        std::lock_guard<std::mutex> lock(m_tiles_mutex);
        initial = m_tiles_current;
    }
    m_current_tile_rois.reserve(initial->size());
    for (const auto &bbox : *initial) {
        m_current_tile_rois.push_back(
            std::make_shared<HailoTileROI>(bbox, 0, 0, 0, 0, SINGLE_SCALE));
    }
    return pipeline::AppStatus::SUCCESS;
}

void DynamicTilingCropStage::prepare_crops(
    pipeline::BufferPtr input_buffer,
    std::vector<dsp_crop_api_t> &crop_resize_dims)
{
    // Snapshot the active tile list for this frame. Any set_bbox_tiles
    // calls that arrived since the previous prepare_crops take effect here.
    std::shared_ptr<const std::vector<HailoBBox>> snapshot;
    {
        std::lock_guard<std::mutex> lock(m_tiles_mutex);
        snapshot = m_tiles_current;
    }

    // Refresh the ROI cache only when the snapshot's underlying pointer
    // differs from the one we used last frame. set_bbox_tiles ALWAYS
    // installs a fresh shared_ptr, so address equality is sufficient —
    // cheaper than per-bbox comparison, and avoids the false-negative
    // case where the application toggles between two configs whose
    // bboxes happen to be element-wise equal at some point.
    if (snapshot.get() != m_last_consumed_snapshot_ptr) {
        m_current_tile_rois.clear();
        m_current_tile_rois.reserve(snapshot->size());
        for (const auto &bbox : *snapshot) {
            m_current_tile_rois.push_back(
                std::make_shared<HailoTileROI>(bbox, 0, 0, 0, 0, SINGLE_SCALE));
        }
        m_last_consumed_snapshot_ptr = snapshot.get();
        // Bump observed_epoch AFTER the cache is in place. Pairs with
        // m_set_epoch increments in set_bbox_tiles — the toggler thread
        // spin-waits on observed_epoch catching up to know "the new tiles
        // are now driving the DSP crop".
        m_observed_epoch.fetch_add(1, std::memory_order_release);
    }

    // Populate crop dims from current ROIs. Mirrors TilingCropStage::prepare_crops.
    HailoMediaLibraryBufferPtr buffer = input_buffer->get_buffer();
    int input_width = buffer->buffer_data->width;
    int input_height = buffer->buffer_data->height;
    for (auto &tile : m_current_tile_rois) {
        HailoBBox bbox = tile->get_bbox();
        prepare_single_crop_dim(bbox, crop_resize_dims, input_width, input_height);
    }
}

HailoBBox DynamicTilingCropStage::get_crop_bbox(int index)
{
    try {
        return m_current_tile_rois.at(index)->get_bbox();
    } catch (const std::out_of_range &e) {
        HAILO_ANALYTICS_LOG_ERROR("DynamicTilingCropStage: get_crop_bbox index {} "
                                  "out of bounds: {}", index, e.what());
        throw;
    }
}

void DynamicTilingCropStage::set_bbox_tiles(std::vector<HailoBBox> new_tiles)
{
    if (new_tiles.size() != m_tile_count) {
        throw std::invalid_argument(
            "DynamicTilingCropStage::set_bbox_tiles: tile count must remain " +
            std::to_string(m_tile_count) + " (got " +
            std::to_string(new_tiles.size()) + "). The aggregator's "
            "static_subframes count is fixed at pipeline construction.");
    }
    auto new_snapshot =
        std::make_shared<const std::vector<HailoBBox>>(std::move(new_tiles));
    {
        std::lock_guard<std::mutex> lock(m_tiles_mutex);
        m_tiles_current = std::move(new_snapshot);
    }
    // Bump AFTER publishing the new snapshot so a reader of set_epoch
    // never sees a counter that's "ahead" of the visible snapshot.
    m_set_epoch.fetch_add(1, std::memory_order_release);
}

std::vector<HailoBBox> DynamicTilingCropStage::get_bbox_tiles() const
{
    std::shared_ptr<const std::vector<HailoBBox>> snapshot;
    {
        std::lock_guard<std::mutex> lock(m_tiles_mutex);
        snapshot = m_tiles_current;
    }
    return *snapshot;
}

} // namespace drone_follow_app
