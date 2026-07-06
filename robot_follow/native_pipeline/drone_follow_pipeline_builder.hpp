#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "hailo_analytics/analytics/tiling.hpp"
#include "hailo_analytics/pipeline/core/pipeline.hpp"
#include "hailo_postprocess_tools/objects/hailo_objects.hpp"

#include "dynamic_tiling_stage.hpp"

namespace drone_follow_app
{

// Bundle returned by build_drone_follow_detection_pipeline: the assembled
// pipeline plus a handle to the dynamic tiling stage. The C++ control
// thread keeps the stage handle so it can call ``set_bbox_tiles`` for
// hot-swaps without rebuilding the pipeline.
struct DroneFollowDetectionPipeline
{
    hailo_analytics::pipeline::PipelinePtr pipeline;
    std::shared_ptr<DynamicTilingCropStage> tiling_stage;
};

// Returns a baseline tiling_detection_config_t tuned for drone-follow:
// YOLOv8s, tracker disabled (Python's ByteTracker handles tracking),
// queue sizes / aggregator multi-scale / static subframes count.
// Callers can mutate fields on the returned config before passing it to
// build_drone_follow_detection_pipeline.
hailo_analytics::analytics::tiling::tiling_detection_config_t
default_detection_config(std::optional<std::vector<HailoBBox>> bbox_tiles = std::nullopt);

// Manually assembles the detection pipeline. Mirrors libhailo_analytics's
// generate_tiling_detection_pipeline (apps/.../tiling.cpp:234) but
// substitutes DynamicTilingCropStage for the framework's static
// TilingCropStage, and pins the aggregator's static_subframes count to
// the initial tile count (so the aggregator's bookkeeping doesn't break
// when geometries change at runtime — only counts are locked).
//
// The returned tiling_stage handle is what main.cpp's control thread
// uses for set_bbox_tiles() runtime updates.
DroneFollowDetectionPipeline
build_drone_follow_detection_pipeline(
    const std::string &pipeline_name,
    hailo_analytics::analytics::tiling::tiling_detection_config_t cfg);

} // namespace drone_follow_app
