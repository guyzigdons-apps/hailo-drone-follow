#include "drone_follow_pipeline_builder.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

#include "hailo_analytics/analytics/ai_models_config.hpp"
#include "hailo_analytics/analytics/common_configs.hpp"
#include "hailo_analytics/analytics/detection.hpp"
#include "hailo_analytics/pipeline/ai/detection_tracker_stage.hpp"
#include "hailo_analytics/pipeline/core/pipeline_builder.hpp"
#include "hailo_analytics/pipeline/cropping/aggregator_stage.hpp"

namespace drone_follow_app
{

namespace tiling = hailo_analytics::analytics::tiling;
namespace detection = hailo_analytics::analytics::detection;
namespace cropping = hailo_analytics::pipeline::cropping;
namespace ai_stages = hailo_analytics::pipeline::ai;
namespace ai_models = hailo_analytics::analytics::ai_models;
namespace pipeline = hailo_analytics::pipeline;

tiling::tiling_detection_config_t default_detection_config(
    std::optional<std::vector<HailoBBox>> bbox_tiles)
{
    auto cfg = tiling::base_config();

    // YOLOv8s 384x640 — preset bound to /home/root/apps/shared/resources/
    // hailo_yolov8s_384_640.hef on the H15 image.
    ai_models::apply_to(ai_models::YOLOV8S, cfg.detection_config);
    cfg.detection_config.ai_config.use_hailort_service = false;

    if (bbox_tiles.has_value() && !bbox_tiles->empty()) {
        cfg.tiling_config.bbox_tiles = bbox_tiles;
        // Batch inference across all tiles of a frame: set the NPU batch size
        // to the tile count so HailoRT groups the per-frame crops into one
        // scheduled batch instead of N separately-scheduled jobs. This also
        // fixes a mismatch — the framework default (5) assumed the old 5-tile
        // grid, so with a different tile count it never filled a batch cleanly.
        // NOTE: on the Hailo dataflow NPU this amortizes scheduling overhead;
        // it does not parallelize compute (total inference work stays ~linear
        // in tile count). Capped so a large region set can't exceed the HEF's
        // supported batch and fail configuration at startup.
        cfg.detection_config.ai_config.batch_size =
            std::min(static_cast<int>(bbox_tiles->size()), 8);
    }
    cfg.tiling_config.queue_size = 2;
    cfg.aggregator_config.main_queue_size = 3;

    // Tracker disabled — Python's ByteTracker runs downstream of the ZMQ
    // pub. Running a second tracker on the C++ side would assign IDs that
    // conflict with the Python ones.
    cfg.tracker_config.enabled = false;

    return cfg;
}

DroneFollowDetectionPipeline build_drone_follow_detection_pipeline(
    const std::string &pipeline_name,
    tiling::tiling_detection_config_t cfg)
{
    // The merge below mirrors generate_tiling_detection_pipeline:
    // start from framework defaults, overlay caller's fields, then build.
    // We bypass `generate_tiling_detection_pipeline` itself because it
    // instantiates the static TilingCropStage internally — we need to
    // substitute our DynamicTilingCropStage at that step.
    tiling::tiling_detection_config_t base = tiling::base_config();
    base.merge_from(cfg);
    cfg = std::move(base);

    if (cfg.tracker_config.enabled.value_or(false)) {
        // Same fixup the framework does — preserve sub-frame tensor metadata
        // through the aggregator so the tracker knows which frames had AI.
        cfg.aggregator_config.copy_sub_frame_tensor_to_metadata = true;
    }

    // Reach into the merged config for the values we need to construct
    // DynamicTilingCropStage. Because base_config() populates every field,
    // these optional<>s are guaranteed to hold values after the merge.
    auto initial_tiles = cfg.tiling_config.bbox_tiles.value();
    if (initial_tiles.size() < 2) {
        throw std::invalid_argument(
            "build_drone_follow_detection_pipeline: tile geometry needs "
            "≥2 tiles (single full-frame tile triggers the framework's "
            "AI-stage-sees-zero edge case)");
    }
    const size_t initial_tile_count = initial_tiles.size();

    // Construct our DynamicTilingCropStage in place of TilingCropStage.
    // The constructor mirrors TilingCropStageBuild::buildptr (tiling_stage.cpp:188)
    // — same arguments, same order. Parameters that flow through the
    // tiling_config map 1:1.
    auto dyn_tiling_stage = std::make_shared<DynamicTilingCropStage>(
        cfg.tiling_config.stage_name.value(),
        cfg.tiling_config.output_pool_size.value(),
        cfg.tiling_config.input_width.value(),
        cfg.tiling_config.input_height.value(),
        cfg.tiling_config.output_width.value(),
        cfg.tiling_config.output_height.value(),
        cfg.tiling_config.main_sub_name.value(),
        cfg.tiling_config.sub_sub_name.value(),
        initial_tiles,
        cfg.tiling_config.queue_size.value(),
        cfg.tiling_config.leaky.value_or(false),
        cfg.tiling_config.trace.value_or(true),
        cfg.tiling_config.pool_mode.value_or(pipeline::StagePoolMode::FAIL_ON_EMPTY_POOL),
        cfg.tiling_config.crop_every_x_frames.value_or(1));

    // Detection sub-pipeline (NPU inference + post-process). Framework
    // helper still does the right thing here — we only needed to replace
    // the tiling stage, not the detection chain.
    auto detection_pipeline_result = detection::generate_detection_pipeline(
        std::string(tiling::DETECTION_SUBPIPELINE), cfg.detection_config);
    if (!detection_pipeline_result.has_value()) {
        throw std::runtime_error(
            "build_drone_follow_detection_pipeline: detection sub-pipeline "
            "creation failed");
    }
    pipeline::PipelinePtr detection_pipeline = detection_pipeline_result.value();

    // Aggregator stage. CRITICAL: we set_static_subframes_opt to the
    // initial tile count. This locks the aggregator's expected-subframe
    // bookkeeping to N for the pipeline's lifetime. Tile geometries can
    // change via DynamicTilingCropStage::set_bbox_tiles at any time, but
    // the count cannot — set_bbox_tiles enforces the invariant.
    cropping::AggregatorStageBuild::Builder agg_builder =
        cropping::AggregatorStageBuild::create();
    cfg.apply_to(agg_builder);
    agg_builder.set_static_subframes_opt(static_cast<int>(initial_tile_count));
    auto aggregator_stage = agg_builder.buildptr();

    // Pipeline graph (matches framework's generate_tiling_detection_pipeline
    // wiring): tiling → aggregator main inlet, tiling → detection
    // sub-pipeline, detection → aggregator sub inlet.
    pipeline::PipelineBuilder pip_builder;
    pip_builder.add_stage(dyn_tiling_stage)
        .add_stage(detection_pipeline)
        .add_stage(aggregator_stage);

    pip_builder.connect(cfg.tiling_config.stage_name.value(),
                        cfg.aggregator_config.stage_name.value());
    pip_builder.connect(cfg.tiling_config.stage_name.value(),
                        detection_pipeline->get_name());
    pip_builder.connect(detection_pipeline->get_name(),
                        cfg.aggregator_config.stage_name.value());

    // No tracker stage — disabled by default_detection_config + Python
    // runs ByteTracker downstream. The framework's helper has a branch
    // for it; we don't need it.

    pipeline::PipelinePtr pipe = pip_builder.build(pipeline_name, true);
    pipe->set_in_stage(dyn_tiling_stage);
    pipe->set_out_stage(aggregator_stage);

    return DroneFollowDetectionPipeline{
        .pipeline = pipe,
        .tiling_stage = dyn_tiling_stage,
    };
}

} // namespace drone_follow_app
