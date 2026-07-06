#pragma once

#include <atomic>
#include <memory>
#include <thread>

#include "dynamic_tiling_stage.hpp"

namespace drone_follow_app
{

// Application-driven tile-toggle thread. Periodically alternates the
// DynamicTilingCropStage's active tiles between two presets and measures
// the latency from set_bbox_tiles() → first prepare_crops() that consumed
// the new geometry. Pairs with --auto-toggle-tiles SECONDS on the CLI.
//
// The two presets are sized to keep the tile count constant (the
// aggregator's static_subframes_opt is locked at startup):
//
//   "grid"  — 4 overlapping 60% quadrants + 1 full frame (DEFAULT_TILES)
//   "full"  — 5 copies of the full frame; the aggregator's NMS dedupes
//             the redundant detections so the effective view is single-
//             frame inference. Inefficient but unambiguous about
//             "matches the no-tile-cropping path".
//
// Latency log line on stdout, one per swap:
//   [toggler] swap N→M  config=<name>  latency=<ms>  observed=<epoch>
class TileToggler
{
public:
    TileToggler(std::shared_ptr<DynamicTilingCropStage> stage,
                double period_seconds);
    ~TileToggler();

    TileToggler(const TileToggler &) = delete;
    TileToggler &operator=(const TileToggler &) = delete;

    void start();
    void stop();

private:
    void run();

    std::shared_ptr<DynamicTilingCropStage> m_stage;
    double m_period_seconds;
    std::thread m_thread;
    std::atomic<bool> m_stop;
};

} // namespace drone_follow_app
