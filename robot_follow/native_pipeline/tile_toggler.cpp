#include "tile_toggler.hpp"

#include <chrono>
#include <cstdio>
#include <utility>
#include <vector>

#include "hailo_analytics/analytics/tiling.hpp"
#include "hailo_analytics/logger/hailo_analytics_logger.hpp"

namespace drone_follow_app
{

namespace
{

// 5-tile presets matching the aggregator's static_subframes count.
const std::vector<HailoBBox> &grid_preset()
{
    // Same as the framework's DEFAULT_TILES (tiling.hpp): four 60%×60%
    // quadrants overlapping by 20%, plus a full-frame tile.
    static const std::vector<HailoBBox> g = {
        HailoBBox(0.0f, 0.0f, 0.6f, 0.6f),
        HailoBBox(0.4f, 0.0f, 0.6f, 0.6f),
        HailoBBox(0.0f, 0.4f, 0.6f, 0.6f),
        HailoBBox(0.4f, 0.4f, 0.6f, 0.6f),
        HailoBBox(0.0f, 0.0f, 1.0f, 1.0f),
    };
    return g;
}

const std::vector<HailoBBox> &full_preset()
{
    // 5 distinct large tiles, each ≥98% of the frame. Semantically
    // ≈ "full-frame only" inference (each tile sees essentially the
    // whole image), but NO two tiles are byte-identical — earlier
    // experiment with 5 identical 1.0×1.0 tiles caused the cropper's
    // worker thread to wedge after the first swap (no further
    // prepare_crops calls). Likely the framework's aggregator/NMS or
    // DSP buffer pool doesn't tolerate fully-redundant tile rectangles.
    static const std::vector<HailoBBox> g = {
        HailoBBox(0.00f, 0.00f, 1.00f, 1.00f),  // full
        HailoBBox(0.00f, 0.00f, 0.99f, 0.99f),  // near-full from origin
        HailoBBox(0.01f, 0.01f, 0.99f, 0.99f),  // near-full both shifted
        HailoBBox(0.00f, 0.01f, 0.99f, 0.99f),  // near-full v-shifted
        HailoBBox(0.01f, 0.00f, 0.99f, 0.99f),  // near-full h-shifted
    };
    return g;
}

} // namespace

TileToggler::TileToggler(std::shared_ptr<DynamicTilingCropStage> stage,
                         double period_seconds)
    : m_stage(std::move(stage)),
      m_period_seconds(period_seconds),
      m_stop(false)
{
}

TileToggler::~TileToggler()
{
    stop();
}

void TileToggler::start()
{
    if (m_thread.joinable()) return;
    m_stop.store(false);
    m_thread = std::thread([this]() { run(); });
}

void TileToggler::stop()
{
    m_stop.store(true);
    if (m_thread.joinable()) {
        m_thread.join();
    }
}

void TileToggler::run()
{
    HAILO_ANALYTICS_LOG_INFO(
        "[toggler] starting: period={}s, toggling grid <-> full",
        m_period_seconds);

    int swap_no = 0;
    bool next_is_grid = true;
    while (!m_stop.load()) {
        const char *config_name = next_is_grid ? "grid" : "full";
        const auto &preset = next_is_grid ? grid_preset() : full_preset();
        next_is_grid = !next_is_grid;
        swap_no++;

        // Snapshot the observed_epoch BEFORE the swap. The toggler then
        // waits for it to advance, which signals "the next prepare_crops
        // saw the new pointer and rebuilt its ROI cache" — i.e., DSP
        // cropping is now using the new geometry.
        const uint64_t before_observed = m_stage->observed_epoch();

        const auto t0 = std::chrono::steady_clock::now();
        try {
            m_stage->set_bbox_tiles(preset);
        } catch (const std::exception &e) {
            HAILO_ANALYTICS_LOG_ERROR("[toggler] set_bbox_tiles failed: {}", e.what());
            std::this_thread::sleep_for(std::chrono::duration<double>(m_period_seconds));
            continue;
        }

        // Spin-wait (cheap; just a few atomic loads) until prepare_crops
        // picks up the new snapshot. Cap at 500 ms — if exceeded, the
        // pipeline is stalled upstream (no frames flowing into the
        // cropper) and we log the timeout instead of hanging the toggler.
        constexpr auto kMaxWait = std::chrono::milliseconds(500);
        while (!m_stop.load() &&
               m_stage->observed_epoch() == before_observed &&
               std::chrono::steady_clock::now() - t0 < kMaxWait) {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
        const auto t1 = std::chrono::steady_clock::now();
        const auto elapsed_us =
            std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
        const uint64_t after_observed = m_stage->observed_epoch();
        const bool picked_up = (after_observed > before_observed);

        // stdout for easy grepping ("[toggler] swap …"). Format chosen
        // to be parseable by a shell awk one-liner for quick stats.
        std::printf("[toggler] swap %d  config=%s  latency_us=%lld  "
                    "picked_up=%d  observed=%llu\n",
                    swap_no, config_name, (long long)elapsed_us,
                    picked_up ? 1 : 0,
                    (unsigned long long)after_observed);
        std::fflush(stdout);

        // Sleep the rest of the period, modulo the time we already
        // consumed waiting for the swap to be picked up.
        const auto period_d = std::chrono::duration<double>(m_period_seconds);
        const auto remainder = period_d -
            std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
        if (remainder.count() > 0.0 && !m_stop.load()) {
            // Sleep in slices so stop() is honored within ~100 ms.
            const auto deadline = std::chrono::steady_clock::now() + remainder;
            while (!m_stop.load() &&
                   std::chrono::steady_clock::now() < deadline) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
    }
    HAILO_ANALYTICS_LOG_INFO("[toggler] stopped after {} swaps", swap_no);
}

} // namespace drone_follow_app
