// Drone Follow native pipeline — runs YOLOv8s detection on the Hailo15
// ISP frontend and publishes raw detections over ZMQ for the Python
// orchestrator to consume (which runs ByteTracker + the follow control
// loop). Modeled on hailo-analytics/apps/face_landmarks/main.cpp with
// the landmarks subpipeline stripped.

#include <chrono>
#include <condition_variable>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <cxxopts/cxxopts.hpp>
#include <media_library/media_library.hpp>
#include <media_library/media_library_api_types.hpp>
#include "media_library/signal_utils.hpp"

// #include "control_server.hpp"  // TEMP: disabled while bisecting launch failure
#include "drone_follow_pipeline_builder.hpp"
#include "tile_toggler.hpp"
#include "hailo_analytics/analytics/vision.hpp"
#include "hailo_analytics/analytics/analytic_metadata_zmq_sender.hpp"
#include "hailo_analytics/analytics/tiling.hpp"
#include "hailo_analytics/logger/hailo_analytics_logger.hpp"
#include "hailo_analytics/pipeline/core/pipeline.hpp"
#include "hailo_analytics/pipeline/core/pipeline_builder.hpp"

// Pipeline names used by PipelineBuilder.connect_frontend / connect.
#define VISION_PIPELINE "vision_pipeline"
#define DETECTION_PIPELINE "detection_pipeline"
#define ANALYTIC_META_SENDER_PIPELINE "analytic_metadata_sender_pipeline"
#define APP_NAME "df_native_pipeline"

// Forced-flushed debug prints — moved above configure_media_library so
// it's in scope wherever we need it. setvbuf in main() makes stderr
// unbuffered so traces survive abrupt deaths.
#define TRACE(msg) do { std::cerr << "[trace] " << msg << std::endl; std::cerr.flush(); } while (0)

// AI_SINK = the FHD inference sink emitted by the ISP frontend. Same as
// face_landmarks_app since we reuse the medialib config.
#define AI_SINK "sink2"
// 4K vision sink — what the recorder and analytic_viewer consume.
#define VISION_SINK "sink0"
// 720p vision sink — what the on-device Python bridge decodes for the web
// UI's MJPEG element. Cheaper to software-decode than the 4K sink0.
#define UI_SINK "sink1"

// Default medialib config — reuses face_landmarks' since it produces the
// 3-stream ISP output (4K / 720p / FHD-inference) we need.
#define MEDIALIB_CONFIG_PATH "/etc/imaging/cfg/medialib_configs/face_landmarks_medialib_config.json"

// Default ZMQ pub port. Python subscriber binds to tcp://127.0.0.1:<port>.
#define DEFAULT_ZMQ_PORT "7000"

// Default control-server REP port. Python /api/tiles POST forwards tile
// updates here for hot-swap; the C++ side calls
// DynamicTilingCropStage::set_bbox_tiles without rebuilding the pipeline.
#define DEFAULT_CONTROL_PORT "7001"

// Admin-scoped multicast group for sink_0's H264 RTP when --multicast is set.
// Lets multiple consumers (analytic_viewer on the laptop, gst-launch recorder
// on the H15) receive the same stream without an in-pipeline tee/filesink
// branch that would require modifying libhailo_analytics.
#define DEFAULT_MULTICAST_GROUP "239.255.0.50"

struct AppResources
{
    MediaLibraryInterfacePtr media_library;
    hailo_analytics::pipeline::PipelinePtr pipeline;
    // Handle into the dynamic tiling stage living inside ``pipeline``.
    // Kept here so the control thread can ``set_bbox_tiles`` for runtime
    // tile swaps without rebuilding anything.
    std::shared_ptr<drone_follow_app::DynamicTilingCropStage> tiling_stage;
    std::string medialib_config_path;
    std::string zmq_port;
    int timeout_seconds;
    bool use_multicast = false;
    // nullopt = use framework's DEFAULT_TILES (4 quadrants + full).
    // non-empty vector = override.
    std::optional<std::vector<HailoBBox>> bbox_tiles;
    std::string control_port;
    double auto_toggle_period_s = 0.0;  // 0 → disabled
    std::unique_ptr<drone_follow_app::TileToggler> tile_toggler;
};

// Parse a tiles string of the form "x,y,w,h;x,y,w,h;..." into a vector of
// HailoBBox. All values are normalized [0, 1]. Throws on malformed input,
// out-of-range values, or fewer than 2 tiles (the framework's tiling stage
// needs ≥2 tiles to drive the DSP cropper).
static std::vector<HailoBBox> parse_tiles_spec(const std::string &spec)
{
    std::vector<HailoBBox> tiles;
    std::stringstream ss(spec);
    std::string entry;
    while (std::getline(ss, entry, ';')) {
        if (entry.empty()) continue;
        std::stringstream es(entry);
        std::string tok;
        float v[4];
        int i = 0;
        while (std::getline(es, tok, ',')) {
            if (i >= 4) {
                throw std::runtime_error("tile spec '" + entry + "' has > 4 fields");
            }
            try {
                v[i++] = std::stof(tok);
            } catch (const std::exception &) {
                throw std::runtime_error("tile spec '" + entry + "' has non-numeric field");
            }
        }
        if (i != 4) {
            throw std::runtime_error("tile spec '" + entry + "' must be x,y,w,h (got "
                                     + std::to_string(i) + " fields)");
        }
        for (int j = 0; j < 4; ++j) {
            if (!(v[j] >= 0.0f && v[j] <= 1.0f)) {
                throw std::runtime_error("tile spec '" + entry + "' value out of [0,1]");
            }
        }
        if (v[2] <= 0.0f || v[3] <= 0.0f) {
            throw std::runtime_error("tile spec '" + entry + "' has zero width/height");
        }
        if (v[0] + v[2] > 1.0f || v[1] + v[3] > 1.0f) {
            throw std::runtime_error("tile spec '" + entry + "' exceeds frame bounds");
        }
        tiles.push_back(HailoBBox(v[0], v[1], v[2], v[3]));
    }
    if (tiles.size() < 2) {
        throw std::runtime_error("--tiles needs at least 2 tiles (got "
                                 + std::to_string(tiles.size())
                                 + "); single-tile path causes the AI stage to "
                                   "see zero buffers");
    }
    return tiles;
}

static cxxopts::Options build_arg_parser()
{
    cxxopts::Options options("df_native_pipeline",
                             "Hailo15 detection pipeline that publishes detections over ZMQ.");
    options.add_options()
        ("h,help",                "Show this help")
        ("t,timeout",             "Run for N seconds (default: 0 = forever)",
            cxxopts::value<int>()->default_value("0"))
        ("c,config-file-path",    "Media library configuration path",
            cxxopts::value<std::string>()->default_value(MEDIALIB_CONFIG_PATH))
        ("z,zmq-port",            "ZMQ publisher port",
            cxxopts::value<std::string>()->default_value(DEFAULT_ZMQ_PORT))
        ("multicast",             "Send sink0 (4K) and sink1 (720p) H264 RTP to a "
                                  "multicast group (" DEFAULT_MULTICAST_GROUP
                                  ") instead of the default unicast laptop host. "
                                  "Lets multiple consumers share the same stream: "
                                  "on-device recorder + analytic_viewer on sink0, "
                                  "and the on-device Python bridge that feeds the "
                                  "web UI on sink1. analytic_viewer must be invoked "
                                  "with --udp-ip " DEFAULT_MULTICAST_GROUP " to keep "
                                  "working.",
            cxxopts::value<bool>()->default_value("false"))
        ("tiles",                 "Override DEFAULT_TILES with a custom tile list. "
                                  "Format: \"x,y,w,h;x,y,w,h;...\" with values in "
                                  "[0,1] normalized to frame. Need ≥2 tiles (single "
                                  "full-frame tile triggers an AI-stage-sees-zero "
                                  "framework edge case). Omit to use the framework "
                                  "default (4 overlapping 60% quadrants + 1 full).",
            cxxopts::value<std::string>()->default_value(""))
        ("control-port",          "ZMQ REP port for runtime tile reconfiguration. "
                                  "Python /api/tiles POST hot-swaps via this socket "
                                  "without restarting the C++ binary.",
            cxxopts::value<std::string>()->default_value(DEFAULT_CONTROL_PORT))
        ("auto-toggle-tiles",     "If > 0, spawn an internal toggler thread that "
                                  "alternates the active tile geometry between "
                                  "'grid' (4 quadrants + full) and 'full' (5 copies "
                                  "of the full frame) every N seconds, logging the "
                                  "set_bbox_tiles → next-prepare_crops latency to "
                                  "stdout for each swap. Use to characterize "
                                  "application-driven swap latency without a UI.",
            cxxopts::value<double>()->default_value("0"));
    return options;
}

static void configure_media_library(std::shared_ptr<AppResources> app_resources)
{
    TRACE("media_library: local MediaLibrary::create");
    auto media_lib_expected = MediaLibrary::create();
    if (!media_lib_expected.has_value()) {
        throw std::runtime_error("Failed to create media library");
    }
    app_resources->media_library = media_lib_expected.value();
    TRACE("media_library: instance created, calling initialize");
    auto rc = app_resources->media_library->initialize(app_resources->medialib_config_path);
    TRACE("media_library: initialize returned");
    if (rc != media_library_return::MEDIA_LIBRARY_SUCCESS) {
        throw std::runtime_error("Failed to initialize media library with config: " +
                                 app_resources->medialib_config_path);
    }
    TRACE("media_library: initialize OK");
}

static void create_pipeline(std::shared_ptr<AppResources> app_resources)
{
    auto output_streams = app_resources->media_library->get_frontend_output_streams();
    if (!output_streams.has_value()) {
        throw std::runtime_error("Failed to get frontend output streams");
    }

    // Vision pipeline owns the ISP frontend. We erase AI_SINK from the
    // auto-generated config so the framework doesn't connect it to a
    // default encoder; our detection pipeline consumes that sink directly.
    auto vision_config = hailo_analytics::analytics::vision::base_vision_config(
        output_streams.value(), /*base_port=*/5000);
    vision_config.outputs.erase(AI_SINK);
    if (app_resources->use_multicast) {
        // GStreamer's udpsink auto-detects 224.0.0.0/4 destinations and switches
        // to multicast send. Multiple udpsrc consumers can then join the group:
        //   - sink0 (4K) → recorder (gst-launch on H15) + analytic_viewer (laptop)
        //   - sink1 (720p) → in-process Python bridge that decodes H264 →
        //     JPEG and feeds the web UI's MJPEG element
        // Both sinks use the same group; port disambiguates (sink0=5000,
        // sink1=5002 per port_from_stream_id base+n*2).
        vision_config.outputs[VISION_SINK].udp_config.host = DEFAULT_MULTICAST_GROUP;
        vision_config.outputs[UI_SINK].udp_config.host = DEFAULT_MULTICAST_GROUP;
        HAILO_ANALYTICS_LOG_INFO(
            "Multicast mode: sink0 → {}:5000, sink1 → {}:5002",
            DEFAULT_MULTICAST_GROUP, DEFAULT_MULTICAST_GROUP);
    }
    auto vision_pipeline_status = hailo_analytics::analytics::vision::generate_vision_pipeline(
        app_resources->media_library, VISION_PIPELINE, vision_config);
    if (!vision_pipeline_status.has_value()) {
        throw std::runtime_error("Failed to create vision pipeline");
    }

    // BISECTION TEST: use the framework's generate_tiling_detection_pipeline
    // directly (static TilingCropStage) instead of our manual assembly with
    // DynamicTilingCropStage. If this binary launches and produces detections,
    // our manual-assembly code is what regressed the launch path.
    auto det_cfg = drone_follow_app::default_detection_config(app_resources->bbox_tiles);
    auto det_pipeline_status =
        hailo_analytics::analytics::tiling::generate_tiling_detection_pipeline(
            DETECTION_PIPELINE, det_cfg);
    if (!det_pipeline_status.has_value()) {
        throw std::runtime_error("Failed to create framework detection pipeline");
    }
    auto detection_pipeline = det_pipeline_status.value();
    // tiling_stage handle isn't available with the framework path — set to null.
    app_resources->tiling_stage = nullptr;

    // ZMQ sender — pub-bind on the configured port. Python subscribes.
    hailo_analytics::analytics::analytic_metadata_zmq_sender::analytic_metadata_zmq_sender_config_t
        sender_config;
    sender_config.analytic_metadata_config.queue_size = 1;
    sender_config.zeromq_config.queue_size = 1;
    sender_config.zeromq_config.pub_address = "tcp://*:" + app_resources->zmq_port;
    auto sender_pipeline_status =
        hailo_analytics::analytics::analytic_metadata_zmq_sender::generate_analytic_metadata_zmq_sender_pipeline(
            ANALYTIC_META_SENDER_PIPELINE, sender_config);
    if (!sender_pipeline_status.has_value()) {
        throw std::runtime_error("Failed to create ZMQ sender pipeline");
    }

    // Chain: vision (ISP sink2) → detection → ZMQ sender
    hailo_analytics::pipeline::PipelineBuilder pip_builder;
    pip_builder
        .add_stage(vision_pipeline_status.value(), hailo_analytics::pipeline::StageType::SOURCE)
        .add_stage(detection_pipeline)
        .add_stage(sender_pipeline_status.value(), hailo_analytics::pipeline::StageType::SINK);
    pip_builder.connect_frontend(VISION_PIPELINE, AI_SINK, DETECTION_PIPELINE);
    pip_builder.connect(DETECTION_PIPELINE, ANALYTIC_META_SENDER_PIPELINE);

    app_resources->pipeline = pip_builder.build(APP_NAME, true);
}

static std::mutex g_stop_mutex;
static std::condition_variable g_stop_cv;

int main(int argc, char *argv[])
{
    TRACE("main:enter");
    setvbuf(stderr, nullptr, _IONBF, 0);
    setvbuf(stdout, nullptr, _IONBF, 0);

    TRACE("main:make_shared AppResources");
    auto app_resources = std::make_shared<AppResources>();

    TRACE("main:build_arg_parser");
    cxxopts::Options options = build_arg_parser();
    TRACE("main:options.parse");
    auto result = options.parse(argc, argv);
    if (result.count("help")) {
        std::cout << options.help() << std::endl;
        return 0;
    }
    TRACE("main:reading options into app_resources");
    app_resources->medialib_config_path = result["config-file-path"].as<std::string>();
    app_resources->zmq_port = result["zmq-port"].as<std::string>();
    app_resources->timeout_seconds = result["timeout"].as<int>();
    app_resources->use_multicast = result["multicast"].as<bool>();
    const auto tiles_spec = result["tiles"].as<std::string>();
    if (!tiles_spec.empty()) {
        app_resources->bbox_tiles = parse_tiles_spec(tiles_spec);
        HAILO_ANALYTICS_LOG_INFO("Custom tile config: {} tiles", app_resources->bbox_tiles->size());
    }
    app_resources->control_port = result["control-port"].as<std::string>();
    app_resources->auto_toggle_period_s = result["auto-toggle-tiles"].as<double>();

    TRACE("main:signal_handler setup");
    signal_utils::SignalHandler signal_handler(false);
    signal_handler.register_signal_handler([](int /*signal*/) {
        HAILO_ANALYTICS_LOG_INFO("Stopping pipeline...");
        g_stop_cv.notify_all();
    });

    TRACE("main:configure_media_library");
    configure_media_library(app_resources);
    TRACE("main:create_pipeline");
    create_pipeline(app_resources);

    TRACE("main:pipeline->start about to call");
    HAILO_ANALYTICS_LOG_INFO("Starting pipeline. Publishing detections on tcp://*:{}",
                             app_resources->zmq_port);
    app_resources->pipeline->start();
    TRACE("main:pipeline->start returned");

    // TEMP: ControlServer disabled while diagnosing launch failure.
    // (If detections come back without it but not with it, the second
    // ZMQ context next to the framework's metadata_zmq_sender is the
    // culprit.)

    // Application-driven tile toggling for latency characterization.
    // Started after pipeline->start() so the cropper is already ticking
    // when the first swap happens — gives a fair "set_bbox_tiles → next
    // prepare_crops" measurement.
    if (app_resources->auto_toggle_period_s > 0.0) {
        app_resources->tile_toggler =
            std::make_unique<drone_follow_app::TileToggler>(
                app_resources->tiling_stage,
                app_resources->auto_toggle_period_s);
        app_resources->tile_toggler->start();
    }

    std::unique_lock<std::mutex> lk(g_stop_mutex);
    if (app_resources->timeout_seconds > 0) {
        g_stop_cv.wait_for(lk, std::chrono::seconds(app_resources->timeout_seconds));
    } else {
        g_stop_cv.wait(lk);
    }

    HAILO_ANALYTICS_LOG_INFO("Stopping pipeline.");
    if (app_resources->tile_toggler) {
        app_resources->tile_toggler->stop();
        app_resources->tile_toggler.reset();
    }
    app_resources->pipeline->stop();
    app_resources->media_library->shutdown();
    return 0;
}
