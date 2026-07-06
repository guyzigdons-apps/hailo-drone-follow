#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include "dynamic_tiling_stage.hpp"

namespace drone_follow_app
{

// A ZMQ REP control loop that accepts in-process tile reconfiguration
// requests and forwards them to a DynamicTilingCropStage at runtime.
//
// Wire format (JSON):
//   request:  {"cmd": "set_tiles", "tiles": [[x,y,w,h], [x,y,w,h], ...]}
//             values normalized [0,1]; tile count must match the
//             pipeline's static_subframes count (locked at startup).
//   request:  {"cmd": "get_tiles"}
//             returns the current tile geometry.
//   response: {"ok": true, "tiles": [[x,y,w,h], ...]} on success
//             {"ok": false, "error": "..."}             on failure
//
// All requests are processed on the control thread; readers/writers of
// DynamicTilingCropStage's tile snapshot are synchronized internally.
class ControlServer
{
public:
    // ``endpoint`` is a ZMQ bind string, e.g. ``tcp://*:7001``.
    ControlServer(std::shared_ptr<DynamicTilingCropStage> stage, std::string endpoint);
    ~ControlServer();

    ControlServer(const ControlServer &) = delete;
    ControlServer &operator=(const ControlServer &) = delete;

    void start();
    void stop();

private:
    void run();

    std::shared_ptr<DynamicTilingCropStage> m_stage;
    std::string m_endpoint;
    std::thread m_thread;
    std::atomic<bool> m_stop;
};

} // namespace drone_follow_app
