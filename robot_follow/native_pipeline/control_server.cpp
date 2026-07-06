#include "control_server.hpp"

#include <chrono>
#include <exception>
#include <vector>

#include <nlohmann/json.hpp>
#include <zmq.hpp>

#include "hailo_analytics/logger/hailo_analytics_logger.hpp"

namespace drone_follow_app
{

using json = nlohmann::json;

ControlServer::ControlServer(std::shared_ptr<DynamicTilingCropStage> stage, std::string endpoint)
    : m_stage(std::move(stage)),
      m_endpoint(std::move(endpoint)),
      m_stop(false)
{
}

ControlServer::~ControlServer()
{
    stop();
}

void ControlServer::start()
{
    if (m_thread.joinable()) {
        return;
    }
    m_stop.store(false);
    m_thread = std::thread([this]() { run(); });
}

void ControlServer::stop()
{
    m_stop.store(true);
    if (m_thread.joinable()) {
        m_thread.join();
    }
}

// Parse a tile descriptor from JSON. Accepts two shapes for friendliness
// with the Python orchestrator (which currently sends list-of-list) and
// the SSE feed (list of {name,x,y,w,h} dicts):
//   - [x, y, w, h]                         (compact)
//   - {"x":..., "y":..., "w":..., "h":...} (named; name field ignored here)
// Throws std::runtime_error on malformed input.
static HailoBBox parse_tile(const json &j)
{
    auto coord = [&](const char *key, std::size_t idx) {
        if (j.is_array()) return j.at(idx).get<float>();
        return j.at(key).get<float>();
    };
    float x = coord("x", 0);
    float y = coord("y", 1);
    float w = coord("w", 2);
    float h = coord("h", 3);
    if (x < 0.0f || y < 0.0f || w <= 0.0f || h <= 0.0f ||
        x + w > 1.0f + 1e-6f || y + h > 1.0f + 1e-6f) {
        throw std::runtime_error(
            "tile out of bounds [0,1] or zero-sized: " + j.dump());
    }
    return HailoBBox(x, y, w, h);
}

static std::string current_tiles_to_json(const DynamicTilingCropStage &stage)
{
    json out;
    out["ok"] = true;
    json tiles_out = json::array();
    for (const auto &t : stage.get_bbox_tiles()) {
        tiles_out.push_back({t.xmin(), t.ymin(), t.width(), t.height()});
    }
    out["tiles"] = tiles_out;
    return out.dump();
}

void ControlServer::run()
{
    zmq::context_t ctx(1);
    zmq::socket_t sock(ctx, zmq::socket_type::rep);
    // 200 ms poll so stop() can be honored within roughly a heartbeat
    // without holding the socket open indefinitely.
    sock.set(zmq::sockopt::rcvtimeo, 200);
    sock.set(zmq::sockopt::linger, 0);
    try {
        sock.bind(m_endpoint);
    } catch (const zmq::error_t &e) {
        HAILO_ANALYTICS_LOG_ERROR("[control] bind to {} failed: {}",
                                  m_endpoint, e.what());
        return;
    }
    HAILO_ANALYTICS_LOG_INFO("[control] REP socket listening on {}", m_endpoint);

    while (!m_stop.load()) {
        zmq::message_t request;
        zmq::recv_result_t recv_result;
        try {
            recv_result = sock.recv(request, zmq::recv_flags::none);
        } catch (const zmq::error_t &e) {
            HAILO_ANALYTICS_LOG_WARN("[control] recv error: {}", e.what());
            continue;
        }
        if (!recv_result) {
            // Timeout — just loop back so we can check m_stop.
            continue;
        }

        std::string reply_str;
        try {
            json req = json::parse(
                std::string(static_cast<char *>(request.data()), request.size()));
            std::string cmd = req.value("cmd", "");

            if (cmd == "set_tiles") {
                std::vector<HailoBBox> new_tiles;
                for (const auto &j : req.at("tiles")) {
                    new_tiles.push_back(parse_tile(j));
                }
                m_stage->set_bbox_tiles(std::move(new_tiles));
                reply_str = current_tiles_to_json(*m_stage);
            } else if (cmd == "get_tiles") {
                reply_str = current_tiles_to_json(*m_stage);
            } else {
                reply_str = json{{"ok", false},
                                 {"error", "unknown cmd: " + cmd}}.dump();
            }
        } catch (const std::exception &e) {
            // Catch JSON parse errors, set_bbox_tiles invariants, etc. The
            // REP socket requires a reply for every recv — we MUST send
            // back something, even on error, or the next request hangs.
            reply_str = json{{"ok", false}, {"error", e.what()}}.dump();
        }

        try {
            sock.send(zmq::buffer(reply_str), zmq::send_flags::none);
        } catch (const zmq::error_t &e) {
            HAILO_ANALYTICS_LOG_WARN("[control] send error: {}", e.what());
        }
    }
    HAILO_ANALYTICS_LOG_INFO("[control] stopped");
}

} // namespace drone_follow_app
