import { useState, useEffect, useRef, useCallback } from "react";

const LOG_POLL_INTERVAL = 500; // ms
const DEBOUNCE_MS = 250;
// Dead-zone radius for overlay rendering, in normalized frame coords.
// While the target centre stays within this distance of the rendered
// position, rendering doesn't move — kills frame-to-frame jitter.
const OVERLAY_DEAD_ZONE = 0.01;
// Per-RAF lerp factor pulling the rendered position toward the target.
const OVERLAY_SMOOTH_ALPHA = 0.5;
const CROSS_HALF_SIZE = 10; // pixels, half the arm length of the cross
const CROSS_STROKE = 3;
const CROSS_HALO_STROKE = 6;

export default function App() {
  const [detections, setDetections] = useState([]);
  const [followingId, setFollowingId] = useState(null);
  const [velocity, setVelocity] = useState(null);
  const [videoDims, setVideoDims] = useState({ width: 0, height: 0 });
  const [logsOpen, setLogsOpen] = useState(true);
  const [logs, setLogs] = useState([]);
  const [config, setConfig] = useState(null);
  const [recording, setRecording] = useState(false);
  const [perf, setPerf] = useState(null);
  const [paused, setPaused] = useState(false);
  const [diag, setDiag] = useState(null);
  const canvasRef = useRef(null);
  const debounceRef = useRef(null);
  const logSinceRef = useRef(0);
  const logEndRef = useRef(null);
  // Per-id latest target bbox (cx, cy, w, h) and smoothly-interpolated
  // rendered bbox, both in normalized coords. RAF lerps rendered → target.
  const targetBoxesRef = useRef(new Map());
  const renderedBoxesRef = useRef(new Map());
  // Mirror of the latest SSE detections so handleFollow can resolve the bbox's
  // current id at click time (React closure binds det.id at render time, which
  // goes stale when ByteTracker re-IDs the same target between SSE and click).
  const detectionsRef = useRef([]);
  // Bumped by the RAF loop to trigger SVG re-renders between detection events.
  const [, setSmoothTick] = useState(0);

  // Frame-synced detections via SSE (replaces polling)
  useEffect(() => {
    const es = new EventSource("/api/detections/stream");
    es.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data);
        // Extract diagnostic entry (if present) from detections list
        const rawDets = data.detections || [];
        const diagIdx = rawDets.findIndex((d) => d._diag);
        if (diagIdx >= 0) {
          setDiag(rawDets[diagIdx]);
          rawDets.splice(diagIdx, 1);
        }
        setDetections(rawDets);
        setFollowingId(data.following_id);
        setVelocity(data.velocity || null);
        setPerf(data.perf || null);
        setPaused(data.paused ?? false);
      } catch {
        // malformed event
      }
    };
    return () => es.close();
  }, []);

  // Poll recording status
  useEffect(() => {
    let active = true;
    const poll = async () => {
      while (active) {
        try {
          const res = await fetch("/api/status");
          if (res.ok) {
            const data = await res.json();
            if (data.recording !== undefined) setRecording(data.recording);
          }
        } catch {
          // server not ready
        }
        await new Promise((r) => setTimeout(r, 1000));
      }
    };
    poll();
    return () => { active = false; };
  }, []);

  // Fetch config on mount, and refresh on every lock/unlock so the
  // Target Size slider tracks the bbox setpoint that the server captures
  // at lock time (manual click *and* AUTO acquisition both rewrite
  // controller_config.target_bbox_height under the hood).
  useEffect(() => {
    let aborted = false;
    fetch("/api/config")
      .then((r) => (r.ok ? r.json() : null))
      .then((data) => {
        if (data && !aborted) setConfig(data);
      })
      .catch(() => {});
    return () => { aborted = true; };
  }, [followingId]);

  // Poll logs
  useEffect(() => {
    if (!logsOpen) return;
    let active = true;
    const poll = async () => {
      while (active) {
        try {
          const res = await fetch(`/api/logs?since_id=${logSinceRef.current}`);
          if (res.ok) {
            const data = await res.json();
            if (data.logs && data.logs.length > 0) {
              logSinceRef.current = data.logs[data.logs.length - 1].id;
              setLogs((prev) => {
                const next = [...prev, ...data.logs];
                return next.length > 200 ? next.slice(-200) : next;
              });
            }
          }
        } catch {
          // server not ready
        }
        await new Promise((r) => setTimeout(r, LOG_POLL_INTERVAL));
      }
    };
    poll();
    return () => {
      active = false;
    };
  }, [logsOpen]);

  // Auto-scroll logs
  useEffect(() => {
    if (logEndRef.current) {
      logEndRef.current.scrollIntoView({ behavior: "smooth" });
    }
  }, [logs]);

  // Push the latest detection bboxes into the target map; prune ids that
  // disappeared from this frame. Newly-seen ids seed the rendered box at
  // the target so they don't glide in from the previous position.
  useEffect(() => {
    const targets = targetBoxesRef.current;
    const rendered = renderedBoxesRef.current;
    const seen = new Set();
    for (const det of detections) {
      if (det.id == null) continue;
      seen.add(det.id);
      const cx = det.bbox.x + det.bbox.w / 2;
      const cy = det.bbox.y + det.bbox.h / 2;
      targets.set(det.id, { cx, cy, w: det.bbox.w, h: det.bbox.h });
      if (!rendered.has(det.id)) {
        rendered.set(det.id, { cx, cy, w: det.bbox.w, h: det.bbox.h });
      }
    }
    for (const k of [...targets.keys()]) if (!seen.has(k)) targets.delete(k);
    for (const k of [...rendered.keys()]) if (!seen.has(k)) rendered.delete(k);
  }, [detections]);

  // RAF loop: lerp rendered boxes toward targets; bump tick to re-render.
  // Centre movement below the dead-zone is ignored so the overlay parks.
  useEffect(() => {
    let raf = 0;
    const tick = () => {
      const targets = targetBoxesRef.current;
      const rendered = renderedBoxesRef.current;
      let dirty = false;
      for (const [id, t] of targets) {
        const cur = rendered.get(id);
        if (!cur) {
          rendered.set(id, { ...t });
          dirty = true;
          continue;
        }
        const dx = t.cx - cur.cx;
        const dy = t.cy - cur.cy;
        if (dx * dx + dy * dy >= OVERLAY_DEAD_ZONE * OVERLAY_DEAD_ZONE) {
          cur.cx += dx * OVERLAY_SMOOTH_ALPHA;
          cur.cy += dy * OVERLAY_SMOOTH_ALPHA;
          dirty = true;
        }
        const dw = t.w - cur.w;
        const dh = t.h - cur.h;
        if (Math.abs(dw) > 1e-4 || Math.abs(dh) > 1e-4) {
          cur.w += dw * OVERLAY_SMOOTH_ALPHA;
          cur.h += dh * OVERLAY_SMOOTH_ALPHA;
          dirty = true;
        }
      }
      if (dirty) setSmoothTick((n) => (n + 1) & 0xffff);
      raf = requestAnimationFrame(tick);
    };
    raf = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(raf);
  }, []);

  // MJPEG stream via fetch + ReadableStream → canvas
  useEffect(() => {
    const controller = new AbortController();
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext("2d");

    const BOUNDARY = "--frame\r\n";
    const HEADER_END = "\r\n\r\n";

    (async () => {
      try {
        const res = await fetch("/api/video", { signal: controller.signal });
        const reader = res.body.getReader();
        let buffer = new Uint8Array(0);

        const concat = (a, b) => {
          const c = new Uint8Array(a.length + b.length);
          c.set(a);
          c.set(b, a.length);
          return c;
        };

        const indexOf = (buf, pattern) => {
          const enc = new TextEncoder();
          const pat = enc.encode(pattern);
          outer: for (let i = 0; i <= buf.length - pat.length; i++) {
            for (let j = 0; j < pat.length; j++) {
              if (buf[i + j] !== pat[j]) continue outer;
            }
            return i;
          }
          return -1;
        };

        while (true) {
          const { done, value } = await reader.read();
          if (done) break;
          buffer = concat(buffer, value);

          // Process all complete frames in the buffer
          while (true) {
            // Find the first boundary
            const bStart = indexOf(buffer, BOUNDARY);
            if (bStart === -1) break;

            // Find the end of headers after the boundary
            const headerStart = bStart + BOUNDARY.length;
            const headerEnd = indexOf(
              buffer.subarray(headerStart),
              HEADER_END
            );
            if (headerEnd === -1) break;

            const jpegStart = headerStart + headerEnd + HEADER_END.length;

            // Find the next boundary to determine the end of JPEG data
            const nextBoundary = indexOf(
              buffer.subarray(jpegStart),
              BOUNDARY
            );
            if (nextBoundary === -1) break;

            // Extract JPEG bytes (strip trailing \r\n before next boundary)
            let jpegEnd = jpegStart + nextBoundary;
            while (jpegEnd > jpegStart && (buffer[jpegEnd - 1] === 0x0a || buffer[jpegEnd - 1] === 0x0d)) {
              jpegEnd--;
            }
            const jpegData = buffer.slice(jpegStart, jpegEnd);

            // Advance buffer past the current frame (keep from next boundary)
            buffer = buffer.slice(jpegStart + nextBoundary);

            // Draw the frame
            try {
              const blob = new Blob([jpegData], { type: "image/jpeg" });
              const bmp = await createImageBitmap(blob);
              if (canvas.width !== bmp.width || canvas.height !== bmp.height) {
                canvas.width = bmp.width;
                canvas.height = bmp.height;
                setVideoDims({ width: bmp.width, height: bmp.height });
              }
              ctx.drawImage(bmp, 0, 0);
              bmp.close();
            } catch {
              // skip corrupt frame
            }
          }
        }
      } catch (err) {
        if (err.name !== "AbortError") {
          console.error("MJPEG stream error:", err);
        }
      }
    })();

    return () => controller.abort();
  }, []);

  const handleFollow = async (clickedDet) => {
    // Resolve the bbox's CURRENT id from the latest SSE frame.
    // React's onClick closure captures det.id at render time; if ByteTracker
    // re-IDed the same visual target between the render and the click firing,
    // that id is stale and the server returns 404 (F3 from 03-12-SUMMARY.md).
    // Pick the detection whose bbox center is nearest the clicked center.
    const latest = detectionsRef.current;
    const cx = clickedDet.bbox.x + clickedDet.bbox.w / 2;
    const cy = clickedDet.bbox.y + clickedDet.bbox.h / 2;
    let pick = clickedDet;
    let best = Infinity;
    for (const d of latest) {
      if (d.id == null) continue;
      const dx = (d.bbox.x + d.bbox.w / 2) - cx;
      const dy = (d.bbox.y + d.bbox.h / 2) - cy;
      const dist2 = dx * dx + dy * dy;
      if (dist2 < best) { best = dist2; pick = d; }
    }
    if (pick.id == null) return;
    try {
      const port = config?.follow_server_port || 8080;
      const host = window.location.hostname;
      // Send the full bbox geometry so the server can: (a) write target_bbox_height
      // from .h directly (bypassing the SharedUIState race [03-15]); and (b) when
      // the requested id has drifted between SSE render and HTTP arrival, match
      // the clicked bbox against shared_state's atomic id→bbox snapshot and
      // recover by geometric overlap (IoU) [03-16] — the operator's click is the
      // visual identification, regardless of which id was current at any layer.
      await fetch(`http://${host}:${port}/follow/${pick.id}`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ bbox: { x: pick.bbox.x, y: pick.bbox.y, w: pick.bbox.w, h: pick.bbox.h } }),
      });
    } catch {
      // ignore
    }
  };

  const handleRecord = async () => {
    try {
      const endpoint = recording ? "/api/record/stop" : "/api/record/start";
      const res = await fetch(endpoint, { method: "POST" });
      if (res.ok) {
        const data = await res.json();
        setRecording(data.recording ?? !recording);
      }
    } catch {
      // ignore
    }
  };

  const handleClear = async () => {
    try {
      const port = config?.follow_server_port || 8080;
      const host = window.location.hostname;
      await fetch(`http://${host}:${port}/follow/clear`, { method: "POST" });
    } catch {
      // ignore
    }
  };

  const [configStatus, setConfigStatus] = useState("");

  const handleConfigSave = async () => {
    try {
      const res = await fetch("/api/config/save", { method: "POST" });
      const data = await res.json().catch(() => ({}));
      if (res.ok && data.saved) {
        setConfigStatus(`Saved → ${data.path}`);
      } else {
        setConfigStatus(`Save failed: ${data.error || res.statusText}`);
      }
    } catch (e) {
      setConfigStatus(`Save error: ${e}`);
    }
    setTimeout(() => setConfigStatus(""), 4000);
  };

  const handleConfigLoad = async () => {
    try {
      const res = await fetch("/api/config/load", { method: "POST" });
      const data = await res.json().catch(() => ({}));
      if (res.ok && data.loaded) {
        // Refresh sliders from the live config after the reload
        const r = await fetch("/api/config");
        if (r.ok) setConfig(await r.json());
        const n = Array.isArray(data.changed) ? data.changed.length : 0;
        setConfigStatus(`Loaded → ${data.path} (${n} changed)`);
      } else {
        setConfigStatus(`Load failed: ${data.error || res.statusText}`);
      }
    } catch (e) {
      setConfigStatus(`Load error: ${e}`);
    }
    setTimeout(() => setConfigStatus(""), 4000);
  };

  // Debounced POST for config changes
  const postConfig = useCallback((updated) => {
    if (debounceRef.current) clearTimeout(debounceRef.current);
    debounceRef.current = setTimeout(() => {
      fetch("/api/config", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(updated),
      })
        .then((r) => (r.ok ? r.json() : null))
        .then((data) => {
          // Merge so fields not in POST response (e.g., follow_server_port)
          // are preserved from the previous state.
          if (data) setConfig((prev) => ({ ...prev, ...data }));
        })
        .catch(() => {});
    }, DEBOUNCE_MS);
  }, []);

  const onSlider = (key, value) => {
    const updated = { ...config, [key]: parseFloat(value) };
    setConfig(updated);
    postConfig({ [key]: parseFloat(value) });
  };

  const onToggle = (key) => {
    const newVal = !config[key];
    const updated = { ...config, [key]: newVal };
    setConfig(updated);
    postConfig({ [key]: newVal });
  };

  const vw = videoDims.width;
  const vh = videoDims.height;

  return (
    <div className="app">
      <div className="status-bar">
        <span className="status-text">
          {followingId != null
            ? `Following: ID ${followingId}`
            : paused ? "Idle (paused)" : "Auto (largest person)"}
        </span>
        {velocity && (
          <span className="velocity-text">
            {velocity.mode} | Fwd {velocity.forward_m_s.toFixed(2)} m/s
            {" "}| Down {velocity.down_m_s.toFixed(2)} m/s | Yaw{" "}
            {velocity.yawspeed_deg_s.toFixed(1)} deg/s
          </span>
        )}
        {perf && (
          <span className="perf-text">
            {perf.fps} FPS | {perf.latency_ms} ms
            | CPU {perf.cpu_percent}%
            | Mem {perf.memory_mb} MB
            {perf.hailo_util_percent > 0 ? ` | NN ${perf.hailo_util_percent}%` : ""}
            {perf.hailo_temp_c > 0 ? ` | ${perf.hailo_temp_c}\u00b0C` : ""}
          </span>
        )}
        {diag && (
          <span className="perf-text">
            {(() => {
              const j = diag.jpeg_pts_ns;
              const i = diag.inference_pts_ns;
              if (j == null || i == null) return "PTS: ?";
              const deltaMs = ((j - i) / 1e6).toFixed(1);
              return `JPEG=${(j / 1e9).toFixed(3)}s INF=${(i / 1e9).toFixed(3)}s \u0394=${deltaMs}ms`;
            })()}
          </span>
        )}
      </div>

      <div className="main-layout">
        <div className="side-panel">
          {config && (
            <div className="controls-panel side-card">
              <div className="controls-header">Controls</div>
              <div className="controls-body">
                {/* --- Operational controls --- */}
                <label className={`control-row${config.yaw_only ? " disabled" : ""}`}>
                  <span className="control-label">Target Size</span>
                  <input
                    type="range"
                    min="0.10"
                    max="0.25"
                    step="0.01"
                    value={config.target_bbox_height}
                    disabled={config.yaw_only}
                    onChange={(e) => onSlider("target_bbox_height", e.target.value)}
                  />
                  <span className="control-value">
                    {(config.target_bbox_height * 100).toFixed(0)}%
                  </span>
                </label>
                <label className="control-row">
                  <span className="control-label">Target Alt</span>
                  <input
                    type="range"
                    min="1"
                    max="8"
                    step="0.5"
                    value={config.target_altitude}
                    onChange={(e) => onSlider("target_altitude", e.target.value)}
                  />
                  <span className="control-value">{config.target_altitude.toFixed(1)}m</span>
                </label>
                <label className="control-row">
                  <span className="control-label">KP Distance</span>
                  <input
                    type="range"
                    min="0"
                    max="3"
                    step="0.1"
                    value={config.kp_distance}
                    onChange={(e) => onSlider("kp_distance", e.target.value)}
                  />
                  <span className="control-value">{config.kp_distance.toFixed(1)}</span>
                </label>
                <label className="control-row">
                  <span className="control-label">KP Dist Back</span>
                  <input
                    type="range"
                    min="0"
                    max="5"
                    step="0.1"
                    value={config.kp_distance_back}
                    onChange={(e) => onSlider("kp_distance_back", e.target.value)}
                  />
                  <span className="control-value">{config.kp_distance_back.toFixed(1)}</span>
                </label>
                <label className="control-row">
                  <span className="control-label">Min Altitude</span>
                  <input
                    type="range"
                    min="1"
                    max="5"
                    step="0.5"
                    value={config.min_altitude}
                    onChange={(e) => onSlider("min_altitude", e.target.value)}
                  />
                  <span className="control-value">{config.min_altitude.toFixed(1)}m</span>
                </label>
                <label className="control-row">
                  <span className="control-label">Max Altitude</span>
                  <input
                    type="range"
                    min="3"
                    max="10"
                    step="1"
                    value={config.max_altitude}
                    onChange={(e) => onSlider("max_altitude", e.target.value)}
                  />
                  <span className="control-value">{config.max_altitude.toFixed(0)}m</span>
                </label>
                <label className="control-row">
                  <span className="control-label">Yaw Only</span>
                  <div className="toggle-wrapper">
                    <button
                      className={`toggle-btn ${config.yaw_only ? "toggle-on" : ""}`}
                      onClick={() => onToggle("yaw_only")}
                    >
                      {config.yaw_only ? "ON" : "OFF"}
                    </button>
                  </div>
                </label>
                <label className="control-row">
                  <span className="control-label">Auto Select</span>
                  <div className="toggle-wrapper">
                    <button
                      className={`toggle-btn ${config.auto_select ? "toggle-on" : ""}`}
                      onClick={() => onToggle("auto_select")}
                    >
                      {config.auto_select ? "ON" : "OFF"}
                    </button>
                  </div>
                </label>
                {/* --- Tuning parameters --- */}
                <label className="control-row">
                  <span className="control-label">KP Yaw</span>
                  <input
                    type="range"
                    min="0"
                    max="10"
                    step="0.1"
                    value={config.kp_yaw}
                    onChange={(e) => onSlider("kp_yaw", e.target.value)}
                  />
                  <span className="control-value">{config.kp_yaw.toFixed(1)}</span>
                </label>
                <label className="control-row">
                  <span className="control-label">Max Yaw Rate</span>
                  <input
                    type="range"
                    min="10"
                    max="180"
                    step="5"
                    value={config.max_yawspeed}
                    onChange={(e) => onSlider("max_yawspeed", e.target.value)}
                  />
                  <span className="control-value">{config.max_yawspeed.toFixed(0)}°/s</span>
                </label>
                <label className="control-row">
                  <span className="control-label">Dead Zone Yaw</span>
                  <input
                    type="range"
                    min="0"
                    max="8"
                    step="0.5"
                    value={config.dead_zone_deg}
                    onChange={(e) => onSlider("dead_zone_deg", e.target.value)}
                  />
                  <span className="control-value">{config.dead_zone_deg.toFixed(1)}°</span>
                </label>
                <label className={`control-row${config.yaw_only ? " disabled" : ""}`}>
                  <span className="control-label">Max Fwd Accel</span>
                  <input
                    type="range"
                    min="0.1"
                    max="5"
                    step="0.1"
                    value={config.max_forward_accel}
                    disabled={config.yaw_only}
                    onChange={(e) => onSlider("max_forward_accel", e.target.value)}
                  />
                  <span className="control-value">{config.max_forward_accel.toFixed(1)} m/s²</span>
                </label>
                <label className={`control-row${config.yaw_only ? " disabled" : ""}`}>
                  <span className="control-label">Dead Zone BBox %</span>
                  <input
                    type="range"
                    min="0"
                    max="25"
                    step="1"
                    value={config.dead_zone_bbox_percent}
                    disabled={config.yaw_only}
                    onChange={(e) => onSlider("dead_zone_bbox_percent", e.target.value)}
                  />
                  <span className="control-value">{config.dead_zone_bbox_percent.toFixed(0)}%</span>
                </label>
                <label className="control-row">
                  <span className="control-label">Yaw Smooth</span>
                  <div className="toggle-wrapper">
                    <button
                      className={`toggle-btn ${config.smooth_yaw ? "toggle-on" : ""}`}
                      onClick={() => onToggle("smooth_yaw")}
                    >
                      {config.smooth_yaw ? "ON" : "OFF"}
                    </button>
                  </div>
                </label>
                <label className="control-row">
                  <span className="control-label">Yaw Alpha</span>
                  <input
                    type="range"
                    min="0.01"
                    max="1.0"
                    step="0.01"
                    value={config.yaw_alpha}
                    onChange={(e) => onSlider("yaw_alpha", e.target.value)}
                  />
                  <span className="control-value">{config.yaw_alpha.toFixed(2)}</span>
                </label>
                <label className={`control-row${config.yaw_only ? " disabled" : ""}`}>
                  <span className="control-label">Fwd Smooth</span>
                  <div className="toggle-wrapper">
                    <button
                      className={`toggle-btn ${config.smooth_forward ? "toggle-on" : ""}`}
                      disabled={config.yaw_only}
                      onClick={() => onToggle("smooth_forward")}
                    >
                      {config.smooth_forward ? "ON" : "OFF"}
                    </button>
                  </div>
                </label>
                <label className={`control-row${config.yaw_only ? " disabled" : ""}`}>
                  <span className="control-label">Fwd Alpha</span>
                  <input
                    type="range"
                    min="0.01"
                    max="1.0"
                    step="0.01"
                    value={config.forward_alpha}
                    disabled={config.yaw_only}
                    onChange={(e) => onSlider("forward_alpha", e.target.value)}
                  />
                  <span className="control-value">{config.forward_alpha.toFixed(2)}</span>
                </label>
                <label className="control-row">
                  <span className="control-label">Down Smooth</span>
                  <div className="toggle-wrapper">
                    <button
                      className={`toggle-btn ${config.smooth_down ? "toggle-on" : ""}`}
                      onClick={() => onToggle("smooth_down")}
                    >
                      {config.smooth_down ? "ON" : "OFF"}
                    </button>
                  </div>
                </label>
                <label className="control-row">
                  <span className="control-label">Down Alpha</span>
                  <input
                    type="range"
                    min="0.01"
                    max="1.0"
                    step="0.01"
                    value={config.down_alpha}
                    onChange={(e) => onSlider("down_alpha", e.target.value)}
                  />
                  <span className="control-value">{config.down_alpha.toFixed(2)}</span>
                </label>
              </div>
            </div>
          )}
        </div>

        <div className="video-column">
          <div className="video-container">
        <canvas
          ref={canvasRef}
          className="video-feed"
        />
        {vw > 0 && vh > 0 && (
          <svg className="overlay" viewBox={`0 0 ${vw} ${vh}`}>
            {detections.map((det, i) => {
              const isFollowing =
                det.id != null && det.id === followingId;
              const hasId = det.id != null;
              // ID-less detections (transient raw boxes) skip smoothing.
              const smoothed = hasId
                ? renderedBoxesRef.current.get(det.id)
                : null;
              const cx = smoothed ? smoothed.cx : det.bbox.x + det.bbox.w / 2;
              const cy = smoothed ? smoothed.cy : det.bbox.y + det.bbox.h / 2;
              const bw = smoothed ? smoothed.w : det.bbox.w;
              const bh = smoothed ? smoothed.h : det.bbox.h;
              const px = (cx - bw / 2) * vw;
              const py = (cy - bh / 2) * vh;
              const pw = bw * vw;
              const ph = bh * vh;
              const pcx = cx * vw;
              const pcy = cy * vh;

              return (
                <g
                  key={hasId ? `id-${det.id}` : `untracked-${i}`}
                  onClick={hasId ? () => handleFollow(det) : undefined}
                  style={{ cursor: hasId ? "pointer" : "default", pointerEvents: "auto" }}
                >
                  {isFollowing ? (
                    <>
                      {/* invisible bbox-sized hit target so the whole person stays clickable */}
                      <rect
                        x={px}
                        y={py}
                        width={pw}
                        height={ph}
                        fill="transparent"
                        stroke="none"
                      />
                      {/* black halo so the cross stays readable on any background */}
                      <line
                        x1={pcx - CROSS_HALF_SIZE}
                        y1={pcy}
                        x2={pcx + CROSS_HALF_SIZE}
                        y2={pcy}
                        stroke="#000000"
                        strokeWidth={CROSS_HALO_STROKE}
                        strokeOpacity={0.75}
                        strokeLinecap="round"
                      />
                      <line
                        x1={pcx}
                        y1={pcy - CROSS_HALF_SIZE}
                        x2={pcx}
                        y2={pcy + CROSS_HALF_SIZE}
                        stroke="#000000"
                        strokeWidth={CROSS_HALO_STROKE}
                        strokeOpacity={0.75}
                        strokeLinecap="round"
                      />
                      <line
                        x1={pcx - CROSS_HALF_SIZE}
                        y1={pcy}
                        x2={pcx + CROSS_HALF_SIZE}
                        y2={pcy}
                        stroke="#80f060"
                        strokeWidth={CROSS_STROKE}
                        strokeOpacity={1.0}
                        strokeLinecap="round"
                      />
                      <line
                        x1={pcx}
                        y1={pcy - CROSS_HALF_SIZE}
                        x2={pcx}
                        y2={pcy + CROSS_HALF_SIZE}
                        stroke="#80f060"
                        strokeWidth={CROSS_STROKE}
                        strokeOpacity={1.0}
                        strokeLinecap="round"
                      />
                    </>
                  ) : (
                    <rect
                      x={px}
                      y={py}
                      width={pw}
                      height={ph}
                      fill="transparent"
                      stroke="#ffffff"
                      strokeWidth={2}
                      strokeOpacity={0.8}
                    />
                  )}
                </g>
              );
            })}
          </svg>
        )}
          </div>

          <div className="logs-panel side-card">
            <button
              className="controls-toggle"
              onClick={() => setLogsOpen((o) => !o)}
            >
              Logs {logsOpen ? "▲" : "▼"}
            </button>
            {logsOpen && (
              <div className="logs-body">
                {logs.map((entry) => (
                  <div key={entry.id} className="log-line">
                    {entry.msg}
                  </div>
                ))}
                <div ref={logEndRef} />
              </div>
            )}
          </div>
        </div>

        <div className="action-panel">
          <button
            className={`record-btn ${recording ? "recording" : ""}`}
            onClick={handleRecord}
          >
            {recording ? "Stop Rec" : "Record"}
          </button>
          <button className="clear-btn" onClick={handleClear}>
            Clear Target
          </button>
          <button className="clear-btn" onClick={handleConfigSave} title="Save current config to df_config.json on the air unit">
            Save Config
          </button>
          <button className="clear-btn" onClick={handleConfigLoad} title="Live-reload config from df_config.json on the air unit">
            Load Config
          </button>
          {configStatus && <span className="config-status">{configStatus}</span>}
        </div>
      </div>
    </div>
  );
}
