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
// Whole-frame default tile geometry (4 overlapping quadrants + full frame).
// POSTed to /api/tiles to revert inference to the whole frame once the
// operator clears all hand-drawn regions. Mirrors DEFAULT_TILES on the
// Python side (subscriber.py) and the C++ framework default.
const DEFAULT_TILE_SPEC =
  "0,0,0.6,0.6;0.4,0,0.6,0.6;0,0.4,0.6,0.6;0.4,0.4,0.6,0.6;0,0,1,1";
// Ignore rubber-band drags smaller than this (normalized) — treats an
// accidental click as "cancel" rather than a zero-area region.
const MIN_REGION_SIZE = 0.02;
// Network input dimensions (YOLOv8s hailo_yolov8s_384_640.hef → 640×384 WxH).
// The DSP crop-resize is a plain stretch to these dims, so a crop whose pixel
// aspect already matches gets scaled uniformly (no warp). We expand each drawn
// region to this aspect before sending it as a tile — see expandToNetAspect().
const NET_INPUT_W = 640;
const NET_INPUT_H = 384;

// --- regions ⇄ tiles ----------------------------------------------------
// "regions" are the ORIGINAL rectangles the operator draws/edits. The pipeline
// "tiles" are those regions expanded to the network aspect (the padded crops
// fed to the chip), plus the ≥2-tile rule (a lone region → two identical
// tiles). regions→tiles happens in buildTiles() at apply time; tiles→regions
// (dedupe) runs once on load/SSE so a reload still shows the active tiles.
// (Expansion is idempotent, so a region reconstructed from a padded tile and
// re-expanded is unchanged.)

// Parse "x,y,w,h;x,y,w,h;..." (optional "name=" prefix) → [{x,y,w,h}].
function parseSpecToRegions(spec) {
  const out = [];
  for (const raw of String(spec).split(";")) {
    const s = raw.trim();
    if (!s) continue;
    const nums = s.replace(/^[^=]*=/, "").split(",").map((n) => Number(n.trim()));
    if (nums.length !== 4 || nums.some((n) => !Number.isFinite(n))) {
      throw new Error(`bad tile "${s}" (need x,y,w,h)`);
    }
    const [x, y, w, h] = nums;
    out.push({ x, y, w, h });
  }
  return out;
}

const regionsToSpec = (regs) =>
  regs.map((r) => `${r.x},${r.y},${r.w},${r.h}`).join(";");

// Remove exactly-duplicate rectangles (e.g. the single-region duplicate).
function dedupeTiles(tiles) {
  const seen = new Set();
  const out = [];
  for (const t of tiles) {
    const k = [t.x, t.y, t.w, t.h].map((v) => Number(v).toFixed(4)).join(",");
    if (seen.has(k)) continue;
    seen.add(k);
    out.push({ x: t.x, y: t.y, w: t.w, h: t.h });
  }
  return out;
}

// The whole-frame default (4 quadrants + full). When the active tiles equal
// this, there are no user regions — so tiles→regions yields an empty list.
const DEFAULT_TILE_SET = parseSpecToRegions(DEFAULT_TILE_SPEC);
function isDefaultTiles(regs) {
  if (regs.length !== DEFAULT_TILE_SET.length) return false;
  const key = (r) => [r.x, r.y, r.w, r.h].map((v) => Number(v).toFixed(3)).join(",");
  const have = new Set(regs.map(key));
  return DEFAULT_TILE_SET.every((r) => have.has(key(r)));
}

// A full-frame tile (always present in the pipeline tile set). Not a user
// region — stripped when reconstructing regions from the active tiles.
const isFullFrameTile = (t) =>
  t.w >= 0.999 && t.h >= 0.999 && t.x <= 0.001 && t.y <= 0.001;

export default function App() {
  const [detections, setDetections] = useState([]);
  const [followingId, setFollowingId] = useState(null);
  const [velocity, setVelocity] = useState(null);
  // Default to the camera's FHD aspect so the SVG overlay still renders
  // when no MJPEG video is being pushed (e.g. native-pipeline mode where
  // the C++ binary owns video over a separate UDP channel). The decoder
  // overwrites this with the real frame size as soon as JPEGs arrive.
  const [videoDims, setVideoDims] = useState({ width: 1920, height: 1080 });
  const [logsOpen, setLogsOpen] = useState(true);
  const [logs, setLogs] = useState([]);
  const [config, setConfig] = useState(null);
  const [recording, setRecording] = useState(false);
  const [perf, setPerf] = useState(null);
  const [paused, setPaused] = useState(false);
  const [diag, setDiag] = useState(null);
  // Per-frame tiling stats from the Python subscriber. None until first SSE
  // event with stats arrives. Used for the tile-boundary overlay legend +
  // small-target verification readout.
  const [tileStats, setTileStats] = useState(null);
  const [showTiles, setShowTiles] = useState(false);
  // Active tile geometry from SSE — populated once the server starts
  // sending. Until then, fall back to the framework default (rendered for
  // visual continuity; replaced as soon as a real event arrives).
  const [tiles, setTiles] = useState([
    { name: "TL", x: 0.0, y: 0.0, w: 0.6, h: 0.6 },
    { name: "TR", x: 0.4, y: 0.0, w: 0.6, h: 0.6 },
    { name: "BL", x: 0.0, y: 0.4, w: 0.6, h: 0.6 },
    { name: "BR", x: 0.4, y: 0.4, w: 0.6, h: 0.6 },
    { name: "FULL", x: 0.0, y: 0.0, w: 1.0, h: 1.0 },
  ]);
  const [tileEditorOpen, setTileEditorOpen] = useState(false);
  const [tileSpecDraft, setTileSpecDraft] = useState("");
  const [tileApplyState, setTileApplyState] = useState({ status: "idle" });
  // Hand-drawn inference regions. `regionArmed` is set by the "+" button /
  // key and means "the next click-drag on the video defines a region".
  // `regions` is the committed list ({x,y,w,h} normalized); when non-empty
  // only these areas are inferred. `drawRect` is the live rubber-band during
  // a drag. drawStartRef holds the drag origin so mouse-up can compute the
  // final rect without depending on stale state.
  const [regionArmed, setRegionArmed] = useState(false);
  const [regions, setRegions] = useState([]);
  const [drawRect, setDrawRect] = useState(null);
  const [regionApplyState, setRegionApplyState] = useState({ status: "idle" });
  const drawStartRef = useRef(null);
  // Guards the one-time reconstruction of `regions` from the pipeline's active
  // tiles on load (tiles→regions). After the first sync, `regions` is the
  // local source of truth and isn't overwritten by the per-frame SSE tiles.
  const regionsInitedRef = useRef(false);
  const canvasRef = useRef(null);
  // Per-tile debug thumbnail canvases (index → <canvas>), populated by
  // callback refs. Filled by the draw effect when "show tiles" is on.
  const tileThumbRefs = useRef(new Map());
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
        if (data.stats && typeof data.stats === "object") {
          setTileStats(data.stats);
        }
        if (Array.isArray(data.tiles) && data.tiles.length) {
          setTiles(data.tiles);
          // One-time tiles→regions sync on load: reconstruct the region list
          // from whatever tiles the pipeline is already running, so the UI
          // reflects them (and "clear"/"edit" work) after a reload. Deduped
          // (undoes the single-region duplicate); the whole-frame default maps
          // to no regions. Skipped forever after — regions is local truth then.
          if (!regionsInitedRef.current) {
            regionsInitedRef.current = true;
            const deduped = dedupeTiles(data.tiles);
            // Strip the always-present full-frame tile — it isn't a user region.
            if (!isDefaultTiles(deduped)) {
              setRegions(deduped.filter((t) => !isFullFrameTile(t)));
            }
          }
        }
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

          // Scan the buffer for ALL complete frames, then draw only the
          // newest one — older frames are discarded. If we instead awaited
          // createImageBitmap per frame, a single slow decode would let
          // the network buffer fill with more frames, the next iteration
          // would parse those too, and lag would grow without bound.
          // Detections don't hit this because each SSE event is small;
          // 43 KB JPEGs accumulate fast.
          let latestJpeg = null;
          let consumedTo = 0;
          while (true) {
            const bStart = indexOf(buffer.subarray(consumedTo), BOUNDARY);
            if (bStart === -1) break;
            const absStart = consumedTo + bStart;
            const headerStart = absStart + BOUNDARY.length;
            const headerEnd = indexOf(
              buffer.subarray(headerStart),
              HEADER_END
            );
            if (headerEnd === -1) break;
            const jpegStart = headerStart + headerEnd + HEADER_END.length;
            const nextBoundary = indexOf(
              buffer.subarray(jpegStart),
              BOUNDARY
            );
            if (nextBoundary === -1) break;
            let jpegEnd = jpegStart + nextBoundary;
            while (jpegEnd > jpegStart && (buffer[jpegEnd - 1] === 0x0a || buffer[jpegEnd - 1] === 0x0d)) {
              jpegEnd--;
            }
            latestJpeg = buffer.slice(jpegStart, jpegEnd);
            // Advance past this frame's data so the next iteration looks
            // at what comes after the next boundary marker.
            consumedTo = jpegStart + nextBoundary;
          }
          if (consumedTo > 0) {
            buffer = buffer.slice(consumedTo);
          }

          if (latestJpeg) {
            try {
              const blob = new Blob([latestJpeg], { type: "image/jpeg" });
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

  // "Edit tiles" edits the region list (same source of truth as draw/clear).
  // Parse the spec into regions, sync them, and apply — so editing tiles and
  // drawing regions stay equivalent.
  const applyTileSpec = async (spec) => {
    let parsed;
    try {
      parsed = parseSpecToRegions(spec);
    } catch (e) {
      setTileApplyState({ status: "error", message: `Parse: ${e.message || e}` });
      return;
    }
    setRegions(parsed);
    setTileApplyState({ status: "applying" });
    const body = parsed.length === 0
      ? { spec: DEFAULT_TILE_SPEC }
      : { tiles: buildTiles(parsed) };
    try {
      const res = await fetch("/api/tiles", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(body),
      });
      const data = await res.json().catch(() => ({}));
      if (!res.ok) {
        setTileApplyState({
          status: "error",
          message: data.error || `HTTP ${res.status}`,
        });
        return;
      }
      setTileApplyState({
        status: "applied",
        message: `${data.tiles?.length ?? 0} tiles active`,
      });
      // Auto-close after a moment so the editor doesn't linger over the
      // video once the apply succeeds.
      setTimeout(() => {
        setTileEditorOpen(false);
        setTileApplyState({ status: "idle" });
      }, 1500);
    } catch (e) {
      setTileApplyState({ status: "error", message: String(e) });
    }
  };

  // --- Hand-drawn inference regions --------------------------------------

  // Convert a viewport pixel coordinate to normalized [0,1] frame coords,
  // relative to the rendered video canvas. The canvas is width:100%/height:auto
  // with the SVG overlay matching it exactly (no letterboxing), so the canvas
  // client rect is the video rect.
  const clientToNorm = useCallback((clientX, clientY) => {
    const el = canvasRef.current;
    if (!el) return null;
    const r = el.getBoundingClientRect();
    if (r.width <= 0 || r.height <= 0) return null;
    const nx = Math.min(1, Math.max(0, (clientX - r.left) / r.width));
    const ny = Math.min(1, Math.max(0, (clientY - r.top) / r.height));
    return { x: nx, y: ny };
  }, []);

  // Expand a drawn region so its CROP has the same pixel aspect ratio as the
  // network input, so the DSP's stretch resize scales it uniformly (no warp).
  // The extra area is filled with real neighbouring scene (not black). Growth
  // is centred on the region and clamped to the frame; only the smaller axis
  // grows, so the drawn region always stays fully inside the expanded crop.
  const expandToNetAspect = (r) => {
    const vw = videoDims.width || 16;
    const vh = videoDims.height || 9;
    // Target tile w/h in NORMALIZED coords = net pixel aspect ÷ frame pixel
    // aspect. e.g. (640/384) / (1920/1080) = 0.9375 for a 16:9 frame.
    const targetWH = (NET_INPUT_W / NET_INPUT_H) * (vh / vw);
    let { x, y, w, h } = r;
    if (w / h < targetWH) {
      const newW = Math.min(1, h * targetWH); // too narrow → widen
      x += (w - newW) / 2;
      w = newW;
    } else {
      const newH = Math.min(1, w / targetWH); // too wide → grow height
      y += (h - newH) / 2;
      h = newH;
    }
    // Keep the (grown) box inside the frame. Shifting is safe: the box only
    // grew, so the original region is still contained after clamping.
    x = Math.max(0, Math.min(x, 1 - w));
    y = Math.max(0, Math.min(y, 1 - h));
    return { x, y, w, h };
  };

  // Build the pipeline tiles from the user's ORIGINAL regions. The full frame
  // is ALWAYS included as a tile (so the whole scene is inferred regardless of
  // regions), followed by each region expanded to the network aspect (the
  // "padded" tile fed to the chip). Regions themselves stay the original drawn
  // shapes — this is the only place expansion happens, so the overlay shows
  // originals while "show tiles" shows these padded tiles. The always-present
  // full tile also satisfies the pipeline's ≥2-tile rule for a single region
  // (full + region), so no duplicate is needed. expandToNetAspect is
  // idempotent (re-expanding a padded region reconstructed from SSE is a no-op).
  const buildTiles = (regs) => [
    { name: "FULL", x: 0, y: 0, w: 1, h: 1 },
    ...regs.map((r, i) => ({ name: `R${i + 1}`, ...expandToNetAspect(r) })),
  ];

  // POST the current region set to the pipeline. Empty list → revert to the
  // whole-frame default. Triggers a brief pipeline restart on the backend.
  const applyRegions = async (regs) => {
    const body = regs.length === 0
      ? { spec: DEFAULT_TILE_SPEC }
      : { tiles: buildTiles(regs) };
    setRegionApplyState({ status: "applying" });
    try {
      const res = await fetch("/api/tiles", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(body),
      });
      const data = await res.json().catch(() => ({}));
      if (!res.ok) {
        setRegionApplyState({
          status: "error",
          message: data.error || `HTTP ${res.status}`,
        });
        return;
      }
      setRegionApplyState({
        status: "applied",
        message: regs.length === 0
          ? "whole frame"
          : `${regs.length} region${regs.length > 1 ? "s" : ""}`,
      });
      setTimeout(() => setRegionApplyState({ status: "idle" }), 2500);
    } catch (e) {
      setRegionApplyState({ status: "error", message: String(e) });
    }
  };

  const onRegionMouseDown = (e) => {
    const p = clientToNorm(e.clientX, e.clientY);
    if (!p) return;
    drawStartRef.current = p;
    setDrawRect({ x: p.x, y: p.y, w: 0, h: 0 });
  };

  const onRegionMouseMove = (e) => {
    const s = drawStartRef.current;
    if (!s) return;
    const p = clientToNorm(e.clientX, e.clientY);
    if (!p) return;
    setDrawRect({
      x: Math.min(s.x, p.x),
      y: Math.min(s.y, p.y),
      w: Math.abs(p.x - s.x),
      h: Math.abs(p.y - s.y),
    });
  };

  const onRegionMouseUp = (e) => {
    const s = drawStartRef.current;
    drawStartRef.current = null;
    setDrawRect(null);
    setRegionArmed(false);
    if (!s) return;
    const p = clientToNorm(e.clientX, e.clientY) || s;
    const rect = {
      x: Math.min(s.x, p.x),
      y: Math.min(s.y, p.y),
      w: Math.abs(p.x - s.x),
      h: Math.abs(p.y - s.y),
    };
    if (rect.w < MIN_REGION_SIZE || rect.h < MIN_REGION_SIZE) return; // cancel
    // Store the ORIGINAL drawn rect — the overlay shows exactly what you drew.
    // Expansion to the network aspect (the padded tile fed to the chip) happens
    // only in buildTiles() at apply time and is shown under "show tiles".
    const next = [...regions, rect];
    setRegions(next);
    applyRegions(next);
  };

  const onRegionMouseLeave = () => {
    // Abort the in-progress drag but stay armed so the operator can retry.
    drawStartRef.current = null;
    setDrawRect(null);
  };

  const clearRegions = () => {
    setRegionArmed(false);
    setDrawRect(null);
    drawStartRef.current = null;
    setRegions([]);
    applyRegions([]);
  };

  // "+" arms region drawing; Escape cancels. Ignored while typing in a field.
  useEffect(() => {
    const onKey = (e) => {
      const tag = (e.target?.tagName || "").toLowerCase();
      if (tag === "input" || tag === "textarea") return;
      if (e.key === "+" || e.key === "=") {
        e.preventDefault();
        setRegionArmed((v) => !v);
      } else if (e.key === "Escape") {
        setRegionArmed(false);
        setDrawRect(null);
        drawStartRef.current = null;
      }
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, []);

  // Debug: reconstruct each pipeline tile's chip-input image (crop → resize to
  // the 640×384 target) from the live display frame while "show tiles" is on.
  // This mirrors the crop/resize geometry the NPU receives (aspect, framing,
  // any edge-clamp warp) — faithful for geometry, though sourced from the
  // display frame, not the literal NV12 buffer on the DSP. There's no black
  // padding to show: the expand-to-aspect crop is scaled uniformly.
  useEffect(() => {
    if (!showTiles || !tiles.length) return undefined;
    let raf;
    let last = 0;
    const draw = (ts) => {
      raf = requestAnimationFrame(draw);
      if (ts - last < 120) return; // throttle to ~8 fps
      last = ts;
      const src = canvasRef.current;
      if (!src || !src.width) return;
      tiles.forEach((t, i) => {
        const cv = tileThumbRefs.current.get(i);
        const g = cv && cv.getContext("2d");
        if (!g) return;
        g.fillStyle = "#000";
        g.fillRect(0, 0, cv.width, cv.height);
        const sx = t.x * src.width;
        const sy = t.y * src.height;
        const sw = Math.max(1, t.w * src.width);
        const sh = Math.max(1, t.h * src.height);
        try {
          g.drawImage(src, sx, sy, sw, sh, 0, 0, cv.width, cv.height);
        } catch {
          // frame not ready yet
        }
      });
    };
    raf = requestAnimationFrame(draw);
    return () => cancelAnimationFrame(raf);
  }, [showTiles, tiles]);

  const TILE_PRESETS = [
    {
      label: "Default (4 quadrants + full)",
      spec: "0,0,0.6,0.6;0.4,0,0.6,0.6;0,0.4,0.6,0.6;0.4,0.4,0.6,0.6;0,0,1,1",
    },
    {
      label: "2×2 no overlap + full",
      spec: "0,0,0.5,0.5;0.5,0,0.5,0.5;0,0.5,0.5,0.5;0.5,0.5,0.5,0.5;0,0,1,1",
    },
    {
      label: "3×3 thirds + full",
      spec:
        "0,0,0.333,0.333;0.333,0,0.334,0.333;0.667,0,0.333,0.333;" +
        "0,0.333,0.333,0.334;0.333,0.333,0.334,0.334;0.667,0.333,0.333,0.334;" +
        "0,0.667,0.333,0.333;0.333,0.667,0.334,0.333;0.667,0.667,0.333,0.333;" +
        "0,0,1,1",
    },
    {
      label: "Centre crop + full",
      spec: "0.25,0.25,0.5,0.5;0,0,1,1",
    },
  ];

  return (
    <div className="app">
      {tileEditorOpen && (
        <div
          style={{
            position: "fixed",
            inset: 0,
            background: "rgba(0,0,0,0.55)",
            zIndex: 1000,
            display: "flex",
            alignItems: "center",
            justifyContent: "center",
          }}
          onClick={() => setTileEditorOpen(false)}
        >
          <div
            onClick={(e) => e.stopPropagation()}
            style={{
              background: "#222",
              color: "#fff",
              padding: 20,
              borderRadius: 6,
              width: 540,
              fontFamily: "monospace",
              fontSize: 13,
            }}
          >
            <div style={{ fontSize: 15, marginBottom: 10 }}>
              Edit tiles (applies and restarts the pipeline — brief video
              pause while the C++ binary respawns)
            </div>
            <div style={{ marginBottom: 8, opacity: 0.85 }}>
              Format: <code>x,y,w,h;x,y,w,h;...</code> — all normalized [0,1]
              to frame. Need ≥ 2 tiles.
            </div>
            <textarea
              value={tileSpecDraft}
              onChange={(e) => setTileSpecDraft(e.target.value)}
              rows={4}
              style={{
                width: "100%",
                background: "#111",
                color: "#fff",
                border: "1px solid #555",
                padding: 6,
                fontFamily: "monospace",
                fontSize: 13,
                boxSizing: "border-box",
              }}
              spellCheck={false}
            />
            <div style={{ marginTop: 8, marginBottom: 10 }}>
              Presets:{" "}
              {TILE_PRESETS.map((p, i) => (
                <button
                  key={i}
                  onClick={() => setTileSpecDraft(p.spec)}
                  style={{
                    marginRight: 6,
                    marginBottom: 4,
                    background: "#444",
                    color: "#fff",
                    border: "1px solid #666",
                    padding: "3px 8px",
                    cursor: "pointer",
                    fontFamily: "monospace",
                    fontSize: 12,
                  }}
                >
                  {p.label}
                </button>
              ))}
            </div>
            {tileApplyState.status === "error" && (
              <div style={{ color: "#ff8080", marginBottom: 8 }}>
                Error: {tileApplyState.message}
              </div>
            )}
            {tileApplyState.status === "applied" && (
              <div style={{ color: "#80f060", marginBottom: 8 }}>
                Applied — {tileApplyState.message}
              </div>
            )}
            <div style={{ display: "flex", gap: 8, justifyContent: "flex-end" }}>
              <button
                onClick={() => setTileEditorOpen(false)}
                style={{
                  background: "#333",
                  color: "#fff",
                  border: "1px solid #555",
                  padding: "6px 12px",
                  cursor: "pointer",
                }}
              >
                Cancel
              </button>
              <button
                onClick={() => applyTileSpec(tileSpecDraft)}
                disabled={tileApplyState.status === "applying"}
                style={{
                  background: "#225",
                  color: "#fff",
                  border: "1px solid #557",
                  padding: "6px 12px",
                  cursor:
                    tileApplyState.status === "applying"
                      ? "wait"
                      : "pointer",
                }}
              >
                {tileApplyState.status === "applying"
                  ? "Applying..."
                  : "Apply"}
              </button>
            </div>
          </div>
        </div>
      )}
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
        {/* Inference-region toolbar. "+ Add region" arms a click-drag on the
            video that defines an area to run inference on. Drawing an area
            replaces whole-frame inference with just the selected region(s);
            "Clear regions" reverts to the whole frame. Also bindable via the
            "+" key. */}
        <div
          style={{
            position: "absolute",
            top: 4,
            left: 4,
            display: "flex",
            gap: 6,
            alignItems: "center",
            zIndex: 20,
            fontFamily: "monospace",
            fontSize: 12,
          }}
        >
          <button
            onClick={() => setRegionArmed((v) => !v)}
            title="Press + then click-drag on the video to select an inference area"
            style={{
              background: regionArmed ? "#1b6" : "rgba(0,0,0,0.55)",
              color: "#fff",
              border: "1px solid #888",
              borderRadius: 3,
              padding: "3px 8px",
              cursor: "pointer",
              fontFamily: "monospace",
              fontSize: 12,
            }}
          >
            {regionArmed ? "Click-drag to select…" : "＋ Add region"}
          </button>
          {regions.length > 0 && (
            <button
              onClick={clearRegions}
              style={{
                background: "rgba(0,0,0,0.55)",
                color: "#fff",
                border: "1px solid #888",
                borderRadius: 3,
                padding: "3px 8px",
                cursor: "pointer",
                fontFamily: "monospace",
                fontSize: 12,
              }}
            >
              Clear regions ({regions.length})
            </button>
          )}
          {regionApplyState.status === "applying" && (
            <span style={{ color: "#ffd54f" }}>applying…</span>
          )}
          {regionApplyState.status === "applied" && (
            <span style={{ color: "#80f060" }}>{regionApplyState.message}</span>
          )}
          {regionApplyState.status === "error" && (
            <span style={{ color: "#ff8080" }}>{regionApplyState.message}</span>
          )}
        </div>
        {/* Transparent capture layer — only mounted while armed. Sits above
            the video + SVG so it intercepts the drag (and blocks detection
            click-to-follow) while a region is being drawn. */}
        {regionArmed && (
          <div
            onMouseDown={onRegionMouseDown}
            onMouseMove={onRegionMouseMove}
            onMouseUp={onRegionMouseUp}
            onMouseLeave={onRegionMouseLeave}
            style={{
              position: "absolute",
              inset: 0,
              zIndex: 10,
              cursor: "crosshair",
            }}
          />
        )}
        {/* Tile stats badge. Shows the per-frame detection count + smallest
            bbox + per-tile attribution, plus toggles for the tile-boundary
            overlay and the [edit tiles] modal. Only renders in native
            pipeline mode (gated on tileStats being populated). */}
        <div
          style={{
            position: "absolute",
            top: 4,
            right: 4,
            background: "rgba(0,0,0,0.55)",
            color: "#fff",
            fontFamily: "monospace",
            fontSize: 12,
            lineHeight: 1.3,
            padding: "4px 6px",
            borderRadius: 3,
            pointerEvents: "none",
            zIndex: 5,
          }}
        >
          {tileStats && (
            <>
              <div style={{ opacity: 0.8 }}>
                n={tileStats.count}, min{" "}
                {(tileStats.smallest_w * 100).toFixed(1)}×
                {(tileStats.smallest_h * 100).toFixed(1)}%
              </div>
              <div style={{ opacity: 0.8 }}>
                {Object.entries(tileStats.per_tile || {})
                  .filter(([, c]) => c > 0)
                  .map(([n, c]) => `${n}${c}`)
                  .join(" ") || "(no dets)"}
              </div>
              <div
                style={{
                  marginTop: 4,
                  cursor: "pointer",
                  pointerEvents: "auto",
                  textDecoration: "underline",
                  opacity: 0.8,
                }}
                onClick={() => setShowTiles((v) => !v)}
              >
                {showTiles ? "[hide tiles]" : "[show tiles]"}
              </div>
              <div
                style={{
                  cursor: "pointer",
                  pointerEvents: "auto",
                  textDecoration: "underline",
                  opacity: 0.8,
                }}
                onClick={() => {
                  // Seed the editor from the current regions (the source of
                  // truth). Editing here updates regions, keeping the two in
                  // sync. Empty → the current active tiles as a starting point.
                  setTileSpecDraft(
                    regions.length
                      ? regionsToSpec(regions)
                      : tiles.map((t) => `${t.x},${t.y},${t.w},${t.h}`).join(";"),
                  );
                  setTileEditorOpen(true);
                }}
              >
                [edit tiles]
              </div>
            </>
          )}
        </div>
        {vw > 0 && vh > 0 && (
          <svg className="overlay" viewBox={`0 0 ${vw} ${vh}`}>
            {/* Tile-boundary overlay. Driven by the SSE `tiles` field so
                custom geometries set via the editor render correctly.
                Full-frame tiles (w==h==1) get a white stroke; everything
                else gets yellow. Rendered first so detections draw on
                top. Toggled via the "show tiles" badge link. */}
            {showTiles &&
              tiles.map((t) => {
                const isFull = t.w >= 0.999 && t.h >= 0.999;
                const color = isFull ? "#ffffff" : "#ffd54f";
                return (
                  <g key={`tile-${t.name}`}>
                    <rect
                      x={t.x * vw}
                      y={t.y * vh}
                      width={t.w * vw}
                      height={t.h * vh}
                      fill="none"
                      stroke={color}
                      strokeWidth={2}
                      strokeDasharray="6 4"
                      opacity={0.55}
                    />
                    <text
                      x={t.x * vw + 6}
                      y={t.y * vh + 18}
                      fill={color}
                      fontSize={14}
                      fontFamily="monospace"
                      opacity={0.75}
                    >
                      {t.name}
                    </text>
                  </g>
                );
              })}
            {/* Inference regions — the ORIGINAL shapes the operator drew.
                The padded tiles actually fed to the chip (each expanded to the
                640×384 aspect) render separately under the "show tiles" toggle
                and in the debug strip below the video. */}
            {regions.map((r, i) => (
              <g key={`roi-${i}`}>
                <rect
                  x={r.x * vw}
                  y={r.y * vh}
                  width={r.w * vw}
                  height={r.h * vh}
                  fill="none"
                  stroke="#00e5ff"
                  strokeWidth={3}
                  opacity={0.9}
                />
                <text
                  x={r.x * vw + 6}
                  y={r.y * vh + 20}
                  fill="#00e5ff"
                  fontSize={16}
                  fontFamily="monospace"
                >
                  ROI {i + 1}
                </text>
              </g>
            ))}
            {drawRect && (
              <rect
                x={drawRect.x * vw}
                y={drawRect.y * vh}
                width={drawRect.w * vw}
                height={drawRect.h * vh}
                fill="rgba(0,229,255,0.15)"
                stroke="#00e5ff"
                strokeWidth={2}
                strokeDasharray="6 4"
              />
            )}
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

          {/* Debug: the actual chip inputs (each tile cropped + resized to the
              640×384 network input). Shown with "show tiles". Includes the
              single-region duplicate, so you see exactly what's fed per frame. */}
          {showTiles && tiles.length > 0 && (
            <div
              style={{
                display: "flex",
                flexWrap: "wrap",
                gap: 8,
                marginTop: 8,
                padding: 8,
                background: "#1a1a1a",
                borderRadius: 6,
              }}
            >
              <div
                style={{
                  width: "100%",
                  fontFamily: "monospace",
                  fontSize: 11,
                  color: "#888",
                }}
              >
                Chip inputs — each tile cropped &amp; resized to 640×384
                (reconstructed from the display frame):
              </div>
              {tiles.map((t, i) => (
                <div key={`thumb-${i}`} style={{ textAlign: "center" }}>
                  <canvas
                    width={240}
                    height={144}
                    ref={(el) => {
                      if (el) tileThumbRefs.current.set(i, el);
                      else tileThumbRefs.current.delete(i);
                    }}
                    style={{
                      width: 200,
                      height: 120,
                      background: "#000",
                      border: "1px solid #00e5ff",
                      borderRadius: 3,
                      display: "block",
                    }}
                  />
                  <div
                    style={{
                      fontFamily: "monospace",
                      fontSize: 10,
                      color: "#aaa",
                      marginTop: 2,
                    }}
                  >
                    {t.name} · {(t.w * 100).toFixed(0)}×{(t.h * 100).toFixed(0)}%
                  </div>
                </div>
              ))}
            </div>
          )}

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
