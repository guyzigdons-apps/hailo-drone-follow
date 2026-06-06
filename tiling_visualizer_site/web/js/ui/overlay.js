// Canvas overlay rendering (DS §3.4 / §3.5 / §7.2 / §7.6 / §7.7 / §7.12).
//
// Draws tiles (background regions) then detection boxes for each enabled run
// onto #overlay-canvas. The canvas lives INSIDE #stage-transform, so the CSS
// zoom/pan transform scales our drawing for free — we draw in stage CSS-px
// coordinates (mapped through contentBox) and only divide stroke widths / font
// sizes by zoom so they stay constant on screen (DS §7.5).
//
// Repaint policy (DS §7.7): full clear+repaint ONLY when one of the keys below
// changes. NEVER on cursorNorm / mousemove.
import { isPhantom, containmentMerge } from '../core/filters.js';

// Run overlay palette — slots 1..8 (DS §3.5). Hardcoded hexes mirror the
// --run-1..8 custom properties in css/tokens.css (canvas can't read CSS vars
// cheaply, so keep these in sync if tokens.css changes).
const RUN_PALETTE = [
  '#00E5FF', // 1 electric cyan
  '#FF3DDB', // 2 magenta
  '#FFA000', // 3 amber
  '#6EE63A', // 4 spring green
  '#A78BFA', // 5 violet
  '#FF6B5C', // 6 coral red
  '#5AA9FF', // 7 sky blue
  '#FF9EC4', // 8 hot pink-white
];

// Tile category → color (DS §3.4). Hardcoded hexes mirror the --tile-* custom
// properties in css/tokens.css.
const TILE_COLORS = {
  'multi-scale': '#22D3EE',
  'single-scale': '#FACC15',
  dynamic: '#34D399',
  'dynamic-merged': '#E879F9',
};

// Categories drawn with a dashed stroke (regions that overlap / search scales);
// dynamic + dynamic-merged are solid (DS §3.4 table).
const DASHED_CATEGORIES = new Set(['multi-scale', 'single-scale']);

// Mirrors --font-mono in tokens.css (canvas can't read CSS vars cheaply).
const MONO_FONT = '"JetBrains Mono","SF Mono","Cascadia Code",Consolas,"Roboto Mono",monospace';

// Keys that, when changed, require a full repaint.
const REDRAW_KEYS = [
  'frame',
  'enabledRuns',
  'runColors',
  'tileSourceRun',
  'confThreshold',
  'hidePhantoms',
  'containmentMerge',
  'contentBox',
  'hoveredDet',
  'runDocs',
  'zoom',
];

const FILTER_CACHE_MAX = 2000;

export function initOverlay(store) {
  const canvas = document.getElementById('overlay-canvas');
  if (!canvas) return;
  const ctx = canvas.getContext('2d');
  if (!ctx) return;

  // filteredCache: key `${runId}|${frame}|${conf}|${hidePhantoms}|${merge}`
  //   → { dets: [...], phantoms: <count>, merged: <count> }
  const filteredCache = new Map();
  let lastCountsSig = null;

  function cacheKey(runId, frame, conf, hidePhantoms, merge) {
    return `${runId}|${frame}|${conf}|${hidePhantoms ? 1 : 0}|${merge ? 1 : 0}`;
  }

  // Run the filter pipeline for one run at the current frame, with caching.
  // Pipeline order (per the task contract): (1) confidence ≥ conf;
  // (2) hide phantoms; (3) containment merge. Counts: phantoms = boxes dropped
  // by step 2; merged = boxes absorbed by step 3.
  function filterDetections(runId, doc, state) {
    const { frame, confThreshold, hidePhantoms, containmentMerge: merge } = state;
    const key = cacheKey(runId, frame, confThreshold, hidePhantoms, merge);
    const hit = filteredCache.get(key);
    if (hit) return hit;

    const raw = doc.byFrame.get(frame)?.detections ?? [];
    // (1) confidence
    let kept = raw.filter((d) => Number(d.confidence) >= confThreshold);

    // (2) phantoms
    let phantoms = 0;
    if (hidePhantoms) {
      const after = [];
      for (const d of kept) {
        if (isPhantom(d, doc.staticTileRects)) phantoms += 1;
        else after.push(d);
      }
      kept = after;
    }

    // (3) containment merge
    let merged = 0;
    if (merge) {
      const before = kept.length;
      kept = containmentMerge(kept, 0.5, 0);
      merged = before - kept.length;
    }

    const result = { dets: kept, phantoms, merged };
    if (filteredCache.size >= FILTER_CACHE_MAX) filteredCache.clear();
    filteredCache.set(key, result);
    return result;
  }

  // ── Drawing helpers (all coords are normalized [0,1] → CSS px) ─────────
  function draw() {
    const state = store.get();
    const { contentBox } = state;
    const dpr = (contentBox && contentBox.dpr) || window.devicePixelRatio || 1;

    // Reset transform & clear the entire backing store.
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    // clearRect in CSS px (the dpr scale handles the backing store).
    ctx.clearRect(0, 0, canvas.width / dpr, canvas.height / dpr);

    if (!contentBox) return;
    const { x: cbX, y: cbY, w: cbW, h: cbH } = contentBox;
    const zoom = state.zoom || 1;

    // normalized → CSS px
    const px = (nx) => cbX + nx * cbW;
    const py = (ny) => cbY + ny * cbH;

    drawTiles(state, px, py, cbW, zoom);
    drawDetections(state, px, py, zoom);

    publishCounts(state);
  }

  function drawTiles(state, px, py, cbW, zoom) {
    const { tileSourceRun, runDocs, frame } = state;
    if (!tileSourceRun) return;
    const doc = runDocs.get(tileSourceRun);
    if (!doc) return;

    // Per-frame tiles (each with its own category) if present, else fall back
    // to the static config rects treated as multi-scale.
    const perFrame = doc.byFrame.get(frame)?.tiles;
    let tiles;
    if (perFrame && perFrame.length) {
      tiles = perFrame;
    } else if (doc.staticTileRects && doc.staticTileRects.length) {
      tiles = doc.staticTileRects.map((r) => ({ ...r, category: 'multi-scale' }));
    } else {
      return;
    }

    const strokeW = 1.5 / zoom;
    const tagFont = 10 / zoom;
    const pad = 6 / zoom;

    for (const t of tiles) {
      const category = t.category || 'multi-scale';
      const color = TILE_COLORS[category] || TILE_COLORS['multi-scale'];
      const x = px(t.x);
      const y = py(t.y);
      const w = t.w * cbW;
      const h = t.h * (py(1) - py(0)); // cbH via py span (avoids passing it in)

      // Low-opacity fill so tiles read as regions, not boxes (DS §3.4.1).
      ctx.globalAlpha = 0.06;
      ctx.fillStyle = color;
      ctx.fillRect(x, y, w, h);
      ctx.globalAlpha = 1;

      ctx.lineWidth = strokeW;
      ctx.strokeStyle = color;
      const dash = DASHED_CATEGORIES.has(category);
      if (dash) ctx.setLineDash([6 / zoom, 4 / zoom]);
      ctx.strokeRect(x, y, w, h);
      if (dash) ctx.setLineDash([]);

      // Category tag at the tile top-left — skip when the tile is too narrow.
      if (w >= 60) {
        ctx.font = `${tagFont}px ${monoFont()}`;
        ctx.textBaseline = 'top';
        const label = category;
        const tw = ctx.measureText(label).width;
        const tagH = tagFont + pad;
        ctx.fillStyle = 'rgba(0,0,0,0.55)';
        ctx.fillRect(x + pad, y + pad, tw + pad, tagH);
        ctx.fillStyle = color;
        ctx.fillText(label, x + pad + pad / 2, y + pad + pad / 2);
      }
    }
  }

  function drawDetections(state, px, py, zoom) {
    const { enabledRuns, runColors, runDocs, hoveredDet } = state;
    if (!enabledRuns || !enabledRuns.length) return;
    const cbHspan = py(1) - py(0);
    const cbWspan = px(1) - px(0);

    for (const runId of enabledRuns) {
      const doc = runDocs.get(runId);
      if (!doc) continue;
      const slot = runColors[runId] || 1;
      const color = RUN_PALETTE[(slot - 1) % RUN_PALETTE.length];

      const { dets } = filterDetections(runId, doc, state);
      for (let i = 0; i < dets.length; i++) {
        const d = dets[i];
        const bbox = d.bbox || [];
        if (bbox.length < 4) continue;
        const x = px(Number(bbox[0]));
        const y = py(Number(bbox[1]));
        const w = Number(bbox[2]) * cbWspan;
        const h = Number(bbox[3]) * cbHspan;

        const isHover =
          hoveredDet && hoveredDet.runId === runId && hoveredDet.index === i;

        // Hover glow: outer white stroke first (DS §4.7.1 highlight).
        if (isHover) {
          ctx.lineWidth = 7 / zoom;
          ctx.strokeStyle = 'rgba(255,255,255,0.5)';
          ctx.strokeRect(x, y, w, h);
        }

        // Dark halo under the colored stroke (DS §3.5.2 / §7.2).
        ctx.lineWidth = 5 / zoom;
        ctx.strokeStyle = 'rgba(0,0,0,0.6)';
        ctx.strokeRect(x, y, w, h);

        // Colored stroke (thicker when hovered).
        ctx.lineWidth = (isHover ? 4 : 3) / zoom;
        ctx.strokeStyle = color;
        ctx.strokeRect(x, y, w, h);

        // Label chip — skip on tiny boxes to keep the canvas readable.
        if (w >= 14) drawChip(d, x, y, color, zoom, py(0));
      }
    }
  }

  function drawChip(d, x, y, color, zoom, contentTop) {
    const fontPx = 11 / zoom;
    const padX = 4 / zoom;
    const padY = 2 / zoom;
    const chipH = fontPx + padY * 2;
    const label = (d.label || 'person');
    const conf = Number.isFinite(Number(d.confidence))
      ? Number(d.confidence).toFixed(2)
      : '';
    let text = conf ? `${label} ${conf}` : label;
    if (d.track_id != null) text += ` #${d.track_id}`;

    ctx.font = `${fontPx}px ${monoFont()}`;
    ctx.textBaseline = 'top';
    const tw = ctx.measureText(text).width;
    const chipW = tw + padX * 2;

    // Above the box top-left; if that clips above the content top (NOT the
    // canvas top — there may be a letterbox bar), draw it inside instead.
    let chipY = y - chipH;
    if (chipY < (contentTop ?? 0)) chipY = y;

    ctx.fillStyle = color;
    ctx.fillRect(x, chipY, chipW, chipH);
    ctx.fillStyle = '#04121F'; // --text-on-accent
    ctx.fillText(text, x + padX, chipY + padY);
  }

  function monoFont() {
    return MONO_FONT;
  }

  // Recompute per-run counts and publish to the store only when the signature
  // changes (avoids 30fps DOM churn in the HUD — DS §7.7).
  function publishCounts(state) {
    const { enabledRuns, runDocs } = state;
    const counts = {};
    const sigParts = [];
    for (const runId of enabledRuns || []) {
      const doc = runDocs.get(runId);
      if (!doc) {
        counts[runId] = { shown: 0, phantoms: 0, merged: 0 };
        sigParts.push(`${runId}:0:0:0`);
        continue;
      }
      const { dets, phantoms, merged } = filterDetections(runId, doc, state);
      const shown = dets.length;
      counts[runId] = { shown, phantoms, merged };
      sigParts.push(`${runId}:${shown}:${phantoms}:${merged}`);
    }
    const sig = sigParts.join('|');
    if (sig !== lastCountsSig) {
      lastCountsSig = sig;
      store.set({ frameCounts: counts });
    }
  }

  // ── Subscription: repaint only on relevant changes (never cursorNorm) ──
  store.subscribe((_state, changed) => {
    // A new runDocs Map means a variant switch — cached filter results for
    // the old docs must not survive (run ids/frames could collide).
    if (changed.has('runDocs')) filteredCache.clear();
    for (const k of REDRAW_KEYS) {
      if (changed.has(k)) {
        draw();
        return;
      }
    }
  });

  // Initial paint.
  draw();
}
