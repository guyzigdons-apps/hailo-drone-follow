# Dynamic Tiling Visualizer Site Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Deliver the dynamic-tiling benchmark visualizer as a professional, static HTML site (video + canvas overlays + metrics tables) packaged with transcoded videos and run data, ready to host on any Hailo server.

**Architecture:** Pure static site — vanilla ES modules + CSS custom properties, no framework, no build step, no backend. A Python packaging script (`package_site.py`) transcodes the 4K HEVC source videos to browser-safe 1080p H.264, copies curated `*.frames.json` / `trials_*.json` run artifacts, and emits a `manifest.json` that drives the UI. The result is a self-contained `dist/` directory servable by nginx or `python3 -m http.server`.

**Tech Stack:** HTML5 `<video>` + `<canvas>` overlay, vanilla JS (ES modules), CSS custom properties, `node --test` for JS unit tests, Python 3 stdlib + `unittest` for the packaging script, ffmpeg/ffprobe for transcoding.

**Design authority:** `docs/superpowers/plans/2026-06-06-tiling-visualizer-site-design-spec.md` (referenced below as **DS §N**). Every implementer MUST read the DS sections named in their task. When this plan and the DS conflict on visual details, the DS wins.

**New top-level directory:** `tiling_visualizer_site/`

```
tiling_visualizer_site/
├── README.md                  # deployment + usage docs (Task 12)
├── package_site.py            # packaging: transcode + copy + manifest (Task 5)
├── site_config.json           # curated content list (Task 5)
├── tests/
│   └── test_package_site.py   # unittest for packaging logic (Task 5)
└── web/                       # the site source (copied verbatim into dist/)
    ├── index.html             # SPA shell (Task 1)
    ├── css/
    │   ├── tokens.css         # design tokens (Task 1)
    │   ├── layout.css         # grid regions (Task 1)
    │   └── components.css     # controls, rails, tables (Tasks 1,6-10)
    └── js/
        ├── core/
        │   ├── geometry.js    # tile-grid math + letterbox mapping (Task 2)
        │   ├── geometry.test.js
        │   ├── filters.js     # phantom + containment-merge ports (Task 3)
        │   ├── filters.test.js
        │   ├── frames.js      # frames.json indexing (Task 4)
        │   └── frames.test.js
        ├── state.js           # central store + URL hash + localStorage (Task 4)
        ├── state.test.js
        ├── manifest.js        # manifest fetch + validation (Task 4)
        ├── main.js            # boot + wiring (Task 6)
        ├── ui/
        │   ├── topbar.js      # video/FOV selectors, mode toggle, status (Task 6)
        │   ├── stage.js       # video+canvas stage, zoom/pan, frame sync (Task 7)
        │   ├── transport.js   # scrubber + playback controls (Task 7)
        │   ├── hud.js         # corner HUD pills (Task 7)
        │   ├── overlay.js     # canvas draw: detections + tiles (Task 8)
        │   ├── filters-panel.js # confidence slider + toggles (Task 8)
        │   ├── runs-panel.js  # run rows + tile-source (Task 9)
        │   ├── inspector.js   # right-rail frame inspector + legend (Task 9)
        │   ├── keyboard.js    # shortcuts (Task 9)
        │   ├── toast.js       # transient status toasts (Task 9)
        │   └── metrics.js     # metrics mode tables (Task 10)
        └── (no other deps)
```

**Verification commands used throughout:**
- JS tests: `node --test tiling_visualizer_site/web/js/**/*.test.js` (Node ≥20 is installed)
- Python tests: `cd tiling_visualizer_site && python3 -m unittest discover -s tests -v`
- Serve locally: `python3 -m http.server 8123 --directory tiling_visualizer_site/dist`

**Git:** All commits on the current `tiling-benchmark` branch. The repo has unrelated dirty files (`reid_manager.py`, submodules, run artifacts) — **stage only `tiling_visualizer_site/` and `docs/superpowers/plans/` files explicitly, never `git add -A`.** `dist/` and transcoded videos must NOT be committed; Task 1 adds a `.gitignore`.

---

## Data formats (reference for all tasks)

### frames.json (input, already exists)
```json
{
  "label": "dynamic-b600",
  "config": {"tiles_x": 8, "tiles_y": 6, "overlap_x": 0.25, "overlap_y": 0.25,
              "include_full_frame": false, "include_center_tile": false,
              "center_tile_size": 0.0, "extra_grids": [[1,1,0,0]], "extra_rects": []},
  "frames": [
    {"frame": 0,
     "detections": [{"label": "person", "confidence": 0.71,
                      "bbox": [0.708, 0.510, 0.011, 0.054], "track_id": 1}],
     "tiles": [{"x": 0.0, "y": 0.0, "w": 0.375, "h": 0.5, "category": "multi-scale"}]}
  ]
}
```
Notes: `config` may be absent or partial. `tiles` per frame is optional. `track_id` optional (GT runs have it). `bbox` = normalized `[x, y, w, h]` top-left + size. Frame indices may have gaps; a missing frame index means "no data for that frame" (render nothing, not previous frame). `category` ∈ `multi-scale | single-scale | dynamic | dynamic-merged`.

### trials.json (input, already exists)
```json
{
  "params": {"video": "...", "budget": 3000.0, "fps": 29.97, "discovery_grid": "8x6", "...": "..."},
  "aggregate": {"n_trials": 2, "mean_coverage": 0.465, "mean_iou": 0.901,
                 "mean_drift_rate": 0.01, "mean_loss_events": 3.5,
                 "mean_time_to_recover": 12.0, "mean_recovery_success": 0.8,
                 "avg_tiles_per_frame": 1.33},
  "per_trial": [{"track_id": 3, "n_frames": 900, "coverage": 0.5, "mean_iou": 0.9,
                  "drift_rate": 0.0, "loss_events": 2, "mean_time_to_recover": 10.0,
                  "recovery_success_rate": 1.0}]
}
```

### manifest.json (output of package_site.py, consumed by the site)
```json
{
  "generated": "2026-06-06T12:00:00Z",
  "title": "Hailo Dynamic Tiling Benchmark",
  "videos": [
    {"id": "0025", "title": "DJI 0025 — two walkers",
     "variants": [
       {"fov": "fov50",
        "video": "data/videos/0025_fov50.mp4",
        "width": 1920, "height": 1080, "fps": 29.97003, "frames": 1314,
        "runs": [
          {"id": "0025_fov50_dyn_t3", "label": "dynamic trial 3", "type": "DYN",
           "frames_json": "data/runs/0025_fov50_dyn_t3.frames.json"}
        ],
        "trials": [
          {"id": "0025_fov50_trials", "label": "dynamic ov0",
           "trials_json": "data/runs/0025_fov50_trials.json"}
        ]}
     ]}
  ]
}
```
`type` ∈ `DYN | DENSE | GT` (drives the run-row tag chip, DS §4.3.3). All paths relative to site root.

---

### Task 1: Scaffold, design tokens, HTML shell, layout CSS

**Files:**
- Create: `tiling_visualizer_site/.gitignore`
- Create: `tiling_visualizer_site/web/index.html`
- Create: `tiling_visualizer_site/web/css/tokens.css`
- Create: `tiling_visualizer_site/web/css/layout.css`
- Create: `tiling_visualizer_site/web/css/components.css`

Read **DS §2 (layout), §3 (visual system)** before starting.

- [ ] **Step 1: `.gitignore`**

```gitignore
dist/
*.mp4
__pycache__/
```

- [ ] **Step 2: `tokens.css`** — transcribe ALL tokens from DS §3.1–3.8 as CSS custom properties on `:root`:

```css
:root {
  /* DS 3.1 surfaces */
  --bg-base: #0A1020; --bg-canvas-stage: #000000;
  --surface-1: #0F1A2E; --surface-2: #15233D; --surface-3: #1C2E4A;
  --surface-inset: #0B1422;
  --border-subtle: #22324F; --border-strong: #2E445F;
  /* DS 3.2 text */
  --text-primary: #E6EEF8; --text-secondary: #9FB2CC;
  --text-tertiary: #647793; --text-on-accent: #04121F;
  /* DS 3.3 accent + semantic */
  --accent: #19E3D6; --accent-hover: #3DEDE2; --accent-muted: #0E5E5A;
  --info: #3AA0FF; --success: #37D67A; --warning: #FFB020; --danger: #FF5C5C;
  /* DS 3.4 tile categories */
  --tile-multi: #22D3EE; --tile-single: #FACC15; --tile-dynamic: #34D399;
  --tile-dynamic-merged: #E879F9;
  /* DS 3.5 run palette: exposed for swatch rendering */
  --run-1: #00E5FF; --run-2: #FF3DDB; --run-3: #FFA000; --run-4: #6EE63A;
  --run-5: #A78BFA; --run-6: #FF6B5C; --run-7: #5AA9FF; --run-8: #FF9EC4;
  /* DS 3.6 type */
  --font-ui: Inter, "Segoe UI", system-ui, -apple-system, Roboto, Helvetica, Arial, sans-serif;
  --font-mono: "JetBrains Mono", "SF Mono", "Cascadia Code", Consolas, "Roboto Mono", monospace;
  /* DS 3.7 spacing */
  --sp-1: 4px; --sp-2: 8px; --sp-3: 12px; --sp-4: 16px; --sp-5: 24px; --sp-6: 32px;
  /* DS 3.8 radius + shadow */
  --r-sm: 4px; --r-md: 8px; --r-lg: 12px;
  --shadow-float: 0 8px 24px rgba(0,0,0,0.45);
}
```

- [ ] **Step 3: `index.html`** — full SPA shell with semantic region containers. JS modules attach into these by id; no inline scripts beyond the single module entry:

```html
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Hailo · Tiling Benchmark Visualizer</title>
<link rel="stylesheet" href="css/tokens.css">
<link rel="stylesheet" href="css/layout.css">
<link rel="stylesheet" href="css/components.css">
</head>
<body>
<header id="topbar" class="topbar">
  <div class="topbar__brand">
    <span class="brand-mark" aria-hidden="true"></span>
    <span class="brand-title">HAILO</span>
    <span class="brand-sub">Tiling Benchmark Visualizer</span>
  </div>
  <div class="topbar__selectors">
    <select id="video-select" class="select" disabled aria-label="Video"></select>
    <div id="fov-segmented" class="segmented" role="tablist" aria-label="FOV variant"></div>
  </div>
  <div id="mode-toggle" class="segmented segmented--accent" role="tablist" aria-label="Mode">
    <button class="segmented__btn is-active" data-mode="viewer" role="tab">Viewer</button>
    <button class="segmented__btn" data-mode="metrics" role="tab">Metrics</button>
  </div>
  <div id="manifest-status" class="topbar__status"><span class="status-dot"></span><span class="status-text">loading…</span></div>
</header>

<main id="viewer-mode" class="workspace">
  <aside id="left-rail" class="rail rail--left">
    <section id="runs-panel" class="panel"></section>
    <section id="tile-source-panel" class="panel"></section>
    <section id="filters-panel" class="panel"></section>
  </aside>
  <section class="stage-column">
    <div id="stage" class="stage">
      <div id="stage-transform" class="stage__transform">
        <video id="video" muted playsinline preload="auto"></video>
        <canvas id="overlay-canvas"></canvas>
      </div>
      <div id="hud" class="hud">
        <div class="hud__pill hud__tl" id="hud-frame"></div>
        <div class="hud__pill hud__tr" id="hud-zoom"></div>
        <div class="hud__pill hud__bl" id="hud-cursor"></div>
        <div class="hud__pill hud__br" id="hud-counts"></div>
      </div>
      <div id="stage-empty" class="stage__empty">Select a video to begin</div>
    </div>
    <div id="transport" class="transport"></div>
  </section>
  <aside id="right-rail" class="rail rail--right">
    <section id="inspector" class="panel panel--grow"></section>
    <section id="legend" class="panel panel--pinned"></section>
  </aside>
</main>

<main id="metrics-mode" class="metrics hidden"></main>
<div id="toast-root" class="toast-root" aria-live="polite"></div>
<script type="module" src="js/main.js"></script>
</body>
</html>
```

- [ ] **Step 4: `layout.css`** — CSS Grid per DS §2.1/§2.7: body grid `48px auto`; workspace grid `280px minmax(640px,1fr) 300px`; media queries at 1600px (`300px/320px`) and 1279px (right rail collapsed via `.rail--right.is-collapsed { display:none }`); `.stage` is a flex-grow black 16:9 letterbox container (`position:relative; background:var(--bg-canvas-stage)`); `.stage__transform` absolute, `transform-origin: 0 0`; `#overlay-canvas` absolute inset 0; `.hud__pill` positioned in corners with `pointer-events:none`; `.transport` fixed 72px; `.hidden { display:none !important }`. Rails: `background:var(--surface-1); border-right/left: 1px solid var(--border-subtle); overflow-y:auto; padding:var(--sp-4)`.

- [ ] **Step 5: `components.css`** — base control styles per DS §3.9: `.btn`, `.icon-btn`, `.btn--primary`, `.toggle` (36×20 switch), `.select`, `.segmented`/`.segmented__btn`, `.slider` (4px track, accent fill via `linear-gradient` on `input[type=range]`), `.panel` (surface-2 card, 12px padding, `--r-md`), `.panel__header` (13px/600 uppercase, letter-spacing .04em, `--text-secondary`), `.hud__pill` (rgba(15,26,46,.7), backdrop-filter blur(6px), 12px mono, `--r-sm`, padding 4px 8px), `.toast` (DS §4.9), focus-visible ring per DS §3.8.3. Brand mark: 8×24px rounded rect in `--accent` (simple CSS block, no image asset). Body: `background:var(--bg-base); color:var(--text-primary); font:13px/1.4 var(--font-ui); margin:0; overflow:hidden`.

- [ ] **Step 6: Visual smoke check** — `python3 -m http.server 8123 --directory tiling_visualizer_site/web` and `curl -s localhost:8123 | head -5` returns the HTML. (Full visual check happens in Task 11.) Kill the server.

- [ ] **Step 7: Commit**

```bash
git add tiling_visualizer_site/.gitignore tiling_visualizer_site/web
git commit -m "feat(visualizer-site): scaffold static site shell with Hailo design tokens"
```

---

### Task 2: Core geometry library (TDD)

**Files:**
- Create: `tiling_visualizer_site/web/js/core/geometry.js`
- Test: `tiling_visualizer_site/web/js/core/geometry.test.js`

Port the tile-grid math from `tiling_benchmark/analyze_pxt.py` (`_grid_to_static_tiles`, `_center_tile_rect`, `_compute_tile_rects`) and add the letterbox content-rect mapping (DS §7.4). Keep the math IDENTICAL to Python: `T = 1/(N-(N-1)*o); S = T*(1-o)`.

- [ ] **Step 1: Write failing tests**

```js
// geometry.test.js
import test from 'node:test';
import assert from 'node:assert/strict';
import { gridToStaticTiles, computeTileRects, contentRect } from './geometry.js';

test('1x1 grid is the full frame', () => {
  assert.deepEqual(gridToStaticTiles(1, 1, 0, 0), [{ x: 0, y: 0, w: 1, h: 1 }]);
});

test('2x2 grid no overlap', () => {
  const rects = gridToStaticTiles(2, 2, 0, 0);
  assert.equal(rects.length, 4);
  assert.deepEqual(rects[0], { x: 0, y: 0, w: 0.5, h: 0.5 });
  assert.deepEqual(rects[3], { x: 0.5, y: 0.5, w: 0.5, h: 0.5 });
});

test('8x6 grid with 0.25 overlap matches python math', () => {
  // T = 1/(8 - 7*0.25) = 0.16; S = 0.16*0.75 = 0.12
  const rects = gridToStaticTiles(8, 6, 0.25, 0.25);
  assert.equal(rects.length, 48);
  assert.ok(Math.abs(rects[0].w - 0.16) < 1e-9);
  assert.ok(Math.abs(rects[1].x - 0.12) < 1e-9);
  // last tile ends at 1.0: x = 7*0.12 = 0.84, +0.16 = 1.0
  assert.ok(Math.abs(rects[7].x + rects[7].w - 1.0) < 1e-9);
});

test('invalid grid returns empty', () => {
  assert.deepEqual(gridToStaticTiles(0, 3, 0, 0), []);
});

test('computeTileRects: full config', () => {
  const rects = computeTileRects({
    tiles_x: 2, tiles_y: 2, overlap_x: 0, overlap_y: 0,
    include_full_frame: true, include_center_tile: true, center_tile_size: 0.4,
    extra_grids: [[1, 1, 0, 0]], extra_rects: [[0.1, 0.2, 0.3, 0.4]],
  });
  // 4 grid + 1 full + 1 center + 1 extra-grid + 1 extra-rect
  assert.equal(rects.length, 8);
  assert.deepEqual(rects[4], { x: 0, y: 0, w: 1, h: 1 });        // full frame
  assert.deepEqual(rects[5], { x: 0.3, y: 0.3, w: 0.4, h: 0.4 }); // center tile
  assert.deepEqual(rects[7], { x: 0.1, y: 0.2, w: 0.3, h: 0.4 }); // extra rect
});

test('computeTileRects: null/missing config', () => {
  assert.deepEqual(computeTileRects(null), []);
  assert.deepEqual(computeTileRects({}), []);
});

test('contentRect: 16:9 video in wider stage pillarboxes horizontally', () => {
  const r = contentRect(2000, 900, 1920, 1080);
  assert.equal(r.h, 900);
  assert.equal(r.w, 1600);            // 900 * 16/9
  assert.equal(r.x, 200);             // (2000-1600)/2
  assert.equal(r.y, 0);
});

test('contentRect: 16:9 video in taller stage letterboxes vertically', () => {
  const r = contentRect(1600, 1000, 1920, 1080);
  assert.equal(r.w, 1600);
  assert.equal(r.h, 900);
  assert.equal(r.x, 0);
  assert.equal(r.y, 50);
});
```

- [ ] **Step 2: Run tests, verify they fail**

Run: `node --test tiling_visualizer_site/web/js/core/geometry.test.js`
Expected: FAIL — `Cannot find module ... geometry.js`

- [ ] **Step 3: Implement `geometry.js`**

```js
// Port of tiling_benchmark/analyze_pxt.py tile math. Keep identical:
// T = 1/(N - (N-1)*o); step S = T*(1-o). N==1 → single full-axis tile.
export function gridToStaticTiles(tilesX, tilesY, overlapX, overlapY) {
  if (tilesX < 1 || tilesY < 1) return [];
  const axis = (n, o) => {
    if (n === 1) return [[0.0, 1.0]];
    const T = 1.0 / (n - (n - 1) * o);
    const S = T * (1.0 - o);
    return Array.from({ length: n }, (_, i) => [i * S, T]);
  };
  const rects = [];
  for (const [y, h] of axis(tilesY, overlapY)) {
    for (const [x, w] of axis(tilesX, overlapX)) {
      rects.push({ x, y, w, h });
    }
  }
  return rects;
}

function centerTileRect(size) {
  const x = (1.0 - size) / 2.0;
  return { x, y: x, w: size, h: size };
}

// Mirror of analyze_pxt._compute_tile_rects: main grid + optional
// full-frame + optional center tile + extra_grids + extra_rects.
export function computeTileRects(config) {
  if (!config) return [];
  const rects = gridToStaticTiles(
    (config.tiles_x | 0) || 0, (config.tiles_y | 0) || 0,
    +config.overlap_x || 0, +config.overlap_y || 0);
  if (config.include_full_frame) rects.push({ x: 0, y: 0, w: 1, h: 1 });
  if (config.include_center_tile) {
    rects.push(centerTileRect(+config.center_tile_size || 0.4));
  }
  for (const g of config.extra_grids || []) {
    rects.push(...gridToStaticTiles(g[0] | 0, g[1] | 0, +g[2] || 0, +g[3] || 0));
  }
  for (const r of config.extra_rects || []) {
    rects.push({ x: +r[0], y: +r[1], w: +r[2], h: +r[3] });
  }
  return rects;
}

// object-fit:contain math (DS §7.4): where the video pixels actually sit
// inside the stage box. All overlay coords map through this rect.
export function contentRect(stageW, stageH, videoW, videoH) {
  const scale = Math.min(stageW / videoW, stageH / videoH);
  const w = videoW * scale, h = videoH * scale;
  return { x: (stageW - w) / 2, y: (stageH - h) / 2, w, h };
}
```

- [ ] **Step 4: Run tests, verify pass**

Run: `node --test tiling_visualizer_site/web/js/core/geometry.test.js`
Expected: all PASS

- [ ] **Step 5: Commit**

```bash
git add tiling_visualizer_site/web/js/core/geometry.js tiling_visualizer_site/web/js/core/geometry.test.js
git commit -m "feat(visualizer-site): tile-grid + letterbox geometry core (port of analyze_pxt math)"
```

---

### Task 3: Core filters library — phantom + containment merge (TDD)

**Files:**
- Create: `tiling_visualizer_site/web/js/core/filters.js`
- Test: `tiling_visualizer_site/web/js/core/filters.test.js`

Faithful port of `tiling_benchmark/analyze_pxt.py` `is_phantom`, `is_contained_fragment`, `containment_merge`. Read those functions in the Python source (lines 102–273) before porting — semantics must match exactly (person-class-only phantoms, exact-tile tol=0.01 match OR ≥75 %-area + center-within-half-tile fallback; containment: same class, strict `area_s < ratio*area_b`, center inside big bbox with slack; area-DESC stable iteration; suppressed bigs don't suppress).

- [ ] **Step 1: Write failing tests**

```js
// filters.test.js
import test from 'node:test';
import assert from 'node:assert/strict';
import { isPhantom, isContainedFragment, containmentMerge } from './filters.js';

const TILES = [{ x: 0, y: 0, w: 0.5, h: 0.5 }, { x: 0.5, y: 0.5, w: 0.5, h: 0.5 }];

test('phantom: person bbox matching a tile rect within tol', () => {
  const det = { label: 'person', confidence: 0.9, bbox: [0.001, 0.002, 0.499, 0.501] };
  assert.equal(isPhantom(det, TILES), true);
});

test('phantom: vehicle never filtered even if tile-shaped', () => {
  const det = { label: 'vehicle', confidence: 0.9, bbox: [0, 0, 0.5, 0.5] };
  assert.equal(isPhantom(det, TILES), false);
});

test('phantom: large-person fallback (>=75% tile area, centered)', () => {
  // tile (0,0,.5,.5): area .25, center (.25,.25). det area .2 >= .1875, center close.
  const det = { label: 'person', bbox: [0.03, 0.03, 0.45, 0.45] };
  assert.equal(isPhantom(det, TILES), true);
});

test('phantom: small person not filtered', () => {
  const det = { label: 'person', bbox: [0.1, 0.1, 0.02, 0.05] };
  assert.equal(isPhantom(det, TILES), false);
});

test('phantom: empty tile list → never phantom', () => {
  const det = { label: 'person', bbox: [0, 0, 0.5, 0.5] };
  assert.equal(isPhantom(det, []), false);
});

test('phantom: class_id===1 path when label missing', () => {
  const det = { class_id: 1, bbox: [0.001, 0.001, 0.5, 0.5] };
  assert.equal(isPhantom(det, TILES), true);
});

test('containment: small same-class det inside big is fragment', () => {
  const big = { label: 'person', bbox: [0.1, 0.1, 0.4, 0.4] };
  const small = { label: 'person', bbox: [0.2, 0.2, 0.1, 0.1] };
  assert.equal(isContainedFragment(small, big), true);
});

test('containment: different class never merges', () => {
  const big = { label: 'person', bbox: [0.1, 0.1, 0.4, 0.4] };
  const small = { label: 'vehicle', bbox: [0.2, 0.2, 0.1, 0.1] };
  assert.equal(isContainedFragment(small, big), false);
});

test('containment: area ratio is strict <', () => {
  const big = { label: 'person', bbox: [0.0, 0.0, 0.4, 0.4] };   // area .16
  const small = { label: 'person', bbox: [0.1, 0.1, 0.4, 0.2] }; // area .08 == .5*.16
  assert.equal(isContainedFragment(small, big, 0.5, 0), false);
});

test('containment: center outside big → keep', () => {
  const big = { label: 'person', bbox: [0.0, 0.0, 0.2, 0.2] };
  const small = { label: 'person', bbox: [0.19, 0.19, 0.1, 0.1] }; // center (.24,.24) outside
  assert.equal(isContainedFragment(small, big), false);
});

test('containmentMerge keeps big, drops contained fragment, preserves order', () => {
  const dets = [
    { label: 'person', bbox: [0.2, 0.2, 0.05, 0.05] }, // fragment of #2
    { label: 'vehicle', bbox: [0.6, 0.6, 0.1, 0.1] },
    { label: 'person', bbox: [0.1, 0.1, 0.4, 0.4] },
  ];
  const kept = containmentMerge(dets);
  assert.equal(kept.length, 2);
  assert.equal(kept[0].label, 'vehicle');  // original order preserved
  assert.equal(kept[1].bbox[2], 0.4);
});

test('containmentMerge with no overlaps returns all', () => {
  const dets = [
    { label: 'person', bbox: [0.0, 0.0, 0.1, 0.1] },
    { label: 'person', bbox: [0.5, 0.5, 0.1, 0.1] },
  ];
  assert.equal(containmentMerge(dets).length, 2);
});
```

- [ ] **Step 2: Run tests, verify fail**

Run: `node --test tiling_visualizer_site/web/js/core/filters.test.js`
Expected: FAIL — module not found

- [ ] **Step 3: Implement `filters.js`**

```js
// Faithful JS port of tiling_benchmark/analyze_pxt.py filtering rules.
// Person-class phantom: bbox matches a tile rect exactly (tol) OR is a
// "large person" (>=75% of a tile's area with center within half a tile
// of the tile center). Vehicles/faces/plates are never phantoms.
export function isPhantom(det, tileRects, tol = 0.01) {
  if (!tileRects || tileRects.length === 0) return false;
  const label = (det.label || '').toLowerCase();
  if (label !== 'person' && det.class_id !== 1) return false;
  const bbox = det.bbox || [];
  if (bbox.length < 4) return false;
  const [bx, by, bw, bh] = bbox.map(Number);
  const detArea = bw * bh;
  const detCx = bx + bw / 2, detCy = by + bh / 2;
  for (const { x: tx, y: ty, w: tw, h: th } of tileRects) {
    if (Math.abs(bx - tx) < tol && Math.abs(by - ty) < tol
        && Math.abs(bw - tw) < tol && Math.abs(bh - th) < tol) return true;
    const tileArea = tw * th;
    if (tileArea > 0 && detArea >= 0.75 * tileArea) {
      const tCx = tx + tw / 2, tCy = ty + th / 2;
      if (Math.abs(detCx - tCx) < 0.5 * tw && Math.abs(detCy - tCy) < 0.5 * th) return true;
    }
  }
  return false;
}

export function isContainedFragment(detSmall, detBig, areaRatioMax = 0.5, centerSlack = 0.0) {
  const lblS = detSmall.label ? detSmall.label.toLowerCase() : null;
  const lblB = detBig.label ? detBig.label.toLowerCase() : null;
  if (lblS !== null && lblB !== null) {
    if (lblS !== lblB) return false;
  } else {
    const cs = detSmall.class_id, cb = detBig.class_id;
    if (cs == null || cb == null || (cs | 0) !== (cb | 0)) return false;
  }
  const bs = detSmall.bbox || [], bb = detBig.bbox || [];
  if (bs.length < 4 || bb.length < 4) return false;
  const [sx, sy, sw, sh] = bs.map(Number);
  const [bx, by, bw, bh] = bb.map(Number);
  const areaS = sw * sh, areaB = bw * bh;
  if (areaB <= 0) return false;
  if (areaS >= areaRatioMax * areaB) return false;          // strict <
  const cx = sx + sw / 2, cy = sy + sh / 2;
  if (cx < bx - centerSlack || cx > bx + bw + centerSlack) return false;
  if (cy < by - centerSlack || cy > by + bh + centerSlack) return false;
  return true;
}

// Area-DESC pass; suppressed dets can't suppress others; returns kept dets
// in ORIGINAL order (matches python containment_merge).
export function containmentMerge(dets, areaRatioMax = 0.5, centerSlack = 0.0) {
  const indexed = dets.map((d, i) => {
    const bbox = d.bbox || [];
    const area = bbox.length >= 4 ? Number(bbox[2]) * Number(bbox[3]) : 0;
    return { i, area, d };
  });
  indexed.sort((a, b) => (b.area - a.area) || (a.i - b.i));
  const suppressed = new Set();
  for (let big = 0; big < indexed.length; big++) {
    if (suppressed.has(indexed[big].i)) continue;
    for (let small = big + 1; small < indexed.length; small++) {
      if (suppressed.has(indexed[small].i)) continue;
      if (isContainedFragment(indexed[small].d, indexed[big].d, areaRatioMax, centerSlack)) {
        suppressed.add(indexed[small].i);
      }
    }
  }
  return dets.filter((_, i) => !suppressed.has(i));
}
```

- [ ] **Step 4: Run tests, verify pass**

Run: `node --test tiling_visualizer_site/web/js/core/filters.test.js`
Expected: all PASS

- [ ] **Step 5: Commit**

```bash
git add tiling_visualizer_site/web/js/core/filters.js tiling_visualizer_site/web/js/core/filters.test.js
git commit -m "feat(visualizer-site): phantom + containment-merge filters (port of analyze_pxt)"
```

---

### Task 4: Frame indexing, manifest loader, state store (TDD for pure logic)

**Files:**
- Create: `tiling_visualizer_site/web/js/core/frames.js` + `frames.test.js`
- Create: `tiling_visualizer_site/web/js/manifest.js`
- Create: `tiling_visualizer_site/web/js/state.js` + `state.test.js`

Read **DS §1.5, §6.9, §7.10, §7.11**.

- [ ] **Step 1: Write failing tests for `frames.js`**

```js
// frames.test.js
import test from 'node:test';
import assert from 'node:assert/strict';
import { indexFrames } from './frames.js';

const DOC = {
  label: 'run-a',
  config: { tiles_x: 2, tiles_y: 2, overlap_x: 0, overlap_y: 0 },
  frames: [
    { frame: 0, detections: [{ label: 'person', confidence: 0.9, bbox: [0.1, 0.1, 0.05, 0.1] }] },
    { frame: 2, detections: [], tiles: [{ x: 0, y: 0, w: 0.5, h: 0.5, category: 'dynamic' }] },
  ],
};

test('indexFrames builds O(1) lookup with gaps as undefined', () => {
  const idx = indexFrames(DOC);
  assert.equal(idx.label, 'run-a');
  assert.equal(idx.maxFrame, 2);
  assert.equal(idx.byFrame.get(0).detections.length, 1);
  assert.equal(idx.byFrame.get(1), undefined);
  assert.equal(idx.byFrame.get(2).tiles.length, 1);
});

test('indexFrames detects per-frame tiles presence', () => {
  assert.equal(indexFrames(DOC).hasTiles, true);
  assert.equal(indexFrames({ label: 'x', frames: [{ frame: 0, detections: [] }] }).hasTiles, false);
});

test('indexFrames precomputes static tile rects from config', () => {
  const idx = indexFrames(DOC);
  assert.equal(idx.staticTileRects.length, 4);
});

test('indexFrames tolerates missing config', () => {
  const idx = indexFrames({ label: 'x', frames: [] });
  assert.deepEqual(idx.staticTileRects, []);
  assert.equal(idx.maxFrame, -1);
});
```

- [ ] **Step 2: Run, verify fail** — `node --test .../frames.test.js` → module not found.

- [ ] **Step 3: Implement `frames.js`**

```js
import { computeTileRects } from './geometry.js';

// One-time index of a frames.json doc: Map frame→entry (O(1) per-draw
// lookup, DS §7.10), static tile rects reconstructed from config (used by
// the phantom filter + tile rendering when per-frame tiles are absent).
export function indexFrames(doc) {
  const byFrame = new Map();
  let maxFrame = -1;
  let hasTiles = false;
  for (const f of doc.frames || []) {
    byFrame.set(f.frame, f);
    if (f.frame > maxFrame) maxFrame = f.frame;
    if (f.tiles && f.tiles.length) hasTiles = true;
  }
  return {
    label: doc.label || 'unnamed',
    config: doc.config || null,
    staticTileRects: computeTileRects(doc.config || null),
    byFrame, maxFrame, hasTiles,
  };
}
```

- [ ] **Step 4: Run frames tests, verify pass.**

- [ ] **Step 5: Write failing tests for `state.js`** — the store is a plain pub/sub with hash (de)serialization kept as PURE functions so they're node-testable:

```js
// state.test.js
import test from 'node:test';
import assert from 'node:assert/strict';
import { createStore, encodeHash, decodeHash } from './state.js';

test('store: set merges and notifies subscribers with changed keys', () => {
  const s = createStore({ a: 1, b: 2 });
  let seen = null;
  s.subscribe((state, changed) => { seen = [state.a, [...changed]]; });
  s.set({ a: 5 });
  assert.deepEqual(seen, [5, ['a']]);
  assert.equal(s.get().b, 2);
});

test('store: set with no actual change does not notify', () => {
  const s = createStore({ a: 1 });
  let calls = 0;
  s.subscribe(() => calls++);
  s.set({ a: 1 });
  assert.equal(calls, 0);
});

test('hash round-trip', () => {
  const sel = { videoId: '0025', fov: 'fov50', runs: ['r1', 'r2'], frame: 312, conf: 0.3 };
  const decoded = decodeHash(encodeHash(sel));
  assert.deepEqual(decoded, sel);
});

test('decodeHash of garbage returns null', () => {
  assert.equal(decodeHash('#not=valid=stuff&&&'), null);
  assert.equal(decodeHash(''), null);
});
```

- [ ] **Step 6: Run, verify fail.**

- [ ] **Step 7: Implement `state.js`**

```js
// Central store (single source of truth, DS §1.5). UI modules subscribe and
// re-render on the keys they care about.
export function createStore(initial) {
  let state = { ...initial };
  const subs = new Set();
  return {
    get: () => state,
    set(patch) {
      const changed = new Set();
      for (const [k, v] of Object.entries(patch)) {
        if (state[k] !== v) changed.add(k);
      }
      if (changed.size === 0) return;
      state = { ...state, ...patch };
      for (const fn of subs) fn(state, changed);
    },
    subscribe(fn) { subs.add(fn); return () => subs.delete(fn); },
  };
}

export const DEFAULT_STATE = {
  manifest: null,                 // parsed manifest.json
  videoId: null, fov: null,
  runDocs: new Map(),             // runId -> indexed frames doc (Task 4 indexFrames)
  enabledRuns: [],                // ordered runIds
  runColors: {},                  // runId -> palette slot 1..8
  tileSourceRun: null,            // runId | null
  confThreshold: 0.25,
  hidePhantoms: true, containmentMerge: true,
  mode: 'viewer',                 // 'viewer' | 'metrics'
  frame: 0, totalFrames: 0, fps: 30,
  zoom: 1, panX: 0, panY: 0,
  playing: false, speed: 1, loop: false,
  hoveredDet: null,               // {runId, index} | null (inspector<->canvas)
};

// URL hash codec (DS §1.5/§7.11): #v=0025&fov=fov50&runs=r1,r2&f=312&conf=0.30
export function encodeHash({ videoId, fov, runs, frame, conf }) {
  const p = new URLSearchParams();
  if (videoId) p.set('v', videoId);
  if (fov) p.set('fov', fov);
  if (runs && runs.length) p.set('runs', runs.join(','));
  if (Number.isFinite(frame)) p.set('f', String(frame));
  if (Number.isFinite(conf)) p.set('conf', conf.toFixed(2));
  return '#' + p.toString();
}

export function decodeHash(hash) {
  if (!hash || hash.length < 2) return null;
  try {
    const p = new URLSearchParams(hash.slice(1));
    const videoId = p.get('v');
    if (!videoId) return null;
    return {
      videoId,
      fov: p.get('fov') || null,
      runs: p.get('runs') ? p.get('runs').split(',') : [],
      frame: p.has('f') ? parseInt(p.get('f'), 10) : 0,
      conf: p.has('conf') ? parseFloat(p.get('conf')) : 0.25,
    };
  } catch { return null; }
}
```

- [ ] **Step 8: Run state tests, verify pass.**

- [ ] **Step 9: Implement `manifest.js`** (fetch wrapper — not unit-tested, exercised in the browser):

```js
// Load + minimally validate data/manifest.json (DS §1.2).
export async function loadManifest(url = 'data/manifest.json') {
  const res = await fetch(url, { cache: 'no-cache' });
  if (!res.ok) throw new Error(`manifest fetch failed: HTTP ${res.status}`);
  const m = await res.json();
  if (!Array.isArray(m.videos)) throw new Error('manifest: missing videos[]');
  for (const v of m.videos) {
    if (!v.id || !Array.isArray(v.variants)) throw new Error(`manifest: bad video entry ${v.id}`);
  }
  return m;
}

export async function loadRunFrames(url) {
  const res = await fetch(url);
  if (!res.ok) throw new Error(`frames fetch failed: HTTP ${res.status} ${url}`);
  return res.json();
}

export async function loadTrials(url) {
  const res = await fetch(url);
  if (!res.ok) throw new Error(`trials fetch failed: HTTP ${res.status} ${url}`);
  return res.json();
}
```

- [ ] **Step 10: Run ALL js tests** — `node --test tiling_visualizer_site/web/js/core/*.test.js tiling_visualizer_site/web/js/state.test.js` → all PASS.

- [ ] **Step 11: Commit**

```bash
git add tiling_visualizer_site/web/js
git commit -m "feat(visualizer-site): frame indexing, manifest loader, state store with URL-hash codec"
```

---

### Task 5: Packaging script + curated config (TDD)

**Files:**
- Create: `tiling_visualizer_site/package_site.py`
- Create: `tiling_visualizer_site/site_config.json`
- Test: `tiling_visualizer_site/tests/test_package_site.py` (+ empty `tests/__init__.py`)

Python 3 stdlib only (json, subprocess, shutil, pathlib, argparse, datetime). ffmpeg/ffprobe are at `/usr/bin/`.

- [ ] **Step 1: Write `site_config.json`** — the curated deliverable content (paths relative to the REPO ROOT; resolved against `--repo-root`, default = two levels up from the script):

```json
{
  "title": "Hailo Dynamic Tiling Benchmark",
  "videos": [
    {
      "id": "0025", "title": "DJI 0025 — baseline walkers",
      "variants": [
        {
          "fov": "fov50",
          "source": "/home/giladn/Videos/Drone/Training/DJI_20260528155151_0025_D_prepared__fov50.mp4",
          "runs": [
            {"id": "0025_fov50_dyn_t3", "label": "dynamic trial 3", "type": "DYN",
             "path": "dynamic_tiling/runs/baseline_0025/frames_fov50/trial_3.frames.json"},
            {"id": "0025_fov50_dyn_t4", "label": "dynamic trial 4", "type": "DYN",
             "path": "dynamic_tiling/runs/baseline_0025/frames_fov50/trial_4.frames.json"},
            {"id": "0025_fov50_dyn_ov25_t3", "label": "dynamic ov25 trial 3", "type": "DYN",
             "path": "dynamic_tiling/runs/baseline_0025/frames_fov50_ov25/trial_3.frames.json"},
            {"id": "0025_fov50_dyn_ov25_t4", "label": "dynamic ov25 trial 4", "type": "DYN",
             "path": "dynamic_tiling/runs/baseline_0025/frames_fov50_ov25/trial_4.frames.json"},
            {"id": "0025_fov50_dense", "label": "dense 12x9 ov25 (GT ref)", "type": "DENSE",
             "path": "dynamic_tiling/runs/dense_0025/fov50/pxt_GT-12x9-25-multi.frames.json"}
          ],
          "trials": [
            {"id": "0025_fov50_trials", "label": "dynamic ov0",
             "path": "dynamic_tiling/runs/baseline_0025/trials_fov50.json"},
            {"id": "0025_fov50_trials_ov25", "label": "dynamic ov25",
             "path": "dynamic_tiling/runs/baseline_0025/trials_fov50_ov25.json"}
          ]
        },
        {
          "fov": "fov60",
          "source": "/home/giladn/Videos/Drone/Training/DJI_20260528155151_0025_D_prepared__fov60.mp4",
          "runs": [
            {"id": "0025_fov60_dyn_ov25_t3", "label": "dynamic ov25 trial 3", "type": "DYN",
             "path": "dynamic_tiling/runs/baseline_0025/frames_fov60_ov25/trial_3.frames.json"},
            {"id": "0025_fov60_dyn_ov25_t4", "label": "dynamic ov25 trial 4", "type": "DYN",
             "path": "dynamic_tiling/runs/baseline_0025/frames_fov60_ov25/trial_4.frames.json"},
            {"id": "0025_fov60_dense", "label": "dense 12x9 ov25 (GT ref)", "type": "DENSE",
             "path": "dynamic_tiling/runs/dense_0025/fov60/pxt_GT-12x9-25-multi.frames.json"}
          ],
          "trials": [
            {"id": "0025_fov60_trials", "label": "dynamic ov0",
             "path": "dynamic_tiling/runs/baseline_0025/trials_fov60.json"},
            {"id": "0025_fov60_trials_ov25", "label": "dynamic ov25",
             "path": "dynamic_tiling/runs/baseline_0025/trials_fov60_ov25.json"}
          ]
        },
        {
          "fov": "fov70",
          "source": "/home/giladn/Videos/Drone/Training/DJI_20260528155151_0025_D_prepared__fov70.mp4",
          "runs": [
            {"id": "0025_fov70_dyn_ov25_t3", "label": "dynamic ov25 trial 3", "type": "DYN",
             "path": "dynamic_tiling/runs/baseline_0025/frames_fov70_ov25/trial_3.frames.json"},
            {"id": "0025_fov70_dyn_ov25_t4", "label": "dynamic ov25 trial 4", "type": "DYN",
             "path": "dynamic_tiling/runs/baseline_0025/frames_fov70_ov25/trial_4.frames.json"},
            {"id": "0025_fov70_dense", "label": "dense 12x9 ov25 (GT ref)", "type": "DENSE",
             "path": "dynamic_tiling/runs/dense_0025/fov70/pxt_GT-12x9-25-multi.frames.json"}
          ],
          "trials": [
            {"id": "0025_fov70_trials", "label": "dynamic ov0",
             "path": "dynamic_tiling/runs/baseline_0025/trials_fov70.json"},
            {"id": "0025_fov70_trials_ov25", "label": "dynamic ov25",
             "path": "dynamic_tiling/runs/baseline_0025/trials_fov70_ov25.json"}
          ]
        }
      ]
    },
    {
      "id": "0026", "title": "DJI 0026 — multi-target",
      "variants": [
        {
          "fov": "fov50",
          "source": "/home/giladn/Videos/Drone/Training/DJI_20260528155239_0026_D_prepared__fov50.mp4",
          "runs": [
            {"id": "0026_fov50_dyn_full", "label": "dynamic full multi", "type": "DYN",
             "path": "dynamic_tiling/runs/dynamic_full_multi_0026_fov50.frames.json"},
            {"id": "0026_fov50_dyn_dense8x6", "label": "dynamic dense8x6 disc2fps", "type": "DYN",
             "path": "dynamic_tiling/runs/dynamic_dense8x6_disc2fps_0026_fov50.frames.json"},
            {"id": "0026_fov50_gt", "label": "GT tracks", "type": "GT",
             "path": "dynamic_tiling/runs/gt_tracks_0026_fov50.frames.json"}
          ],
          "trials": []
        }
      ]
    }
  ]
}
```

If a listed `path` doesn't exist at packaging time, `package_site.py` must FAIL with a clear error listing the missing file (no silent skip — the deliverable must be complete).

- [ ] **Step 2: Write failing tests** — test the pure logic: config validation, manifest building, ffprobe parsing, transcode-command construction, and skip-if-fresh logic. Mock subprocess.

```python
# tests/test_package_site.py
import json
import unittest
from pathlib import Path
from unittest import mock

import sys
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import package_site as ps


class TestProbe(unittest.TestCase):
    def test_parse_ffprobe_output(self):
        out = "h264,1920,1080,30000/1001,1314"
        info = ps.parse_probe_csv(out)
        self.assertEqual(info["width"], 1920)
        self.assertEqual(info["height"], 1080)
        self.assertAlmostEqual(info["fps"], 29.97002997, places=5)
        self.assertEqual(info["frames"], 1314)

    def test_parse_ffprobe_na_frames(self):
        info = ps.parse_probe_csv("h264,1920,1080,30/1,N/A")
        self.assertIsNone(info["frames"])


class TestTranscodeCmd(unittest.TestCase):
    def test_cmd_shape(self):
        cmd = ps.transcode_cmd(Path("/src/a.mp4"), Path("/out/b.mp4"))
        self.assertEqual(cmd[0], "ffmpeg")
        joined = " ".join(cmd)
        self.assertIn("libx264", joined)
        self.assertIn("+faststart", joined)
        self.assertIn("scale=1920:-2", joined)
        self.assertIn("yuv420p", joined)
        self.assertIn("-an", joined)          # no audio track needed

    def test_needs_transcode(self):
        with mock.patch.object(Path, "exists", return_value=False):
            self.assertTrue(ps.needs_transcode(Path("/s.mp4"), Path("/d.mp4")))


class TestManifest(unittest.TestCase):
    def make_cfg(self, tmp):
        run = tmp / "run.frames.json"
        run.write_text(json.dumps({"label": "r", "frames": []}))
        trials = tmp / "trials.json"
        trials.write_text(json.dumps({"aggregate": {}}))
        return {
            "title": "T",
            "videos": [{"id": "v1", "title": "V1", "variants": [{
                "fov": "fov50", "source": str(tmp / "src.mp4"),
                "runs": [{"id": "r1", "label": "run 1", "type": "DYN", "path": run.name}],
                "trials": [{"id": "t1", "label": "trials 1", "path": trials.name}],
            }]}],
        }

    def test_build_manifest_entry(self):
        import tempfile
        with tempfile.TemporaryDirectory() as d:
            tmp = Path(d)
            cfg = self.make_cfg(tmp)
            probe = {"width": 1920, "height": 1080, "fps": 29.97, "frames": 1314}
            man = ps.build_manifest(cfg, probe_lookup=lambda p: probe)
            v = man["videos"][0]["variants"][0]
            self.assertEqual(v["video"], "data/videos/v1_fov50.mp4")
            self.assertEqual(v["fps"], 29.97)
            self.assertEqual(v["runs"][0]["frames_json"], "data/runs/r1.frames.json")
            self.assertEqual(v["trials"][0]["trials_json"], "data/runs/t1.json")

    def test_missing_run_file_raises(self):
        import tempfile
        with tempfile.TemporaryDirectory() as d:
            tmp = Path(d)
            cfg = self.make_cfg(tmp)
            cfg["videos"][0]["variants"][0]["runs"][0]["path"] = "missing.json"
            with self.assertRaises(ps.ConfigError):
                ps.validate_config(cfg, repo_root=tmp)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 3: Run, verify fail** — `cd tiling_visualizer_site && python3 -m unittest discover -s tests -v` → ImportError/AttributeError.

- [ ] **Step 4: Implement `package_site.py`** with this exact public surface (functions used by tests) + a `main()`:

```python
#!/usr/bin/env python3
"""Package the tiling visualizer site into a self-contained dist/ directory.

Reads site_config.json, transcodes source videos to browser-safe 1080p
H.264 (faststart), copies curated frames/trials JSONs, copies web/ and
writes data/manifest.json.

Usage:
    python3 package_site.py [--config site_config.json] [--out dist]
                            [--repo-root ../..] [--skip-transcode]
"""
from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path


class ConfigError(Exception):
    pass


def parse_probe_csv(line: str) -> dict:
    """Parse `codec,width,height,fps_frac,nb_frames` from ffprobe csv output."""
    parts = line.strip().split(",")
    num, den = parts[3].split("/")
    frames = None if parts[4] in ("N/A", "") else int(parts[4])
    return {"width": int(parts[1]), "height": int(parts[2]),
            "fps": float(num) / float(den), "frames": frames}


def probe_video(path: Path) -> dict:
    out = subprocess.run(
        ["ffprobe", "-v", "error", "-select_streams", "v:0",
         "-show_entries", "stream=codec_name,width,height,r_frame_rate,nb_frames",
         "-of", "csv=p=0", str(path)],
        check=True, capture_output=True, text=True).stdout
    info = parse_probe_csv(out)
    if info["frames"] is None:
        # container without nb_frames: count via duration * fps fallback
        dur = subprocess.run(
            ["ffprobe", "-v", "error", "-show_entries", "format=duration",
             "-of", "csv=p=0", str(path)],
            check=True, capture_output=True, text=True).stdout.strip()
        info["frames"] = int(float(dur) * info["fps"])
    # output dimensions after the 1080p transcode
    scale = min(1, 1920 / info["width"])
    info["out_width"] = int(info["width"] * scale)
    info["out_height"] = int(info["height"] * scale) // 2 * 2
    return info


def transcode_cmd(src: Path, dst: Path) -> list[str]:
    return ["ffmpeg", "-y", "-i", str(src),
            "-vf", "scale=1920:-2", "-c:v", "libx264", "-preset", "slow",
            "-crf", "22", "-pix_fmt", "yuv420p", "-movflags", "+faststart",
            "-an", str(dst)]


def needs_transcode(src: Path, dst: Path) -> bool:
    return not dst.exists() or dst.stat().st_mtime < src.stat().st_mtime


def validate_config(cfg: dict, repo_root: Path) -> None:
    missing = []
    for video in cfg.get("videos", []):
        for var in video.get("variants", []):
            src = Path(var["source"])
            if not src.is_absolute():
                src = repo_root / src
            if not src.exists():
                missing.append(str(src))
            for entry in list(var.get("runs", [])) + list(var.get("trials", [])):
                p = repo_root / entry["path"]
                if not p.exists():
                    missing.append(str(p))
    if missing:
        raise ConfigError("missing input files:\n  " + "\n  ".join(missing))


def build_manifest(cfg: dict, probe_lookup) -> dict:
    videos = []
    for video in cfg.get("videos", []):
        variants = []
        for var in video.get("variants", []):
            probe = probe_lookup(Path(var["source"]))
            variants.append({
                "fov": var["fov"],
                "video": f"data/videos/{video['id']}_{var['fov']}.mp4",
                "width": probe.get("out_width", probe["width"]),
                "height": probe.get("out_height", probe["height"]),
                "fps": probe["fps"], "frames": probe["frames"],
                "runs": [{"id": r["id"], "label": r["label"], "type": r["type"],
                           "frames_json": f"data/runs/{r['id']}.frames.json"}
                          for r in var.get("runs", [])],
                "trials": [{"id": t["id"], "label": t["label"],
                             "trials_json": f"data/runs/{t['id']}.json"}
                            for t in var.get("trials", [])],
            })
        videos.append({"id": video["id"], "title": video["title"], "variants": variants})
    return {"generated": datetime.now(timezone.utc).isoformat(),
            "title": cfg.get("title", "Tiling Benchmark"), "videos": videos}


def main(argv=None) -> int:
    here = Path(__file__).resolve().parent
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default=str(here / "site_config.json"))
    ap.add_argument("--out", default=str(here / "dist"))
    ap.add_argument("--repo-root", default=str(here.parent))
    ap.add_argument("--skip-transcode", action="store_true",
                    help="copy/refresh everything except videos (fast iteration)")
    args = ap.parse_args(argv)

    cfg = json.loads(Path(args.config).read_text())
    repo_root = Path(args.repo_root).resolve()
    out = Path(args.out).resolve()
    validate_config(cfg, repo_root)

    # 1. site source -> dist root
    (out / "data" / "videos").mkdir(parents=True, exist_ok=True)
    (out / "data" / "runs").mkdir(parents=True, exist_ok=True)
    shutil.copytree(here / "web", out, dirs_exist_ok=True)
    # tests don't belong in the deliverable
    for t in out.rglob("*.test.js"):
        t.unlink()

    # 2. run/trial JSONs
    for video in cfg["videos"]:
        for var in video["variants"]:
            for r in var.get("runs", []):
                shutil.copy2(repo_root / r["path"], out / "data" / "runs" / f"{r['id']}.frames.json")
            for t in var.get("trials", []):
                shutil.copy2(repo_root / t["path"], out / "data" / "runs" / f"{t['id']}.json")

    # 3. videos
    probes: dict[str, dict] = {}
    for video in cfg["videos"]:
        for var in video["variants"]:
            src = Path(var["source"])
            probes[str(src)] = probe_video(src)
            dst = out / "data" / "videos" / f"{video['id']}_{var['fov']}.mp4"
            if args.skip_transcode:
                print(f"[skip] {dst.name}")
            elif needs_transcode(src, dst):
                print(f"[transcode] {src.name} -> {dst.name}")
                subprocess.run(transcode_cmd(src, dst), check=True,
                               capture_output=True)
            else:
                print(f"[fresh] {dst.name}")

    # 4. manifest
    manifest = build_manifest(cfg, probe_lookup=lambda p: probes[str(p)])
    (out / "data" / "manifest.json").write_text(json.dumps(manifest, indent=2))
    print(f"manifest written: {out / 'data' / 'manifest.json'}")
    print(f"dist ready: {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 5: Run tests, verify pass** — `cd tiling_visualizer_site && python3 -m unittest discover -s tests -v` → all PASS.

- [ ] **Step 6: Commit**

```bash
git add tiling_visualizer_site/package_site.py tiling_visualizer_site/site_config.json tiling_visualizer_site/tests
git commit -m "feat(visualizer-site): packaging script — transcode, curate runs, emit manifest"
```

---

### Task 6: Top bar, boot wiring, data loading

**Files:**
- Create: `tiling_visualizer_site/web/js/main.js`
- Create: `tiling_visualizer_site/web/js/ui/topbar.js`
- Create: `tiling_visualizer_site/web/js/ui/toast.js`
- Modify: `tiling_visualizer_site/web/css/components.css` (selector/segmented/status styles as needed)

Read **DS §1.3, §1.4, §4.1, §4.9, §6.1, §6.2**. No unit tests for DOM modules — they're verified in the browser smoke test (Task 11) and by review; keep ALL conditional logic that can live in `core/`/`state.js` there.

Behavior contract:
- [ ] **Step 1: `toast.js`** — `showToast(msg, {kind: 'info'|'error', sticky: false})` appends a `.toast` div to `#toast-root`, auto-removes after 4 s unless sticky; errors get a Dismiss button. Export a single function.
- [ ] **Step 2: `topbar.js`** — `initTopbar(store)`:
  - Populates `#video-select` from `store.get().manifest.videos` (option label = `title`); on change → selects video, auto-picks first variant (DS §1.4.2), updates store.
  - Renders FOV segmented buttons for the chosen video's variants; active per DS §4.1.3.
  - Mode toggle buttons set `store.set({mode})`; main.js shows/hides `#viewer-mode`/`#metrics-mode`.
  - `#manifest-status` reflects loading/ready (`ready · N videos`)/failed states per DS §4.1.5.
- [ ] **Step 3: `main.js`** — boot sequence:
  1. `createStore(DEFAULT_STATE)`; expose as singleton import for ui modules.
  2. `loadManifest()` → store. On failure: status=failed + sticky error toast.
  3. Apply `decodeHash(location.hash)` if valid (select video/fov/runs/frame/conf), else defaults; restore `confThreshold/hidePhantoms/containmentMerge/speed` from `localStorage` (key `tiling-viz-prefs`, JSON).
  4. On video/fov selection change: fetch all runs' frames.json for that variant in parallel (`Promise.allSettled`), `indexFrames` each, populate `store.runDocs`; enable the first DYN run by default (DS §1.4.2); failed runs get an error toast but don't block others (DS §6.1).
  5. Subscribe a hash-writer: on `videoId/fov/enabledRuns/frame/confThreshold` change, `history.replaceState` the new `encodeHash` (throttled to 250 ms).
  6. Subscribe a prefs-writer to localStorage.
  7. Init all ui modules (topbar now; stage/transport/etc. as they land in later tasks — keep imports commented-in as tasks complete is NOT allowed; instead main.js this task imports only what exists: topbar, toast. Later tasks each add their own `init*` import line to main.js).
- [ ] **Step 4: Manual check** — serve `web/` with a hand-written minimal `web/data/manifest.json` fixture (two fake videos, no real video files needed):

```bash
mkdir -p tiling_visualizer_site/web/data
cat > tiling_visualizer_site/web/data/manifest.json <<'EOF'
{"generated":"x","title":"t","videos":[{"id":"0025","title":"DJI 0025","variants":[{"fov":"fov50","video":"data/videos/none.mp4","width":1920,"height":1080,"fps":29.97,"frames":100,"runs":[],"trials":[]}]}]}
EOF
python3 -m http.server 8123 --directory tiling_visualizer_site/web &
curl -s localhost:8123/data/manifest.json | python3 -m json.tool >/dev/null && echo OK
kill %1
```
Expected: `OK`. (Browser-level verification deferred to Task 11; the fixture stays untracked — `web/data/` is wiped by packaging anyway. Add `web/data/` to `.gitignore`.)
- [ ] **Step 5: Commit** — `git add tiling_visualizer_site/web tiling_visualizer_site/.gitignore && git commit -m "feat(visualizer-site): top bar, boot wiring, manifest-driven selection"`

---

### Task 7: Video stage — frame sync, zoom/pan, transport, HUD

**Files:**
- Create: `tiling_visualizer_site/web/js/ui/stage.js`
- Create: `tiling_visualizer_site/web/js/ui/transport.js`
- Create: `tiling_visualizer_site/web/js/ui/hud.js`
- Modify: `tiling_visualizer_site/web/js/main.js` (add init imports)
- Modify: `tiling_visualizer_site/web/css/components.css` (transport styles)

Read **DS §2.2, §2.3, §2.4, §2.6, §6.5, §6.6, §7.1, §7.3, §7.5**. This is the precision-critical task.

Behavior contract:
- [ ] **Step 1: `stage.js` — `initStage(store)`**:
  - Sets `video.src` from the selected variant; on `loadedmetadata` → `store.set({totalFrames, fps})` from manifest values (manifest is authoritative for fps/frames, not the media element).
  - **Frame sync (DS §7.1):** use `video.requestVideoFrameCallback((now, meta) => { store.set({frame: Math.round(meta.mediaTime * fps)}); reschedule; })`; fallback to rAF + `currentTime` mapping when rVFC is unavailable.
  - **Canvas sizing (DS §7.3):** `ResizeObserver` on `.stage`; backing store = `clientWidth*dpr × clientHeight*dpr`; `ctx.setTransform(dpr,0,0,dpr,0,0)`; recompute `contentRect(stageW, stageH, manifest.width, manifest.height)` and publish it via `store.set({contentBox})` for overlay.js. The `<video>` uses `object-fit: contain` filling the stage; the canvas covers the whole stage and maps normalized coords through contentRect.
  - **Zoom/pan (DS §2.6):** wheel → anchor-zoom toward cursor, clamp zoom [1, 8]; drag pan when zoom > 1, hard-clamped; double-click reset. Apply `transform: translate(panX,panY) scale(zoom)` to `#stage-transform` (video+canvas together, DS §7.5). Store zoom/pan in store.
  - **Seek stepping (DS §6.6):** export `seekToFrame(n)` → clamps to [0,totalFrames-1], pauses, sets `currentTime=(n+0.5)/fps`, redraw fires on rVFC/`seeked`.
  - Empty state `#stage-empty` visible when no video selected; hidden once playing (DS §6.2).
- [ ] **Step 2: `transport.js` — `initTransport(store, {seekToFrame})`** renders into `#transport` per DS §2.3: scrubber (range 0..totalFrames-1, input → seekToFrame, with hover tooltip `frame N · t=ss.mmm` per DS §6.4), buttons ⏮ ◀| ▶/⏸ |▶ ⏭, speed dropdown (0.25/0.5/1/2/4 → `video.playbackRate`), loop toggle, all wired to store. Reverse: omitted from v1 controls per DS §7.13 (frame-step back covers the need).
- [ ] **Step 3: `hud.js` — `initHud(store)`**: fills the four pills per DS §2.4 (frame/time, zoom, cursor normalized coords from stage mousemove mapped through contentRect AND zoom/pan inverse, per-run counts pushed by overlay.js via `store.set({frameCounts})`). DOM-text updates only, no canvas involvement (DS §7.7). Auto-fade per DS §2.4.5 honoring `prefers-reduced-motion` (DS §6.12).
- [ ] **Step 4: Run all existing JS tests still green**: `node --test tiling_visualizer_site/web/js/core/*.test.js tiling_visualizer_site/web/js/state.test.js`
- [ ] **Step 5: Commit** — `git commit -m "feat(visualizer-site): video stage with rVFC frame sync, zoom/pan, transport, HUD"` (explicit adds as usual).

---

### Task 8: Overlay rendering + filters panel

**Files:**
- Create: `tiling_visualizer_site/web/js/ui/overlay.js`
- Create: `tiling_visualizer_site/web/js/ui/filters-panel.js`
- Modify: `tiling_visualizer_site/web/js/main.js`, `components.css`

Read **DS §3.4, §3.5, §7.2, §7.6, §7.8, §7.12** and the data-format reference at the top of this plan.

Behavior contract:
- [ ] **Step 1: `overlay.js` — `initOverlay(store, canvas)`**. Redraw triggers: `frame, enabledRuns, runColors, tileSourceRun, confThreshold, hidePhantoms, containmentMerge, contentBox, hoveredDet` change. Draw order: tiles first (background), then detections.
  - **Tiles** (only for `tileSourceRun`): per-frame `tiles` array if the frame entry has one; else the run's `staticTileRects` (category `multi-scale`). Style per DS §3.4: 1.5px stroke in category color, dashed for multi/single-scale, solid for dynamic; fill `rgba(category, 0.06)`; small category tag text at the tile's top-left corner (6px padding, 10px mono), skipped when the tile is narrower than 60 CSS px to avoid clutter.
  - **Detections per enabled run**: take `runDoc.byFrame.get(frame)?.detections ?? []`, apply in order: confidence floor → `isPhantom` drop (when hidePhantoms, using that run's `staticTileRects`) → `containmentMerge` (when enabled, areaRatioMax 0.5, centerSlack 0). Cache the filtered result per `(runId, frame, conf, hideP, cm)` in a small Map keyed by string, cleared on any filter change (DS §7.8).
  - Box style per DS §3.5: 3px stroke in run color over a 5px `rgba(0,0,0,0.6)` halo stroke; label chip = filled run-color rect, 11px mono `--text-on-accent`-style dark text: `person 0.71` (+ `#track_id` if present). Hovered det (`store.hoveredDet`) gets a 1.25× line width + white outer glow.
  - Coordinate mapping: `px = contentBox.x + nx * contentBox.w` etc. The canvas sits inside the zoom/pan-transformed wrapper, so NO zoom math in draw code — but divide line widths and font sizes by `store.zoom` so strokes stay hairline-constant on screen while zoomed.
  - After each draw, push `store.set({frameCounts})` = `{runId: {shown, phantoms, merged}}` for HUD + run badges.
- [ ] **Step 2: `filters-panel.js` — `initFiltersPanel(store)`** renders into `#filters-panel` per DS §4.5/§4.6: panel header `FILTERS`; confidence slider (0–1 step 0.01, mono readout, live store update on `input`); two toggle switches with `(i)` tooltips ("Person-class boxes matching a tile rectangle (exact ±0.01 or ≥75% area centered) are detector artefacts" / "Same-class boxes with <50% area whose center lies inside a larger box are merged"). Histogram sparkline (DS §4.5.2) is OPTIONAL — implement only if trivial; skip without leaving placeholder UI.
- [ ] **Step 3: Tests still green** (same command as Task 7 Step 4).
- [ ] **Step 4: Commit** — `git commit -m "feat(visualizer-site): canvas overlay rendering with tile categories + live filters"`.

---

### Task 9: Runs panel, tile source, inspector, legend, keyboard

**Files:**
- Create: `tiling_visualizer_site/web/js/ui/runs-panel.js`
- Create: `tiling_visualizer_site/web/js/ui/inspector.js`
- Create: `tiling_visualizer_site/web/js/ui/keyboard.js`
- Modify: `tiling_visualizer_site/web/js/main.js`, `components.css`

Read **DS §4.2–4.4, §4.7, §6.3, §6.7, §6.10, §6.11, §7.9**.

Behavior contract:
- [ ] **Step 1: `runs-panel.js`** per DS §4.3: 40px rows with color swatch (click → 8-swatch popover reassigning `runColors`), eye toggle (alt-click = solo, DS §6.7), type chip (`DYN`/`DENSE`/`GT`, GT outlined), truncated label with `title` tooltip, tile-source radio (disabled + "no tiles" tooltip when `!runDoc.hasTiles && runDoc.staticTileRects.length === 0`), live det-count badge from `store.frameCounts`. Header `RUNS (N)` + Solo/Show-all buttons (DS §4.2). Color slots assigned in enable-order, lowest free slot, sticky while enabled (DS §7.9). Tile-source panel renders the `TILES: <run>` summary + None radio (DS §4.4).
- [ ] **Step 2: `inspector.js`** per DS §4.7: per-enabled-run collapsible groups listing the current frame's FILTERED detections (`label conf [x y w h]` mono, conf-desc). Row hover ↔ `store.set({hoveredDet})` for canvas highlight (both directions; canvas hit-test on mousemove throttled to rAF). Legend pinned at the bottom: 3 tile categories + active run swatches.
- [ ] **Step 3: `keyboard.js`** per DS §6.10/§6.11: Space, `[`/`]`, Home/End, `f`, `+`/`-`, `1`–`8`, `t`, `p`, `m`, `,`/`.`, `g` (small centered prompt for frame number), `?` (shortcut overlay modal). All no-ops when `event.target` is an input/select. Wire through store + `seekToFrame`.
- [ ] **Step 4: Tests green; commit** — `git commit -m "feat(visualizer-site): runs panel, frame inspector, legend, keyboard shortcuts"`.

---

### Task 10: Metrics mode

**Files:**
- Create: `tiling_visualizer_site/web/js/ui/metrics.js`
- Modify: `tiling_visualizer_site/web/js/main.js`, `components.css`

Read **DS §5 (all)**. Data = the variant's `trials[]` entries (each one a column).

Behavior contract:
- [ ] **Step 1: `metrics.js` — `initMetrics(store)`** renders into `#metrics-mode` when `mode === 'metrics'`:
  - Header strip: `«video title» · «fov»` + chips per trials entry (DS §5.1.1; "Sync with Viewer" omitted — trials entries are per-config not per-run here; chips toggle column visibility instead).
  - **Aggregate table** (DS §5.2): metric rows in the exact order/format of §5.2.1/§5.2.5 — Coverage %, Mean IoU (3 dp), Drift rate %, Loss events (1 dp), Time to recover (frames, 1 dp), Recovery rate %, Avg tiles/frame (2 dp, "cost"). Direction arrows on row labels; best-per-row highlight per §5.2.2 (skip `—` cells per §5.4); inline microbars per §5.2.4 (a `div` with width % of row max, background = column accent at 25% opacity).
  - **Per-trial table** (DS §5.3): grouped by trials-config (collapsible group header bar), columns `track id, n_frames, coverage, mean IoU, drift, loss, t-recover, recovery`; sortable headers; group mean footer (§5.3.6).
  - **Copy as Markdown** button per table (DS §5.5) → `navigator.clipboard.writeText` of a pipe-table; toast confirms.
  - Empty state when the variant has no trials: "No trials data for this variant."
  - Loads trials JSONs lazily on first metrics-mode entry per variant (cached in store).
- [ ] **Step 2: Tests green; commit** — `git commit -m "feat(visualizer-site): metrics mode — aggregate comparison + per-trial tables"`.

---

### Task 11: Package, serve, browser smoke test

**Files:**
- Create: `tiling_visualizer_site/dist/` (generated, NOT committed)

- [ ] **Step 1: Build** — `cd tiling_visualizer_site && python3 package_site.py` (first run transcodes 4 videos ≈ several minutes each; run in background, monitor). Expected: `dist ready: ...` and `dist/data/manifest.json` exists, 4 mp4s in `dist/data/videos/` each well under the source size, all curated JSONs in `dist/data/runs/`.
- [ ] **Step 2: Serve** — `python3 -m http.server 8123 --directory tiling_visualizer_site/dist` (background).
- [ ] **Step 3: Automated checks** — `curl` the index, manifest, one frames.json, one video (Range request → expect `206 Partial Content` proving scrub-ability):

```bash
curl -s -o /dev/null -w "%{http_code}\n" localhost:8123/                      # 200
curl -s localhost:8123/data/manifest.json | python3 -m json.tool > /dev/null && echo MANIFEST_OK
curl -s -o /dev/null -w "%{http_code}\n" -H "Range: bytes=0-1000" localhost:8123/data/videos/0025_fov50.mp4   # 206
```

- [ ] **Step 4: Browser verification** — open in a real browser (DISPLAY=:0 chromium/firefox) or drive headless screenshots:

```bash
chromium --headless --disable-gpu --screenshot=/tmp/viz_smoke.png --window-size=1440,900 --virtual-time-budget=8000 "http://localhost:8123/#v=0025&fov=fov50&f=300"
```
Read the screenshot. Expected: dark navy UI, video frame visible, run rows in left rail, boxes on canvas. Check the JS console for errors via `chromium --headless --dump-dom` + a page-error capture, or use Node + puppeteer if available; at minimum confirm no blank page and overlays render. Fix anything broken before proceeding.
- [ ] **Step 5: Functional spot checks in the browser** (manually via screenshots at different hashes): frame 0 vs frame 300 show different boxes; `conf=0.9` hash shows fewer boxes than `conf=0.1`; metrics mode renders tables for 0025/fov50.
- [ ] **Step 6: Commit any fixes** — `git commit -m "fix(visualizer-site): smoke-test fixes"` (only if changes were needed).

---

### Task 12: README + deployment docs

**Files:**
- Create: `tiling_visualizer_site/README.md`

- [ ] **Step 1: Write README** covering: what the tool is (1 paragraph + screenshot reference), repo layout, how to add a run to `site_config.json`, packaging (`python3 package_site.py`, `--skip-transcode` for iteration), local serving, and **deployment to a Hailo server** — three documented options since the target server is TBD: (a) nginx static root snippet (with `mp4` mime + Range support note — nginx serves Range by default), (b) `python3 -m http.server` behind the firewall for quick internal sharing, (c) rsync one-liner `rsync -av dist/ <server>:/var/www/tiling-viz/`. Include the dist size estimate and the note that everything is static — no Python/Hailo runtime needed server-side. Also document the URL-hash deep-link format and keyboard shortcuts table.
- [ ] **Step 2: Commit** — `git add tiling_visualizer_site/README.md && git commit -m "docs(visualizer-site): usage + deployment guide"`.

---

## Self-Review (completed at planning time)

1. **Spec coverage:** video+FOV+run selection (T6/T9), canvas overlays with tiles+detections (T8), confidence slider (T8), transport+speeds+scrubber (T7), zoom/pan (T7), HUD (T7), phantom/containment filters (T3/T8), metrics tables (T10), keyboard (T9), professional Hailo look (T1 + DS), hosting package (T5/T11/T12). Reverse playback deliberately dropped per DS §7.13 (frame-step back retained) — documented in README.
2. **Placeholder scan:** none — optional items (histogram sparkline) are explicitly skip-allowed without placeholder UI.
3. **Type consistency:** store field names (`enabledRuns, runColors, tileSourceRun, confThreshold, hidePhantoms, containmentMerge, frame, contentBox, frameCounts, hoveredDet`) are used identically across T4/T7/T8/T9/T10; `indexFrames` return shape (`label, config, staticTileRects, byFrame, maxFrame, hasTiles`) matches its consumers in T8/T9; manifest field names (`video, fps, frames, runs[].frames_json, trials[].trials_json`) consistent between T5 builder and T6 consumer.

## Post-plan phases (PM-managed, not plan tasks)

- **UI expert peer review:** dispatch code-reviewer + a UI/UX-expert subagent against the design spec; triage findings; fix.
- **Final report:** commit log summary, deployment instructions, open decisions (target server, Jira linkage) for Gilad.
