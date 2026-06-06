# Dynamic Tiling Benchmark Visualizer — Design Specification

> Produced by the app-designer subagent on 2026-06-06. This is the authoritative visual/UX
> spec for the hosted HTML visualizer. The implementation plan
> (`2026-06-06-tiling-visualizer-site.md`) references section numbers in this document.

A self-contained design spec for a static (no-backend, no-build) HTML/CSS/JS internal tool that visualizes drone-video object-detection benchmark runs. Targets Hailo engineers on 1080p–1440p monitors. Everything is numbered for direct implementation.

---

## 1. Information Architecture

### 1.1 Top-level model
Single-page application (SPA), no router/framework required. Two primary **modes** switched by a segmented control in the global top bar, never a full page reload:

- **1.1.1 Viewer mode** (default) — the video + canvas-overlay workspace. This is where 90% of time is spent.
- **1.1.2 Metrics mode** — the trials.json comparison tables.

Both modes share one global selection context (selected video, FOV, set of active runs, confidence threshold). Switching modes preserves context. Rationale: an engineer inspects a frame in Viewer, flips to Metrics to see the aggregate number, flips back — context must never reset.

### 1.2 Selection hierarchy
The data is a 3-level tree, surfaced left-to-right:

```
1.2.1  Video        (e.g. baseline_0025, dense_0027)
1.2.2  └ FOV variant   (fov50 / fov60 / fov70)
1.2.3     └ Runs       (dynamic scheduler, dense baseline, GT tracks, …) — multi-select, color-coded
```

`manifest.json` is loaded once on boot and drives all three selectors. The manifest is grouped by video; each video lists its FOV variants; each variant lists its runs with paths to `frames.json` and the experiment's `trials.json`.

### 1.3 Navigation model
- **1.3.1** Global top bar (persistent): app title + Hailo wordmark (left), Video + FOV selectors (center-left), Viewer/Metrics segmented toggle (center), manifest status / reload (right).
- **1.3.2** Left rail (Viewer mode): Run list + global filters + tile-source selector. Collapsible.
- **1.3.3** Center stage (Viewer mode): video canvas + HUD overlay + transport bar.
- **1.3.4** Right rail (Viewer mode): per-frame inspector — live detection list for the current frame, per-run counts, legend. Collapsible.

### 1.4 Canonical user flow ("pick a video → compare two runs")
1. **1.4.1** Boot → manifest loads → Video selector populated; nothing else selected. Center stage shows an empty state (§6.2).
2. **1.4.2** User picks a Video → FOV selector populates and auto-selects the first variant (fov50). Run list populates with all runs for that variant, all **off** by default except the primary dynamic run (auto-enabled, assigned color slot 1).
3. **1.4.3** First frame of the enabled run renders on the canvas. Transport becomes active.
4. **1.4.4** User toggles a second run on (e.g. dense baseline) → it gets color slot 2; both overlay simultaneously, frame-synced.
5. **1.4.5** User picks which run's **tiles** to show via the tile-source radio (only one at a time — §2.2). Detections from all enabled runs stay visible.
6. **1.4.6** User scrubs / plays / steps to find a divergence frame. HUD shows per-run det counts so disagreement is obvious.
7. **1.4.7** User flips to Metrics mode → both runs are pre-selected as comparison columns → reads coverage / mean IoU / drift / recovery side by side, best-in-column highlighted.
8. **1.4.8** Flip back to Viewer; everything (frame index, zoom, threshold) intact.

### 1.5 State that must persist across mode switches (in-memory; optionally URL hash for shareable deep-links)
`videoId, fovVariant, enabledRunIds[], runColorMap, tileSourceRunId, confThreshold, currentFrame, zoom, pan, hidePhantoms, containmentMerge`. Encode in `location.hash` (e.g. `#v=baseline_0025&fov=50&runs=dyn,dense&f=312&conf=0.30`) so a found frame can be shared by URL — no backend needed.

---

## 2. Layout Spec — Main Viewer

### 2.1 Region grid
CSS Grid, three columns + a top bar. Reference width 1440px; values scale per §2.7.

```
┌─────────────────────────────────────────────────────────────────────┐
│ TOP BAR  (h: 48px, fixed)                                             │
├──────────────┬───────────────────────────────────────┬──────────────┤
│ LEFT RAIL    │            CENTER STAGE               │ RIGHT RAIL   │
│ (w: 280px)   │            (fluid, min 640px)         │ (w: 300px)   │
│              │ ┌───────────────────────────────────┐ │              │
│ Runs         │ │   Canvas / video  (16:9 letterbox)│ │ Frame        │
│ Filters      │ │   HUD overlay (corners)           │ │ Inspector    │
│ Tile source  │ │                                   │ │ Legend       │
│              │ └───────────────────────────────────┘ │              │
│              │   TRANSPORT BAR (h: 72px)             │              │
└──────────────┴───────────────────────────────────────┴──────────────┘
```

- **2.1.1** Top bar: fixed 48px height, full width, sits above all rails.
- **2.1.2** Left rail: default 280px, collapsible to a 44px icon strip (toggle button at its top-right). Vertical scroll if run list overflows.
- **2.1.3** Center stage: fluid, `min-width: 640px`. Contains the canvas area (flex-grow) and the transport bar pinned to the bottom.
- **2.1.4** Right rail: default 300px, collapsible to 0 (slide-out). Vertical scroll.
- **2.1.5** Collapsing either rail gives the canvas its width back immediately (Grid `auto`/`fr` recalculation, no JS resize of canvas needed beyond the existing resize observer).

### 2.2 Canvas area (center)
- **2.2.1** The `<video>` element is the base layer; a single `<canvas>` is absolutely positioned exactly over it (same CSS box). Both are letterboxed inside a black 16:9 stage container so 1080p footage maps cleanly.
- **2.2.2** Canvas backing-store resolution = stage CSS size × `devicePixelRatio` (sharp boxes on HiDPI). All overlay coordinates are normalized [0,1]; multiply by stage pixel dimensions at draw time.
- **2.2.3** The video itself is NOT scaled by zoom/pan transforms. Instead, zoom/pan is applied as a shared CSS `transform: translate() scale()` on a wrapper containing BOTH video and canvas, so boxes and pixels move together (§2.6, pitfall §7.5).
- **2.2.4** HUD info renders as DOM elements (not canvas text) absolutely positioned in the four corners of the stage, OUTSIDE the zoom/pan transform (so HUD stays fixed while content pans). See §2.4.

### 2.3 Transport bar (bottom of center stage)
Fixed 72px, two rows:
- **2.3.1 Row 1 — Scrubber:** full-width frame scrubber (range slider styled as a timeline, §3.9.4). Left label `frame N / total`, right label `time mm:ss.mmm`. Buffered/decoded portion shown as a lighter fill behind the handle.
- **2.3.2 Row 2 — Controls (left→right):** Jump-to-start `⏮`, step −1 `◀|`, Play/Pause `▶/⏸` (primary, larger), step +1 `|▶`, jump-to-end `⏭`, then a divider, then Speed dropdown (`0.25× 0.5× 1× 2× 4×`, default 1×), then a Reverse toggle (`⮌`, optional/off by default), then a divider, then a Loop toggle, then (right-aligned) the FPS/decode indicator.
- **2.3.3** Buttons are 32×32 icon buttons; the play button is 40×40 and accent-tinted (§3.9.1).

### 2.4 HUD overlay (corners of canvas stage)
Translucent pills (§3.1 surfaces, ~70% opacity, backdrop-blur), 12px mono text, never intercept pointer events (`pointer-events: none`):
- **2.4.1 Top-left:** `frame 312 / 1320` and `t = 10.40s`.
- **2.4.2 Top-right:** `zoom 2.4×`.
- **2.4.3 Bottom-left:** cursor position in normalized coords `x 0.412  y 0.207` (updates on mousemove; shows `—` when cursor off-canvas).
- **2.4.4 Bottom-right:** per-run detection counts for the current frame, one line per enabled run, each prefixed by its color swatch: `▮ dynamic 4   ▮ dense 7`.
- **2.4.5** HUD pills auto-hide (fade to 0 over 1.5s) during continuous playback after 3s idle of the mouse, reappear on mousemove — keeps footage clean while reviewing.

### 2.5 Left rail contents (top→bottom)
- **2.5.1 Runs panel** (§4.3 run rows).
- **2.5.2 Tile source** selector — radio group, one selectable run + an explicit "None" option (§4.4).
- **2.5.3 Filters** panel — Confidence threshold slider (§4.5), "Hide phantoms" toggle, "Containment merge" toggle (§4.6).

### 2.6 Zoom/pan behavior
- **2.6.1** Wheel = zoom toward cursor (anchor zoom): compute the normalized point under the cursor, scale, re-translate so that point stays under the cursor. Range 1×–8×, wheel step ~1.1× per notch.
- **2.6.2** Drag (left-button) = pan; cursor becomes `grabbing`. Hard clamp pan so the content edge never moves past the stage center.
- **2.6.3** Double-click = reset to 1× / centered. Keyboard `f` does the same.
- **2.6.4** At 1× zoom, panning is disabled (cursor stays default).

### 2.7 Responsive behavior (engineer monitors only — no mobile)
- **2.7.1 ≥1600px:** left 300 / right 320, everything comfortable.
- **2.7.2 1280–1599px (reference):** left 280 / right 300.
- **2.7.3 1024–1279px:** right rail auto-collapses to a toggle; its content (frame inspector) available via a slide-over. Left rail stays.
- **2.7.4 <1024px:** both rails collapse to icon strips by default; this is a degraded state, supported but not optimized.
- **2.7.5** Transport bar and top bar never collapse.

---

## 3. Visual Design System

Dark, dense, instrument-panel aesthetic — Hailo deep-navy with cyan/teal accents. High contrast for reading data over aerial footage.

### 3.1 Color — background & surface layers (elevation by lightness)
| Token | Hex | Use |
|---|---|---|
| `--bg-base` | `#0A1020` | App background (deepest navy) |
| `--bg-canvas-stage` | `#000000` | Letterbox behind video |
| `--surface-1` | `#0F1A2E` | Rails, top bar |
| `--surface-2` | `#15233D` | Panels, cards within rails |
| `--surface-3` | `#1C2E4A` | Raised rows on hover, inputs |
| `--surface-inset` | `#0B1422` | Sunken wells (scrubber track, table body) |
| `--border-subtle` | `#22324F` | Hairline dividers, panel borders |
| `--border-strong` | `#2E445F` | Focused input border |

### 3.2 Color — text hierarchy
| Token | Hex | Use |
|---|---|---|
| `--text-primary` | `#E6EEF8` | Values, headings |
| `--text-secondary` | `#9FB2CC` | Labels, captions |
| `--text-tertiary` | `#647793` | Disabled, hints, units |
| `--text-on-accent` | `#04121F` | Text on cyan accent fills |

### 3.3 Color — accent & semantic
| Token | Hex | Use |
|---|---|---|
| `--accent` | `#19E3D6` | Primary accent (Hailo cyan/teal): active toggles, primary button, focus ring |
| `--accent-hover` | `#3DEDE2` | Hover state of accent |
| `--accent-muted` | `#0E5E5A` | Accent fill backgrounds at low intensity |
| `--info` | `#3AA0FF` | Informational |
| `--success` | `#37D67A` | Best-in-column highlight, "online" |
| `--warning` | `#FFB020` | Drift / caution metrics |
| `--danger` | `#FF5C5C` | Loss events, errors |

### 3.4 Tile category colors (fixed semantic mapping — never reassigned)
| Category | Token | Hex | Stroke style |
|---|---|---|---|
| Multi-scale | `--tile-multi` | `#22D3EE` (cyan) | 2px dashed |
| Single-scale | `--tile-single` | `#FACC15` (yellow) | 2px dashed |
| Dynamic | `--tile-dynamic` | `#34D399` (green) | 2px solid |
- **3.4.1** Tiles always drawn with low-opacity fill (`rgba(...,0.06)`) + colored stroke + a small top-left category tag, so they read as "regions" not "detections" and never compete with bbox colors.

### 3.5 Run overlay palette (8 distinguishable colors that survive aerial footage)
Chosen for separation in hue AND luminance, all readable over green/brown/grey drone terrain. Detection boxes use 3px solid strokes in these colors with a matching filled label chip.

| Slot | Name | Hex |
|---|---|---|
| 1 | Electric cyan | `#00E5FF` |
| 2 | Magenta | `#FF3DDB` |
| 3 | Amber | `#FFA000` |
| 4 | Spring green | `#6EE63A` |
| 5 | Violet | `#A78BFA` |
| 6 | Coral red | `#FF6B5C` |
| 7 | Sky blue | `#5AA9FF` |
| 8 | Hot pink-white | `#FF9EC4` |
- **3.5.1** Slots are assigned in enable-order and stick to a run until it's disabled (color persists in `runColorMap`). The user can manually reassign via the swatch (§4.3.1).
- **3.5.2** Every overlay box gets a 1px dark outer halo (`rgba(0,0,0,0.6)`) drawn under the colored stroke so boxes stay visible over both bright and dark terrain (pitfall §7.2).

### 3.6 Typography
- **3.6.1 UI font stack:** `Inter, "Segoe UI", system-ui, -apple-system, Roboto, Helvetica, Arial, sans-serif`. (Inter optional via local file; stack degrades gracefully — no build step required.)
- **3.6.2 Mono font stack (all numeric/data/coords/metrics):** `"JetBrains Mono", "SF Mono", "Cascadia Code", Consolas, "Roboto Mono", monospace`.
- **3.6.3 Sizes:** Page/section heading 16px/600; panel header 13px/600 uppercase tracking +0.04em; body label 13px/500; secondary label 12px/400; mono data 12.5px/500; HUD mono 12px; table cell mono 12.5px; badge 11px/600.
- **3.6.4** Line-height 1.4 body, 1.2 for dense tables. Tabular figures (`font-variant-numeric: tabular-nums`) on ALL numeric columns so digits align.

### 3.7 Spacing scale
4px base. Tokens: `--sp-1=4, --sp-2=8, --sp-3=12, --sp-4=16, --sp-5=24, --sp-6=32`. Rail padding 16px; panel inner padding 12px; row vertical padding 8px; control gaps 8px.

### 3.8 Radius & elevation
- **3.8.1 Radius:** `--r-sm=4px` (chips, badges, inputs), `--r-md=8px` (cards, panels, buttons), `--r-lg=12px` (modals/slide-overs), pills fully rounded (`9999px`).
- **3.8.2 Elevation:** flat by default (this is an instrument panel — avoid heavy drop shadows). Use border + background-lightness for depth. Only floating layers (dropdowns, tooltips, scrubber preview) get a shadow: `0 8px 24px rgba(0,0,0,0.45)` + 1px `--border-subtle`.
- **3.8.3 Focus ring:** 2px `--accent` outline at 2px offset, on every interactive element (keyboard accessibility, since this is a power tool).

### 3.9 Control styling
- **3.9.1 Buttons:** 32px tall, 8px radius, `--surface-2` bg, `--text-primary`, `--border-subtle` border; hover → `--surface-3`; active/pressed → inset. Primary button (Play) = `--accent` fill, `--text-on-accent` glyph.
- **3.9.2 Icon buttons:** square, transparent bg, icon `--text-secondary`; hover bg `--surface-3`, icon `--text-primary`; active/toggled → `--accent` icon + faint `--accent-muted` bg.
- **3.9.3 Toggles (switches):** 36×20 pill; off = `--surface-inset` track + `--text-tertiary` knob; on = `--accent` track + white knob; 120ms ease transition.
- **3.9.4 Sliders (confidence, scrubber):** track 4px, `--surface-inset`; filled portion `--accent` (confidence) or `--text-secondary` (scrubber position fill); thumb 14px circle `--text-primary` with 2px `--bg-base` ring; thumb grows to 16px on grab. Value bubble (mono) appears above thumb while dragging.
- **3.9.5 Dropdowns (Video/FOV/Speed):** `--surface-2`, 8px radius, chevron `--text-secondary`; open menu = floating layer (§3.8.2) with hover-highlighted rows.
- **3.9.6 Radios (tile source):** custom 16px circle; selected = `--accent` dot.
- **3.9.7 Inputs:** `--surface-3` bg, `--border-subtle`, focus → `--border-strong` + focus ring.

---

## 4. Component Inventory (with states)

### 4.1 Top bar
- **4.1.1 App title block:** Hailo wordmark + "Tiling Benchmark Visualizer". Static.
- **4.1.2 Video selector** (dropdown): states `empty/disabled` (no manifest), `populated`, `selected`, `loading` (spinner in trailing slot while frames fetch).
- **4.1.3 FOV variant selector** (segmented control `50 / 60 / 70`): disabled until a video is chosen; only shows variants that exist for the chosen video; selected segment = `--accent` underline + `--text-primary`.
- **4.1.4 Mode toggle** (segmented `Viewer / Metrics`): selected = accent-filled segment.
- **4.1.5 Manifest status:** dot + label — `loading…` (spinner), `ready · 6 videos` (success dot), `failed` (danger dot, click to retry). Includes a reload icon button.

### 4.2 Runs panel header
- **4.2.1** Title `RUNS` + count `(3)`. Trailing: "Solo/Show all" small text buttons (solo = isolate one run quickly; show-all re-enables).

### 4.3 Run row (the key component)
Single 40px row, left→right:
- **4.3.1 Color swatch** (16×16, the run's slot color, `--r-sm`): hover shows it's clickable; click opens a tiny 8-swatch palette popover to reassign color. State reflects current slot.
- **4.3.2 Visibility eye toggle** (icon): `visible` (filled eye, `--text-primary`) / `hidden` (eye-off, `--text-tertiary`). Toggling hides that run's detections from the canvas immediately.
- **4.3.3 Run label** (e.g. "dynamic dense8x6 disc2fps"): truncates with ellipsis; full name in `title` tooltip. Distinguish run **type** with a tiny leading tag chip: `DYN` / `DENSE` / `GT` (GT styled distinctly — outline, no fill — because ground truth is reference, not a competitor).
- **4.3.4 Tile-source radio** (small, trailing): selecting it sets this run as the tile source (mutually exclusive across rows). Disabled/greyed if the run has no tiles in its frames.json (e.g. GT, dense). Tooltip "no tiles" when disabled.
- **4.3.5 Detection-count badge** (trailing, mono): count of dets in the CURRENT frame after threshold+filters, e.g. `4`. Color = run color at low opacity bg. Updates every frame.
- **4.3.6 Row states:** default; hover (`--surface-3`); selected-as-tile-source (left 2px `--accent` border + tile category dot); disabled-visibility (label dimmed to `--text-tertiary`, swatch desaturated); loading (skeleton shimmer while its frames.json fetches).

### 4.4 Tile-source selector summary
A header above the run radios: `TILES: ▮ dynamic` (shows current source + its category color) with a "None" radio to draw no tiles. Only one source at a time by design (pitfall §7.6 — overlapping tile sets are unreadable).

### 4.5 Confidence threshold slider
- **4.5.1** Labeled `Confidence ≥`, mono value readout `0.30` updating live. Range 0.00–1.00, step 0.01. Live-filters all runs' detections (no debounce needed for canvas redraw of current frame; debounce only any count recomputation across frames).
- **4.5.2** A faint histogram of confidence distribution (current frame, all enabled runs) sits behind the track as a sparkline — engineers immediately see where the threshold bites.

### 4.6 Filter toggles
- **4.6.1 Hide phantoms** (switch + info `(i)` tooltip explaining the phantom definition used). On = drop boxes flagged phantom.
- **4.6.2 Containment merge** (switch + tooltip): on = merge boxes where one contains another above a fixed containment ratio, drawing the merged box and dimming the absorbed ones. Tooltip states the ratio used.

### 4.7 Frame inspector (right rail)
- **4.7.1 Per-run section** (one collapsible group per enabled run, headed by swatch + name + count): a compact list of that frame's detections — `label  conf  [x y w h]` in mono, sorted by confidence desc. Row hover → highlights/pulses the corresponding box on canvas (and vice-versa: hovering a box highlights its list row). Phantom/merged rows tagged with a small icon.
- **4.7.2 Legend** (pinned bottom of right rail): tile category colors + run swatches currently active. Always visible reference.

### 4.8 Tooltip / popover
Floating layer, `--surface-2`, `--border-subtle`, shadow, 12px text, 6px radius, 8px padding, arrow pointer. Used for truncated names, filter explanations, scrubber preview (§6.4).

### 4.9 Toast / status line
Bottom-center transient toast for non-blocking events: "Loaded dense_0027 / fov50 — 4 runs", "frames.json missing for GT track", error fetches. Auto-dismiss 4s; errors are dismiss-manual with a retry button.

---

## 5. Metrics View Design

Goal: read aggregate + per-trial metrics for several run configs at a glance, with best-in-column instantly obvious.

### 5.1 Layout
- **5.1.1 Header strip:** echoes context — `baseline_0025 · fov50` + a run multi-select chip bar (the comparison columns). Chips reuse run colors. A "Sync with Viewer" toggle (default on) keeps these columns equal to the Viewer's enabled runs.
- **5.1.2 Two stacked sections:** (A) Aggregate comparison table at top; (B) Per-trial breakdown table below.

### 5.2 Aggregate comparison — the hero block
Render as a **transposed comparison table**: metrics are ROWS, run configs are COLUMNS (one column per selected run config, header = color swatch + run name). This beats cards because the user compares the SAME metric across runs (read across a row), and metrics differ in directionality.

- **5.2.1 Metric rows (in this order):** Coverage, Mean IoU, Drift rate, Loss events, Recovery rate, Avg tiles/frame. Each row label includes a unit and a tiny ▲better / ▼better arrow indicating which direction is good (Coverage/IoU/Recovery ▲; Drift/Loss ▼). Avg tiles/frame is labeled "cost" (▼ cheaper, but neutral-toned — it's a tradeoff, not a defect).
- **5.2.2 Best-per-row highlighting:** for each metric row, highlight the winning cell — `--success` left-bar + bold value + a small ★. For "Avg tiles/frame", highlight the lowest in neutral accent (not green) since cheaper-but-worse isn't a clean win.
- **5.2.3 Delta vs reference:** if a GT run is among the selection, show non-GT cells' delta vs GT in `--text-tertiary` mono beneath the value (e.g. `0.91  (−0.06)`). Color the delta `--success`/`--danger` by direction-aware goodness.
- **5.2.4 Inline microbars:** each value cell has a thin horizontal bar (0→max across the row) tinted by the run color — turns the table into a mini bar chart without a chart lib.
- **5.2.5 Cell value = mono, tabular-nums.** Coverage/Recovery/Drift as `%` to 1 decimal; IoU to 3 decimals; Loss events to 1 decimal (mean); tiles/frame to 2 decimals.

### 5.3 Per-trial breakdown
- **5.3.1** A dense data table, one row per trial. Columns: Trial/track id, n_frames, coverage, mean IoU, drift rate, loss events, mean time-to-recover, recovery success. Sticky header.
- **5.3.2 Grouping:** rows grouped by run config (group header bar in the run color). Collapsible groups.
- **5.3.3 Sortable** columns (click header → asc/desc, arrow indicator). Default sort: by run, then trial id.
- **5.3.4 Per-column heat:** optional toggle "Heatmap" tints each numeric cell on a min→max color ramp within its column (sequential cyan ramp on dark) so outliers pop. Off by default to keep it clean.
- **5.3.5 Row hover:** full-row highlight.
- **5.3.6 Aggregate footer:** per-group mean footer row (distinct surface) so the aggregate↔per-trial relationship is visible in one place.

### 5.4 Empty/partial metrics
If a run lacks a metric, render `—` in `--text-tertiary`, never `0` (avoids false "worst" highlighting). Exclude `—` cells from best-in-row computation.

### 5.5 Export
A "Copy as CSV/Markdown" button on each table (clipboard only — no backend) so numbers drop straight into a Confluence page or PR description.

---

## 6. Micro-interactions & Polish

- **6.1 Loading states:** Manifest load = top-bar spinner. frames.json fetch per run = run-row skeleton shimmer + disabled transport until at least one run's frames are ready. Never block the whole UI on one slow run — render what's ready, fill in the rest.
- **6.2 Empty states:** (a) No video selected → centered icon + "Select a video to begin" + a hint listing keyboard shortcuts. (b) Video selected but all runs hidden → "All runs hidden — enable a run in the left panel." (c) Frame has zero detections after filtering → subtle centered "no detections at conf ≥ 0.30" hint (auto-fades; never an error).
- **6.3 Hover behaviors:** bidirectional box↔inspector-row highlight (§4.7.1); run-row hover faintly emphasizes that run's boxes (others dim to 60%) — quick visual isolation without clicking.
- **6.4 Scrubber preview:** hovering the scrubber shows a floating tooltip with the target `frame N · t=ss.mmm`. Text-only — don't fake a thumbnail.
- **6.5 Playback smoothness:** drive overlay redraw off `requestVideoFrameCallback` when available (frame-accurate sync to the actual displayed video frame), falling back to `requestAnimationFrame` + `video.currentTime → frame index` mapping. This is the single most important polish item (pitfall §7.1).
- **6.6 Step precision:** frame step ±1 pauses playback, sets `currentTime = (frame ± 1 + 0.5)/fps` (mid-frame offset to avoid landing on a boundary that decodes the wrong frame). Show the new frame index immediately.
- **6.7 Solo interaction:** Alt-click a run's eye = solo that run (hide all others); Alt-click again restores. Mirrors the header Solo button.
- **6.8 Color reassign:** swatch popover changes update everywhere (canvas, HUD, inspector, legend, metrics columns) within the same frame — single source of truth in `runColorMap`.
- **6.9 Persistence:** confidence threshold, filter toggles, rail collapse states, playback speed persist to `localStorage` so the tool reopens the way the engineer left it (selection/frame come from URL hash, §1.5).
- **6.10 Keyboard shortcuts** (active in Viewer; show via `?` overlay): `Space` play/pause · `[` step −1 · `]` step +1 · `Home` first frame · `End` last frame · `f` reset view · `+`/`-` zoom · `1`–`8` toggle run N visibility · `t` cycle tile source · `p` hide-phantoms toggle · `m` containment-merge toggle · `,`/`.` decrease/increase speed · `g` go-to-frame (numeric entry). Shortcuts disabled while a text input is focused.
- **6.11 Frame jump input:** `g` (or clicking the `frame N/total` HUD) opens a tiny numeric entry to jump to an exact frame — essential for "go to frame 312 that the metrics flagged."
- **6.12 Reduced motion:** respect `prefers-reduced-motion` — disable HUD auto-fade and shimmer.

---

## 7. Pitfalls to Avoid (canvas-over-video overlays)

- **7.1 Overlay/video desync.** Driving the canvas from a `setInterval` or naive rAF clock drifts from the actual decoded video frame, so boxes lag the footage. Use `requestVideoFrameCallback` for true frame-sync; when stepping, seek to a mid-frame timestamp (§6.6) and redraw on the `seeked` event, not optimistically.
- **7.2 Boxes invisible over matching terrain.** A green box vanishes over grass, a yellow over sand. Always draw a dark halo/outline under the colored stroke (§3.5.2) and keep label chips opaque with contrasting text.
- **7.3 DPR / scaling blur & misalignment.** If the canvas backing store isn't sized to `clientWidth*DPR` (and the context scaled by DPR), boxes are blurry and, worse, offset. Recompute on every resize via `ResizeObserver`; never hard-code pixel sizes.
- **7.4 Letterbox coordinate errors.** 16:9 footage in a non-16:9 stage produces black bars; normalized coords must map to the VIDEO content rect, not the full stage rect, or every box shifts into the letterbox. Compute the content rect (object-fit: contain math) once per resize and map through it.
- **7.5 Zoom/pan applied to canvas but not video (or vice-versa).** They must share ONE transformed wrapper (§2.2.3); transforming only the canvas makes boxes slide off the pixels. Keep HUD outside that transform so it doesn't zoom.
- **7.6 Overlapping tile sets become noise.** Showing tiles from multiple runs at once produces an unreadable thicket of dashed rectangles. Enforce single tile-source (§2.2/4.4). Keep tile fills near-transparent and strokes thinner than detection boxes so tiles read as background regions.
- **7.7 Redrawing the whole canvas per mousemove.** Cursor-coord HUD updates must NOT trigger a full overlay repaint. Update HUD text in DOM only; repaint canvas only on frame change, threshold/filter change, zoom/pan, or hover-highlight (and throttle hover-highlight to rAF).
- **7.8 Confidence filter recomputed across all frames on every slider tick.** Filter only the current frame's draw immediately; debounce any all-frames recomputation so the slider stays smooth.
- **7.9 Color collisions / non-determinism.** Don't randomly assign run colors — use the fixed 8-slot palette in enable order (§3.5.1) so a run keeps its identity across sessions and matches the metrics columns. Never reuse a slot still in use; if >8 runs, cycle with a dashed-stroke variant to disambiguate.
- **7.10 Huge frames.json blocking the main thread.** Index detections by frame number into a flat array up front; don't `.find()` per draw. Keep per-frame lookup O(1).
- **7.11 Loss of state on mode switch / reload.** Persist via URL hash + localStorage (§1.5/6.9). An engineer who found frame 312 and refreshes must land back on frame 312.
- **7.12 Tile vs detection visual confusion.** Solid 3px = detections; dashed (or solid-green per §3.4) thinner = tiles; tiles always carry a category tag. Never let them share a stroke style/weight, or reviewers misread regions as boxes.
- **7.13 Reverse playback expectations.** True reverse decoding is expensive/unreliable in browsers. Implement reverse as stepped seeking at the chosen speed (not real-time decode); label it clearly and keep it optional/off by default so nobody expects smooth 4× reverse.

---

### Implementation notes (non-binding)
Pure static stack: vanilla ES modules + CSS custom properties for the tokens above; one `<video>` + one `<canvas>` + DOM HUD; no framework required. All data via `fetch()` of static JSON next to the HTML. This spec is sufficient to build the Viewer and Metrics modes without further design questions.
