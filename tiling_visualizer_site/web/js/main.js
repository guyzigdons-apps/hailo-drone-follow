// Boot orchestrator: builds the store, loads the manifest, drives variant
// selection, and wires hash/localStorage persistence. Later tasks attach
// their own init() calls (stage, transport, runs panel, metrics, …) below.
import { createStore, defaultState, encodeHash, decodeHash } from './state.js';
import { loadManifest, loadRunFrames } from './manifest.js';
import { indexFrames } from './core/frames.js';
import { showToast } from './ui/toast.js';
import { initTopbar, setStatus } from './ui/topbar.js';
import { initStage } from './ui/stage.js';
import { initTransport } from './ui/transport.js';
import { initHud } from './ui/hud.js';
import { initOverlay } from './ui/overlay.js';
import { initFiltersPanel } from './ui/filters-panel.js';
import { initRunsPanel } from './ui/runs-panel.js';
import { initInspector } from './ui/inspector.js';
import { initKeyboard } from './ui/keyboard.js';
import { initMetrics } from './ui/metrics.js';
import { initRails } from './ui/rails.js';

const store = createStore(defaultState());
export { store };

const PREFS_KEY = 'tiling-viz-prefs';
const RUN_PALETTE_SLOTS = 8;

let currentFetchToken = 0;

// ── localStorage prefs (DS §6.9) ─────────────────────────────
function loadPrefs() {
  try {
    const raw = localStorage.getItem(PREFS_KEY);
    if (!raw) return null;
    const p = JSON.parse(raw);
    const out = {};
    if (Number.isFinite(p.confThreshold)) out.confThreshold = p.confThreshold;
    if (typeof p.hidePhantoms === 'boolean') out.hidePhantoms = p.hidePhantoms;
    if (typeof p.containmentMerge === 'boolean') out.containmentMerge = p.containmentMerge;
    if (Number.isFinite(p.speed)) out.speed = p.speed;
    return out;
  } catch {
    return null;
  }
}

function throttleTrailing(fn, ms) {
  let timer = null;
  let lastArgs = null;
  return (...args) => {
    lastArgs = args;
    if (timer) return;
    timer = setTimeout(() => {
      timer = null;
      fn(...lastArgs);
    }, ms);
  };
}

// ── Selection flow (DS §1.4, §6.1) ───────────────────────────
async function selectVariant(videoId, fov, { enabledRuns, frame } = {}) {
  const token = ++currentFetchToken;
  const manifest = store.get().manifest;
  const video = manifest && manifest.videos.find((v) => v.id === videoId);
  if (!video) return;
  let variant = video.variants.find((vv) => vv.fov === fov);
  if (!variant) variant = video.variants[0];
  if (!variant) return;

  // Reset run-derived state for the new variant; render what's ready as it
  // arrives rather than blocking the whole UI on one slow run.
  store.set({
    videoId: video.id,
    fov: variant.fov,
    frame: frame ?? 0,
    totalFrames: variant.frames || 0,
    fps: variant.fps || 30,
    runDocs: new Map(),
    enabledRuns: [],
    runColors: {},
    tileSourceRun: null,
    frameCounts: {},
    hoveredDet: null,
    hoverRun: null,
  });

  const runs = variant.runs || [];
  const results = await Promise.allSettled(
    runs.map((r) => loadRunFrames(r.frames_json))
  );

  const docs = new Map();
  let loadedCount = 0;
  results.forEach((res, i) => {
    const run = runs[i];
    if (res.status === 'fulfilled') {
      docs.set(run.id, indexFrames(res.value));
      loadedCount += 1;
    } else {
      const reason = res.reason && res.reason.message ? res.reason.message : res.reason;
      showToast(`Failed to load run "${run.label || run.id}": ${reason}`, { kind: 'error' });
    }
  });

  // Bail if the user switched variants while we were fetching.
  if (token !== currentFetchToken) return;

  store.set({ runDocs: docs });

  // ── Default-enable runs + assign palette colors ────────────
  const loadedIds = [...docs.keys()];
  let enabled;
  if (Array.isArray(enabledRuns) && enabledRuns.length) {
    enabled = enabledRuns.filter((id) => loadedIds.includes(id));
  } else {
    const primary =
      runs.find((r) => r.type === 'DYN' && docs.has(r.id)) ||
      runs.find((r) => docs.has(r.id));
    enabled = primary ? [primary.id] : [];
  }

  const runColors = {};
  enabled.forEach((id, i) => {
    runColors[id] = (i % RUN_PALETTE_SLOTS) + 1;
  });

  // Tile source: first enabled run whose doc actually carries tiles.
  let tileSourceRun = null;
  for (const id of enabled) {
    const doc = docs.get(id);
    if (doc && (doc.hasTiles || (doc.staticTileRects && doc.staticTileRects.length > 0))) {
      tileSourceRun = id;
      break;
    }
  }

  store.set({ enabledRuns: enabled, runColors, tileSourceRun });

  showToast(`Loaded ${video.title || video.id} / ${variant.fov} — ${loadedCount} runs`);
}

// ── Mode switching (DS §1.3) ─────────────────────────────────
function initModeSwitching() {
  const viewer = document.getElementById('viewer-mode');
  const metrics = document.getElementById('metrics-mode');
  const apply = (mode) => {
    if (viewer) viewer.classList.toggle('hidden', mode !== 'viewer');
    if (metrics) metrics.classList.toggle('hidden', mode !== 'metrics');
  };
  store.subscribe((state, changed) => {
    if (changed.has('mode')) apply(state.mode);
  });
  apply(store.get().mode);
}

// ── Hash writer (DS §1.5, §7.11) ─────────────────────────────
function initHashWriter() {
  const write = throttleTrailing((s) => {
    history.replaceState(
      null,
      '',
      encodeHash({
        videoId: s.videoId,
        fov: s.fov,
        runs: s.enabledRuns,
        frame: s.frame,
        conf: s.confThreshold,
        mode: s.mode,
      })
    );
  }, 250);
  store.subscribe((state, changed) => {
    if (
      changed.has('videoId') ||
      changed.has('fov') ||
      changed.has('enabledRuns') ||
      changed.has('frame') ||
      changed.has('confThreshold') ||
      changed.has('mode')
    ) {
      write(state);
    }
  });
}

// ── Prefs writer (DS §6.9) ───────────────────────────────────
function initPrefsWriter() {
  const write = throttleTrailing((s) => {
    try {
      localStorage.setItem(
        PREFS_KEY,
        JSON.stringify({
          confThreshold: s.confThreshold,
          hidePhantoms: s.hidePhantoms,
          containmentMerge: s.containmentMerge,
          speed: s.speed,
        })
      );
    } catch {
      /* storage unavailable — non-fatal */
    }
  }, 250);
  store.subscribe((state, changed) => {
    if (
      changed.has('confThreshold') ||
      changed.has('hidePhantoms') ||
      changed.has('containmentMerge') ||
      changed.has('speed')
    ) {
      write(state);
    }
  });
}

// ── Boot ─────────────────────────────────────────────────────
async function boot() {
  const prefs = loadPrefs() || {};

  initModeSwitching();
  initHashWriter();
  initPrefsWriter();
  const stage = initStage(store);
  initTransport(store, stage);
  initHud(store);
  initOverlay(store);
  initFiltersPanel(store);
  initRunsPanel(store);
  initInspector(store);
  initKeyboard(store, stage);
  initMetrics(store);
  initRails();

  setStatus('loading', 'loading…');

  let manifest;
  try {
    manifest = await loadManifest();
  } catch (err) {
    setStatus('failed', 'failed');
    showToast(`Manifest failed to load: ${err.message}`, { kind: 'error', sticky: true });
    return;
  }

  store.set({ manifest });
  const n = manifest.videos.length;
  setStatus('ready', `ready · ${n} video${n === 1 ? '' : 's'}`);

  // Init the top bar now that videos exist so the dropdown populates.
  initTopbar(store, { onSelect: (videoId, fov, opts) => selectVariant(videoId, fov, opts) });

  // ── Restore from hash, or stay in the empty state (DS §6.2a) ──
  const hash = decodeHash(location.hash);
  // Merge hash.conf into prefs before the single store.set so confThreshold
  // is only written once and the prefs writer sees the right value.
  if (hash && Number.isFinite(hash.conf)) prefs.confThreshold = hash.conf;
  if (Object.keys(prefs).length) store.set(prefs);

  if (hash && hash.videoId) {
    const video = manifest.videos.find((v) => v.id === hash.videoId);
    if (video) {
      const fov =
        (video.variants.find((vv) => vv.fov === hash.fov) || video.variants[0] || {}).fov || null;
      await selectVariant(video.id, fov, { enabledRuns: hash.runs, frame: hash.frame });
      if (hash.mode === 'metrics') store.set({ mode: 'metrics' });
    }
  }
  // else: empty state — do NOT auto-select a video.
}

if (typeof document !== 'undefined') {
  (async () => {
    try {
      await boot();
    } catch (err) {
      showToast(`Unexpected error during boot: ${err && err.message ? err.message : err}`, {
        kind: 'error',
        sticky: true,
      });
    }
  })();
}
