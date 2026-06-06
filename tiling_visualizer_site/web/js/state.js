// Central store (single source of truth). UI modules subscribe and
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

// Factory (not a shared constant): runDocs/enabledRuns/runColors are mutable
// references — a shared module-level object would leak state between stores.
export function defaultState() {
  return {
  manifest: null,
  videoId: null, fov: null,
  runDocs: new Map(),             // runId -> indexFrames() result
  enabledRuns: [],                // ordered runIds
  runColors: {},                  // runId -> palette slot 1..8
  tileSourceRun: null,
  confThreshold: 0.25,
  hidePhantoms: true, containmentMerge: true,
  mode: 'viewer',
  frame: 0, totalFrames: 0, fps: 30,
  zoom: 1, panX: 0, panY: 0,
  playing: false, speed: 1, loop: false,
  contentBox: null,               // {x,y,w,h} published by stage.js
  frameCounts: {},                // runId -> {shown, phantoms, merged} from overlay.js
  hoveredDet: null,               // {runId, index} | null
  hoverRun: null,                 // runId | null
  };
}

// URL hash codec: #v=0025&fov=fov50&runs=r1,r2&f=312&conf=0.30&m=metrics
export function encodeHash({ videoId, fov, runs, frame, conf, mode }) {
  const p = new URLSearchParams();
  if (videoId) p.set('v', videoId);
  if (fov) p.set('fov', fov);
  if (runs && runs.length) p.set('runs', runs.join(','));
  if (Number.isFinite(frame)) p.set('f', String(frame));
  if (Number.isFinite(conf)) p.set('conf', conf.toFixed(2));
  if (mode && mode !== 'viewer') p.set('m', mode);
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
      mode: p.get('m') === 'metrics' ? 'metrics' : 'viewer',
    };
  } catch { return null; }
}
