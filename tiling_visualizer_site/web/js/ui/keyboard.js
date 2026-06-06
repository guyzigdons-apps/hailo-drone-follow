// Keyboard shortcuts (DS §6.10 / §6.11). Active in Viewer mode; ignored while
// a text input is focused or a platform modifier (Cmd/Ctrl) is held.
import { toggleRun, currentVariantRuns } from './runs-panel.js';

const SPEEDS = [0.25, 0.5, 1, 2, 4];
const ZOOM_MIN = 1;
const ZOOM_MAX = 8;

function isTypingTarget(el) {
  if (!el) return false;
  const tag = el.tagName;
  return tag === 'INPUT' || tag === 'SELECT' || tag === 'TEXTAREA' || el.isContentEditable;
}

export function initKeyboard(store, stage) {
  const seek = (n) => stage && stage.seekToFrame && stage.seekToFrame(n);

  function zoomBy(factor) {
    const z = store.get().zoom || 1;
    const next = Math.min(ZOOM_MAX, Math.max(ZOOM_MIN, z * factor));
    if (next !== z) store.set({ zoom: next }); // stage re-clamps pan
  }

  function stepSpeed(dir) {
    const cur = store.get().speed || 1;
    let idx = SPEEDS.indexOf(cur);
    if (idx < 0) idx = SPEEDS.indexOf(1);
    idx = Math.min(SPEEDS.length - 1, Math.max(0, idx + dir));
    store.set({ speed: SPEEDS[idx] });
  }

  function cycleTileSource() {
    const state = store.get();
    const runs = currentVariantRuns(state);
    const withTiles = runs.filter((r) => {
      const doc = state.runDocs && state.runDocs.get(r.id);
      return doc && (doc.hasTiles || (doc.staticTileRects && doc.staticTileRects.length));
    });
    // Cycle: [run0, run1, …, null] → null wraps back to run0.
    const order = [...withTiles.map((r) => r.id), null];
    const cur = state.tileSourceRun;
    const i = order.indexOf(cur);
    const next = order[(i + 1) % order.length];
    store.set({ tileSourceRun: next });
  }

  function handle(e) {
    if (store.get().mode !== 'viewer') return;
    if (e.metaKey || e.ctrlKey) return;
    if (isTypingTarget(e.target)) return;

    const { frame, totalFrames } = store.get();
    const last = Math.max(0, (totalFrames || 1) - 1);

    switch (e.key) {
      case ' ':
        e.preventDefault();
        store.set({ playing: !store.get().playing });
        return;
      case '[':
        seek(frame - 1);
        return;
      case ']':
        seek(frame + 1);
        return;
      case 'Home':
        e.preventDefault();
        seek(0);
        return;
      case 'End':
        e.preventDefault();
        seek(last);
        return;
      case 'f':
        store.set({ zoom: 1, panX: 0, panY: 0 });
        return;
      case '+':
      case '=':
        zoomBy(1.25);
        return;
      case '-':
        zoomBy(1 / 1.25);
        return;
      case 't':
        cycleTileSource();
        return;
      case 'p':
        store.set({ hidePhantoms: !store.get().hidePhantoms });
        return;
      case 'm':
        store.set({ containmentMerge: !store.get().containmentMerge });
        return;
      case ',':
        stepSpeed(-1);
        return;
      case '.':
        stepSpeed(+1);
        return;
      case 'g':
        e.preventDefault();
        openGoto(store, seek);
        return;
      case '?':
        e.preventDefault();
        openShortcuts();
        return;
      default:
        break;
    }

    // Run visibility toggles 1..8 (manifest order).
    if (e.key >= '1' && e.key <= '8') {
      const runs = currentVariantRuns(store.get());
      const idx = Number(e.key) - 1;
      if (idx < runs.length) toggleRun(store, runs[idx].id, { alt: e.altKey });
    }
  }

  window.addEventListener('keydown', handle);
}

// ── Go-to-frame prompt (DS §6.11) ───────────────────────────────────────────
function openGoto(store, seek) {
  if (document.querySelector('.goto-overlay')) return;
  const ov = document.createElement('div');
  ov.className = 'goto-overlay';
  const card = document.createElement('div');
  card.className = 'goto-card';
  const lbl = document.createElement('label');
  lbl.textContent = 'Go to frame';
  const input = document.createElement('input');
  input.type = 'number';
  input.min = '0';
  input.value = String(store.get().frame || 0);
  const go = document.createElement('button');
  go.type = 'button';
  go.className = 'btn btn--primary';
  go.textContent = 'Go';

  const close = () => {
    document.removeEventListener('keydown', onKey, true);
    ov.remove();
  };
  const submit = () => {
    const n = parseInt(input.value, 10);
    if (Number.isFinite(n)) seek(n);
    close();
  };
  const onKey = (e) => {
    if (e.key === 'Escape') { e.stopPropagation(); close(); }
    else if (e.key === 'Enter') { e.stopPropagation(); submit(); }
  };
  go.addEventListener('click', submit);
  ov.addEventListener('mousedown', (e) => { if (e.target === ov) close(); });
  document.addEventListener('keydown', onKey, true);

  card.appendChild(lbl);
  card.appendChild(input);
  card.appendChild(go);
  ov.appendChild(card);
  document.body.appendChild(ov);
  input.focus();
  input.select();
}

// ── Shortcuts overlay (DS §6.10) ────────────────────────────────────────────
const BINDINGS = [
  ['Space', 'Play / pause'],
  ['[  ]', 'Step frame −1 / +1'],
  ['Home  End', 'First / last frame'],
  ['f', 'Reset view'],
  ['+  −', 'Zoom in / out'],
  ['1 – 8', 'Toggle run visibility (Alt = solo)'],
  ['t', 'Cycle tile source'],
  ['p', 'Toggle hide-phantoms'],
  ['m', 'Toggle containment merge'],
  [',  .', 'Speed down / up'],
  ['g', 'Go to frame'],
  ['?', 'This help'],
];

function openShortcuts() {
  if (document.querySelector('.kbd-overlay')) {
    document.querySelector('.kbd-overlay').remove();
    return;
  }
  const ov = document.createElement('div');
  ov.className = 'kbd-overlay';
  const card = document.createElement('div');
  card.className = 'kbd-card';
  const h = document.createElement('div');
  h.className = 'panel__header';
  h.textContent = 'KEYBOARD SHORTCUTS';
  card.appendChild(h);
  const table = document.createElement('div');
  table.className = 'kbd-table';
  for (const [keys, desc] of BINDINGS) {
    const k = document.createElement('kbd');
    k.textContent = keys;
    const d = document.createElement('span');
    d.textContent = desc;
    table.appendChild(k);
    table.appendChild(d);
  }
  card.appendChild(table);
  ov.appendChild(card);

  const close = () => {
    document.removeEventListener('keydown', onKey, true);
    ov.remove();
  };
  const onKey = (e) => {
    if (e.key === 'Escape' || e.key === '?') { e.stopPropagation(); close(); }
  };
  ov.addEventListener('mousedown', () => close());
  document.addEventListener('keydown', onKey, true);
  document.body.appendChild(ov);
}
