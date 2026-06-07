// Runs panel + tile-source panel (DS §4.2 / §4.3 / §4.4 / §6.3 / §6.7 / §7.9).
//
// Renders one row per run in the CURRENT variant (manifest order): color
// swatch (click → 8-slot palette popover), visibility eye (alt-click = solo),
// type chip (DYN/DENSE/GT), label, tile-source radio, det-count badge. A
// separate TILES panel summarises the current tile source + a None radio.
//
// Slot assignment (DS §7.9 / §3.5.1): on enable, take the LOWEST free slot
// 1..8 not used by another enabled run; the slot sticks while the run stays
// enabled. Never random — a run keeps its identity across sessions.
import { RUN_PALETTE } from './overlay.js';

const PALETTE_SLOTS = 8;

// Inline eye / eye-off icons (16px, stroke currentColor) for the run row
// visibility toggle. Replaces the old ◉/○ glyphs (F10).
const EYE_SVG =
  '<svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" ' +
  'stroke-width="1.6" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true">' +
  '<path d="M1.5 12S5 5 12 5s10.5 7 10.5 7-3.5 7-10.5 7S1.5 12 1.5 12z"></path>' +
  '<circle cx="12" cy="12" r="3"></circle></svg>';
const EYE_OFF_SVG =
  '<svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" ' +
  'stroke-width="1.6" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true">' +
  '<path d="M9.9 5.2A9.6 9.6 0 0 1 12 5c7 0 10.5 7 10.5 7a17 17 0 0 1-3.2 4.1"></path>' +
  '<path d="M6.3 6.6A16.7 16.7 0 0 0 1.5 12S5 19 12 19a9.7 9.7 0 0 0 5-1.3"></path>' +
  '<path d="M9.9 9.9a3 3 0 0 0 4.2 4.2"></path>' +
  '<path d="M2 2l20 20"></path></svg>';

// Solo memory (DS §6.7): the enabled set captured the moment solo engaged, so
// a second alt-click can restore it. Module-level so keyboard `1`-`8` and the
// eye toggle share one notion of "previous set".
let preSoloEnabled = null;

// ── Slot helpers ──────────────────────────────────────────────────────────
function lowestFreeSlot(runColors, enabledRuns, exceptId) {
  const used = new Set();
  for (const id of enabledRuns) {
    if (id === exceptId) continue;
    const s = runColors[id];
    if (s) used.add(s);
  }
  for (let s = 1; s <= PALETTE_SLOTS; s++) if (!used.has(s)) return s;
  return ((enabledRuns.length) % PALETTE_SLOTS) + 1; // >8 runs: cycle
}

function slotColor(slot) {
  return RUN_PALETTE[((slot || 1) - 1) % RUN_PALETTE.length];
}

// Variant runs for the current video/fov (manifest order). Exported shape:
// [{id, label, type, frames_json}]. Returns [] when nothing selected.
export function currentVariantRuns(state) {
  const { manifest, videoId, fov } = state;
  if (!manifest || !videoId) return [];
  const video = manifest.videos.find((v) => v.id === videoId);
  if (!video) return [];
  const variant = (video.variants || []).find((vv) => vv.fov === fov) || video.variants[0];
  return (variant && variant.runs) || [];
}

function docHasTiles(doc) {
  return !!(doc && (doc.hasTiles || (doc.staticTileRects && doc.staticTileRects.length)));
}

// ── Toggle a run's visibility (shared with keyboard `1`-`8`) ────────────────
// alt = solo (enable only this run; alt again when already solo restores).
export function toggleRun(store, runId, { alt = false } = {}) {
  const state = store.get();
  const enabled = state.enabledRuns || [];
  const runColors = state.runColors || {};

  if (alt) {
    const isSoloed = enabled.length === 1 && enabled[0] === runId;
    if (isSoloed && preSoloEnabled) {
      // Restore the pre-solo set, re-deriving colors that still hold.
      const restore = preSoloEnabled.filter(Boolean);
      preSoloEnabled = null;
      const nextColors = {};
      restore.forEach((id) => {
        nextColors[id] = runColors[id] || lowestFreeSlot(nextColors, restore, id);
      });
      store.set({ enabledRuns: restore, runColors: { ...runColors, ...nextColors } });
      return;
    }
    preSoloEnabled = [...enabled];
    const slot = runColors[runId] || 1;
    store.set({ enabledRuns: [runId], runColors: { ...runColors, [runId]: slot } });
    return;
  }

  // Plain toggle membership.
  const isOn = enabled.includes(runId);
  if (isOn) {
    const next = enabled.filter((id) => id !== runId);
    store.set({ enabledRuns: next });
    return;
  }
  const next = [...enabled, runId];
  const keep = runColors[runId];
  const free = !keep || next.some((id) => id !== runId && runColors[id] === keep);
  const slot = free ? lowestFreeSlot(runColors, next, runId) : keep;
  store.set({ enabledRuns: next, runColors: { ...runColors, [runId]: slot } });
}

// ── Init ────────────────────────────────────────────────────────────────────
export function initRunsPanel(store) {
  const runsPanel = document.getElementById('runs-panel');
  const tilesPanel = document.getElementById('tile-source-panel');
  if (!runsPanel || !tilesPanel) return;

  let openPopover = null; // { el, onDocClick } — active swatch palette popover

  function closePopover() {
    if (!openPopover) return;
    document.removeEventListener('mousedown', openPopover.onDocClick, true);
    document.removeEventListener('keydown', openPopover.onKey, true);
    openPopover.el.remove();
    openPopover = null;
  }

  function openSwatchPopover(anchor, runId) {
    closePopover();
    const state = store.get();
    const cur = state.runColors[runId] || 1;
    const pop = document.createElement('div');
    pop.className = 'swatch-pop';
    for (let s = 1; s <= PALETTE_SLOTS; s++) {
      const cell = document.createElement('button');
      cell.type = 'button';
      cell.className = 'swatch-pop__cell' + (s === cur ? ' is-current' : '');
      cell.style.background = slotColor(s);
      cell.title = `Slot ${s}`;
      cell.addEventListener('click', () => {
        const c = store.get().runColors;
        store.set({ runColors: { ...c, [runId]: s } });
        closePopover();
      });
      pop.appendChild(cell);
    }
    // Position under the swatch (panel is position:relative via CSS).
    const aRect = anchor.getBoundingClientRect();
    const pRect = runsPanel.getBoundingClientRect();
    pop.style.left = `${aRect.left - pRect.left}px`;
    pop.style.top = `${aRect.bottom - pRect.top + 4}px`;
    runsPanel.appendChild(pop);

    const onDocClick = (e) => {
      if (!pop.contains(e.target) && e.target !== anchor) closePopover();
    };
    const onKey = (e) => {
      if (e.key === 'Escape') closePopover();
    };
    document.addEventListener('mousedown', onDocClick, true);
    document.addEventListener('keydown', onKey, true);
    openPopover = { el: pop, onDocClick, onKey };
  }

  function render() {
    closePopover();
    const state = store.get();
    const runs = currentVariantRuns(state);
    const { enabledRuns = [], runColors = {}, tileSourceRun, runDocs, frameCounts = {} } = state;

    // ── RUNS panel ───────────────────────────────────────────────────────
    runsPanel.innerHTML = '';
    const header = document.createElement('div');
    header.className = 'panel__header';
    // Left cluster: title + All/None actions together; right: a TILES column
    // label sitting above the per-row tile-source radios (clarifies what the
    // rightmost radio column does).
    const left = document.createElement('span');
    left.className = 'runs-head-left';
    const title = document.createElement('span');
    title.textContent = `RUNS (${runs.length})`;
    const actions = document.createElement('span');
    actions.className = 'runs-actions';
    const allBtn = mkTextBtn('All', 'Show all runs (assign free color slots)');
    const noneBtn = mkTextBtn('None', 'Hide all runs');
    allBtn.addEventListener('click', () => showAll(store, runs));
    noneBtn.addEventListener('click', () => store.set({ enabledRuns: [] }));
    actions.appendChild(allBtn);
    actions.appendChild(noneBtn);
    left.appendChild(title);
    left.appendChild(actions);
    const tilesLabel = document.createElement('span');
    tilesLabel.className = 'runs-tiles-label';
    tilesLabel.textContent = 'Tiles';
    tilesLabel.title = 'Which run’s tile rectangles are drawn on the video (one at a time)';
    header.appendChild(left);
    header.appendChild(tilesLabel);
    runsPanel.appendChild(header);

    if (!runs.length) {
      const empty = document.createElement('div');
      empty.className = 'panel-empty';
      empty.textContent = 'No variant selected';
      runsPanel.appendChild(empty);
    }

    for (const run of runs) {
      const doc = runDocs && runDocs.get ? runDocs.get(run.id) : undefined;
      // Loading skeleton while this run's frames.json hasn't indexed yet.
      if (!doc) {
        const sk = document.createElement('div');
        sk.className = 'run-row skeleton';
        runsPanel.appendChild(sk);
        continue;
      }

      const enabled = enabledRuns.includes(run.id);
      const slot = runColors[run.id] || 1;
      const isTileSource = tileSourceRun === run.id;

      const row = document.createElement('div');
      row.className = 'run-row';
      if (isTileSource) row.classList.add('is-tile-source');
      if (!enabled) row.classList.add('is-disabled');
      row.addEventListener('mouseenter', () => store.set({ hoverRun: run.id }));
      row.addEventListener('mouseleave', () => store.set({ hoverRun: null }));

      // Color swatch.
      const swatch = document.createElement('button');
      swatch.type = 'button';
      swatch.className = 'swatch';
      swatch.style.background = enabled ? slotColor(slot) : 'var(--text-tertiary)';
      swatch.title = enabled ? 'Reassign color' : 'Run hidden';
      swatch.disabled = !enabled;
      swatch.addEventListener('click', (e) => {
        e.stopPropagation();
        if (enabled) openSwatchPopover(swatch, run.id);
      });
      row.appendChild(swatch);

      // Eye toggle.
      const eye = document.createElement('button');
      eye.type = 'button';
      eye.className = 'eye-btn';
      eye.innerHTML = enabled ? EYE_SVG : EYE_OFF_SVG;
      eye.title = enabled
        ? 'Hide run (Alt-click = solo)'
        : 'Show run (Alt-click = solo)';
      eye.setAttribute('aria-pressed', enabled ? 'true' : 'false');
      eye.addEventListener('click', (e) => {
        e.stopPropagation();
        toggleRun(store, run.id, { alt: e.altKey });
      });
      row.appendChild(eye);

      // Type chip.
      const chip = document.createElement('span');
      const type = (run.type || '').toUpperCase();
      chip.className = 'run-chip' + (type === 'GT' ? ' run-chip--gt' : '');
      chip.textContent = type || '?';
      row.appendChild(chip);

      // Label (truncated).
      const label = document.createElement('span');
      label.className = 'run-label';
      label.textContent = run.label || run.id;
      label.title = run.label || run.id;
      row.appendChild(label);

      // Det-count badge.
      const badge = document.createElement('span');
      badge.className = 'count-badge';
      const shown = frameCounts[run.id]?.shown;
      badge.textContent = shown ?? '–';
      const c = slotColor(slot);
      badge.style.background = enabled ? hexToRgba(c, 0.18) : 'transparent';
      badge.style.color = enabled ? c : 'var(--text-tertiary)';
      row.appendChild(badge);

      // Tile-source radio.
      const radio = document.createElement('input');
      radio.type = 'radio';
      radio.name = 'tile-source';
      radio.className = 'tile-radio';
      const hasTiles = docHasTiles(doc);
      radio.disabled = !hasTiles;
      radio.checked = isTileSource;
      if (!hasTiles) radio.title = 'no tiles';
      radio.addEventListener('change', () => {
        if (radio.checked) store.set({ tileSourceRun: run.id });
      });
      row.appendChild(radio);

      runsPanel.appendChild(row);
    }

    // ── TILES panel ────────────────────────────────────────────────────────
    tilesPanel.innerHTML = '';
    const tHeader = document.createElement('div');
    tHeader.className = 'panel__header';
    tHeader.textContent = 'TILES';
    tilesPanel.appendChild(tHeader);

    const summary = document.createElement('div');
    summary.className = 'tiles-summary';
    const srcRun = runs.find((r) => r.id === tileSourceRun);
    if (srcRun) {
      const dot = document.createElement('span');
      dot.className = 'tiles-dot';
      dot.style.background = slotColor(runColors[srcRun.id] || 1);
      summary.appendChild(dot);
      const lbl = document.createElement('span');
      lbl.className = 'tiles-summary__label';
      lbl.textContent = srcRun.label || srcRun.id;
      lbl.title = srcRun.label || srcRun.id;
      summary.appendChild(lbl);
    } else {
      const lbl = document.createElement('span');
      lbl.className = 'tiles-summary__label is-none';
      lbl.textContent = 'No tile source';
      summary.appendChild(lbl);
    }
    tilesPanel.appendChild(summary);

    // "None" radio (same group as the row radios).
    const noneRow = document.createElement('label');
    noneRow.className = 'tiles-none-row';
    const noneRadio = document.createElement('input');
    noneRadio.type = 'radio';
    noneRadio.name = 'tile-source';
    noneRadio.checked = !tileSourceRun;
    noneRadio.addEventListener('change', () => {
      if (noneRadio.checked) store.set({ tileSourceRun: null });
    });
    const noneTxt = document.createElement('span');
    noneTxt.textContent = 'None';
    noneRow.appendChild(noneRadio);
    noneRow.appendChild(noneTxt);
    tilesPanel.appendChild(noneRow);
  }

  const KEYS = [
    'manifest', 'videoId', 'fov', 'enabledRuns', 'runColors',
    'tileSourceRun', 'runDocs', 'frameCounts',
  ];
  store.subscribe((_state, changed) => {
    if (changed.has('videoId') || changed.has('fov') || changed.has('manifest')) {
      preSoloEnabled = null;
    }
    for (const k of KEYS) if (changed.has(k)) { render(); return; }
  });
  render();
}

// Enable every loaded run, assigning the lowest free slot to each newly
// enabled one; keep existing slots for already-enabled runs.
function showAll(store, runs) {
  const state = store.get();
  const runDocs = state.runDocs;
  const next = [];
  const colors = { ...state.runColors };
  for (const run of runs) {
    if (!(runDocs && runDocs.get && runDocs.get(run.id))) continue; // only loaded
    next.push(run.id);
  }
  next.forEach((id) => {
    const cur = colors[id];
    const free = !cur || next.some((o) => o !== id && colors[o] === cur);
    if (free) colors[id] = lowestFreeSlot(colors, next, id);
  });
  store.set({ enabledRuns: next, runColors: colors });
}

function mkTextBtn(text, title) {
  const b = document.createElement('button');
  b.type = 'button';
  b.className = 'text-btn';
  b.textContent = text;
  b.title = title;
  return b;
}

// '#RRGGBB' → 'rgba(r,g,b,a)'. Used for the faint count-badge background.
function hexToRgba(hex, a) {
  const h = hex.replace('#', '');
  const r = parseInt(h.slice(0, 2), 16);
  const g = parseInt(h.slice(2, 4), 16);
  const b = parseInt(h.slice(4, 6), 16);
  return `rgba(${r},${g},${b},${a})`;
}
