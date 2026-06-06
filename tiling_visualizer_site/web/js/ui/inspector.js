// Frame inspector + legend (DS §4.7). The inspector lists, per enabled run,
// the FILTERED detections for the current frame using the SAME pure pipeline
// the overlay draws with (core/pipeline.js) — so hoveredDet.index, an index
// into that filtered list, maps 1:1 between an inspector row and a canvas box.
//
// NOTE on ordering: DS §4.7.1 asks for confidence-desc, but the canvas draws
// the boxes in pipeline order and hoveredDet.index points into THAT order. To
// keep the bidirectional highlight correct we render in pipeline order (no
// re-sort) — index association with the canvas matters more than sort order.
import { RUN_PALETTE } from './overlay.js';
import { filterDetectionsPure } from '../core/pipeline.js';
import { currentVariantRuns } from './runs-panel.js';

const TILE_LEGEND = [
  { name: 'multi-scale', color: 'var(--tile-multi)', dash: true },
  { name: 'single-scale', color: 'var(--tile-single)', dash: true },
  { name: 'dynamic', color: 'var(--tile-dynamic)', dash: false },
];

function slotColor(slot) {
  return RUN_PALETTE[((slot || 1) - 1) % RUN_PALETTE.length];
}

function fmt(n, dp) {
  return Number.isFinite(Number(n)) ? Number(n).toFixed(dp) : '–';
}

export function initInspector(store) {
  const panel = document.getElementById('inspector');
  const legend = document.getElementById('legend');
  if (!panel) return;

  function renderInspector() {
    const state = store.get();
    const {
      enabledRuns = [], runDocs, runColors = {}, frame,
      confThreshold, hidePhantoms, containmentMerge: merge, hoveredDet,
    } = state;

    panel.innerHTML = '';
    const header = document.createElement('div');
    header.className = 'panel__header';
    header.textContent = 'FRAME INSPECTOR';
    panel.appendChild(header);

    if (!enabledRuns.length) {
      panel.appendChild(emptyLine('No runs enabled'));
      return;
    }

    for (const runId of enabledRuns) {
      const doc = runDocs && runDocs.get ? runDocs.get(runId) : undefined;
      if (!doc) continue;
      const slot = runColors[runId] || 1;
      const color = slotColor(slot);
      const { dets } = filterDetectionsPure(doc, frame, confThreshold, hidePhantoms, merge);

      const group = document.createElement('details');
      group.className = 'insp-group';
      group.open = true;

      const summary = document.createElement('summary');
      summary.className = 'insp-summary';
      const sw = document.createElement('span');
      sw.className = 'insp-swatch';
      sw.style.background = color;
      const lbl = document.createElement('span');
      lbl.className = 'insp-summary__label';
      lbl.textContent = doc.label || runId;
      lbl.title = doc.label || runId;
      const cnt = document.createElement('span');
      cnt.className = 'insp-summary__count';
      cnt.textContent = String(dets.length);
      summary.appendChild(sw);
      summary.appendChild(lbl);
      summary.appendChild(cnt);
      group.appendChild(summary);

      if (!dets.length) {
        group.appendChild(emptyLine(`no detections at conf ≥ ${fmt(confThreshold, 2)}`));
      } else {
        // Render in PIPELINE order — index i is the hoveredDet.index the
        // overlay uses (see module note above).
        dets.forEach((d, i) => {
          const row = document.createElement('div');
          row.className = 'inspector-row';
          if (hoveredDet && hoveredDet.runId === runId && hoveredDet.index === i) {
            row.classList.add('is-hot');
          }
          const bbox = (d.bbox || []).map(Number);
          const coords = bbox.length >= 4
            ? `[${fmt(bbox[0], 3)} ${fmt(bbox[1], 3)} ${fmt(bbox[2], 3)} ${fmt(bbox[3], 3)}]`
            : '[–]';
          const trk = d.track_id != null ? ` #${d.track_id}` : '';
          row.textContent = `${d.label || 'person'} ${fmt(d.confidence, 2)} ${coords}${trk}`;
          row.addEventListener('mouseenter', () =>
            store.set({ hoveredDet: { runId, index: i } }));
          row.addEventListener('mouseleave', () => store.set({ hoveredDet: null }));
          group.appendChild(row);
        });
      }
      panel.appendChild(group);
    }
  }

  function renderLegend() {
    if (!legend) return;
    const state = store.get();
    const { enabledRuns = [], runDocs, runColors = {} } = state;
    const runs = currentVariantRuns(state);

    legend.innerHTML = '';
    const header = document.createElement('div');
    header.className = 'panel__header';
    header.textContent = 'LEGEND';
    legend.appendChild(header);

    // Tile categories.
    for (const t of TILE_LEGEND) {
      const line = document.createElement('div');
      line.className = 'legend-line';
      const mark = document.createElement('span');
      mark.className = 'legend-mark' + (t.dash ? ' legend-mark--dashed' : '');
      mark.style.borderColor = t.color;
      const txt = document.createElement('span');
      txt.textContent = `${t.name} (${t.dash ? 'dashed' : 'solid'})`;
      line.appendChild(mark);
      line.appendChild(txt);
      legend.appendChild(line);
    }

    // Active runs.
    if (enabledRuns.length) {
      const sep = document.createElement('div');
      sep.className = 'legend-sep';
      legend.appendChild(sep);
      for (const runId of enabledRuns) {
        const doc = runDocs && runDocs.get ? runDocs.get(runId) : undefined;
        const run = runs.find((r) => r.id === runId);
        const line = document.createElement('div');
        line.className = 'legend-line';
        const sw = document.createElement('span');
        sw.className = 'legend-swatch';
        sw.style.background = slotColor(runColors[runId] || 1);
        const txt = document.createElement('span');
        txt.className = 'legend-line__label';
        txt.textContent = (doc && doc.label) || (run && run.label) || runId;
        txt.title = txt.textContent;
        line.appendChild(sw);
        line.appendChild(txt);
        legend.appendChild(line);
      }
    }
  }

  const INSP_KEYS = [
    'frame', 'enabledRuns', 'runDocs', 'confThreshold', 'hidePhantoms',
    'containmentMerge', 'runColors', 'hoveredDet',
  ];
  const LEGEND_KEYS = ['enabledRuns', 'runColors', 'runDocs', 'manifest', 'videoId', 'fov'];

  store.subscribe((_state, changed) => {
    if (INSP_KEYS.some((k) => changed.has(k))) renderInspector();
    if (LEGEND_KEYS.some((k) => changed.has(k))) renderLegend();
  });
  renderInspector();
  renderLegend();
}

function emptyLine(text) {
  const el = document.createElement('div');
  el.className = 'insp-empty';
  el.textContent = text;
  return el;
}
