// Filters panel (DS §4.5 / §4.6). Renders once into #filters-panel:
//   • Confidence threshold slider (live readout, drives confThreshold).
//   • "Hide phantoms" toggle + (i) tooltip.
//   • "Containment merge" toggle + (i) tooltip.
// All controls reflect external store changes (subscribe). The slider has no
// debounce — the overlay redraws the current frame only and caches filtered
// results per (run, frame, conf, …), so dragging stays smooth (DS §7.8).

const PHANTOM_TIP =
  'Person-class boxes matching a tile rectangle (exact ±0.01, or ≥75% of tile ' +
  'area centered on it) are detector artefacts of the tiling grid';
const MERGE_TIP =
  'Same-class boxes with <50% area whose center lies inside a larger box are ' +
  'merged into it';

export function initFiltersPanel(store) {
  const panel = document.getElementById('filters-panel');
  if (!panel) return;
  const s0 = store.get();

  panel.innerHTML = '';

  // Header.
  const header = document.createElement('div');
  header.className = 'panel__header';
  header.textContent = 'FILTERS';
  panel.appendChild(header);

  // ── Confidence row ────────────────────────────────────────────────────
  const slider = document.createElement('div');
  slider.className = 'slider';

  const labelRow = document.createElement('div');
  labelRow.className = 'slider__label-row';
  const labelText = document.createElement('span');
  labelText.textContent = 'Confidence ≥';
  const readout = document.createElement('span');
  readout.className = 'slider__value';
  readout.textContent = (s0.confThreshold ?? 0).toFixed(2);
  labelRow.appendChild(labelText);
  labelRow.appendChild(readout);

  const range = document.createElement('input');
  range.type = 'range';
  range.min = '0';
  range.max = '1';
  range.step = '0.01';
  range.value = String(s0.confThreshold ?? 0);
  range.setAttribute('aria-label', 'Confidence threshold');

  range.addEventListener('input', () => {
    const v = parseFloat(range.value);
    readout.textContent = v.toFixed(2);
    store.set({ confThreshold: v });
  });

  slider.appendChild(labelRow);
  slider.appendChild(range);
  panel.appendChild(slider);

  // ── Filter toggles ────────────────────────────────────────────────────
  const phantom = buildToggle(
    'Hide phantoms',
    PHANTOM_TIP,
    !!s0.hidePhantoms,
    (checked) => store.set({ hidePhantoms: checked })
  );
  const merge = buildToggle(
    'Containment merge',
    MERGE_TIP,
    !!s0.containmentMerge,
    (checked) => store.set({ containmentMerge: checked })
  );
  panel.appendChild(phantom.row);
  panel.appendChild(merge.row);

  // ── Reflect external store changes ────────────────────────────────────
  store.subscribe((state, changed) => {
    if (changed.has('confThreshold')) {
      const v = state.confThreshold ?? 0;
      // Avoid clobbering the caret mid-drag: only update if it differs.
      if (parseFloat(range.value) !== v) range.value = String(v);
      readout.textContent = v.toFixed(2);
    }
    if (changed.has('hidePhantoms')) phantom.input.checked = !!state.hidePhantoms;
    if (changed.has('containmentMerge')) merge.input.checked = !!state.containmentMerge;
  });
}

// A labeled toggle row: <label text> <(i) info> <switch>.
function buildToggle(text, tip, initial, onChange) {
  const row = document.createElement('div');
  row.className = 'filter-row';

  const label = document.createElement('span');
  label.className = 'filter-row__label';
  label.textContent = text;

  const info = document.createElement('span');
  info.className = 'filter-row__info';
  info.textContent = '(i)';
  info.title = tip;
  info.setAttribute('aria-label', tip);

  const spacer = document.createElement('span');
  spacer.className = 'filter-row__spacer';

  // Toggle switch (DS §3.9.3) — reuses the .toggle component styles.
  const toggle = document.createElement('label');
  toggle.className = 'toggle';
  const input = document.createElement('input');
  input.type = 'checkbox';
  input.checked = initial;
  const track = document.createElement('span');
  track.className = 'toggle__track';
  toggle.appendChild(input);
  toggle.appendChild(track);

  input.addEventListener('change', () => onChange(input.checked));

  row.appendChild(label);
  row.appendChild(info);
  row.appendChild(spacer);
  row.appendChild(toggle);

  return { row, input };
}
