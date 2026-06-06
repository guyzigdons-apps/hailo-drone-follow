// Metrics mode (DS §5): trials.json comparison tables.
//
// Renders into #metrics-mode. Two stacked tables:
//   (A) Aggregate comparison — transposed: metric ROWS × trials-config COLUMNS,
//       best-per-row highlighting, microbars, n_trials sub-line.
//   (B) Per-trial breakdown — grouped by trials-config, sortable, group-mean
//       footer, sticky header.
//
// Lazy + cached: trials JSONs are fetched only on first metrics entry for a
// given variant, keyed `videoId|fov|trialsId`, in a module-level Map. Work is
// done ONLY while mode === 'metrics' (DS §5; re-render on mode/videoId/fov/
// manifest changes).
import { loadTrials } from '../manifest.js';
import { showToast } from './toast.js';
import { RUN_PALETTE } from './overlay.js';

const TERTIARY = '—';

// videoId|fov|trialsId -> { data } | { error } ; absence = not yet fetched.
const trialsCache = new Map();

// ── Aggregate row definitions (DS §5.2.1 / §5.2.5) ───────────────────────────
// dir: 'up' = higher better, 'down' = lower better. neutral: cost row (tiles).
const AGG_ROWS = [
  { key: 'mean_coverage', label: 'Coverage', unit: '%', dir: 'up', scale: 100, dp: 1 },
  { key: 'mean_iou', label: 'Mean IoU', unit: '', dir: 'up', scale: 1, dp: 3 },
  { key: 'mean_drift_rate', label: 'Drift rate', unit: '%', dir: 'down', scale: 100, dp: 1 },
  { key: 'mean_loss_events', label: 'Loss events', unit: '', dir: 'down', scale: 1, dp: 1 },
  { key: 'mean_time_to_recover', label: 'Time to recover', unit: 'frames', dir: 'down', scale: 1, dp: 1 },
  { key: 'mean_recovery_success', label: 'Recovery rate', unit: '%', dir: 'up', scale: 100, dp: 1 },
  { key: 'avg_tiles_per_frame', label: 'Avg tiles/frame', unit: '', dir: 'down', scale: 1, dp: 2, neutral: true },
];

// ── Per-trial column definitions (DS §5.3) ───────────────────────────────────
const TRIAL_COLS = [
  { key: 'track_id', label: 'track', scale: 1, dp: 0 },
  { key: 'n_frames', label: 'frames', scale: 1, dp: 0 },
  { key: 'coverage', label: 'coverage %', scale: 100, dp: 1, unit: '%' },
  { key: 'mean_iou', label: 'mean IoU', scale: 1, dp: 3 },
  { key: 'drift_rate', label: 'drift %', scale: 100, dp: 1, unit: '%' },
  { key: 'loss_events', label: 'loss', scale: 1, dp: 0 },
  { key: 'mean_time_to_recover', label: 't-recover', scale: 1, dp: 1 },
  { key: 'recovery_success_rate', label: 'recovery %', scale: 100, dp: 1, unit: '%' },
];

function paletteHex(i) {
  return RUN_PALETTE[i % RUN_PALETTE.length];
}

// rgba string for a hex at the given alpha (microbar tint).
function hexAlpha(hex, alpha) {
  const h = hex.replace('#', '');
  const r = parseInt(h.slice(0, 2), 16);
  const g = parseInt(h.slice(2, 4), 16);
  const b = parseInt(h.slice(4, 6), 16);
  return `rgba(${r},${g},${b},${alpha})`;
}

function isNum(v) {
  return typeof v === 'number' && Number.isFinite(v);
}

// Format a raw metric value per its row/col spec; missing -> em dash.
function fmt(raw, scale, dp, unit) {
  if (!isNum(raw)) return TERTIARY;
  const v = raw * scale;
  const s = v.toFixed(dp);
  return unit === '%' ? `${s}%` : s;
}

export function initMetrics(store) {
  const root = document.getElementById('metrics-mode');
  if (!root) return;

  // Per-table sort state for the per-trial table (one shared sort applied
  // within every group). { key, dir: 'asc'|'desc' } | null (= file order).
  let trialSort = null;
  // Column visibility, keyed by trialsId. true = visible.
  let visibleCols = {};
  let lastVariantKey = null;
  let fetchToken = 0;

  function variantKey(s) {
    return s.videoId && s.fov ? `${s.videoId}|${s.fov}` : null;
  }

  function currentVariant(s) {
    const m = s.manifest;
    if (!m || !s.videoId) return null;
    const video = m.videos.find((v) => v.id === s.videoId);
    if (!video) return null;
    const variant = video.variants.find((vv) => vv.fov === s.fov) || video.variants[0];
    return variant ? { video, variant } : null;
  }

  // Fetch (cached) every trials entry for the variant, then re-render.
  async function ensureTrials(s) {
    const cv = currentVariant(s);
    if (!cv) return;
    const entries = cv.variant.trials || [];
    const token = ++fetchToken;
    const pending = [];
    for (const e of entries) {
      const ck = `${s.videoId}|${cv.variant.fov}|${e.id}`;
      if (trialsCache.has(ck)) continue;
      pending.push(
        loadTrials(e.trials_json)
          .then((data) => trialsCache.set(ck, { data }))
          .catch((err) => {
            trialsCache.set(ck, { error: err && err.message ? err.message : String(err) });
            showToast(`Failed to load trials "${e.label || e.id}": ${err.message || err}`, {
              kind: 'error',
            });
          })
      );
    }
    // No render needed here: maybeRender() always render()s before calling us,
    // so a fully-cached variant is already on screen.
    if (!pending.length) return;
    await Promise.allSettled(pending);
    if (token !== fetchToken) return; // variant changed mid-fetch
    if (store.get().mode === 'metrics') render(store.get());
  }

  // ── Markdown export (DS §5.5) ──────────────────────────────────────────────
  function copyMarkdown(headers, rows) {
    const esc = (c) => String(c).replace(/\|/g, '\\|');
    const lines = [];
    lines.push(`| ${headers.map(esc).join(' | ')} |`);
    lines.push(`| ${headers.map(() => '---').join(' | ')} |`);
    for (const r of rows) lines.push(`| ${r.map(esc).join(' | ')} |`);
    const md = lines.join('\n');
    const done = () => showToast('Copied');
    if (navigator.clipboard && navigator.clipboard.writeText) {
      navigator.clipboard.writeText(md).then(done, () =>
        showToast('Copy failed', { kind: 'error' })
      );
    } else {
      done();
    }
  }

  function emptyMessage(text) {
    const div = document.createElement('div');
    div.className = 'mx-empty';
    div.textContent = text;
    return div;
  }

  // ── Render ─────────────────────────────────────────────────────────────────
  function render(s) {
    root.replaceChildren();
    const cv = currentVariant(s);
    if (!cv) {
      root.appendChild(emptyMessage('Select a video first.'));
      return;
    }
    const entries = cv.variant.trials || [];
    if (!entries.length) {
      root.appendChild(emptyMessage('No trials data for this variant.'));
      return;
    }

    // Reset column visibility when the variant changes (default all on).
    const vk = variantKey(s);
    if (vk !== lastVariantKey) {
      visibleCols = {};
      for (const e of entries) visibleCols[e.id] = true;
      lastVariantKey = vk;
    }

    // Resolve each entry against the cache. Loading until all are resolved.
    const cols = entries.map((e, i) => {
      const ck = `${s.videoId}|${cv.variant.fov}|${e.id}`;
      const cached = trialsCache.get(ck);
      return {
        id: e.id,
        label: e.label || e.id,
        color: paletteHex(i),
        cached,
        visible: visibleCols[e.id] !== false,
      };
    });

    if (cols.some((c) => !c.cached)) {
      const loading = document.createElement('div');
      loading.className = 'mx-empty';
      loading.textContent = 'Loading trials…';
      root.appendChild(loading);
      return;
    }

    // Drop columns whose fetch failed (DS: failed fetch -> column omitted).
    const okCols = cols.filter((c) => c.cached && c.cached.data);
    if (!okCols.length) {
      root.appendChild(emptyMessage('No trials data for this variant.'));
      return;
    }

    root.appendChild(buildHeader(s, cv, okCols));
    root.appendChild(buildAggregate(okCols));
    root.appendChild(buildPerTrial(okCols));
  }

  // ── Header strip + chip bar (DS §5.1.1) ─────────────────────────────────────
  function buildHeader(s, cv, cols) {
    const head = document.createElement('div');
    head.className = 'mx-head';

    const title = document.createElement('div');
    title.className = 'mx-head__title';
    title.textContent = `${cv.video.title || cv.video.id} · ${cv.variant.fov}`;
    head.appendChild(title);

    const chips = document.createElement('div');
    chips.className = 'mx-chipbar';
    for (const col of cols) {
      const chip = document.createElement('button');
      chip.type = 'button';
      chip.className = 'metrics-chip' + (col.visible ? '' : ' is-off');
      chip.style.borderLeftColor = col.color;
      chip.textContent = col.label;
      chip.title = 'Toggle column';
      chip.addEventListener('click', () => {
        visibleCols[col.id] = !col.visible;
        render(store.get());
      });
      chips.appendChild(chip);
    }
    head.appendChild(chips);
    return head;
  }

  // ── Aggregate comparison table (DS §5.2) ────────────────────────────────────
  function buildAggregate(allCols) {
    const cols = allCols.filter((c) => c.visible);
    const section = document.createElement('section');
    section.className = 'mx-section';

    const bar = document.createElement('div');
    bar.className = 'mx-section__bar';
    const h = document.createElement('h2');
    h.className = 'mx-section__title';
    h.textContent = 'Aggregate comparison';
    bar.appendChild(h);
    bar.appendChild(copyButton(() => aggMarkdown(cols)));
    section.appendChild(bar);

    if (!cols.length) {
      section.appendChild(emptyMessage('All columns hidden.'));
      return section;
    }

    const table = document.createElement('table');
    table.className = 'mx-table mx-table--agg';

    // Header: blank corner + one column per visible config.
    const thead = document.createElement('thead');
    const hr = document.createElement('tr');
    hr.appendChild(thEl('Metric', 'mx-rowhead-col'));
    for (const c of cols) {
      const th = document.createElement('th');
      const lab = document.createElement('div');
      lab.className = 'mx-colhead';
      const sw = document.createElement('span');
      sw.className = 'mx-swatch';
      sw.style.background = c.color;
      lab.appendChild(sw);
      lab.appendChild(document.createTextNode(c.label));
      th.appendChild(lab);
      const agg = c.cached.data.aggregate || {};
      const sub = document.createElement('div');
      sub.className = 'mx-subline';
      sub.textContent = isNum(agg.n_trials) ? `n=${agg.n_trials}` : 'n=—';
      th.appendChild(sub);
      hr.appendChild(th);
    }
    thead.appendChild(hr);
    table.appendChild(thead);

    const tbody = document.createElement('tbody');
    for (const row of AGG_ROWS) {
      const tr = document.createElement('tr');

      const rh = document.createElement('th');
      rh.className = 'mx-rowhead';
      rh.scope = 'row';
      rh.appendChild(document.createTextNode(row.label));
      if (row.unit) {
        const u = document.createElement('span');
        u.className = 'mx-unit';
        u.textContent = ` (${row.unit})`;
        rh.appendChild(u);
      }
      const arrow = document.createElement('span');
      arrow.className = 'mx-arrow';
      arrow.textContent = row.dir === 'up' ? ' ▲' : ' ▼';
      arrow.title = row.dir === 'up' ? 'higher is better' : 'lower is better';
      rh.appendChild(arrow);
      tr.appendChild(rh);

      // Gather raw values for best-of + microbar max.
      const vals = cols.map((c) => {
        const agg = c.cached.data.aggregate || {};
        return isNum(agg[row.key]) ? agg[row.key] : null;
      });
      const present = vals.filter((v) => v != null);
      const rowMax = present.length ? Math.max(...present.map(Math.abs)) : 0;

      // Best index (skip nulls). Neutral row still computes a winner but with
      // a neutral highlight + no star (DS §5.2.2).
      let bestIdx = -1;
      let bestVal = null;
      vals.forEach((v, i) => {
        if (v == null) return;
        if (bestVal == null) {
          bestVal = v;
          bestIdx = i;
        } else if (row.dir === 'up' ? v > bestVal : v < bestVal) {
          bestVal = v;
          bestIdx = i;
        }
      });

      cols.forEach((c, i) => {
        const td = document.createElement('td');
        td.className = 'mx-cell mono';
        const raw = vals[i];

        if (i === bestIdx && raw != null) {
          td.classList.add(row.neutral ? 'mx-best--neutral' : 'mx-best');
        }

        const valEl = document.createElement('span');
        valEl.className = 'mx-val';
        valEl.textContent = fmt(raw, row.scale, row.dp, row.unit);
        if (raw == null) valEl.classList.add('mx-na');
        if (i === bestIdx && raw != null && !row.neutral) {
          const star = document.createElement('span');
          star.className = 'mx-star';
          star.textContent = ' ★';
          valEl.appendChild(star);
        }
        td.appendChild(valEl);

        // Microbar (DS §5.2.4): width = |value|/rowMax, tinted by col color.
        if (raw != null && rowMax > 0) {
          const bar = document.createElement('div');
          bar.className = 'mx-microbar';
          bar.style.width = `${Math.min(100, (Math.abs(raw) / rowMax) * 100)}%`;
          bar.style.background = hexAlpha(c.color, 0.25);
          td.appendChild(bar);
        }
        tr.appendChild(td);
      });
      tbody.appendChild(tr);
    }
    table.appendChild(tbody);
    section.appendChild(table);
    return section;
  }

  function aggMarkdown(cols) {
    const headers = ['Metric', ...cols.map((c) => c.label)];
    const rows = AGG_ROWS.map((row) => {
      const unit = row.unit ? ` (${row.unit})` : '';
      const cells = cols.map((c) => {
        const agg = c.cached.data.aggregate || {};
        return fmt(isNum(agg[row.key]) ? agg[row.key] : null, row.scale, row.dp, row.unit);
      });
      return [`${row.label}${unit} ${row.dir === 'up' ? '▲' : '▼'}`, ...cells];
    });
    return { headers, rows };
  }

  // ── Per-trial breakdown table (DS §5.3) ─────────────────────────────────────
  function buildPerTrial(allCols) {
    const cols = allCols.filter((c) => c.visible);
    const section = document.createElement('section');
    section.className = 'mx-section';

    const bar = document.createElement('div');
    bar.className = 'mx-section__bar';
    const h = document.createElement('h2');
    h.className = 'mx-section__title';
    h.textContent = 'Per-trial breakdown';
    bar.appendChild(h);
    bar.appendChild(copyButton(() => trialMarkdown(cols)));
    section.appendChild(bar);

    if (!cols.length) {
      section.appendChild(emptyMessage('All columns hidden.'));
      return section;
    }

    const table = document.createElement('table');
    table.className = 'mx-table mx-table--trial';

    // Sortable header (DS §5.3.3): sorts WITHIN each group.
    const thead = document.createElement('thead');
    const hr = document.createElement('tr');
    for (const col of TRIAL_COLS) {
      const th = document.createElement('th');
      th.className = 'mx-sortable';
      const lab = document.createElement('span');
      lab.textContent = col.label;
      th.appendChild(lab);
      if (trialSort && trialSort.key === col.key) {
        const ind = document.createElement('span');
        ind.className = 'mx-sortind';
        ind.textContent = trialSort.dir === 'asc' ? ' ▲' : ' ▼';
        th.appendChild(ind);
      }
      th.addEventListener('click', () => {
        if (trialSort && trialSort.key === col.key) {
          trialSort = trialSort.dir === 'asc' ? { key: col.key, dir: 'desc' } : null;
        } else {
          trialSort = { key: col.key, dir: 'asc' };
        }
        render(store.get());
      });
      hr.appendChild(th);
    }
    thead.appendChild(hr);
    table.appendChild(thead);

    const tbody = document.createElement('tbody');
    for (const c of cols) {
      const perTrial = (c.cached.data.per_trial || []).slice();

      // Group header (DS §5.3.2).
      const ghr = document.createElement('tr');
      ghr.className = 'mx-group-head';
      const gth = document.createElement('th');
      gth.colSpan = TRIAL_COLS.length;
      gth.style.borderLeftColor = c.color;
      gth.textContent = `${c.label}  ·  ${perTrial.length} trial${perTrial.length === 1 ? '' : 's'}`;
      ghr.appendChild(gth);
      tbody.appendChild(ghr);

      // Sort within the group (default = file order).
      let ordered = perTrial;
      if (trialSort) {
        ordered = perTrial.slice().sort((a, b) => {
          const av = a[trialSort.key];
          const bv = b[trialSort.key];
          const an = isNum(av) ? av : null;
          const bn = isNum(bv) ? bv : null;
          if (an == null && bn == null) return 0;
          if (an == null) return 1; // missing sinks
          if (bn == null) return -1;
          return trialSort.dir === 'asc' ? an - bn : bn - an;
        });
      }

      // Data rows.
      for (const t of ordered) {
        const tr = document.createElement('tr');
        for (const col of TRIAL_COLS) {
          const td = document.createElement('td');
          td.className = 'mx-cell mono';
          const raw = t[col.key];
          const txt = fmt(isNum(raw) ? raw : null, col.scale, col.dp, col.unit || '');
          td.textContent = txt;
          if (!isNum(raw)) td.classList.add('mx-na');
          tr.appendChild(td);
        }
        tbody.appendChild(tr);
      }

      // Group-mean footer (DS §5.3.6).
      const fr = document.createElement('tr');
      fr.className = 'mx-foot';
      TRIAL_COLS.forEach((col, ci) => {
        const td = document.createElement('td');
        td.className = 'mx-cell mono';
        if (ci === 0) {
          td.textContent = 'mean';
          td.classList.add('mx-foot__label');
        } else {
          const nums = perTrial.map((t) => t[col.key]).filter(isNum);
          const mean = nums.length ? nums.reduce((a, b) => a + b, 0) / nums.length : null;
          td.textContent = fmt(mean, col.scale, col.dp, col.unit || '');
          if (mean == null) td.classList.add('mx-na');
        }
        fr.appendChild(td);
      });
      tbody.appendChild(fr);
    }
    table.appendChild(tbody);
    section.appendChild(table);
    return section;
  }

  function trialMarkdown(cols) {
    const headers = TRIAL_COLS.map((c) => c.label);
    const rows = [];
    for (const c of cols) {
      rows.push([`**${c.label}**`, ...Array(TRIAL_COLS.length - 1).fill('')]);
      const perTrial = (c.cached.data.per_trial || []);
      let ordered = perTrial;
      if (trialSort) {
        ordered = perTrial.slice().sort((a, b) => {
          const an = isNum(a[trialSort.key]) ? a[trialSort.key] : null;
          const bn = isNum(b[trialSort.key]) ? b[trialSort.key] : null;
          if (an == null && bn == null) return 0;
          if (an == null) return 1;
          if (bn == null) return -1;
          return trialSort.dir === 'asc' ? an - bn : bn - an;
        });
      }
      for (const t of ordered) {
        rows.push(
          TRIAL_COLS.map((col) => {
            const raw = t[col.key];
            return fmt(isNum(raw) ? raw : null, col.scale, col.dp, col.unit || '');
          })
        );
      }
    }
    return { headers, rows };
  }

  function copyButton(build) {
    const btn = document.createElement('button');
    btn.type = 'button';
    btn.className = 'btn mx-copy';
    btn.textContent = 'Copy as Markdown';
    btn.addEventListener('click', () => {
      const { headers, rows } = build();
      copyMarkdown(headers, rows);
    });
    return btn;
  }

  function thEl(text, cls) {
    const th = document.createElement('th');
    if (cls) th.className = cls;
    th.textContent = text;
    return th;
  }

  // ── Subscription: only work in metrics mode (DS §5) ─────────────────────────
  function maybeRender(s) {
    if (s.mode !== 'metrics') return;
    render(s);
    ensureTrials(s);
  }

  store.subscribe((state, changed) => {
    if (
      changed.has('mode') ||
      changed.has('videoId') ||
      changed.has('fov') ||
      changed.has('manifest')
    ) {
      maybeRender(state);
    }
  });

  // Initial (in case we boot directly into metrics mode).
  maybeRender(store.get());
}
