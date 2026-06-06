// HUD overlay (DS §2.4). Four corner pills fed from the store. Text/DOM
// updates only on relevant changes (DS §7.7 — never repaint the canvas here).
// Auto-fades during playback after 3s of mouse idle on the stage (DS §2.4.5),
// suppressed under prefers-reduced-motion (DS §6.12).

export function initHud(store) {
  const hud = document.getElementById('hud');
  const stage = document.getElementById('stage');
  const elFrame = document.getElementById('hud-frame');
  const elZoom = document.getElementById('hud-zoom');
  const elCursor = document.getElementById('hud-cursor');
  const elCounts = document.getElementById('hud-counts');
  if (!hud || !elFrame || !elZoom || !elCursor || !elCounts) return;

  const reduceMotion =
    typeof window !== 'undefined' &&
    window.matchMedia &&
    window.matchMedia('(prefers-reduced-motion: reduce)').matches;

  // ── Top-left: frame + time (DS §2.4.1) ────────────────────────────────
  function syncFrame(state) {
    const t = state.fps ? (state.frame / state.fps).toFixed(2) : '0.00';
    elFrame.textContent = `frame ${state.frame} / ${state.totalFrames} · t=${t}s`;
  }

  // ── Top-right: zoom, hidden at 1× (DS §2.4.2) ─────────────────────────
  function syncZoom(state) {
    if (state.zoom <= 1.0001) {
      elZoom.hidden = true;
    } else {
      elZoom.hidden = false;
      elZoom.textContent = `zoom ${state.zoom.toFixed(1)}×`;
    }
  }

  // ── Bottom-left: cursor normalized coords (DS §2.4.3) ─────────────────
  function syncCursor(state) {
    const c = state.cursorNorm;
    if (!c) {
      elCursor.textContent = 'x —  y —';
    } else {
      elCursor.textContent = `x ${c.x.toFixed(3)}  y ${c.y.toFixed(3)}`;
    }
  }

  // ── Bottom-right: per-run detection counts (DS §2.4.4) ────────────────
  // frameCounts is populated by the overlay task later; render empty (and a
  // swatch+label+count line per enabled run) until it exists.
  function syncCounts(state) {
    const { enabledRuns = [], runColors = {}, frameCounts = {}, manifest, videoId, fov } = state;
    elCounts.innerHTML = '';
    if (!enabledRuns.length) return;

    // Resolve run labels from the manifest variant (best-effort).
    let runMeta = [];
    const v = manifest && manifest.videos.find((vv) => vv.id === videoId);
    const variant = v && (v.variants.find((x) => x.fov === fov) || v.variants[0]);
    if (variant && Array.isArray(variant.runs)) runMeta = variant.runs;

    for (const id of enabledRuns) {
      const slot = runColors[id] || 1;
      const meta = runMeta.find((r) => r.id === id);
      const label = (meta && (meta.label || meta.id)) || id;
      const counts = frameCounts[id];
      const shown = counts && Number.isFinite(counts.shown) ? counts.shown : 0;

      const line = document.createElement('span');
      line.className = 'hud-count-line';
      const sw = document.createElement('span');
      sw.className = 'hud-swatch';
      sw.style.background = `var(--run-${slot})`;
      const txt = document.createElement('span');
      txt.textContent = `${label} ${shown}`;
      line.appendChild(sw);
      line.appendChild(txt);
      elCounts.appendChild(line);
    }
  }

  store.subscribe((state, changed) => {
    if (changed.has('frame') || changed.has('totalFrames') || changed.has('fps')) {
      syncFrame(state);
    }
    if (changed.has('zoom')) syncZoom(state);
    if (changed.has('cursorNorm')) syncCursor(state);
    if (
      changed.has('enabledRuns') ||
      changed.has('runColors') ||
      changed.has('frameCounts') ||
      changed.has('videoId') ||
      changed.has('fov')
    ) {
      syncCounts(state);
    }
  });

  // ── Auto-fade (DS §2.4.5 / §6.12) ─────────────────────────────────────
  let idleTimer = null;
  function clearIdle() {
    if (idleTimer) {
      clearTimeout(idleTimer);
      idleTimer = null;
    }
  }
  function scheduleFade() {
    clearIdle();
    if (reduceMotion) return;
    if (!store.get().playing) return;
    idleTimer = setTimeout(() => {
      if (store.get().playing) hud.classList.add('hud--faded');
    }, 3000);
  }
  if (stage) {
    stage.addEventListener('mousemove', () => {
      hud.classList.remove('hud--faded');
      scheduleFade();
    });
  }
  store.subscribe((state, changed) => {
    if (changed.has('playing')) {
      if (state.playing) {
        scheduleFade();
      } else {
        clearIdle();
        hud.classList.remove('hud--faded');
      }
    }
  });

  // Initial paint.
  const s0 = store.get();
  syncFrame(s0);
  syncZoom(s0);
  syncCursor(s0);
  syncCounts(s0);
}
