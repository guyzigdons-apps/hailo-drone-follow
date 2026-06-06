// Top bar (DS §4.1): video dropdown, FOV segmented control, Viewer/Metrics
// mode toggle, manifest status. Selection logic itself lives in main.js —
// this module only renders controls and calls back via onSelect.

let topbarUnsub = null;

// Manifest status indicator (DS §4.1.5). state ∈ {loading, ready, failed}.
export function setStatus(state, text) {
  const root = document.getElementById('manifest-status');
  if (!root) return;
  const dot = root.querySelector('.status-dot');
  const label = root.querySelector('.status-text');
  if (dot) dot.classList.remove('is-ready', 'is-failed');
  root.classList.remove('is-clickable');
  root.onclick = null;

  if (state === 'ready') {
    if (dot) dot.classList.add('is-ready');
  } else if (state === 'failed') {
    if (dot) dot.classList.add('is-failed');
    root.classList.add('is-clickable');
    root.onclick = () => location.reload();
  }
  if (label) label.textContent = text;
}

export function initTopbar(store, { onSelect } = {}) {
  if (topbarUnsub) topbarUnsub();
  topbarUnsub = null;

  const select = document.getElementById('video-select');
  const fovEl = document.getElementById('fov-segmented');
  const modeEl = document.getElementById('mode-toggle');

  // ── Populate the video dropdown ────────────────────────────
  const { manifest } = store.get();
  const videos = (manifest && manifest.videos) || [];
  if (select) {
    select.innerHTML = '';
    const placeholder = document.createElement('option');
    placeholder.value = '';
    placeholder.textContent = videos.length ? 'Select a video…' : 'No videos';
    placeholder.disabled = true;
    placeholder.selected = true;
    select.appendChild(placeholder);
    for (const v of videos) {
      const opt = document.createElement('option');
      opt.value = v.id;
      opt.textContent = v.title || v.id;
      select.appendChild(opt);
    }
    select.disabled = videos.length === 0;

    select.onchange = () => {
      const vid = videos.find((v) => v.id === select.value);
      if (!vid) return;
      const fov = (vid.variants[0] && vid.variants[0].fov) || null;
      onSelect && onSelect(vid.id, fov);
    };
  }

  // ── Render FOV segmented buttons for the current video ─────
  function renderFov(state) {
    if (!fovEl) return;
    const vid = videos.find((v) => v.id === state.videoId);
    fovEl.innerHTML = '';
    if (!vid) return;
    for (const variant of vid.variants) {
      const btn = document.createElement('button');
      btn.type = 'button';
      btn.className = 'segmented__btn';
      btn.setAttribute('role', 'tab');
      btn.dataset.fov = variant.fov;
      // Strip the leading "fov" so the button reads "50" / "60" / "70".
      btn.textContent = String(variant.fov).replace(/^fov/i, '');
      if (variant.fov === state.fov) btn.classList.add('is-active');
      btn.addEventListener('click', () => {
        if (variant.fov === store.get().fov) return;
        onSelect && onSelect(vid.id, variant.fov);
      });
      fovEl.appendChild(btn);
    }
  }

  // ── Mode toggle ────────────────────────────────────────────
  if (modeEl) {
    modeEl.onclick = (e) => {
      const btn = e.target.closest('.segmented__btn');
      if (!btn || !btn.dataset.mode) return;
      store.set({ mode: btn.dataset.mode });
    };
  }

  // ── Keep controls in sync with store (hash restore, etc.) ──
  function sync(state) {
    if (select && select.value !== (state.videoId || '')) {
      select.value = state.videoId || '';
    }
    if (fovEl) {
      for (const btn of fovEl.querySelectorAll('.segmented__btn')) {
        btn.classList.toggle('is-active', btn.dataset.fov === state.fov);
      }
    }
    if (modeEl) {
      for (const btn of modeEl.querySelectorAll('.segmented__btn')) {
        btn.classList.toggle('is-active', btn.dataset.mode === state.mode);
      }
    }
  }

  topbarUnsub = store.subscribe((state, changed) => {
    if (changed.has('videoId')) renderFov(state);
    if (changed.has('videoId') || changed.has('fov') || changed.has('mode')) {
      sync(state);
    }
  });

  // Initial paint (in case state was pre-populated from the hash).
  renderFov(store.get());
  sync(store.get());
}
