// Stage (DS §2.2 / §2.6 / §6.5 / §6.6 / §7.1 / §7.3 / §7.5 / §7.7).
//
// Owns the <video> element, frame-accurate sync (requestVideoFrameCallback
// with a rAF fallback), canvas sizing + contentBox publishing, zoom/pan on a
// shared transform wrapper, playback state, and cursor tracking for the HUD.
//
// This module does NOT draw on the overlay canvas — it only sizes the canvas
// and publishes contentBox (DS §7.3 / §7.4); the overlay task (later) consumes
// it. Cursor moves update the store only (DS §7.7) — no canvas repaint here.
//
// Transient store keys introduced here (not in defaultState — store.set
// tolerates new keys):
//   cursorNorm: {x, y} | null   — normalized content-space cursor for the HUD.
// And contentBox is published as {x, y, w, h, dpr} (dpr added for the overlay).
import { contentRect } from '../core/geometry.js';

export function initStage(store) {
  const stage = document.getElementById('stage');
  const transform = document.getElementById('stage-transform');
  const video = document.getElementById('video');
  const canvas = document.getElementById('overlay-canvas');
  const empty = document.getElementById('stage-empty');

  // Defensive: if the DOM isn't present (e.g. imported in a test harness),
  // return a no-op seekToFrame so callers don't crash.
  if (!stage || !transform || !video || !canvas) {
    return { seekToFrame() {} };
  }

  // ── Per-load sync params (authoritative from manifest via store) ──────
  let fps = store.get().fps || 30;
  let totalFrames = store.get().totalFrames || 0;
  let rvfcHandle = null; // current rVFC registration (we can't cancel, so guard by token)
  let rvfcToken = 0; // bump on each video load to invalidate stale callbacks
  let rafHandle = null; // fallback rAF loop handle
  let currentVariant = null; // {width, height} for contentRect
  let pendingSeekFrame = null; // hash-restore target to seek once metadata loads

  const hasRVFC =
    typeof HTMLVideoElement !== 'undefined' &&
    'requestVideoFrameCallback' in HTMLVideoElement.prototype;

  function timeToFrame(t) {
    if (!fps) return 0;
    return Math.min(Math.max(totalFrames - 1, 0), Math.max(0, Math.round(t * fps)));
  }

  function pushFrameFromTime(t) {
    store.set({ frame: timeToFrame(t) });
  }

  // ── Frame sync (DS §6.5 / §7.1) ───────────────────────────────────────
  function startSync() {
    rvfcToken += 1;
    const token = rvfcToken;
    if (hasRVFC) {
      const cb = (now, meta) => {
        if (token !== rvfcToken) return; // a newer load superseded this loop
        store.set({ frame: timeToFrame(meta.mediaTime) });
        rvfcHandle = video.requestVideoFrameCallback(cb);
      };
      rvfcHandle = video.requestVideoFrameCallback(cb);
    } else {
      // Fallback: rAF reading currentTime. Only meaningful while playing,
      // but harmless when paused (it just re-publishes the same frame).
      const loop = () => {
        if (token !== rvfcToken) return;
        pushFrameFromTime(video.currentTime);
        rafHandle = requestAnimationFrame(loop);
      };
      if (rafHandle != null) cancelAnimationFrame(rafHandle);
      rafHandle = requestAnimationFrame(loop);
    }
  }

  // Push the exact frame immediately after a seek settles (DS §7.1: redraw on
  // the seeked event, not optimistically).
  video.addEventListener('seeked', () => pushFrameFromTime(video.currentTime));
  video.addEventListener('loadedmetadata', () => {
    if (pendingSeekFrame != null && fps) {
      const f = Math.min(Math.max(totalFrames - 1, 0), Math.max(0, pendingSeekFrame));
      pendingSeekFrame = null;
      video.currentTime = (f + 0.5) / fps; // seeked handler pushes the frame
    }
  });
  video.addEventListener('loadeddata', () => {
    if (pendingSeekFrame == null) pushFrameFromTime(video.currentTime);
  });

  // ── Video source wiring (DS §2.2.1 / §6.1) ────────────────────────────
  function applyVideoSource(state) {
    const { manifest, videoId, fov } = state;
    if (!videoId || !manifest) {
      currentVariant = null;
      video.removeAttribute('src');
      video.load();
      if (empty) empty.classList.remove('hidden');
      return;
    }
    const v = manifest.videos.find((vv) => vv.id === videoId);
    const variant = v && (v.variants.find((x) => x.fov === fov) || v.variants[0]);
    if (!variant || !variant.video) {
      currentVariant = null;
      if (empty) empty.classList.remove('hidden');
      return;
    }
    currentVariant = { width: variant.width || 1920, height: variant.height || 1080 };
    // Manifest is authoritative for fps/frames (DS §6.5) — main.js already
    // wrote totalFrames/fps into the store; mirror them locally for sync math.
    fps = state.fps || variant.fps || 30;
    totalFrames = state.totalFrames || variant.frames || 0;

    // A deep-link / hash restore may have set a non-zero frame in the store
    // before the video existed; seek the media to it once it's loaded so the
    // overlay and footage land on the requested frame (DS §7.11).
    pendingSeekFrame = state.frame > 0 ? state.frame : null;

    if (video.src !== new URL(variant.video, location.href).href) {
      video.src = variant.video;
      video.load();
    } else {
      // src unchanged -> no loadedmetadata coming; don't leave a stale
      // pending seek for a future, unrelated load
      pendingSeekFrame = null;
    }
    if (empty) empty.classList.add('hidden');
    startSync();
    resizeCanvas(); // recompute contentBox for the new variant aspect
  }

  // ── Canvas sizing + contentBox publish (DS §7.3 / §7.4) ───────────────
  function resizeCanvas() {
    const dpr = window.devicePixelRatio || 1;
    const cssW = stage.clientWidth;
    const cssH = stage.clientHeight;
    // Backing store sized to CSS × DPR for crisp overlay strokes; CSS size
    // stays 100% via the stylesheet.
    canvas.width = Math.max(1, Math.round(cssW * dpr));
    canvas.height = Math.max(1, Math.round(cssH * dpr));

    const vw = (currentVariant && currentVariant.width) || 16;
    const vh = (currentVariant && currentVariant.height) || 9;
    const rect = contentRect(cssW, cssH, vw, vh); // CSS px
    store.set({ contentBox: { x: rect.x, y: rect.y, w: rect.w, h: rect.h, dpr } });
  }

  const ro = new ResizeObserver(() => resizeCanvas());
  ro.observe(stage);

  // ── Zoom / pan (DS §2.6 / §7.5) ───────────────────────────────────────
  const ZOOM_MIN = 1;
  const ZOOM_MAX = 8;
  const ZOOM_STEP = 1.1;

  function clamp(v, lo, hi) {
    return Math.min(hi, Math.max(lo, v));
  }

  // Clamp pan so content never pulls its edges inside the stage. With
  // origin 0 0, content spans [pan, pan + stage*zoom]; we require
  // pan ≤ 0 and pan + stage*zoom ≥ stage, i.e. pan ∈ [stage*(1-zoom), 0].
  function clampPan(panX, panY, zoom) {
    if (zoom <= 1) return { panX: 0, panY: 0 };
    const w = stage.clientWidth;
    const h = stage.clientHeight;
    return {
      panX: clamp(panX, w * (1 - zoom), 0),
      panY: clamp(panY, h * (1 - zoom), 0),
    };
  }

  function applyTransform(state) {
    transform.style.transform =
      `translate(${state.panX}px, ${state.panY}px) scale(${state.zoom})`;
    // Pan cursor affordance only when zoomed in (DS §2.6.4).
    stage.style.cursor = state.zoom > 1 ? 'grab' : 'default';
  }

  stage.addEventListener(
    'wheel',
    (e) => {
      e.preventDefault();
      const { zoom, panX, panY } = store.get();
      const newZoom = clamp(zoom * (e.deltaY < 0 ? ZOOM_STEP : 1 / ZOOM_STEP), ZOOM_MIN, ZOOM_MAX);
      if (newZoom === zoom) return;
      // Anchor toward the cursor: keep the content point under the cursor
      // fixed. s = c*zoom + pan ⇒ pan' = s - (s - pan) * (zoom'/zoom).
      const r = stage.getBoundingClientRect();
      const sx = e.clientX - r.left;
      const sy = e.clientY - r.top;
      const ratio = newZoom / zoom;
      let nPanX = sx - (sx - panX) * ratio;
      let nPanY = sy - (sy - panY) * ratio;
      ({ panX: nPanX, panY: nPanY } = clampPan(nPanX, nPanY, newZoom));
      store.set({ zoom: newZoom, panX: nPanX, panY: nPanY });
    },
    { passive: false }
  );

  // Pointer-drag pan (only when zoomed in) — DS §2.6.2.
  let dragging = false;
  let dragStart = null; // {x, y, panX, panY}
  stage.addEventListener('pointerdown', (e) => {
    if (e.button !== 0) return;
    if (store.get().zoom <= 1) return;
    dragging = true;
    const { panX, panY } = store.get();
    dragStart = { x: e.clientX, y: e.clientY, panX, panY };
    stage.setPointerCapture(e.pointerId);
    stage.style.cursor = 'grabbing';
  });
  stage.addEventListener('pointermove', (e) => {
    if (!dragging || !dragStart) return;
    const { zoom } = store.get();
    let nPanX = dragStart.panX + (e.clientX - dragStart.x);
    let nPanY = dragStart.panY + (e.clientY - dragStart.y);
    ({ panX: nPanX, panY: nPanY } = clampPan(nPanX, nPanY, zoom));
    store.set({ panX: nPanX, panY: nPanY });
  });
  function endDrag(e) {
    if (!dragging) return;
    dragging = false;
    dragStart = null;
    try {
      stage.releasePointerCapture(e.pointerId);
    } catch {
      /* pointer already released */
    }
    stage.style.cursor = store.get().zoom > 1 ? 'grab' : 'default';
  }
  stage.addEventListener('pointerup', endDrag);
  stage.addEventListener('pointercancel', endDrag);

  // Double-click → reset view (DS §2.6.3).
  stage.addEventListener('dblclick', () => {
    store.set({ zoom: 1, panX: 0, panY: 0 });
  });

  // ── Cursor tracking for HUD (DS §2.4.3 / §7.7) ────────────────────────
  // Map screen px → content-normalized [0,1] through the zoom/pan inverse and
  // the contentBox. rAF-throttled; updates the store only (no canvas redraw).
  let cursorScheduled = false;
  let pendingCursorEvent = null;
  stage.addEventListener('mousemove', (e) => {
    pendingCursorEvent = e;
    if (cursorScheduled) return;
    cursorScheduled = true;
    requestAnimationFrame(() => {
      cursorScheduled = false;
      const ev = pendingCursorEvent;
      pendingCursorEvent = null;
      if (!ev) return;
      const { zoom, panX, panY, contentBox } = store.get();
      if (!contentBox) {
        store.set({ cursorNorm: null });
        return;
      }
      const r = stage.getBoundingClientRect();
      const mx = ev.clientX - r.left; // stage CSS px (untransformed)
      const my = ev.clientY - r.top;
      // Undo the wrapper transform, then map through the content rect.
      const cx = ((mx - panX) / zoom - contentBox.x) / contentBox.w;
      const cy = ((my - panY) / zoom - contentBox.y) / contentBox.h;
      const inside = cx >= 0 && cx <= 1 && cy >= 0 && cy <= 1;
      store.set({ cursorNorm: inside ? { x: cx, y: cy } : null });
    });
  });
  stage.addEventListener('mouseleave', () => store.set({ cursorNorm: null }));

  // ── Playback state (DS §2.3 controls drive the store) ─────────────────
  // Native events → store. The store's change-detection prevents feedback
  // loops (setting playing to its current value notifies no subscribers).
  video.addEventListener('play', () => store.set({ playing: true }));
  video.addEventListener('pause', () => store.set({ playing: false }));
  video.addEventListener('ended', () => {
    if (!video.loop) store.set({ playing: false });
  });

  // ── Store subscription: react to relevant changes ─────────────────────
  store.subscribe((state, changed) => {
    if (changed.has('videoId') || changed.has('fov') || changed.has('manifest')) {
      applyVideoSource(state);
    }
    if (changed.has('totalFrames')) totalFrames = state.totalFrames || 0;
    if (changed.has('fps')) fps = state.fps || 30;

    if (changed.has('zoom') || changed.has('panX') || changed.has('panY')) {
      applyTransform(state);
    }
    if (changed.has('playing')) {
      if (state.playing) {
        const p = video.play();
        if (p && typeof p.catch === 'function') p.catch(() => {});
      } else {
        video.pause();
      }
    }
    if (changed.has('speed')) video.playbackRate = state.speed;
    if (changed.has('loop')) video.loop = state.loop;
  });

  // ── seekToFrame (DS §6.6) ─────────────────────────────────────────────
  function seekToFrame(n) {
    if (!totalFrames) return;
    const f = clamp(Math.round(n), 0, totalFrames - 1);
    video.pause();
    store.set({ playing: false });
    // Mid-frame offset avoids landing on a boundary that decodes the wrong
    // frame (DS §6.6). seeked → pushFrameFromTime updates the store.
    video.currentTime = (f + 0.5) / fps;
  }

  // Initial paint from current store state (hash restore may have happened).
  applyVideoSource(store.get());
  applyTransform(store.get());

  return { seekToFrame };
}
