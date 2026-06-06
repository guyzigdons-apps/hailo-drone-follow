// Transport bar (DS §2.3 / §6.4). Two rows: a frame scrubber with time labels
// and a hover tooltip, and a control row (jump/step/play/speed/loop). Renders
// once into #transport and stays in sync with the store. Drives the video via
// the seekToFrame() handle returned by initStage and via store.set() for play
// state / speed / loop.

// Format a frame index as mm:ss.mmm given fps (DS §2.3.1 / §6.4).
function fmtTime(frame, fps) {
  if (!fps) return '00:00.000';
  const secs = frame / fps;
  const mm = Math.floor(secs / 60);
  const ss = Math.floor(secs % 60);
  const mmm = Math.round((secs - Math.floor(secs)) * 1000);
  return `${String(mm).padStart(2, '0')}:${String(ss).padStart(2, '0')}.${String(mmm).padStart(3, '0')}`;
}

export function initTransport(store, stage) {
  const root = document.getElementById('transport');
  if (!root) return;
  const seekToFrame = (stage && stage.seekToFrame) || (() => {});

  // ── Build DOM once ─────────────────────────────────────────────────────
  root.innerHTML = `
    <div class="transport__row transport__row--scrubber">
      <span class="transport__label mono" id="transport-frame-label">frame 0 / 0</span>
      <div class="transport__scrub-wrap">
        <input type="range" class="transport__scrubber" id="transport-scrubber"
               min="0" max="0" step="1" value="0" aria-label="Frame scrubber">
        <div class="transport__tooltip tooltip" id="transport-tooltip" hidden></div>
      </div>
      <span class="transport__label mono" id="transport-time-label">00:00.000</span>
    </div>
    <div class="transport__row transport__row--controls">
      <button class="icon-btn" id="t-first" aria-label="Jump to first frame" title="First frame (Home)">⏮</button>
      <button class="icon-btn" id="t-prev" aria-label="Step back one frame" title="Step −1 ([)">◀|</button>
      <button class="icon-btn icon-btn--play" id="t-play" aria-label="Play" title="Play/Pause (Space)">▶</button>
      <button class="icon-btn" id="t-next" aria-label="Step forward one frame" title="Step +1 (])">|▶</button>
      <button class="icon-btn" id="t-last" aria-label="Jump to last frame" title="Last frame (End)">⏭</button>
      <span class="transport__divider" aria-hidden="true"></span>
      <label class="transport__speed-wrap">
        <span class="transport__speed-label">Speed</span>
        <select class="select transport__speed" id="t-speed" aria-label="Playback speed" title="Playback speed">
          <option value="0.25">0.25×</option>
          <option value="0.5">0.5×</option>
          <option value="1">1×</option>
          <option value="2">2×</option>
          <option value="4">4×</option>
        </select>
      </label>
      <span class="transport__divider" aria-hidden="true"></span>
      <button class="icon-btn" id="t-loop" aria-label="Toggle loop" aria-pressed="false" title="Loop">⟳</button>
      <span class="transport__spacer"></span>
    </div>
  `;

  const frameLabel = root.querySelector('#transport-frame-label');
  const timeLabel = root.querySelector('#transport-time-label');
  const scrubWrap = root.querySelector('.transport__scrub-wrap');
  const scrubber = root.querySelector('#transport-scrubber');
  const tooltip = root.querySelector('#transport-tooltip');
  const btnFirst = root.querySelector('#t-first');
  const btnPrev = root.querySelector('#t-prev');
  const btnPlay = root.querySelector('#t-play');
  const btnNext = root.querySelector('#t-next');
  const btnLast = root.querySelector('#t-last');
  const speedSel = root.querySelector('#t-speed');
  const btnLoop = root.querySelector('#t-loop');

  const controls = [btnFirst, btnPrev, btnPlay, btnNext, btnLast, speedSel, btnLoop, scrubber];

  let scrubbing = false; // don't fight the user while they drag the handle

  // ── Scrubber (DS §2.3.1) ───────────────────────────────────────────────
  scrubber.addEventListener('pointerdown', () => { scrubbing = true; });
  const stopScrub = () => { scrubbing = false; };
  scrubber.addEventListener('pointerup', stopScrub);
  scrubber.addEventListener('pointercancel', stopScrub);
  scrubber.addEventListener('change', stopScrub);
  scrubber.addEventListener('input', () => {
    seekToFrame(+scrubber.value);
  });

  // ── Scrubber hover preview tooltip (DS §6.4) — text only ───────────────
  scrubWrap.addEventListener('mousemove', (e) => {
    const { totalFrames, fps } = store.get();
    if (!totalFrames) { tooltip.hidden = true; return; }
    const r = scrubber.getBoundingClientRect();
    const frac = r.width ? (e.clientX - r.left) / r.width : 0;
    const f = Math.min(totalFrames - 1, Math.max(0, Math.round(frac * (totalFrames - 1))));
    tooltip.textContent = `frame ${f} · t=${fmtTime(f, fps)}`;
    tooltip.hidden = false;
    // Position above the cursor, clamped to the track width.
    const wrapRect = scrubWrap.getBoundingClientRect();
    const left = Math.min(Math.max(e.clientX - wrapRect.left, 0), wrapRect.width);
    tooltip.style.left = `${left}px`;
  });
  scrubWrap.addEventListener('mouseleave', () => { tooltip.hidden = true; });

  // ── Control row (DS §2.3.2) ────────────────────────────────────────────
  btnFirst.addEventListener('click', () => seekToFrame(0));
  btnPrev.addEventListener('click', () => seekToFrame(store.get().frame - 1));
  btnNext.addEventListener('click', () => seekToFrame(store.get().frame + 1));
  btnLast.addEventListener('click', () => seekToFrame(store.get().totalFrames - 1));
  btnPlay.addEventListener('click', () => store.set({ playing: !store.get().playing }));
  speedSel.addEventListener('change', () => store.set({ speed: +speedSel.value }));
  btnLoop.addEventListener('click', () => store.set({ loop: !store.get().loop }));

  // ── Sync DOM ← store ───────────────────────────────────────────────────
  function syncFrame(state) {
    const { frame, totalFrames, fps } = state;
    const max = Math.max(0, totalFrames - 1);
    if (+scrubber.max !== max) scrubber.max = String(max);
    if (!scrubbing && +scrubber.value !== frame) scrubber.value = String(frame);
    frameLabel.textContent = `frame ${frame} / ${totalFrames}`;
    timeLabel.textContent = fmtTime(frame, fps);
  }

  function syncPlay(state) {
    btnPlay.textContent = state.playing ? '⏸' : '▶';
    btnPlay.setAttribute('aria-label', state.playing ? 'Pause' : 'Play');
    btnPlay.classList.toggle('is-active', state.playing);
  }

  function syncEnabled(state) {
    const disabled = !state.videoId;
    for (const c of controls) c.disabled = disabled;
    root.classList.toggle('is-disabled', disabled);
  }

  store.subscribe((state, changed) => {
    if (changed.has('frame') || changed.has('totalFrames') || changed.has('fps')) {
      syncFrame(state);
    }
    if (changed.has('playing')) syncPlay(state);
    if (changed.has('speed') && +speedSel.value !== state.speed) {
      speedSel.value = String(state.speed);
    }
    if (changed.has('loop')) {
      btnLoop.classList.toggle('is-active', state.loop);
      btnLoop.setAttribute('aria-pressed', state.loop ? 'true' : 'false');
    }
    if (changed.has('videoId')) syncEnabled(state);
  });

  // Initial paint from current store state.
  const s0 = store.get();
  syncFrame(s0);
  syncPlay(s0);
  syncEnabled(s0);
  speedSel.value = String(s0.speed);
  btnLoop.classList.toggle('is-active', s0.loop);
  btnLoop.setAttribute('aria-pressed', s0.loop ? 'true' : 'false');
}
