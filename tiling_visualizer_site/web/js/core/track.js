// Smoothed target trajectory for the crosshair overlay.
//
// frames.json carries only raw per-frame detections (no separate tracker
// signal), so we derive a tracker-like smoothed position with a CAUSAL
// exponential moving average (EMA) over the centers of the 'target' detection.
// The result is precomputed once per doc and indexed by frame number, so
// scrubbing / jumping to any frame is deterministic (independent of playback
// order — unlike smoothing live during playback).

const TARGET_LABEL = 'target';

// A long absence (re-acquisition elsewhere after occlusion) restarts the
// filter so the crosshair doesn't glide across the frame through empty space.
const DEFAULT_RESET_GAP = 30;

// Center [cx, cy] (normalized) of the first 'target' detection in a frame, or
// null when the target is absent.
export function targetCenter(frameEntry) {
  for (const d of (frameEntry && frameEntry.detections) || []) {
    if (d && d.label === TARGET_LABEL && Array.isArray(d.bbox) && d.bbox.length >= 4) {
      const x = Number(d.bbox[0]);
      const y = Number(d.bbox[1]);
      const w = Number(d.bbox[2]);
      const h = Number(d.bbox[3]);
      return { cx: x + w / 2, cy: y + h / 2 };
    }
  }
  return null;
}

// Map<frameNumber, {cx, cy}> of EMA-smoothed target centers. Only frames that
// actually contain a target detection get an entry (the crosshair hides when
// the target is absent). alpha in (0,1]: higher = snappier / less lag.
export function smoothTargetTrack(frames, alpha = 0.18, resetGap = DEFAULT_RESET_GAP) {
  const out = new Map();
  if (!Array.isArray(frames) || frames.length === 0) return out;
  const ordered = [...frames].sort((a, b) => ((a.frame | 0) - (b.frame | 0)));
  let sx = null;
  let sy = null;
  let prevFrame = null;
  for (const f of ordered) {
    const c = targetCenter(f);
    if (!c) continue;
    const reset = sx === null || (prevFrame !== null && f.frame - prevFrame > resetGap);
    if (reset) {
      sx = c.cx;
      sy = c.cy;
    } else {
      sx = alpha * c.cx + (1 - alpha) * sx;
      sy = alpha * c.cy + (1 - alpha) * sy;
    }
    out.set(f.frame, { cx: sx, cy: sy });
    prevFrame = f.frame;
  }
  return out;
}
