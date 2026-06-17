import { computeTileRects } from './geometry.js';
import { smoothTargetTrack } from './track.js';

// One-time index of a frames.json doc: Map frame→entry (O(1) per-draw
// lookup), static tile rects reconstructed from config (used by the
// phantom filter + tile rendering when per-frame tiles are absent).
export function indexFrames(doc) {
  const byFrame = new Map();
  let maxFrame = -1;
  let hasTiles = false;
  for (const f of doc.frames || []) {
    byFrame.set(f.frame, f);
    if (f.frame > maxFrame) maxFrame = f.frame;
    if (f.tiles && f.tiles.length) hasTiles = true;
  }
  return {
    label: doc.label || 'unnamed',
    config: doc.config || null,
    staticTileRects: computeTileRects(doc.config || null),
    // Smoothed (EMA) target trajectory, frame→{cx,cy}, for the crosshair.
    targetTrack: smoothTargetTrack(doc.frames || []),
    byFrame, maxFrame, hasTiles,
  };
}
