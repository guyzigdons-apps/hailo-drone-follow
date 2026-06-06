// Pure detection filter pipeline (extracted from overlay.js so the inspector
// and overlay share ONE definition of "what's shown this frame").
//
// Pipeline order (load-bearing — hoveredDet.index refers to an index in the
// RESULT list, so overlay + inspector MUST run the same steps in this order):
//   (1) confidence ≥ conf
//   (2) hide phantoms        (phantoms = boxes dropped here)
//   (3) containment merge    (merged   = boxes absorbed here)
//
// `doc` is an indexFrames() result ({ byFrame, staticTileRects, ... }).
import { isPhantom, containmentMerge } from './filters.js';

export function filterDetectionsPure(doc, frame, confThreshold, hidePhantoms, merge) {
  const raw = doc.byFrame.get(frame)?.detections ?? [];

  // (1) confidence
  let kept = raw.filter((d) => Number(d.confidence) >= confThreshold);

  // (2) phantoms
  let phantoms = 0;
  if (hidePhantoms) {
    const after = [];
    for (const d of kept) {
      if (isPhantom(d, doc.staticTileRects)) phantoms += 1;
      else after.push(d);
    }
    kept = after;
  }

  // (3) containment merge
  let merged = 0;
  if (merge) {
    const before = kept.length;
    kept = containmentMerge(kept, 0.5, 0);
    merged = before - kept.length;
  }

  return { dets: kept, phantoms, merged };
}
