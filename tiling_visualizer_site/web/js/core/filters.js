// Faithful JS port of tiling_benchmark/analyze_pxt.py filtering rules.
export function isPhantom(det, tileRects, tol = 0.01) {
  if (!tileRects || tileRects.length === 0) return false;
  const label = (det.label || '').toLowerCase();
  if (label !== 'person' && det.class_id !== 1) return false;
  const bbox = det.bbox || [];
  if (bbox.length < 4) return false;
  const [bx, by, bw, bh] = bbox.map(Number);
  const detArea = bw * bh;
  const detCx = bx + bw / 2, detCy = by + bh / 2;
  for (const { x: tx, y: ty, w: tw, h: th } of tileRects) {
    if (Math.abs(bx - tx) < tol && Math.abs(by - ty) < tol
        && Math.abs(bw - tw) < tol && Math.abs(bh - th) < tol) return true;
    const tileArea = tw * th;
    if (tileArea > 0 && detArea >= 0.75 * tileArea) {
      const tCx = tx + tw / 2, tCy = ty + th / 2;
      if (Math.abs(detCx - tCx) < 0.5 * tw && Math.abs(detCy - tCy) < 0.5 * th) return true;
    }
  }
  return false;
}

export function isContainedFragment(detSmall, detBig, areaRatioMax = 0.5, centerSlack = 0.0) {
  const lblS = detSmall.label ? detSmall.label.toLowerCase() : null;
  const lblB = detBig.label ? detBig.label.toLowerCase() : null;
  if (lblS !== null && lblB !== null) {
    if (lblS !== lblB) return false;
  } else {
    const cs = detSmall.class_id, cb = detBig.class_id;
    if (cs == null || cb == null || (cs | 0) !== (cb | 0)) return false;
  }
  const bs = detSmall.bbox || [], bb = detBig.bbox || [];
  if (bs.length < 4 || bb.length < 4) return false;
  const [sx, sy, sw, sh] = bs.map(Number);
  const [bx, by, bw, bh] = bb.map(Number);
  const areaS = sw * sh, areaB = bw * bh;
  if (areaB <= 0) return false;
  if (areaS >= areaRatioMax * areaB) return false;          // strict <
  const cx = sx + sw / 2, cy = sy + sh / 2;
  if (cx < bx - centerSlack || cx > bx + bw + centerSlack) return false;
  if (cy < by - centerSlack || cy > by + bh + centerSlack) return false;
  return true;
}

// Area-DESC pass; suppressed dets can't suppress others; returns kept dets
// in ORIGINAL order (matches python containment_merge).
export function containmentMerge(dets, areaRatioMax = 0.5, centerSlack = 0.0) {
  const indexed = dets.map((d, i) => {
    const bbox = d.bbox || [];
    const area = bbox.length >= 4 ? Number(bbox[2]) * Number(bbox[3]) : 0;
    return { i, area, d };
  });
  indexed.sort((a, b) => (b.area - a.area) || (a.i - b.i));
  const suppressed = new Set();
  for (let big = 0; big < indexed.length; big++) {
    if (suppressed.has(indexed[big].i)) continue;
    for (let small = big + 1; small < indexed.length; small++) {
      if (suppressed.has(indexed[small].i)) continue;
      if (isContainedFragment(indexed[small].d, indexed[big].d, areaRatioMax, centerSlack)) {
        suppressed.add(indexed[small].i);
      }
    }
  }
  return dets.filter((_, i) => !suppressed.has(i));
}
