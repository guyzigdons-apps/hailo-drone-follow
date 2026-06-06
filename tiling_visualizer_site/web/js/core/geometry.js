// Port of tiling_benchmark/analyze_pxt.py tile math. Keep identical:
// T = 1/(N - (N-1)*o); step S = T*(1-o). N==1 → single full-axis tile.
export function gridToStaticTiles(tilesX, tilesY, overlapX, overlapY) {
  if (tilesX < 1 || tilesY < 1) return [];
  const axis = (n, o) => {
    if (n === 1) return [[0.0, 1.0]];
    const T = 1.0 / (n - (n - 1) * o);
    const S = T * (1.0 - o);
    return Array.from({ length: n }, (_, i) => [i * S, T]);
  };
  const rects = [];
  for (const [y, h] of axis(tilesY, overlapY)) {
    for (const [x, w] of axis(tilesX, overlapX)) {
      rects.push({ x, y, w, h });
    }
  }
  return rects;
}

function centerTileRect(size) {
  const x = (1.0 - size) / 2.0;
  return { x, y: x, w: size, h: size };
}

// Mirror of analyze_pxt._compute_tile_rects: main grid + optional
// full-frame + optional center tile + extra_grids + extra_rects.
export function computeTileRects(config) {
  if (!config) return [];
  const rects = gridToStaticTiles(
    (config.tiles_x | 0) || 0, (config.tiles_y | 0) || 0,
    +config.overlap_x || 0, +config.overlap_y || 0);
  if (config.include_full_frame) rects.push({ x: 0, y: 0, w: 1, h: 1 });
  if (config.include_center_tile) {
    rects.push(centerTileRect(+config.center_tile_size || 0.4));
  }
  for (const g of config.extra_grids || []) {
    rects.push(...gridToStaticTiles(g[0] | 0, g[1] | 0, +g[2] || 0, +g[3] || 0));
  }
  for (const r of config.extra_rects || []) {
    rects.push({ x: +r[0], y: +r[1], w: +r[2], h: +r[3] });
  }
  return rects;
}

// object-fit:contain math: where the video pixels actually sit inside the
// stage box. All overlay coords map through this rect.
export function contentRect(stageW, stageH, videoW, videoH) {
  const scale = Math.min(stageW / videoW, stageH / videoH);
  const w = videoW * scale, h = videoH * scale;
  return { x: (stageW - w) / 2, y: (stageH - h) / 2, w, h };
}
