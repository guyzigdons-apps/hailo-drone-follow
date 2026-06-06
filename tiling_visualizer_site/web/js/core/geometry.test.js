// geometry.test.js
import test from 'node:test';
import assert from 'node:assert/strict';
import { gridToStaticTiles, computeTileRects, contentRect } from './geometry.js';

test('1x1 grid is the full frame', () => {
  assert.deepEqual(gridToStaticTiles(1, 1, 0, 0), [{ x: 0, y: 0, w: 1, h: 1 }]);
});

test('2x2 grid no overlap', () => {
  const rects = gridToStaticTiles(2, 2, 0, 0);
  assert.equal(rects.length, 4);
  assert.deepEqual(rects[0], { x: 0, y: 0, w: 0.5, h: 0.5 });
  assert.deepEqual(rects[3], { x: 0.5, y: 0.5, w: 0.5, h: 0.5 });
});

test('8x6 grid with 0.25 overlap matches python math', () => {
  // T = 1/(8 - 7*0.25) = 0.16; S = 0.16*0.75 = 0.12
  const rects = gridToStaticTiles(8, 6, 0.25, 0.25);
  assert.equal(rects.length, 48);
  assert.ok(Math.abs(rects[0].w - 0.16) < 1e-9);
  assert.ok(Math.abs(rects[1].x - 0.12) < 1e-9);
  assert.ok(Math.abs(rects[7].x + rects[7].w - 1.0) < 1e-9);
});

test('invalid grid returns empty', () => {
  assert.deepEqual(gridToStaticTiles(0, 3, 0, 0), []);
});

test('computeTileRects: full config', () => {
  const rects = computeTileRects({
    tiles_x: 2, tiles_y: 2, overlap_x: 0, overlap_y: 0,
    include_full_frame: true, include_center_tile: true, center_tile_size: 0.4,
    extra_grids: [[1, 1, 0, 0]], extra_rects: [[0.1, 0.2, 0.3, 0.4]],
  });
  // 4 grid + 1 full + 1 center + 1 extra-grid + 1 extra-rect
  assert.equal(rects.length, 8);
  assert.deepEqual(rects[4], { x: 0, y: 0, w: 1, h: 1 });
  assert.ok(Math.abs(rects[5].x - 0.3) < 1e-9 && Math.abs(rects[5].w - 0.4) < 1e-9);
  assert.deepEqual(rects[7], { x: 0.1, y: 0.2, w: 0.3, h: 0.4 });
});

test('computeTileRects: null/missing config', () => {
  assert.deepEqual(computeTileRects(null), []);
  assert.deepEqual(computeTileRects({}), []);
});

test('contentRect: 16:9 video in wider stage pillarboxes horizontally', () => {
  const r = contentRect(2000, 900, 1920, 1080);
  assert.equal(r.h, 900);
  assert.equal(r.w, 1600);
  assert.equal(r.x, 200);
  assert.equal(r.y, 0);
});

test('contentRect: 16:9 video in taller stage letterboxes vertically', () => {
  const r = contentRect(1600, 1000, 1920, 1080);
  assert.equal(r.w, 1600);
  assert.equal(r.h, 900);
  assert.equal(r.x, 0);
  assert.equal(r.y, 50);
});
