// frames.test.js
import test from 'node:test';
import assert from 'node:assert/strict';
import { indexFrames } from './frames.js';

const DOC = {
  label: 'run-a',
  config: { tiles_x: 2, tiles_y: 2, overlap_x: 0, overlap_y: 0 },
  frames: [
    { frame: 0, detections: [{ label: 'person', confidence: 0.9, bbox: [0.1, 0.1, 0.05, 0.1] }] },
    { frame: 2, detections: [], tiles: [{ x: 0, y: 0, w: 0.5, h: 0.5, category: 'dynamic' }] },
  ],
};

test('indexFrames builds O(1) lookup with gaps as undefined', () => {
  const idx = indexFrames(DOC);
  assert.equal(idx.label, 'run-a');
  assert.equal(idx.maxFrame, 2);
  assert.equal(idx.byFrame.get(0).detections.length, 1);
  assert.equal(idx.byFrame.get(1), undefined);
  assert.equal(idx.byFrame.get(2).tiles.length, 1);
});

test('indexFrames detects per-frame tiles presence', () => {
  assert.equal(indexFrames(DOC).hasTiles, true);
  assert.equal(indexFrames({ label: 'x', frames: [{ frame: 0, detections: [] }] }).hasTiles, false);
});

test('indexFrames precomputes static tile rects from config', () => {
  const idx = indexFrames(DOC);
  assert.equal(idx.staticTileRects.length, 4);
});

test('indexFrames tolerates missing config', () => {
  const idx = indexFrames({ label: 'x', frames: [] });
  assert.deepEqual(idx.staticTileRects, []);
  assert.equal(idx.maxFrame, -1);
});
