import { test } from 'node:test';
import assert from 'node:assert/strict';
import { filterDetectionsPure } from './pipeline.js';

// Minimal indexFrames-shaped doc.
function mkDoc(detections, staticTileRects = []) {
  return {
    byFrame: new Map([[0, { frame: 0, detections }]]),
    staticTileRects,
  };
}

test('confidence filter drops below-threshold dets', () => {
  const doc = mkDoc([
    { label: 'person', confidence: 0.9, bbox: [0.1, 0.1, 0.1, 0.2] },
    { label: 'person', confidence: 0.2, bbox: [0.4, 0.4, 0.1, 0.2] },
  ]);
  const { dets } = filterDetectionsPure(doc, 0, 0.5, false, false);
  assert.equal(dets.length, 1);
  assert.equal(dets[0].confidence, 0.9);
});

test('missing frame yields empty list', () => {
  const doc = mkDoc([]);
  const { dets, phantoms, merged } = filterDetectionsPure(doc, 99, 0, true, true);
  assert.deepEqual(dets, []);
  assert.equal(phantoms, 0);
  assert.equal(merged, 0);
});

test('phantom filter counts and removes tile-matching person boxes', () => {
  const tile = { x: 0.0, y: 0.0, w: 0.5, h: 0.5 };
  const doc = mkDoc(
    [
      { label: 'person', confidence: 0.9, bbox: [0.0, 0.0, 0.5, 0.5] }, // == tile → phantom
      { label: 'person', confidence: 0.9, bbox: [0.6, 0.6, 0.1, 0.2] }, // real
    ],
    [tile]
  );
  const { dets, phantoms } = filterDetectionsPure(doc, 0, 0, true, false);
  assert.equal(phantoms, 1);
  assert.equal(dets.length, 1);
});

test('containment merge counts absorbed fragments', () => {
  const doc = mkDoc([
    { label: 'person', confidence: 0.9, bbox: [0.0, 0.0, 0.5, 0.8] }, // big
    { label: 'person', confidence: 0.9, bbox: [0.1, 0.1, 0.1, 0.1] }, // contained fragment
  ]);
  const { dets, merged } = filterDetectionsPure(doc, 0, 0, false, true);
  assert.equal(merged, 1);
  assert.equal(dets.length, 1);
});
