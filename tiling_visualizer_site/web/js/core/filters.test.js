// filters.test.js
import test from 'node:test';
import assert from 'node:assert/strict';
import { isPhantom, isContainedFragment, containmentMerge } from './filters.js';

const TILES = [{ x: 0, y: 0, w: 0.5, h: 0.5 }, { x: 0.5, y: 0.5, w: 0.5, h: 0.5 }];

test('phantom: person bbox matching a tile rect within tol', () => {
  const det = { label: 'person', confidence: 0.9, bbox: [0.001, 0.002, 0.499, 0.501] };
  assert.equal(isPhantom(det, TILES), true);
});

test('phantom: vehicle never filtered even if tile-shaped', () => {
  const det = { label: 'vehicle', confidence: 0.9, bbox: [0, 0, 0.5, 0.5] };
  assert.equal(isPhantom(det, TILES), false);
});

test('phantom: large-person fallback (>=75% tile area, centered)', () => {
  const det = { label: 'person', bbox: [0.03, 0.03, 0.45, 0.45] };
  assert.equal(isPhantom(det, TILES), true);
});

test('phantom: small person not filtered', () => {
  const det = { label: 'person', bbox: [0.1, 0.1, 0.02, 0.05] };
  assert.equal(isPhantom(det, TILES), false);
});

test('phantom: empty tile list → never phantom', () => {
  const det = { label: 'person', bbox: [0, 0, 0.5, 0.5] };
  assert.equal(isPhantom(det, []), false);
});

test('phantom: class_id===1 path when label missing', () => {
  const det = { class_id: 1, bbox: [0.001, 0.001, 0.5, 0.5] };
  assert.equal(isPhantom(det, TILES), true);
});

test('containment: small same-class det inside big is fragment', () => {
  const big = { label: 'person', bbox: [0.1, 0.1, 0.4, 0.4] };
  const small = { label: 'person', bbox: [0.2, 0.2, 0.1, 0.1] };
  assert.equal(isContainedFragment(small, big), true);
});

test('containment: different class never merges', () => {
  const big = { label: 'person', bbox: [0.1, 0.1, 0.4, 0.4] };
  const small = { label: 'vehicle', bbox: [0.2, 0.2, 0.1, 0.1] };
  assert.equal(isContainedFragment(small, big), false);
});

test('containment: area ratio is strict <', () => {
  const big = { label: 'person', bbox: [0.0, 0.0, 0.4, 0.4] };   // area .16
  const small = { label: 'person', bbox: [0.1, 0.1, 0.4, 0.2] }; // area .08 == .5*.16
  assert.equal(isContainedFragment(small, big, 0.5, 0), false);
});

test('containment: center outside big → keep', () => {
  const big = { label: 'person', bbox: [0.0, 0.0, 0.2, 0.2] };
  const small = { label: 'person', bbox: [0.19, 0.19, 0.1, 0.1] };
  assert.equal(isContainedFragment(small, big), false);
});

test('containmentMerge keeps big, drops contained fragment, preserves order', () => {
  const dets = [
    { label: 'person', bbox: [0.2, 0.2, 0.05, 0.05] },
    { label: 'vehicle', bbox: [0.6, 0.6, 0.1, 0.1] },
    { label: 'person', bbox: [0.1, 0.1, 0.4, 0.4] },
  ];
  const kept = containmentMerge(dets);
  assert.equal(kept.length, 2);
  assert.equal(kept[0].label, 'vehicle');
  assert.equal(kept[1].bbox[2], 0.4);
});

test('containmentMerge with no overlaps returns all', () => {
  const dets = [
    { label: 'person', bbox: [0.0, 0.0, 0.1, 0.1] },
    { label: 'person', bbox: [0.5, 0.5, 0.1, 0.1] },
  ];
  assert.equal(containmentMerge(dets).length, 2);
});
