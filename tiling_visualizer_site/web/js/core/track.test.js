// track.test.js — smoothed target trajectory.
import test from 'node:test';
import assert from 'node:assert/strict';
import { targetCenter, smoothTargetTrack } from './track.js';

const tgt = (frame, cx, cy) => ({
  frame,
  detections: [{ label: 'target', confidence: 0.9, bbox: [cx - 0.05, cy - 0.05, 0.1, 0.1] }],
});

test('targetCenter returns bbox center of the target detection', () => {
  assert.deepEqual(targetCenter(tgt(0, 0.5, 0.4)), { cx: 0.5, cy: 0.4 });
});

test('targetCenter ignores non-target detections / empty frames', () => {
  assert.equal(targetCenter({ detections: [{ label: 'vehicle', bbox: [0, 0, 0.1, 0.1] }] }), null);
  assert.equal(targetCenter({ detections: [] }), null);
  assert.equal(targetCenter({}), null);
});

test('first sample seeds the EMA exactly at the raw center', () => {
  const m = smoothTargetTrack([tgt(0, 0.2, 0.2)], 0.35);
  assert.deepEqual(m.get(0), { cx: 0.2, cy: 0.2 });
});

test('EMA lags raw motion (smoothed sits between previous and new)', () => {
  const m = smoothTargetTrack([tgt(0, 0.0, 0.0), tgt(1, 1.0, 1.0)], 0.5);
  // seed at 0, then 0.5*1 + 0.5*0 = 0.5
  assert.deepEqual(m.get(0), { cx: 0, cy: 0 });
  assert.deepEqual(m.get(1), { cx: 0.5, cy: 0.5 });
});

test('frames without a target get no entry (crosshair hides)', () => {
  const frames = [tgt(0, 0.3, 0.3), { frame: 1, detections: [] }, tgt(2, 0.3, 0.3)];
  const m = smoothTargetTrack(frames, 0.5);
  assert.ok(m.has(0));
  assert.ok(!m.has(1));
  assert.ok(m.has(2));
});

test('a long gap resets the filter (no glide across empty space)', () => {
  // target jumps far after a > resetGap absence; smoothed should snap, not lag.
  const frames = [tgt(0, 0.1, 0.1), tgt(100, 0.9, 0.9)];
  const m = smoothTargetTrack(frames, 0.5, 30);
  assert.deepEqual(m.get(100), { cx: 0.9, cy: 0.9 }); // reset → exact, no blend
});

test('is order-independent (sorts by frame before smoothing)', () => {
  const a = smoothTargetTrack([tgt(0, 0, 0), tgt(1, 1, 1)], 0.5);
  const b = smoothTargetTrack([tgt(1, 1, 1), tgt(0, 0, 0)], 0.5);
  assert.deepEqual(b.get(1), a.get(1));
});
