// overlay.test.js — color resolution for detection boxes.
import test from 'node:test';
import assert from 'node:assert/strict';
import { colorForDetection, TARGET_COLOR } from './overlay.js';

const RUN = '#00E5FF';

test('target detection is always red, overriding the run color', () => {
  assert.equal(colorForDetection({ label: 'target' }, RUN), TARGET_COLOR);
});

test('non-target detections keep the run color', () => {
  assert.equal(colorForDetection({ label: 'vehicle' }, RUN), RUN);
  assert.equal(colorForDetection({ label: 'person' }, RUN), RUN);
});

test('missing / unlabeled detection falls back to the run color', () => {
  assert.equal(colorForDetection({}, RUN), RUN);
  assert.equal(colorForDetection(null, RUN), RUN);
});
