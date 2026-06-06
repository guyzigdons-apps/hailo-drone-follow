// state.test.js
import test from 'node:test';
import assert from 'node:assert/strict';
import { createStore, encodeHash, decodeHash } from './state.js';

test('store: set merges and notifies subscribers with changed keys', () => {
  const s = createStore({ a: 1, b: 2 });
  let seen = null;
  s.subscribe((state, changed) => { seen = [state.a, [...changed]]; });
  s.set({ a: 5 });
  assert.deepEqual(seen, [5, ['a']]);
  assert.equal(s.get().b, 2);
});

test('store: set with no actual change does not notify', () => {
  const s = createStore({ a: 1 });
  let calls = 0;
  s.subscribe(() => calls++);
  s.set({ a: 1 });
  assert.equal(calls, 0);
});

test('hash round-trip', () => {
  const sel = { videoId: '0025', fov: 'fov50', runs: ['r1', 'r2'], frame: 312, conf: 0.3 };
  const decoded = decodeHash(encodeHash(sel));
  // mode defaults to 'viewer' when absent from the hash
  assert.deepEqual(decoded, { ...sel, mode: 'viewer' });
});

test('hash round-trip carries metrics mode', () => {
  const sel = { videoId: '0025', fov: 'fov50', runs: [], frame: 0, conf: 0.25, mode: 'metrics' };
  const decoded = decodeHash(encodeHash(sel));
  assert.equal(decoded.mode, 'metrics');
});

test('decodeHash of garbage returns null', () => {
  assert.equal(decodeHash('#not=valid=stuff&&&'), null);
  assert.equal(decodeHash(''), null);
});
