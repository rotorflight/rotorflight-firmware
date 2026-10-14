'use strict';
const assert = require('node:assert/strict');
const bounds = require('./iteration_time.cjs');
// A 25 ms host stall does not make the next iteration's timestamp the write time.
const iterations = [100, 102, 104], times = [100000, 125000, 127000];
assert.deepEqual(bounds(iterations, times, 102), [100000, 125000]);
assert.deepEqual(bounds(iterations, times, 101), [100000, 125000]);
const [earliest, latest] = bounds(iterations, times, 102);
assert.ok(earliest <= 102000 && latest >= 101000); // write during the stalled interval
assert.ok(latest < 150000); // a later unrelated write must still be rejected
assert.deepEqual(bounds(iterations, times, 100), [-Infinity, 100000]);
assert.deepEqual(bounds(iterations, times, 105), [127000, Infinity]);
console.log('Iteration-time bounds: 6 checks passed');
