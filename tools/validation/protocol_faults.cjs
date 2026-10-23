'use strict';
// Fault injection against captured firmware output and an independent synthetic old-format file.
const fs = require('node:fs'), path = require('node:path'), assert = require('node:assert/strict');
const root = process.env.BBP_VIEWER_ROOT || path.resolve(__dirname, '../../../viewer');
const lib = require(path.join(root, 'tools/autotune/lib.cjs'));
const PL = require(path.join(root, 'tools/autotune/param_log.cjs'));
const enc = require(path.join(root, 'test/helpers/bbl_encode.cjs'));
const app = lib.loadApp();
const raw = fs.readFileSync(process.argv[2]);
const log = new app.FlightLog(raw); assert.ok(log.openLog(0));
const sc = log.getSysConfig(), got = PL.collect(log), clean = PL.assemble(got.strings);
assert.equal(clean.crcErrors.length, 0); assert.equal(clean.gaps.length, 0);
let checks = 0;
const text = got.strings.map(x => x.text);
const c = text.findIndex(x => /^PC/.test(x)); assert.ok(c >= 0);
const bad = text.slice(); bad[c] = bad[c].slice(0, -1) + (bad[c].endsWith('0') ? '1' : '0');
const corrupt = PL.assemble(bad); assert.equal(corrupt.crcErrors.length, 1); checks++;
const ct = PL.buildTimeline({sysConfig: sc, records: corrupt, frames: got.frames});
assert.ok(ct.flags.some(f => f.type === 'crc')); checks++;
const seq = PL.parseRecord(text[c]).seq;
const lost = PL.assemble(text.filter(x => PL.parseRecord(x).seq !== seq));
assert.ok(lost.gaps.length); checks++;
const duplicate = PL.assemble([...text, text[c]]); assert.ok(duplicate.duplicates.includes(seq)); checks++;
const plus = text.findIndex(x => /^P\+/.test(x));
if (plus >= 0) { const j = PL.assemble(text.filter((_, i) => i !== plus)); assert.ok(j.incomplete.length); checks++; }
for (const cut of [1, 8, Math.floor(text[c].length / 2), text[c].length - 1]) {
    const j = PL.assemble([...text.slice(0, c), text[c].slice(0, cut)]);
    assert.ok(!j.records.has(seq) || j.records.get(seq).crcOk === false); checks++;
}
const h = Array.from(sc.paramHeader, x => Array.from(x));
for (const n of [1, Math.floor(h.length / 2), h.length - 1]) {
    const partial = PL.parseParamHeader(h.slice(0, n)); assert.equal(partial.complete, false); checks++;
}
// Old header arrays belong to individual logs, including reverse opens and parser reuse.
const a = enc.simulateFlight({seconds: 1, airborne: null}), b = enc.simulateFlight({seconds: 1, airborne: null});
a.header = {...a.header, 'legacy_probe': 'first', 'looptime': '250'};
b.header = {...b.header, 'legacy_probe': 'second', 'looptime': '500'};
const two = new app.FlightLog(enc.encode([a, b]).bytes);
for (const i of [0, 1, 0, 1]) {
    assert.ok(two.openLog(i));
    const cfg = two.getSysConfig();
    assert.equal(cfg.unknownHeaders.filter(x => x.name === 'legacy_probe').length, 1);
    assert.equal(cfg.unknownHeaders.find(x => x.name === 'legacy_probe').value, i ? 'second' : 'first');
    assert.equal(cfg.looptime, i ? 500 : 250); checks++;
}
console.log(JSON.stringify({checks, firmwareStrings: text.length, continuationFaultCovered: plus >= 0, passed: true}, null, 2));
