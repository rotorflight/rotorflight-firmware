'use strict';
// Compare the H lines of each log of two .bbl files (e.g. blackbox_params = OFF vs the stock build).
//   node compare_headers.cjs <a.bbl> <b.bbl>
const fs = require('fs');
const assert = require('node:assert/strict');
function headers(file) {
    const buf = fs.readFileSync(file), logs = [];
    let i = 0;
    const MARK = Buffer.from('H Product:');
    while ((i = buf.indexOf(MARK, i)) >= 0) {
        const lines = []; let p = i;
        while (buf[p] === 0x48 && buf[p + 1] === 0x20) {   // "H "
            const e = buf.indexOf(0x0a, p); lines.push(buf.slice(p + 2, e).toString('latin1')); p = e + 1;
        }
        logs.push({ start: i, headerBytes: p - i, lines });
        i = p;
    }
    return logs;
}
const [A, B] = process.argv.slice(2).map(headers);
assert.ok(A.length > 0); assert.equal(A.length, B.length);
console.log(`logs: ${A.length} vs ${B.length}`);
for (let k = 0; k < Math.max(A.length, B.length); k++) {
    const a = A[k], b = B[k];
    if (!a || !b) { console.log(`log ${k + 1}: missing in one file`); continue; }
    console.log(`log ${k + 1}: ${a.lines.length} lines ${a.headerBytes} B vs ${b.lines.length} lines ${b.headerBytes} B`);
    const ka = a.lines.map(l => l.slice(0, l.indexOf(':'))), kb = b.lines.map(l => l.slice(0, l.indexOf(':')));
    assert.deepEqual(ka, kb, 'header keys and order');
    if (ka.join('\n') !== kb.join('\n')) {
        console.log('  key sequence differs');
        console.log('  only in a:', ka.filter(x => !kb.includes(x)).slice(0, 20));
        console.log('  only in b:', kb.filter(x => !ka.includes(x)).slice(0, 20));
    } else console.log('  same key sequence');
    for (let j = 0; j < a.lines.length; j++) {
        if (/^(Firmware revision|Firmware date|Log start datetime):/.test(a.lines[j])) continue;
        assert.equal(a.lines[j], b.lines[j], `header value log ${k + 1}, line ${j + 1}`);
    }
    for (const l of a.lines) if (/^Field S/.test(l)) console.log('  a ' + l);
}
for (const [n, f] of [['a', process.argv[2]], ['b', process.argv[3]]]) {
    const buf = fs.readFileSync(f);
    assert.equal(buf.indexOf('Param log'), -1);
    assert.equal((buf.toString('latin1').match(/P[CARMLQ+][0-9a-f]{1,8} [^*]*\*[0-9A-F]{2}/g) || []).length, 0);
    console.log(`${n}: "Param log" ${buf.indexOf('Param log') >= 0}, journal strings ${(buf.toString('latin1').match(/P[CARMLQ+][0-9a-f]{1,8} [^*]*\*[0-9A-F]{2}/g) || []).length}`);
}
