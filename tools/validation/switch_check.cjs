'use strict';
const fs = require('node:fs'), path = require('node:path'), assert = require('node:assert/strict');
const root = process.env.BBP_VIEWER_ROOT || path.resolve(__dirname, '../../../viewer');
const app = require(path.join(root, 'tools/autotune/lib.cjs')).loadApp();
const f = new app.FlightLog(fs.readFileSync(process.argv[2])); assert.equal(f.getLogCount(), 2);
let frames = 0;
for(let n=0;n<2;n++) { assert.ok(f.openLog(n)); let end=0, resume=0; for(const c of f.getChunksInTimeRange(f.getMinTime(),f.getMaxTime())) { frames+=c.frames.length; for(const e of c.events) {if(e.event===255)end++; if(e.event===14)resume++;}} assert.equal(end,1); assert.equal(resume,0); assert.equal(f.getStats(n).totalCorruptFrames,0); }
console.log(JSON.stringify({logs:2, frames, endedWithoutResume:true}));
