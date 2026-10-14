'use strict';
const fs = require('node:fs'), path = require('node:path'), assert = require('node:assert/strict'), crypto = require('node:crypto');
const root = process.env.BBP_VIEWER_ROOT || path.resolve(__dirname, '../../../viewer');
const lib = require(path.join(root, 'tools/autotune/lib.cjs'));
const current = lib.loadApp(root), old = lib.loadApp(path.resolve(root, '../viewer-2.3.0'));
const reports=[];
for(const f of process.argv.slice(2)) {
 const bytes=fs.readFileSync(f), a=new current.FlightLog(bytes), b=new old.FlightLog(bytes);
 assert.equal(a.getLogCount(), b.getLogCount()); let total=0;
 for(let i=0;i<a.getLogCount();i++) {
  if(a.getLogError(i)||b.getLogError(i)){assert.equal(String(a.getLogError(i)),String(b.getLogError(i)));continue;}
  assert.ok(a.openLog(i));assert.ok(b.openLog(i));
  const names=Array.from(a.getMainFieldNames()); assert.deepEqual(names,Array.from(b.getMainFieldNames()));
  function digest(log){let n=0; const hash=crypto.createHash('sha256');for(const c of log.getChunksInTimeRange(log.getMinTime(), log.getMaxTime()))for(const row of c.frames){hash.update(JSON.stringify(row)); n++;}return {n,hash:hash.digest('hex')};}
  const x=digest(a), y=digest(b);assert.deepEqual(x,y); total+=x.n;
  assert.equal(a.getStats(i).totalCorruptFrames, b.getStats(i).totalCorruptFrames);
  assert.equal(a.getSysConfig().paramHeader, undefined);
 }
 reports.push({file:path.basename(f),sha256:crypto.createHash('sha256').update(bytes).digest('hex'),logs:a.getLogCount(),frames:total,identical:true});
}
console.log(JSON.stringify(reports,null,2));
