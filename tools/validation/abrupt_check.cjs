'use strict';
const fs = require('node:fs'), path = require('node:path'), assert = require('node:assert/strict');
const root = process.env.BBP_VIEWER_ROOT || path.resolve(__dirname, '../../../viewer');
const app = require(path.join(root, 'tools/autotune/lib.cjs')).loadApp();
const PL = require(path.join(root, 'tools/autotune/param_log.cjs'));
const PS = require(path.join(root, 'tools/autotune/param_semantics.cjs'));
const semantics = require('./semantics.cjs');
let reports=[];
for(const dir of process.argv.slice(2)) {
 const file=new app.FlightLog(fs.readFileSync(path.join(dir,'blackbox.bbl'))); assert.ok(file.openLog(0));
 const sc=file.getSysConfig(), sem=semantics(PS,sc['Firmware revision'],PL.parseParamHeader(sc.paramHeader).fixes);
 const got=PL.collect(file), journal=PL.assemble(got.strings), timeline=PL.buildTimeline({semantics:sem,sysConfig:file.getSysConfig(),records:journal,frames:got.frames,resumes:got.resumes});
 assert.ok(got.frames.iter.length>0); assert.equal(journal.crcErrors.length,0);
 let ends=0;for(const c of file.getChunksInTimeRange(file.getMinTime(),file.getMaxTime()))for(const e of c.events)if(e.event===255)ends++;
 assert.equal(ends,0,'power loss has no LOG_END');
 const host=JSON.parse(fs.readFileSync(path.join(dir,'timeline.json'))), writes=host.events.filter(e=>e.write && e.truth);
 let checked=0;
 for(const w of writes)for(const [key,[old,value]] of Object.entries(w.truth))for(let i=0;i<got.frames.iter.length;i++) {
   if(got.frames.time[i] < w.t_sitl_reply_us+10000)continue;
   if(writes.some(q=>q!==w&&q.truth[key]&&q.t_sitl_send_us>w.t_sitl_send_us&&q.t_sitl_send_us-10000<got.frames.time[i]))continue;
   const v=timeline.valueAt(key,got.frames.iter[i]); if(v.status==='exact') {assert.equal(String(v.stored),value); checked++;}
 }
 const top=Math.max(...got.frames.paramSeq); if(top>journal.maxSeq)assert.ok(timeline.trailing);
 reports.push({dir,frames:got.frames.iter.length,exactValuesChecked:checked,maxSequence:journal.maxSeq,frameSequence:top,trailingLoss:timeline.trailing,corrupt:file.getStats(0).totalCorruptFrames});
}
console.log(JSON.stringify(reports,null,2));
