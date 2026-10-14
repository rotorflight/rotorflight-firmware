'use strict';
/*
 * End-to-end check of one SITL run of tools/sitl/scenario_journal.py (or scenario_regrace.py): decode blackbox.bbl with
 * the viewer (lib.cjs loadApp + param_log.cjs) and compare it with the ground truth of timeline.json.
 *
 *   node analyze.cjs <run dir> [--json out.json] [--old <viewer root of an old build>] [--tol-ms 10]
 *
 * Prints one block per log and a JSON summary (also to --json).
 */
const fs = require('fs'), path = require('path');
const VIEWER = process.env.BBP_VIEWER_ROOT || path.resolve(__dirname, '../../../viewer');
const lib = require(path.join(VIEWER, 'tools/autotune/lib.cjs'));
const PL = require(path.join(VIEWER, 'tools/autotune/param_log.cjs'));
const PS = require(path.join(VIEWER, 'tools/autotune/param_semantics.cjs'));
const semantics = require('./semantics.cjs');
const iterationTimeBounds = require('./iteration_time.cjs');

const argv = process.argv.slice(2);
const opt = (name, d) => { const i = argv.indexOf(name); return i >= 0 ? argv[i + 1] : d; };
const dir = argv[0];
const TOL = +opt('--tol-ms', 10) * 1000;
const OLD = opt('--old', process.env.BBP_OLD_VIEWER_ROOT || path.resolve(VIEWER, '../viewer-2.3.0'));
const bytes = fs.readFileSync(path.join(dir, 'blackbox.bbl'));
const tlj = JSON.parse(fs.readFileSync(path.join(dir, 'timeline.json'), 'utf8'));
const events = tlj.events;
const writes = events.filter(e => e.write);

const app = lib.loadApp();
const flog = new app.FlightLog(bytes);
let oldApp = null;
try { if (OLD && fs.existsSync(path.join(OLD, 'js/flightlog.js'))) oldApp = lib.loadApp(OLD); } catch (e) { oldApp = null; console.error('old viewer:', e.message); }

if (!oldApp) throw new Error('The released viewer is required for the compatibility gate: ' + OLD);

const summary = { dir, logs: [], writes: writes.length, checks: {} };
const fail = (log, check, msg) => { (log.fail[check] = log.fail[check] || []).push(msg); };

function framesOf(fl) {
    const idx = (n) => fl.getMainFieldIndexByName(n);
    const names = ['axisP[0]', 'axisP[2]', 'setpoint[0]', 'setpoint[2]', 'gyroADC[0]', 'gyroADC[2]', 'flightModeFlags', 'pidProfile', 'rateProfile', 'armed', 'paramSeq'];
    const cols = {}; for (const n of names) cols[n] = idx(n);
    const out = { iter: [], time: [], rows: [], events: [] };
    for (const c of fl.getChunksInTimeRange(fl.getMinTime(), fl.getMaxTime())) {
        for (const f of c.frames) { out.iter.push(f[0]); out.time.push(f[1]); out.rows.push(f.slice ? Array.from(f) : f); }
        for (const e of c.events) out.events.push(e);
    }
    out.col = (name, i) => cols[name] === undefined ? undefined : out.rows[i][cols[name]];
    out.cols = cols;
    return out;
}

// ground truth: value of key at SITL time t (µs): { value, determinate }
function truthAt(key, t, initial) {
    let v = initial.get(key), det = true;
    for (const w of writes) {
        if (!w.truth || !w.truth[key]) continue;
        const a = w.t_sitl_send_us - TOL, b = w.t_sitl_reply_us + TOL;
        if (t >= b) v = w.truth[key][1];
        else if (t > a) det = false;
    }
    return { value: v, determinate: det };
}
const initial = new Map();
for (const w of writes) for (const k in (w.truth || {})) if (!initial.has(k)) initial.set(k, w.truth[k][0]);
// pid_profile/rate_profile start at the 'ready' status
const ready = events.find(e => e.event === 'ready');
if (ready && ready.status) { if (!initial.has('pid_profile')) initial.set('pid_profile', String(ready.status.pid_profile)); if (!initial.has('rate_profile')) initial.set('rate_profile', String(ready.status.rate_profile)); }

const timeOfIter = (F, n) => {   // SITL µs of loopIteration n (interpolated between logged frames)
    let lo = 0, hi = F.iter.length - 1;
    if (n <= F.iter[0]) return F.time[0] - (F.iter[0] - n) * ((F.time[hi] - F.time[0]) / (F.iter[hi] - F.iter[0]));
    if (n >= F.iter[hi]) return F.time[hi];
    while (hi - lo > 1) { const m = (lo + hi) >> 1; if (F.iter[m] <= n) lo = m; else hi = m; }
    return F.time[lo] + (F.time[hi] - F.time[lo]) * (n - F.iter[lo]) / (F.iter[hi] - F.iter[lo]);
};

for (let li = 0; li < flog.getLogCount(); li++) {
    const log = { index: li + 1, fail: {}, info: {} };
    summary.logs.push(log);
    if (flog.getLogError(li) || !flog.openLog(li)) { log.error = String(flog.getLogError(li)); fail(log, 'decode', log.error); continue; }
    const sc = flog.getSysConfig(), stats = flog.getStats(li);
    const F = framesOf(flog);
    log.info.frames = F.iter.length;
    if (!F.iter.length) fail(log, 'empty', 'No main frames');
    log.info.corrupt = stats.totalCorruptFrames;
    log.info.frameStats = Object.fromEntries(Object.entries(stats.frame || {}).map(([k, v]) => [k, { valid: v.validCount, corrupt: v.corruptCount, desync: v.desyncCount, bytes: v.bytes }]));
    log.info.tStart = F.time[0]; log.info.tEnd = F.time[F.time.length - 1];
    log.info.iter = [F.iter[0], F.iter[F.iter.length - 1]];
    log.info.sFields = (sc.frameDefs && sc.frameDefs.S) ? sc.frameDefs.S.name : null;
    log.info.paramLog = PL.isParamLog(sc);
    log.info.headerLines = null;
    if (log.info.corrupt) fail(log, 'corrupt', `${log.info.corrupt} corrupt frames`);
    // iteration continuity: steps of the P interval only
    const steps = new Map();
    for (let i = 1; i < F.iter.length; i++) { const d = F.iter[i] - F.iter[i - 1]; steps.set(d, (steps.get(d) || 0) + 1); }
    log.info.iterSteps = Object.fromEntries(steps);
    const expectedStep = sc.frameIntervalPDenom / sc.frameIntervalPNum;
    if ([...steps.keys()].some(x => x !== expectedStep)) fail(log, 'continuity', JSON.stringify(log.info.iterSteps));
    const ev101 = F.events.filter(e => e.event === 101 && e.data && typeof e.data.string === 'string');
    log.info.event101 = ev101.length;
    log.info.events = {}; for (const e of F.events) log.info.events[e.event] = (log.info.events[e.event] || 0) + 1;

    // blind window: arm command (host) -> first frame (SITL clock)
    const arms = events.filter(e => e.event === 'arm_cmd' && e.t_sitl_us !== undefined);
    const armBefore = arms.filter(e => e.t_sitl_us < F.time[0]).pop();
    if (armBefore) {
        const armed = events.find(e => e.event === 'armed' && e.t_monotonic >= armBefore.t_monotonic);
        log.info.blind = { armCmdToFirstFrameMs: +((F.time[0] - armBefore.t_sitl_us) / 1000).toFixed(1), armedPollToFirstFrameMs: armed ? +((F.time[0] - armed.t_sitl_us) / 1000).toFixed(1) : null };
    }

    // writes of this log: in the frame time range; writes before the first frame and after the previous log: 'v' (or 'p')
    const prevEnd = li > 0 && summary.logs[li - 1].info.tEnd !== undefined ? summary.logs[li - 1].info.tEnd : -Infinity;
    const inLog = writes.filter(w => w.t_sitl_send_us >= F.time[0] && w.t_sitl_send_us <= log.info.tEnd);
    const before = writes.filter(w => w.t_sitl_send_us < F.time[0] && w.t_sitl_send_us > prevEnd);
    log.info.writesInLog = inLog.length; log.info.writesBefore = before.length;

    if (!log.info.paramLog) {
        log.info.mode = 'stock/OFF';
        if (ev101.length) fail(log, 'no101', `${ev101.length} event 101 in a log without Param log`);
        log.info.unknownHeaders = (sc.unknownHeaders || []).map(h => h.name);
        if (oldApp) compareOld(li, F, log);
        continue;
    }

    const got = PL.collect(flog), journal = PL.assemble(got.strings);
    const sem = semantics(PS, sc['Firmware revision'], PL.parseParamHeader(sc.paramHeader).fixes);
    log.info.semanticScope = sem.firmware;
    const tl = PL.buildTimeline({ semantics: sem, sysConfig: sc, records: journal, frames: got.frames, resumes: got.resumes });
    const H = tl.header;
    const denomCase = path.basename(dir).match(/^denom-(\d+)$/);
    if (denomCase && H.loop.pidDenom !== +denomCase[1]) fail(log, 'denominator', `requested ${denomCase[1]}, actual ${H.loop.pidDenom}`);
    log.info.mode = H.mode;
    log.info.header = { complete: H.complete, lines: H.lines, end: H.end, hash: H.hash !== null ? H.hash.toString(16) : null, long: H.long.length, extra: H.extra.length,
        loop: H.loop, phase: H.phase, pid0: H.pid0, rate0: H.rate0, master: H.master.size, pid: H.pid.size, rate: H.rate.size, el: H.el.size, raw: H.raw.size, boot: H.boot.size };
    if (!H.complete) fail(log, 'header', `param_end ${JSON.stringify(H.end)} vs counted ${H.lines} lines hash ${log.info.header.hash}`);
    log.info.records = journal.records.size;
    log.info.journal = { gaps: journal.gaps, crcErrors: journal.crcErrors.length, incomplete: journal.incomplete, orphans: journal.orphans, duplicates: journal.duplicates, maxSeq: journal.maxSeq, trailing: tl.trailing, losses: tl.losses.length, flags: tl.flags };
    if (journal.crcErrors.length) fail(log, 'crc', `${journal.crcErrors.length} CRC errors`);
    if (journal.gaps.length) fail(log, 'gaps', JSON.stringify(journal.gaps));
    if (journal.incomplete.length) fail(log, 'gaps', `incomplete ${journal.incomplete}`);
    const recs = [...journal.records.values()];
    const uy = recs.filter(r => r.type === 'C' && (r.src === 'u' || r.src === 'y'));
    const Ls = recs.filter(r => r.type === 'L');
    if (uy.length || Ls.length) fail(log, 'uy', uy.concat(Ls).map(r => r.text).join(' | '));
    const Q = recs.find(r => r.type === 'Q');
    log.info.Q = Q ? { unsent: Q.unsent, lostrec: Q.lostrec } : null;
    if (Q && (Q.unsent || Q.lostrec)) fail(log, 'Q', Q.text);
    log.info.selfTest = PL.selfTest(tl, sc);
    if (!log.info.selfTest.ok) fail(log, 'selftest', JSON.stringify(log.info.selfTest.mismatches));
    log.info.recordTypes = {}; for (const r of recs) { const k = r.type + (r.type === 'C' ? ':' + r.src : r.type === 'A' ? ':' + r.loader : ''); log.info.recordTypes[k] = (log.info.recordTypes[k] || 0) + 1; }

    // paramSeq: monotonic, at most maxSeq
    const ps = F.cols.paramSeq;
    for (const name of ['paramSeq', 'pidProfile', 'rateProfile', 'armed']) if (F.cols[name] === undefined) fail(log, 'slowFields', name + ' missing');
    let psBad = 0, psMax = 0;
    for (let i = 0; i < F.iter.length; i++) { const v = F.rows[i][ps]; if (v > journal.maxSeq) psBad++; if (v < psMax) psBad++; psMax = Math.max(psMax, v); }
    log.info.paramSeqMax = psMax;
    if (psBad) fail(log, 'paramSeq', `${psBad} frames with paramSeq above the max SEQ or going down`);

    // ---- every ground-truth write has its C record with the right point ----
    const cItems = [];
    for (const r of recs) if (r.type === 'C') for (const it of r.items) cItems.push({ r, it });
    const matched = new Set();
    log.writes = [];
    const expectWrites = inLog.concat(before);
    for (const w of expectWrites) {
        const keys = Object.keys(w.truth || {});
        const row = { event: w.event, truth: w.truth, tSend: w.t_sitl_send_us, tReply: w.t_sitl_reply_us, records: [] };
        log.writes.push(row);
        for (const k of keys) {
            const [o, n] = w.truth[k];
            const isBefore = before.includes(w);
            const cand = cItems.filter(x => !matched.has(x) && x.it.key === k && x.it.value === n && x.it.old === o && (isBefore ? x.r.at && x.r.at.pre : !(x.r.at && x.r.at.pre)));
            if (!cand.length) { fail(log, 'C', `${w.event} ${k}=${n}<${o}: no C record`); continue; }
            // the one whose point is closest to the write
            let best = cand[0];
            if (!isBefore) {
                const dt = (x) => x.r.at.n !== undefined ? Math.abs(timeOfIter(F, x.r.at.n) - (w.t_sitl_send_us + w.t_sitl_reply_us) / 2) : Infinity;
                best = cand.reduce((a, b) => dt(b) < dt(a) ? b : a);
            }
            matched.add(best);
            const rr = { key: k, seq: best.r.seq, at: PL.pointText(best.r.at), src: best.r.src + (best.r.arg !== null ? '.' + best.r.arg : '') };
            if (isBefore) { if (best.r.src !== (w.expected_source || 'v')) fail(log, 'C', `${k}: pre-frame record source ${best.r.src}, expected ${w.expected_source || 'v'}`); }
            else {
                if (best.r.at.n === undefined) fail(log, 'C', `${k}: not an exact point ${rr.at}`);
                else {
                    const tp = timeOfIter(F, best.r.at.n);
                    rr.afterSendMs = +((tp - w.t_sitl_send_us) / 1000).toFixed(2);
                    rr.beforeReplyMs = +((w.t_sitl_reply_us - tp) / 1000).toFixed(2);
                    // A journal point identifies a scheduling position, not a wall-clock timestamp.
                    // It occurs after the previous frame and by this frame's Blackbox tick.
                    // Interpolation alone invents precision when Docker pauses between frames.
                    const bounds = iterationTimeBounds(F.iter, F.time, best.r.at.n);
                    rr.frameTimeBoundsUs = bounds;
                    if (bounds[1] < w.t_sitl_send_us - TOL || bounds[0] > w.t_sitl_reply_us + TOL) {
                        fail(log, 'C', `${k}: point ${rr.at} frame interval ${bounds} does not overlap MSP write ${w.t_sitl_send_us}..${w.t_sitl_reply_us}`);
                    }
                }
                const extraSources = {copy_active_pid: 'm.183', reset_active_pid: 'm.219', set_governor_config: 'm.143', set_servo_with_arg: 'm.124', set_filter_config: 'm.93'};
                const wantSrc = w.journal_source || extraSources[w.event] || (w.event === 'eeprom_write' ? 'm.250' : w.event.startsWith('select') ? 'm.210' : 'm.202');
                if (rr.src !== wantSrc) fail(log, 'C', `${k}: source ${rr.src}, expected ${wantSrc}`);
            }
            row.records.push(rr);
        }
    }
    // C items that no write explains
    const extra = cItems.filter(x => !matched.has(x));
    log.info.unexplainedItems = extra.map(x => `${x.r.text}`);
    if (extra.length) fail(log, 'C', `${extra.length} C items with no ground-truth write: ${extra.map(x => x.it.key).join(',')}`);

    // ---- valueAt equals the ground truth outside flagged frames ----
    const tkeys = [...initial.keys()];
    const statusCount = {}, mismatch = [], flaggedMismatch = [], keyFrames = {};
    let det = 0, indet = 0;
    for (let i = 0; i < F.iter.length; i++) {
        const it = F.iter[i], t = F.time[i];
        for (const k of tkeys) {
            const tr = truthAt(k, t, initial);
            const v = tl.valueAt(k, it);
            statusCount[v.status] = (statusCount[v.status] || 0) + 1;
            if (!tr.determinate) { indet++; continue; }
            det++;
            keyFrames[k] = (keyFrames[k] || 0) + 1;
            const flagged = v.status === 'mixed' || v.status === 'uncertain' || v.status === 'unknown';
            if (String(v.stored) !== String(tr.value)) (flagged ? flaggedMismatch : mismatch).push({ i, iter: it, key: k, stored: v.stored, truth: tr.value, status: v.status });
        }
    }
    log.info.valueAt = { keys: tkeys, comparisons: det, indeterminate: indet, statusCount, mismatches: mismatch.length, flaggedMismatches: flaggedMismatch.length };
    if (mismatch.length) fail(log, 'valueAt', `${mismatch.length} mismatches, first ${JSON.stringify(mismatch.slice(0, 3))}`);

    // ---- S-frame profile indices: truth and journal ----
    let profBad = 0, profJ = 0, rateBad = 0;
    for (let i = 0; i < F.iter.length; i++) {
        const t = F.time[i], it = F.iter[i];
        const tp = truthAt('pid_profile', t, initial), tr = truthAt('rate_profile', t, initial);
        const sp = F.col('pidProfile', i), sr = F.col('rateProfile', i);
        if (tp.determinate && String(sp) !== String(tp.value)) profBad++;
        if (tr.determinate && String(sr) !== String(tr.value)) rateBad++;
        const jv = tl.valueAt('pid_profile', it, 'bb');
        if (jv.status === 'exact' && String(jv.stored) !== String(sp)) profJ++;
    }
    log.info.profiles = { sframeVsTruthPid: profBad, sframeVsTruthRate: rateBad, sframeVsJournal: profJ };
    if (profBad || rateBad || profJ) fail(log, 'profile', JSON.stringify(log.info.profiles));

    // ---- S-frame armed against the host arm and disarm commands ----
    {
        const sw = [];   // [t0, t1, state]: the state is known outside the windows command..confirmation
        for (const e of events) if (e.event === 'arm_cmd' || e.event === 'disarm_cmd') sw.push({ t: e.t_sitl_us, to: e.event === 'arm_cmd' ? 1 : 0 });
        const conf = events.filter(e => e.event === 'armed' || e.event === 'disarmed');
        let bad = 0, n = 0;
        for (let i = 0; i < F.iter.length; i++) {
            const t = F.time[i];
            let st = 0, det = true;
            for (const c of sw) {
                const done = conf.find(x => x.t_sitl_us >= c.t);
                if (done && t >= done.t_sitl_us + TOL) st = c.to;
                else if (t > c.t - TOL) det = false;
            }
            if (!det) continue;
            n++;
            if (F.col('armed', i) !== st) bad++;
        }
        log.info.armedCheck = { frames: n, mismatches: bad };
        if (bad) fail(log, 'armed', `${bad} frames with S-frame armed different from the host commands`);
    }
    // ---- final state against the read-back of the 'done' step (the active PID profile) ----
    const done = events.find(e => e.event === 'done' && e.pid && e.t_sitl_us > F.time[0] && (li + 1 === flog.getLogCount() || e.t_sitl_us < F.time[F.time.length - 1] + 20e6));
    if (done && li + 1 === flog.getLogCount()) {
        const last = F.iter[F.iter.length - 1], prof = done.status.pid_profile, out = {};
        let compared = 2, unknown = 0;
        for (const axis of ['roll', 'pitch', 'yaw']) for (const [t, n] of [['P', 'p'], ['I', 'i'], ['D', 'd'], ['F', 'f']]) {
            const v = tl.valueAt(`p${prof}.${axis}_${n}_gain`, last);
            if (H.mode === 'CHANGES' && !cItems.some(x => x.it.key === `p${prof}.${axis}_${n}_gain`)) {
                unknown++;
                if (v.stored !== null || v.status === 'exact') fail(log, 'changesUnknown', `${axis}.${t} falsely known`);
                continue;
            }
            compared++;
            if (String(v.stored) !== String(done.pid[axis][t])) out[`${axis}.${t}`] = [v.stored, done.pid[axis][t]];
        }
        if (String(tl.valueAt('pid_profile', last).stored) !== String(prof)) out.pid_profile = [tl.valueAt('pid_profile', last).stored, prof];
        if (String(tl.valueAt('rate_profile', last).stored) !== String(done.status.rate_profile)) out.rate_profile = [tl.valueAt('rate_profile', last).stored, done.status.rate_profile];
        log.info.finalState = { compared, unknown, mismatches: out };
        if (Object.keys(out).length) fail(log, 'final', JSON.stringify(out));
    }

    // ---- physics: axisP = k * P(running, pid tick) * (setpoint - gyro) ----
    log.physics = {};
    for (const [ax, axis] of [[0, 'roll'], [2, 'yaw']]) {
        const rows = [];
        for (let i = 0; i < F.iter.length; i++) {
            const err = F.col(`setpoint[${ax}]`, i) - F.col(`gyroADC[${ax}]`, i), p = F.col(`axisP[${ax}]`, i);
            // the PID profile at the tick of the PID subtask (the S-frame has it at the blackbox tick)
            const pj = tl.valueAt('pid_profile', F.iter[i], 'pid'), prof = pj.stored !== null ? pj.stored : F.col('pidProfile', i);
            const v = tl.valueAt(`p${prof}.${axis}_p_gain`, F.iter[i], 'pid');
            // running value; where the viewer gives none (uncertain), the stored value, counted apart
            const P = v.running !== null ? +v.running : v.stored !== null ? +v.stored : null;
            rows.push({ i, err, p, P, fromStored: v.running === null, status: v.status, flags: v.flags, armed: F.col('armed', i) });
        }
        const use = rows.filter(r => r.P !== null && Math.abs(r.err) > 0);
        let num = 0, den = 0; for (const r of use) { num += r.p * r.P * r.err; den += (r.P * r.err) ** 2; }
        const k = den ? num / den : null;
        let bad = [], maxRes = 0; const zeros = rows.filter(r => r.p === 0).length;
        for (const r of use) { const res = r.p - k * r.P * r.err; maxRes = Math.max(maxRes, Math.abs(res)); if (Math.abs(res) > 1.01) bad.push({ i: r.i, iter: F.iter[r.i], p: r.p, pred: +(k * r.P * r.err).toFixed(2), P: r.P, err: r.err, status: r.status, armed: r.armed }); }
        const errs = [...new Set(use.map(r => r.err))];
        const st = {}; for (const r of rows) { const key = r.status + (r.flags && r.flags.length ? '/' + r.flags.join('+') : ''); st[key] = (st[key] || 0) + 1; }
        const storedRows = use.filter(r => r.fromStored);
        log.physics[axis] = { k, frames: use.length, maxResidual: +maxRes.toFixed(3), bad: bad.length, badFirst: bad.slice(0, 8), zeroP: zeros, errorValues: errs.slice(0, 6),
            statusOfP: st, framesWithoutRunning: storedRows.length, framesWithoutRunningMatchStored: storedRows.filter(r => Math.abs(r.p - k * r.P * r.err) <= 1.01).length };
        if (bad.length) fail(log, 'physics', `${axis}: ${bad.length} frames with |axisP - k P e| > 1`);
    }
    // the tick rule at each roll P record: logged frame N exists?
    const D = tl.pidDenom, phase = tl.phase;
    log.ticks = [];
    for (const r of recs) {
        if (r.type !== 'C' || !r.at || r.at.n === undefined) continue;
        const it = r.items.find(x => /roll_p_gain$/.test(x.key) || x.key === 'pid_profile');
        if (!it) continue;
        const fi = F.iter.indexOf(r.at.n);
        const row = { seq: r.seq, at: PL.pointText(r.at), key: it.key, frameLogged: fi >= 0, mixed: tl.mixed.has(r.at.n) };
        if (fi >= 0 && /roll_p_gain$/.test(it.key)) {
            // COPY/RESET changes stored gains without reloading the active PID controller on stock 4.6.
            // Their running values are checked above; the write point alone does not apply them.
            if (r.src === 'm' && [183, 219].includes(r.arg)) {
                row.pending = true; log.ticks.push(row); continue;
            }
            const err = F.col('setpoint[0]', fi) - F.col('gyroADC[0]', fi), p = F.col('axisP[0]', fi), k = log.physics.roll.k;
            const isNew = Math.abs(p - k * (+it.value) * err) <= 1.01, isOld = Math.abs(p - k * (+it.old) * err) <= 1.01;
            row.frameN = isNew && !isOld ? 'new' : isOld && !isNew ? 'old' : 'ambiguous';
            row.expected = (phase.pid >= r.at.c) ? 'new' : 'old';
            if (row.frameN !== row.expected) fail(log, 'tick', `${row.at}: frame N is ${row.frameN}, the tick rule gives ${row.expected}`);
        }
        log.ticks.push(row);
    }

    // epochs that are not exact
    log.info.epochs = tl.epochs.length;
    log.info.nonExact = tl.epochs.filter(e => e.status !== 'exact').map(e => ({ i0: e.i0, i1: e.i1, status: e.status, reasons: e.reasons, uncertain: e.uncertain, mixed: e.mixed, pending: e.pending, unknown: e.unknown }));
    log.info.uncertain = tl.uncertain.slice(0, 20);
    log.info.mixedFrames = [...tl.mixed];
    if (oldApp) compareOld(li, F, log);
}

function compareOld(li, F, log) {
    try {
        const ol = new oldApp.FlightLog(bytes);
        if (ol.getLogError(li) || !ol.openLog(li)) { fail(log, 'old', `old viewer: ${ol.getLogError(li)}`); return; }
        const G = framesOf(ol);
        if (JSON.stringify(Array.from(ol.getMainFieldNames())) !== JSON.stringify(Array.from(flog.getMainFieldNames()))) fail(log, 'oldFields', 'Field names or order differ');
        const os = ol.getStats(li);
        const res = { frames: G.iter.length, corrupt: os.totalCorruptFrames, event101: G.events.filter(e => e.event === 101).length, events: G.events.length };
        let diff = 0;
        const n = Math.min(F.rows.length, G.rows.length);
        for (let i = 0; i < n; i++) { const a = F.rows[i], b = G.rows[i]; for (let j = 0; j < Math.min(a.length, b.length); j++) if (a[j] !== b[j]) { diff++; break; } }
        res.framesDiffer = diff;
        const s1 = F.events.filter(e => e.event === 101).map(e => e.data.string).join('\n'), s2 = G.events.filter(e => e.event === 101).map(e => e.data.string).join('\n');
        res.same101 = s1 === s2;
        res.unknownHeaders = (ol.getSysConfig().unknownHeaders || []).length;
        log.old = res;
        if (res.frames !== F.iter.length || diff || res.corrupt || !res.same101) fail(log, 'old', JSON.stringify(res));
    } catch (e) { fail(log, 'old', 'old viewer threw ' + e.message); }
}

for (const log of summary.logs) {
    console.log(`== log ${log.index}: ${log.info.mode} frames ${log.info.frames} corrupt ${log.info.corrupt} records ${log.info.records} writes ${log.info.writesInLog}+${log.info.writesBefore}`);
    console.log(JSON.stringify({ info: log.info, writes: log.writes, physics: log.physics, ticks: log.ticks, old: log.old }, null, 1));
    console.log('FAIL:', Object.keys(log.fail).length ? JSON.stringify(log.fail, null, 1) : 'none');
}
const out = opt('--json');
if (out) fs.writeFileSync(out, JSON.stringify(summary, null, 1));

if (!summary.logs.length) { console.error('No logs decoded'); process.exitCode = 1; }
if (summary.logs.some(log => Object.keys(log.fail).length || log.error)) process.exitCode = 1;
