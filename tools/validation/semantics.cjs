'use strict';

// Validation-only consumer rules reviewed against master a88bec8f8 (4.7.0).
// This deliberately covers the scenario keys, not the desktop application's full 4.7 support.
module.exports = function validationSemantics(PS, revision, fixes) {
    if (/^Rotorflight\s+4\.6\./.test(revision)) return PS.forFirmware(revision, fixes);
    if (!/^Rotorflight\s+4\.7\./.test(revision)) throw new Error('Unreviewed firmware semantics: ' + revision);
    if (fixes && fixes.some(f => f && f !== '-')) throw new Error('Unreviewed firmware behavior fixes');

    // Reuse the decoder's loader/slot machinery, replacing its consumer and header rules below.
    const table = PS.forFirmware('Rotorflight 4.6.0', []);
    table.revision = revision;
    table.firmware = 'Rotorflight 4.7 validation subset (master a88bec8f8)';
    table.keys = [
        { match: /^pid_profile$/, use: [{ by: 'live', consumer: 'pid' }] },
        { match: /^rate_profile$/, use: [{ by: 'live', consumer: 'sp' }] },
        // pidLoadProfile() copies gains and the new speed-dependent decay coefficient.
        { match: /^p[0-5]\.(?:(?:roll|pitch|yaw)_[pidfbo]_gain|error_decay_(?:time_cyclic|limit_cyclic|gain_cyclic))$/,
            use: [{ by: 'loader', loader: 'pid', consumer: 'pid' }] },
        // setpointInitProfile() caches this array; adjustment 69 still only changes its stored value.
        { match: /^r[0-5]\.setpoint_boost_gain$/, use: [{ by: 'loader', loader: 'sp', consumer: 'sp' }] },
        // governorInit() is boot-only; the MSP setter does not reload it.
        { match: /^gov_startup_time$/, use: [{ by: 'boot', consumer: 'pid' }] },
        { match: /^gyro_lpf1_static_hz$/, use: [{ by: 'loader', loader: 'gyrof', consumer: 'filter' }] },
        // servoUpdate() reads the midpoint live; the rate remains initialized at boot.
        { match: /^el\.servo\.\d+$/, use: [{ by: 'live', consumer: 'mot' }], fields: { 5: { by: 'boot' } } },
    ];
    table.classify = key => table.keys.find(rule => rule.match.test(String(key).replace(/\[\d+\]$/, ''))) || null;
    table.adjustments = Object.fromEntries([1, 2, 18, 69].map(id => [id, PS.ADJUSTMENTS_46[id]]));
    const adjustmentRefreshes = table.adjustmentRefreshes;
    table.adjustmentRefreshes = (id, key, itemKey) => !!table.adjustments[id] && adjustmentRefreshes(id, key, itemKey);
    const classicKeys = new Set(['rollPID', 'pitchPID', 'yawPID', 'looptime', 'pid_process_denom', 'filter_process_denom']);
    table.classic = PS.CLASSIC_46.filter(([name]) => classicKeys.has(name)).concat([
        ['error_decay', [{ p: 'error_decay_time_cyclic' }, { p: 'error_decay_limit_cyclic' }]],
    ]);
    return table;
};
