'use strict';

// pointNow() stamps the next Blackbox iteration: the operation lies between
// the last captured earlier iteration and the first captured equal/later one.
// Unlogged iterations and host stalls leave an interval, not an exact timestamp.
module.exports = function iterationTimeBounds(iterations, times, iteration) {
    let lo = 0, hi = iterations.length;
    while (lo < hi) {
        const mid = (lo + hi) >> 1;
        if (iterations[mid] < iteration) lo = mid + 1;
        else hi = mid;
    }
    return [lo > 0 ? times[lo - 1] : -Infinity, lo < times.length ? times[lo] : Infinity];
};
