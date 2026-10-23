#!/usr/bin/env python3
#
# Static audit of the parameter log (blackbox_params), RF-PARAM-1 section 6.2. Run it on every firmware update.
#
#  1. Every write to a tracked parameter group outside init() must be inside an operation
#     (blackboxParamsOpBegin/End, the MSP bracket) or boot-only, or in the CLI (the CLI stops the log).
#     A write is a call of <group>Mutable(), an assignment through currentPidProfile-> or
#     currentControlRateProfile->, or a use of <group>_System / _SystemArray outside src/main/pg.
#  2. The loader regions (blackbox_params_tables.c) cover the tracked groups that each loader and its callees read.
#  3. pgRegistry_t.copy (the shadow) is used only in cli.c and the blackbox_params files.
#  4. The MSP allowlist covers reachable tracked writes and explicitly justified indirect mutations only.
#
# The call graph comes from the text of src/main. Of each #if group the first branch counts; write sites in
# other branches are listed for a manual check. Static inline functions in headers are not followed. Functions that are
# called through pointers are given their dispatchers in POINTER_DISPATCH. A write that the audit cannot place is
# reported as GAP: the journal scan finds such writes as 'u' records with an interval.
#
# usage: python3 src/utils/blackbox_params_audit.py [src/main]      exit status 1 when there is a gap

import os
import re
import sys
from collections import defaultdict, deque

ROOT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'main')
ROOT = os.path.normpath(ROOT)

KEYWORDS = {'if', 'for', 'while', 'switch', 'return', 'sizeof', 'case', 'do', 'else', 'defined', 'ARRAYLEN',
            'offsetof', 'UNUSED', 'MIN', 'MAX', 'BIT', 'typeof', '__typeof__', 'STATIC_ASSERT', 'constrain',
            'constrainf'}

# Functions whose body (between the begin and the end call) is an operation
BRACKET_BEGIN = re.compile(r'\bblackboxParams(OpBegin|MspBegin)\s*\(')
BRACKET_END = re.compile(r'\bblackboxParamsOpEnd\s*\(')

# Roots that are not operations
BOOT_ROOTS = {'init', 'main', 'systemInit'}
CLI_FILES = {'cli/cli.c'}

# Functions called through pointers, and the functions that call them. 'op:<why>' marks a dispatcher that is
# inside an operation; 'boot' a dispatcher that runs at boot only.
POINTER_DISPATCH = [
    (re.compile(r'^set_ADJUSTMENT_'), ['op:processRcAdjustments cfgSet']),
    (re.compile(r'^pgResetFn_'), ['pgResetInstance']),
    (re.compile(r'^task[A-Z]'), ['scheduler']),
    (re.compile(r'Fn$|^msp.*PostProcess'), ['mspPostProcess']),
    (re.compile(r'^dispatch'), ['dispatchProcess']),
]

# Write sites that are not writes, or writes that the journal records in another way. Each needs a reason.
KNOWN = {
    'taskUpdateAccelerometer': 'READ: passes the trims to accUpdate() by pointer, which reads them',
    'cmsx_initPidProfile': 'READ: keeps a pointer for the CMS menu entries, which write in cmsHandleKey (an operation)',
    'cmsx_initRateProfile': 'READ: the profile name for the CMS text',
    'blackboxValidateConfig': 'LOGSTART: blackboxStart() fixes the device before blackboxParamsStart(); the change is a v record',
    'accResetRollAndPitchTrims': 'UNUSED: no caller',
}

# The bodies of these are known: their calls do not need text analysis
KNOWN_ROOT_KINDS = {
    'scheduler': 'task',
    'mspPostProcess': 'msp post-process (after the operation)',
    'dispatchProcess': 'dispatch task',
}


def strip_comments_and_strings(text):
    out = []
    i = 0
    n = len(text)
    while i < n:
        c = text[i]
        if text.startswith('//', i):
            j = text.find('\n', i)
            i = n if j < 0 else j
        elif text.startswith('/*', i):
            j = text.find('*/', i + 2)
            seg = text[i:(n if j < 0 else j + 2)]
            out.append('\n' * seg.count('\n'))
            i = n if j < 0 else j + 2
        elif c == '"' or c == "'":
            q = c
            j = i + 1
            while j < n and text[j] != q:
                j += 2 if text[j] == '\\' else 1
            out.append(q + q)
            i = j + 1
        else:
            out.append(c)
            i += 1
    return ''.join(out)


def strip_preprocessor(text):
    # Directives go. Of each #if group only the first branch stays, so that the braces stay balanced; the
    # write sites in other branches are reported as not analysed.
    lines = text.split('\n')
    out = []
    cont = False
    stack = []          # for each open #if: the current branch is kept
    for line in lines:
        stripped = line.lstrip()
        if cont or stripped.startswith('#'):
            directive = re.match(r'#\s*(\w+)', stripped) if not cont else None
            cont = line.rstrip().endswith('\\')
            if directive:
                d = directive.group(1)
                if d in ('if', 'ifdef', 'ifndef'):
                    stack.append(True)
                elif d in ('elif', 'else') and stack:
                    stack[-1] = False
                elif d == 'endif' and stack:
                    stack.pop()
            out.append('')
        elif all(stack):
            out.append(line)
        else:
            out.append('')
    return '\n'.join(out)


FUNC_HEAD = re.compile(r'([A-Za-z_]\w*)\s*\(([^;{}()]|\([^;{}()]*\))*\)\s*(__attribute__\s*\(\(.*?\)\)\s*)?$', re.S)


class Function:
    def __init__(self, name, path, line, body, body_line):
        self.name = name
        self.path = path
        self.line = line
        self.body = body
        self.body_line = body_line


def parse_functions(path, rel):
    raw = open(path, encoding='utf-8', errors='replace').read()
    text = strip_preprocessor(strip_comments_and_strings(raw))
    funcs = []
    depth = 0
    i = 0
    start = 0
    n = len(text)
    while i < n:
        c = text[i]
        if c == '{':
            if depth == 0:
                head = text[start:i]
                m = FUNC_HEAD.search(head.rstrip())
                if m and '=' not in head[head.rfind(';') + 1:] and not re.search(r'\b(struct|enum|union)\s*\w*\s*$', head):
                    name = m.group(1)
                    if name not in KEYWORDS:
                        # body until the matching brace
                        d = 0
                        j = i
                        while j < n:
                            if text[j] == '{':
                                d += 1
                            elif text[j] == '}':
                                d -= 1
                                if d == 0:
                                    break
                            j += 1
                        body = text[i:j + 1]
                        funcs.append(Function(name, rel, text.count('\n', 0, start + m.start()) + 1, body,
                                              text.count('\n', 0, i) + 1))
                        i = j + 1
                        start = i
                        continue
            depth += 1
        elif c == '}':
            depth -= 1
            if depth == 0:
                start = i + 1
        elif c == ';' and depth == 0:
            start = i + 1
        i += 1
    return funcs, text


CALL = re.compile(r'\b([A-Za-z_]\w*)\s*\(')
IDENT = re.compile(r'\b([A-Za-z_]\w*)\b')


def main():
    files = []
    for d, _, names in os.walk(ROOT):
        for name in names:
            if name.endswith('.c'):
                p = os.path.join(d, name)
                rel = os.path.relpath(p, ROOT)
                if rel.startswith('target/') and not rel.startswith('target/STM32_UNIFIED'):
                    continue
                files.append((p, rel))

    functions = defaultdict(list)       # name -> [Function]
    texts = {}
    for p, rel in files:
        funcs, text = parse_functions(p, rel)
        texts[rel] = text
        for f in funcs:
            functions[f.name].append(f)

    # Tracked groups and loader regions from the tables
    tables = open(os.path.join(ROOT, 'blackbox', 'blackbox_params_tables.c')).read()
    tracked = re.findall(r'X\((PG_\w+),\s*(\w+?)_System(?:Array)?,', tables)
    accessor_of = {pg: acc for pg, acc in tracked}
    pg_of_accessor = {acc: pg for pg, acc in tracked}
    regions = {}
    for name, body in re.findall(r'static const pgn_t (region\w+)\[\] = \{(.*?)\};', tables, re.S):
        regions[name] = set(re.findall(r'PG_\w+', body))

    # Edges: caller -> callee, with 'in operation' when the call is between the begin and the end of a bracket
    callers = defaultdict(set)          # callee -> {(caller, inop)}
    referenced = defaultdict(set)       # function name used without a call -> {file}
    names = set(functions)
    for name, defs in functions.items():
        for f in defs:
            body = f.body
            begin = BRACKET_BEGIN.search(body)
            ends = list(BRACKET_END.finditer(body))
            lo = begin.end() if begin else -1
            hi = ends[-1].start() if (begin and ends) else -1
            for m in CALL.finditer(body):
                callee = m.group(1)
                if callee in names and callee != name and callee not in KEYWORDS:
                    callers[callee].add((name, lo <= m.start() < hi))
            for m in IDENT.finditer(body):
                ident = m.group(1)
                after = body[m.end():m.end() + 2].lstrip()
                if ident in names and not after.startswith('('):
                    referenced[ident].add(f.path)
    # References at file level (tables of function pointers)
    for rel, text in texts.items():
        for m in IDENT.finditer(text):
            ident = m.group(1)
            if ident in names and not text[m.end():m.end() + 3].lstrip().startswith('('):
                referenced[ident].add(rel)

    # CMS menus: the slot of each function gives its dispatcher
    cms_slot = {}
    for rel, text in texts.items():
        if rel.startswith('cms/') or rel.startswith('io/'):
            for slot, fn in re.findall(r'\.(onEnter|onExit|onDisplayUpdate)\s*=\s*(\w+)', text):
                cms_slot[fn] = slot

    def slot_dispatchers(slot):
        """The functions that call a CMS menu slot, each with 'in operation' for the call sites"""
        out = []
        for d in {'onEnter': ['cmsMenuChange'], 'onExit': ['cmsMenuBack', 'cmsMenuExit'],
                  'onDisplayUpdate': ['cmsDrawMenu']}[slot]:
            for f in functions.get(d, []):
                begin = BRACKET_BEGIN.search(f.body)
                ends = list(BRACKET_END.finditer(f.body))
                calls = [m.start() for m in re.finditer(r'\b' + slot + r'\s*\(', f.body)]
                inop = bool(begin and ends and calls and all(begin.end() <= c < ends[-1].start() for c in calls))
                out.append(('op:' + d + ' ' + slot) if inop else d)
        return out

    def dispatchers(fname):
        if fname in cms_slot:
            return slot_dispatchers(cms_slot[fname])
        defs = functions.get(fname, [])
        if any(d.path.startswith('cms/') for d in defs) and fname in referenced:
            return ['cmsHandleKeyInner']        # menu entry functions and their change callbacks
        for pattern, targets in POINTER_DISPATCH:
            if pattern.search(fname):
                return targets
        return None

    def classify(fname):
        """All root paths of fname: a set of (kind, path) where kind is OP, BOOT, CLI or GAP."""
        results = set()
        seen = set()
        queue = deque([(fname, (fname,))])
        while queue:
            cur, path = queue.popleft()
            if cur in seen:
                continue
            seen.add(cur)
            defs = functions.get(cur, [])
            if any(d.path in CLI_FILES for d in defs):
                results.add(('CLI', ' <- '.join(path)))
                continue
            if cur in BOOT_ROOTS:
                results.add(('BOOT', ' <- '.join(path)))
                continue
            if cur in KNOWN_ROOT_KINDS:
                results.add(('GAP', ' <- '.join(path) + ' [' + KNOWN_ROOT_KINDS[cur] + ']'))
                continue
            ups = callers.get(cur, set())
            for caller, inop in ups:
                if inop:
                    results.add(('OP', ' <- '.join(path + (caller,))))
                else:
                    queue.append((caller, path + (caller,)))
            if cur in referenced or not ups:
                targets = dispatchers(cur)
                if targets is None:
                    if cur in referenced:
                        results.add(('GAP', ' <- '.join(path) + ' [called through a pointer from ' +
                                     ','.join(sorted(referenced[cur])) + ']'))
                    elif not ups:
                        results.add(('GAP', ' <- '.join(path) + ' [no caller]'))
                else:
                    for t in targets:
                        if t.startswith('op:'):
                            results.add(('OP', ' <- '.join(path) + ' [' + t[3:] + ']'))
                        else:
                            queue.append((t, path + (t,)))
        return results

    # 1. Writes
    acc_alt = '|'.join(sorted(map(re.escape, pg_of_accessor), key=len, reverse=True))
    write_patterns = [
        ('Mutable', re.compile(r'\b(' + acc_alt + r')Mutable\s*\(')),
        ('currentPidProfile', re.compile(r'\bcurrentPidProfile\s*->[\w.\[\]\s]*?([-+*/|&^]?=(?!=)|\+\+|--)')),
        ('currentControlRateProfile', re.compile(r'\bcurrentControlRateProfile\s*->[\w.\[\]\s]*?([-+*/|&^]?=(?!=)|\+\+|--)')),
        ('System', re.compile(r'\b(' + acc_alt + r')_System(Array)?\b')),
    ]
    sites = defaultdict(list)       # function -> [(path, line, kind, group)]
    inside = defaultdict(list)      # function -> write sites inside its own operation
    analysed = set()
    for name, defs in functions.items():
        for f in defs:
            if f.path.startswith('pg/') or f.path.startswith('blackbox/blackbox_params'):
                continue
            begin = BRACKET_BEGIN.search(f.body)
            ends = list(BRACKET_END.finditer(f.body))
            for kind, pat in write_patterns:
                for m in pat.finditer(f.body):
                    group = pg_of_accessor.get(m.group(1), '') if kind in ('Mutable', 'System') else \
                        ('PG_PID_PROFILE' if kind == 'currentPidProfile' else 'PG_CONTROL_RATE_PROFILES')
                    line = f.body_line + f.body.count('\n', 0, m.start())
                    analysed.add((f.path, line))
                    if begin and ends and begin.end() <= m.start() < ends[-1].start():
                        inside[name].append((f.path, line, kind, group))
                    else:
                        sites[name].append((f.path, line, kind, group))

    gaps = 0
    rows = []
    for name in sorted(sites):
        if name in KNOWN:
            groups = sorted({g.replace('PG_', '') for _, _, _, g in sites[name]})
            where = sorted({'%s:%d' % (p, l) for p, l, _, _ in sites[name]})
            rows.append((KNOWN[name].split(':')[0], name, where, groups, {('NOTE', KNOWN[name])}))
            continue
        status = classify(name)
        kinds = sorted({k for k, _ in status})
        verdict = 'GAP' if 'GAP' in kinds else '+'.join(kinds) if kinds else 'GAP'
        if verdict == 'GAP' or 'GAP' in kinds:
            gaps += 1
        groups = sorted({g.replace('PG_', '') for _, _, _, g in sites[name]})
        where = sorted({'%s:%d' % (p, l) for p, l, _, _ in sites[name]})
        rows.append((verdict, name, where, groups, status))

    for name in sorted(inside):
        if name not in sites:
            groups = sorted({g.replace('PG_', '') for _, _, _, g in inside[name]})
            where = sorted({'%s:%d' % (p, l) for p, l, _, _ in inside[name]})
            rows.append(('OP', name, where, groups, {('OP', name + ' (its own operation)')}))

    print('== Writes to tracked groups outside src/main/pg (%d functions)' % len(rows))
    for verdict, name, where, groups, status in sorted(rows, key=lambda r: (r[0] != 'GAP', r[1])):
        print('%-12s %-44s %s  [%s]' % (verdict, name, ', '.join(groups), where[0] + (' +%d' % (len(where) - 1) if len(where) > 1 else '')))
        for kind, path in sorted(status):
            if kind == 'GAP' or os.environ.get('AUDIT_VERBOSE'):
                print('             %s: %s' % (kind, path))

    # Write sites that the parser did not place in a function (other #if branches, macros)
    print('\n== Write sites that the audit did not analyse (other #if branches): read them')
    for rel, raw_path in ((r, os.path.join(ROOT, r)) for r in sorted(texts)):
        if rel.startswith('pg/') or rel.startswith('blackbox/blackbox_params') or rel in CLI_FILES:
            continue
        raw = strip_comments_and_strings(open(raw_path, encoding='utf-8', errors='replace').read())
        for kind, pat in write_patterns:
            for m in pat.finditer(raw):
                line = raw.count('\n', 0, m.start()) + 1
                if (rel, line) not in analysed:
                    print('  %s:%d %s' % (rel, line, m.group(0).strip()))

    # 2. Loader regions
    loaders = {
        'pidLoadProfile': 'regionPid', 'governorInitProfile': 'regionGovernor', 'rescueInitProfile': 'regionRescue',
        'setpointInitProfile': 'regionSetpoint', 'gyroInitFilters': 'regionGyroFilter',
        'rpmFilterInit': 'regionRpmFilter', 'mixerInitConfig': 'regionMixer', 'rcControlsInit': 'regionRcControls',
        'featureInit': 'regionFeature',
    }
    other_loaders = set(loaders) | {'activateConfig'}
    print('\n== Loader regions: tracked groups that a loader and its callees read')
    region_gaps = 0
    read_pat = re.compile(r'\b(' + acc_alt + r')(Mutable)?\s*\(')
    for loader, region in loaders.items():
        if loader not in functions:
            print('%-22s not found' % loader)
            continue
        reads = set()
        queue = deque([(loader, 0)])
        seen = set()
        while queue:
            cur, depth = queue.popleft()
            if cur in seen or depth > 4:
                continue
            seen.add(cur)
            for f in functions.get(cur, []):
                body = f.body
                reads |= {pg_of_accessor[m.group(1)] for m in read_pat.finditer(body)}
                if re.search(r'\bcurrentPidProfile\b|\bpidProfile\s*->', body):
                    reads.add('PG_PID_PROFILE')
                if re.search(r'\bcurrentControlRateProfile\b', body):
                    reads.add('PG_CONTROL_RATE_PROFILES')
                for m in CALL.finditer(body):
                    callee = m.group(1)
                    if callee in functions and callee not in other_loaders and not callee.startswith('blackboxParams'):
                        queue.append((callee, depth + 1))
        missing = sorted(g for g in reads if g not in regions.get(region, set()) and g in accessor_of)
        print('%-22s region %-18s %-8s reads %s' % (loader, region, 'OK' if not missing else 'MISSING',
                                                   ', '.join(sorted(g.replace('PG_', '') for g in reads))))
        region_gaps += bool(missing)

    # 3. The shadow
    print('\n== pgRegistry_t.copy users')
    copy_users = []
    for d, _, fnames in os.walk(ROOT):
        for fname in fnames:
            if fname.endswith('.c') or fname.endswith('.h'):
                rel = os.path.relpath(os.path.join(d, fname), ROOT)
                text = strip_comments_and_strings(open(os.path.join(d, fname), encoding='utf-8', errors='replace').read())
                for m in re.finditer(r'->\s*copy\b', text):
                    copy_users.append('%s:%d' % (rel, text.count('\n', 0, m.start()) + 1))
    copy_users.sort()
    bad = [u for u in copy_users if not (u.startswith('cli/cli.c') or u.startswith('blackbox/blackbox_params'))]
    for u in copy_users:
        print('  ' + u)
    print('only cli.c and blackbox_params: %s' % ('yes' if not bad else 'NO: ' + ', '.join(bad)))

    # 4. The MSP skip list
    print('\n== MSP tracked-write allowlist and reachable writers')
    skip_bad = check_skip_list(functions, write_patterns)

    print('\nsummary: %d writer functions, %d with a gap; %d loader regions with missing groups; copy %s; MSP allowlist %s' %
          (len(rows), gaps, region_gaps, 'ok' if not bad else 'misused', 'ok' if not skip_bad else 'differs'))
    return 1 if (gaps or region_gaps or bad or skip_bad) else 0


def check_skip_list(functions, write_patterns):
    """Check the declared allowlist and every MSP case with reachable tracked writes."""
    code = strip_comments_and_strings(open(os.path.join(ROOT, 'blackbox', 'blackbox_params.c')).read())
    m = re.search(r'bool blackboxParamsMspBegin\(.*?\{(.*?)\nstatic uint32_t runtimeValue', code, re.S)
    if not m:
        print('blackboxParamsMspBegin() not found')
        return True
    allowed = set(re.findall(r'case (MSP\w+):', m.group(1)))
    indirect = {
        'MSP_ACC_CALIBRATION',       # starts asynchronous calibration writes
        'MSP_SET_ESC_PARAMETERS',    # external ESC write, recorded by its hash marker
        'MSP_SET_RESET_CURR_PID',    # PG reset callback, not a direct Mutable() call
    }

    def writes_tracked(body, seen=None):
        seen = set() if seen is None else seen
        if any(pat.search(body) for _, pat in write_patterns):
            return True
        for call in CALL.finditer(body):
            name = call.group(1)
            if name in seen or name.startswith('blackboxParams') or name in KNOWN:
                continue
            seen.add(name)
            for function in functions.get(name, []):
                if writes_tracked(function.body, seen):
                    return True
        return False

    required = set()
    for name in ['mspProcessInCommand', 'mspCommonProcessInCommand', 'mspFcProcessOutCommandWithArg']:
        for function in functions.get(name, []):
            labels = list(re.finditer(r'case (MSP\w+):', function.body))
            for i, label in enumerate(labels):
                end = labels[i + 1].start() if i + 1 < len(labels) else len(function.body)
                body = function.body[label.end():end]
                body = re.split(r'\n    default:', body)[0]
                if label.group(1) == 'MSP_MULTIPLE_MSP' and re.search(r'\bmspFcProcessCommand\s*\(', body):
                    # Each nested command goes through the same checked dispatcher.
                    continue
                if writes_tracked(body):
                    required.add(label.group(1))
    missing = required - allowed
    forbidden = {'MSP_SET_RAW_RC', 'MSP_DATAFLASH_READ', 'MSP_SET_PASSTHROUGH', 'MSP_MULTIPLE_MSP',
                 'MSP_REBOOT', 'MSP_BOXNAMES', 'MSP_BOXIDS', 'MSP_GET_SERVO_CONFIG', 'MSP_GET_MIXER_INPUT',
                 'MSP_GET_ADJUSTMENT_RANGE', 'MSP_RPM_FILTER_V2', 'MSP2_SEND_DSHOT_COMMAND'}
    for name in sorted(required | allowed):
        print('  %-32s %s' % (name, 'MISSING BRACKET' if name in missing else 'OK'))
    unexpected = allowed - required - indirect
    if unexpected:
        print('unjustified MSP brackets:', sorted(unexpected))
    missing |= indirect - allowed
    if missing:
        print('missing MSP brackets:', sorted(missing))
    if allowed & forbidden:
        print('read or passthrough commands instrumented:', sorted(allowed & forbidden))
    return bool(missing or unexpected or allowed & forbidden)


if __name__ == '__main__':
    sys.exit(main())
