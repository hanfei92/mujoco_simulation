#!/usr/bin/env python3
"""Summarise an observation-manager run from its JSONL log (or a session dir).

Usage:
    python3 scripts/summarize_pf_session.py tmp/sim_sessions/<tag>
    python3 scripts/summarize_pf_session.py run_a run_b     # side by side
"""

import glob
import json
import os
import sys


def load(path):
    if os.path.isdir(path):
        files = sorted(glob.glob(os.path.join(path, '**', 'observe-*.jsonl'),
                                 recursive=True),
                       key=os.path.getmtime)
        if not files:
            files = sorted(glob.glob(os.path.join(path, '*.jsonl')),
                           key=os.path.getmtime)
        if not files:
            raise SystemExit('no observe-*.jsonl under {}'.format(path))
        path = files[-1]
    rows = []
    with open(path) as handle:
        for line in handle:
            line = line.strip()
            if line:
                try:
                    rows.append(json.loads(line))
                except json.JSONDecodeError:
                    pass
    return path, rows


def summarise(rows):
    decisions = [r for r in rows if r.get('type') == 'decision']
    metrics = [r for r in rows if r.get('type') == 'metrics']
    last = metrics[-1] if metrics else {}
    out = {}
    out['decisions'] = len(decisions)
    if decisions:
        out['duration'] = decisions[-1]['t'] - decisions[0]['t']
        out['visible_ratio'] = (sum(1 for d in decisions if d.get('visible'))
                                / float(len(decisions)))
    else:
        out['duration'] = 0.0
        out['visible_ratio'] = 0.0
    out['state_share'] = last.get('state_share') or {}
    out['actions'] = last.get('actions') or {}
    out['blocked'] = last.get('blocked') or {}
    out['moves'] = last.get('moves', 0)
    out['gain'] = last.get('gain') or {}
    out['ratios'] = last.get('ratios') or {}
    out['mean_terms'] = last.get('mean_current_terms') or {}

    transitions = []
    previous = None
    for row in decisions:
        key = (row.get('state'), row.get('action'))
        if key != previous:
            transitions.append((round(row['t'], 1), key[0], key[1],
                                (row.get('reason') or '')[:48],
                                row.get('blocked_by') or ''))
            previous = key
    out['transitions'] = transitions
    return out


def fmt_pct(value):
    return '{:.0f}%'.format(100.0 * float(value))


def report(name, data):
    print('== {} =='.format(name))
    print('decisions {}  duration {:.1f}s  visible {}'.format(
        data['decisions'], data['duration'], fmt_pct(data['visible_ratio'])))
    print('states   {}'.format(', '.join(
        '{} {}'.format(k, fmt_pct(v))
        for k, v in sorted(data['state_share'].items(),
                           key=lambda kv: -kv[1]))))
    print('actions  {}'.format(', '.join(
        '{} {}'.format(k, v) for k, v in sorted(data['actions'].items(),
                                                key=lambda kv: -kv[1]))))
    print('blocked  {}'.format(', '.join(
        '{} {}'.format(k, v) for k, v in sorted(data['blocked'].items(),
                                                key=lambda kv: -kv[1])[:6])))
    print('moves {}  gain mean={:.2f} max={:.2f}'.format(
        data['moves'], data['gain'].get('mean', 0.0),
        data['gain'].get('max', 0.0)))
    if data['ratios']:
        print('ratios   {}'.format(', '.join(
            '{} {}'.format(k, fmt_pct(v))
            for k, v in data['ratios'].items())))
    if data['mean_terms']:
        print('terms    {}'.format(', '.join(
            '{} {:.2f}'.format(k, v) for k, v in data['mean_terms'].items())))
    print('-- transitions ({} total) --'.format(len(data['transitions'])))
    for stamp, state, action, reason, blocked in data['transitions'][:20]:
        print('  {:>9.1f}  {:>11} {:>6}  {}{}'.format(
            stamp, state, action, reason,
            '  [{}]'.format(blocked) if blocked else ''))
    if len(data['transitions']) > 20:
        print('  ... {} more'.format(len(data['transitions']) - 20))
    print()


def compare(name_a, a, name_b, b):
    print('== {} vs {} =='.format(name_a, name_b))
    keys = [('decisions', 'decisions'), ('duration', 'duration'),
            ('moves', 'moves'), ('visible_ratio', 'visible_ratio'),
            ('gain', 'gain.mean')]
    print('{:<16}{:>14}{:>14}'.format('metric', name_a[:14], name_b[:14]))
    for label, key in keys:
        va = a
        vb = b
        for part in key.split('.'):
            va = va.get(part, 0.0) if isinstance(va, dict) else 0.0
            vb = vb.get(part, 0.0) if isinstance(vb, dict) else 0.0
        if isinstance(va, float) and label in ('duration', 'visible_ratio'):
            print('{:<16}{:>14.2f}{:>14.2f}'.format(label, va, vb))
        elif isinstance(va, float):
            print('{:<16}{:>14.2f}{:>14.2f}'.format(label, va, vb))
        else:
            print('{:<16}{:>14}{:>14}'.format(label, va, vb))
    for state in sorted(set(a['state_share']) | set(b['state_share'])):
        print('{:<16}{:>13.0f}%{:>13.0f}%'.format(
            'state ' + state,
            100 * a['state_share'].get(state, 0.0),
            100 * b['state_share'].get(state, 0.0)))
    print()


def main():
    args = sys.argv[1:]
    if not args:
        print(__doc__)
        return 2
    if len(args) == 1:
        path, rows = load(args[0])
        report(os.path.basename(os.path.dirname(path)) or path,
               summarise(rows))
        return 0
    path_a, rows_a = load(args[0])
    path_b, rows_b = load(args[1])
    compare(os.path.basename(args[0].rstrip('/')),
            summarise(rows_a), os.path.basename(args[1].rstrip('/')),
            summarise(rows_b))
    return 0


if __name__ == '__main__':
    sys.exit(main())
