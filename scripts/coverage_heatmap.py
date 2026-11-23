#!/usr/bin/env python3
"""Draw a coverage heatmap from the observation manager's JSONL log.

Each decision contributes the person's position (target) and whether the
manager could see it.  Cells are coloured by the fraction of decisions where
the person was visible at that spot, so red patches are the blind spots of the
chosen camera/observation strategy.

    python3 scripts/coverage_heatmap.py tmp/sim_logs --map tmp/pf_map.json \
        --out tmp/coverage.png

Only the standard library is used (pure-Python PNG writer), so it runs in the
container as well as on the host.
"""

import argparse
import glob
import json
import math
import os
import struct
import zlib


def latest_jsonl(path):
    if os.path.isfile(path):
        return path
    files = sorted(glob.glob(os.path.join(path, '**', 'observe-*.jsonl'),
                             recursive=True), key=os.path.getmtime)
    if not files:
        raise SystemExit('no observe-*.jsonl under {}'.format(path))
    return files[-1]


def load_trace(path):
    samples = []
    if not path or not os.path.isfile(path):
        return samples
    with open(path) as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            try:
                row = json.loads(line)
            except json.JSONDecodeError:
                continue
            if 't' in row and 'x' in row:
                samples.append((row['t'], row['x'], row['y']))
    samples.sort()
    return samples


def nearest_trace(samples, stamp, tolerance=0.5):
    if not samples:
        return None
    import bisect
    keys = [s[0] for s in samples]
    index = bisect.bisect_left(keys, stamp)
    best = None
    for candidate in (index - 1, index):
        if 0 <= candidate < len(samples):
            delta = abs(samples[candidate][0] - stamp)
            if delta <= tolerance and (best is None or delta < best[0]):
                best = (delta, samples[candidate])
    return None if best is None else (best[1][1], best[1][2])


def load_decisions(path, trace=None):
    """Each decision keeps its visibility plus a best-effort position.

    When the manager lost the person it has no usable target, so the ground
    truth trace (if available) supplies the position instead; otherwise the
    stale estimate is used as a rough proxy.
    """
    decisions = []
    with open(path) as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            try:
                row = json.loads(line)
            except json.JSONDecodeError:
                continue
            if row.get('type') != 'decision':
                continue
            target = row.get('target')
            position = None
            if target:
                stale = target.get('stale', 0.0)
                extrapolated = target.get('extrapolated')
                position = (target['x'], target['y'])
                if trace and (stale > 0.5 or extrapolated):
                    position = nearest_trace(trace, row['t']) or position
            if position is None and trace:
                position = nearest_trace(trace, row['t'])
            if position is None:
                continue
            row['_pos'] = position
            decisions.append(row)
    return decisions


def load_obstacles(path):
    if not path or not os.path.isfile(path):
        return None
    with open(path) as handle:
        data = json.load(handle)
    return {
        'resolution': float(data['resolution']),
        'origin_x': float(data['origin_x']),
        'origin_y': float(data['origin_y']),
        'width': int(data['width']),
        'height': int(data['height']),
        'data': data['data'],
    }


def cell_colour(ratio):
    """red -> yellow -> green for ratio 0 -> 0.5 -> 1."""
    if ratio <= 0.5:
        t = ratio / 0.5
        return (220, int(60 + 170 * t), 60)
    t = (ratio - 0.5) / 0.5
    return (int(220 - 160 * t), int(230 - 60 * t), 60)


def write_png(path, pixels, width, height):
    raw = bytearray()
    for y in range(height):
        raw.append(0)
        raw.extend(pixels[y * width * 3:(y + 1) * width * 3])

    def chunk(tag, payload):
        return (struct.pack('>I', len(payload)) + tag + payload +
                struct.pack('>I', zlib.crc32(tag + payload) & 0xffffffff))

    png = b'\x89PNG\r\n\x1a\n'
    png += chunk(b'IHDR', struct.pack('>IIBBBBB', width, height, 8, 2, 0, 0, 0))
    png += chunk(b'IDAT', zlib.compress(bytes(raw), 6))
    png += chunk(b'IEND', b'')
    with open(path, 'wb') as handle:
        handle.write(png)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('log', help='session dir or observe-*.jsonl')
    parser.add_argument('--map', default='tmp/pf_map.json',
                        help='occupancy map json (obstacles overlay)')
    parser.add_argument('--out', default='tmp/coverage.png')
    parser.add_argument('--cell', type=float, default=0.25,
                        help='heatmap cell size in metres')
    parser.add_argument('--px', type=int, default=14,
                        help='pixels per cell')
    parser.add_argument('--min-samples', type=int, default=2)
    parser.add_argument('--trace', default='',
                        help='person trace jsonl from dev_person_trace.py')
    args = parser.parse_args()

    path = latest_jsonl(args.log)
    trace = load_trace(args.trace)
    decisions = load_decisions(path, trace)
    if not decisions:
        raise SystemExit('no usable decisions in {}'.format(path))

    xs = [d['_pos'][0] for d in decisions]
    ys = [d['_pos'][1] for d in decisions]
    cell = args.cell
    min_x, max_x = min(xs) - cell, max(xs) + cell
    min_y, max_y = min(ys) - cell, max(ys) + cell
    cols = max(1, int(math.ceil((max_x - min_x) / cell)))
    rows = max(1, int(math.ceil((max_y - min_y) / cell)))

    total = {}
    visible = {}
    blocked = {}
    robot_cells = set()
    for decision in decisions:
        cx = int((decision['_pos'][0] - min_x) / cell)
        cy = int((decision['_pos'][1] - min_y) / cell)
        key = (cy, cx)
        total[key] = total.get(key, 0) + 1
        if decision.get('visible'):
            visible[key] = visible.get(key, 0) + 1
        else:
            reason = decision.get('blocked_by') or 'lost'
            blocked[reason] = blocked.get(reason, 0) + 1
        robot = decision.get('robot') or {}
        if 'x' in robot:
            rx = int((robot['x'] - min_x) / cell)
            ry = int((robot['y'] - min_y) / cell)
            robot_cells.add((ry, rx))

    obstacles = load_obstacles(args.map)

    def is_obstacle(x, y):
        if not obstacles:
            return False
        ix = int((x - obstacles['origin_x']) / obstacles['resolution'])
        iy = int((y - obstacles['origin_y']) / obstacles['resolution'])
        if not (0 <= ix < obstacles['width'] and 0 <= iy < obstacles['height']):
            return False
        return obstacles['data'][iy * obstacles['width'] + ix] >= 50

    scale = args.px
    width, height = cols * scale, rows * scale
    background = (46, 49, 56)
    pixels = [background[0], background[1], background[2]] * (width * height)

    sampled = []
    for key, count in total.items():
        if count < args.min_samples:
            continue
        ratio = visible.get(key, 0) / float(count)
        sampled.append((key, ratio, count))

    for (ry, cx), ratio, _count in sampled:
        colour = cell_colour(ratio)
        for py in range(ry * scale, (ry + 1) * scale):
            base = py * width * 3
            for px in range(cx * scale, (cx + 1) * scale):
                offset = base + px * 3
                pixels[offset:offset + 3] = list(colour)

    # obstacle overlay from the static map
    if obstacles:
        res = obstacles['resolution']
        for iy in range(obstacles['height']):
            for ix in range(obstacles['width']):
                if obstacles['data'][iy * obstacles['width'] + ix] < 50:
                    continue
                x = obstacles['origin_x'] + (ix + 0.5) * res
                y = obstacles['origin_y'] + (iy + 0.5) * res
                cx = int((x - min_x) / cell)
                cy = int((y - min_y) / cell)
                if not (0 <= cx < cols and 0 <= cy < rows):
                    continue
                if total.get((cy, cx)):
                    continue
                for py in range(cy * scale, (cy + 1) * scale):
                    base = py * width * 3
                    for px in range(cx * scale, (cx + 1) * scale):
                        offset = base + px * 3
                        pixels[offset:offset + 3] = [10, 10, 12]

    for (ry, cx) in robot_cells:
        if not (0 <= ry < rows and 0 <= cx < cols):
            continue
        for py in range(max(0, ry * scale - 1), min(height, ry * scale + 2)):
            base = py * width * 3
            for px in range(max(0, cx * scale - 1), min(width, cx * scale + 2)):
                offset = base + px * 3
                pixels[offset:offset + 3] = [80, 160, 255]

    # colour bar: red (never visible) -> green (always visible), plus a legend
    bar_height = 18
    height += bar_height
    pixels.extend([background[0], background[1], background[2]]
                  * (width * bar_height))
    for x in range(width):
        colour = cell_colour(x / float(max(1, width - 1)))
        for row in range(bar_height - 6, bar_height - 2):
            offset = ((height - bar_height + row) * width + x) * 3
            pixels[offset:offset + 3] = list(colour)
    for x in range(0, width, 2):
        for row in range(bar_height - 12, bar_height - 9):
            offset = ((height - bar_height + row) * width + x) * 3
            pixels[offset:offset + 3] = [200, 200, 200]

    os.makedirs(os.path.dirname(args.out) or '.', exist_ok=True)
    write_png(args.out, pixels, width, height)

    good = sum(1 for _key, ratio, _n in sampled if ratio >= 0.8)
    poor = sum(1 for _key, ratio, _n in sampled if ratio <= 0.2)
    print('log: {}  decisions: {}'.format(path, len(decisions)))
    print('cells sampled: {}  always visible (>=80%): {}  blind (<=20%): {}'.format(
        len(sampled), good, poor))
    print('overall visibility: {:.0%}'.format(
        sum(visible.values()) / float(sum(total.values()))))
    if blocked:
        print('lost reasons:', ', '.join(
            '{} {}'.format(k, v)
            for k, v in sorted(blocked.items(), key=lambda kv: -kv[1])[:5]))
    print('heatmap: {} ({}x{} px, cell {:.2f} m)'.format(
        args.out, width, height, cell))


if __name__ == '__main__':
    main()
