#!/usr/bin/env python3
"""Count ZMQ messages per topic for a few seconds (bridge debugging).

Usage:
    python3 dev_zmq_sniff.py [endpoint] [duration]
"""

import collections
import sys
import time

import zmq

import common_pb2


def main():
    endpoint = sys.argv[1] if len(sys.argv) > 1 else 'tcp://127.0.0.1:5555'
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0

    context = zmq.Context()
    sub = context.socket(zmq.SUB)
    sub.connect(endpoint)
    sub.setsockopt(zmq.SUBSCRIBE, b'')
    sub.setsockopt(zmq.RCVTIMEO, 500)

    counts = collections.Counter()
    sizes = {}
    start = time.time()
    while time.time() - start < duration:
        try:
            raw = sub.recv()
        except zmq.Again:
            continue
        envelope = common_pb2.ZmqEnvelope()
        try:
            envelope.ParseFromString(raw)
        except Exception:
            continue
        counts[envelope.topic] += 1
        sizes[envelope.topic] = len(envelope.payload)

    if not counts:
        print('no messages on {} within {:.0f}s'.format(endpoint, duration))
        return 1
    for topic, count in counts.most_common():
        print('{:12s} {:4d} msgs  {:5.1f} Hz  payload {} B'.format(
            topic, count, count / duration, sizes[topic]))
    return 0


if __name__ == '__main__':
    sys.exit(main())
