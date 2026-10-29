#!/usr/bin/env python3
"""Triggered cycle storage and offline decoder. Wire schema matches cycle_capture.hpp."""
from array import array
import base64
from collections import deque
import csv
import json
import sys
import zlib

HEADER = ["cycle", "sample_steady_ns", "scheduled_steady_ns", "lateness_ns", "interval_ns",
          "working_counter", "wc_state", "previous_send_result", "previous_send_duration_ns",
          "dropped_frames", "al_states", "slaves_responding", "link_up", "master_state_result"]
AXIS = ["actual_counts", "target_counts", "velocity_raw", "torque_raw", "status_word",
        "control_word", "error_code", "transition_flags", "mode_command"]
WIDTH = len(HEADER) + 7 * len(AXIS)


def encode(values):
    data = array('q', values)
    if sys.byteorder != 'little':
        data.byteswap()
    return base64.b64encode(zlib.compress(data.tobytes(), level=1)).decode('ascii')


def decode(packet):
    if packet['schema'] != 1 or packet['width'] != WIDTH or packet['codec'] != 'zlib-base64-i64le':
        raise ValueError('Unsupported cycle schema')
    raw = zlib.decompress(base64.b64decode(packet['payload']))
    data = array('q')
    data.frombytes(raw)
    if sys.byteorder != 'little':
        data.byteswap()
    if len(data) != packet['frames'] * WIDTH:
        raise ValueError('Cycle packet length mismatch')
    return data


class TriggeredCapture:
    def __init__(self, emit, pre_seconds=2, post_seconds=10):
        self.emit = emit
        self.pre_ns = int(pre_seconds * 1e9)
        self.post_ns = int(post_seconds * 1e9)
        self.history = {0: deque(maxlen=256), 1: deque(maxlen=256)}
        self.previous = {}
        self.written = {}
        self.until_ns = 0
        self.gaps = 0
        self.ring_drops = {0: 0, 1: 0}

    def store(self, master, packet):
        if packet['last_cycle'] > self.written.get(master, -1):
            self.emit('waveforms', 'cycle_block', packet)
            self.written[master] = packet['last_cycle']

    def receive(self, wire):
        if len(wire) < 3 or tuple(wire[:1]) != (1,) or wire[1] not in (0, 1) or wire[2] != WIDTH:
            raise ValueError('Invalid cycle capture header')
        master = int(wire[1])
        data = wire[3:]
        if not data or len(data) % WIDTH or len(data) > WIDTH * 256:
            raise ValueError('Invalid cycle capture batch size')
        trigger = False
        for offset in range(0, len(data), WIDTH):
            frame = data[offset:offset + WIDTH]
            cycle, stamp = int(frame[0]), int(frame[1])
            previous = self.previous.get(master)
            if previous is not None and (cycle <= previous[0] or stamp <= previous[1]):
                # Reconfigure/restarted publisher: do not mix two incarnations.
                self.emit('events', 'cycle_stream_reset', {'master': master, 'cycle': cycle})
                self.history[master].clear()
                self.written.pop(master, None)
                previous = None
                trigger = True
            if previous is not None and cycle != previous[0] + 1:
                self.gaps += max(0, cycle - previous[0] - 1)
                self.emit('events', 'cycle_gap', {'master': master, 'previous_cycle': previous[0], 'cycle': cycle,
                                                 'sample_steady_ns': stamp, 'ring_dropped': int(frame[9])})
                trigger = True
            if int(frame[9]) != self.ring_drops[master]:
                self.emit('events', 'cycle_ring_drops', {'master': master, 'total': int(frame[9]), 'sample_steady_ns': stamp})
                self.ring_drops[master] = int(frame[9])
                trigger = True
            # Bus change or late cycle is captured alongside drive transitions.
            bus_fields = (5, 6, 7, 10, 11, 12, 13)
            bus_changed = previous is not None and any(frame[k] != previous[k] for k in bus_fields)
            late = frame[3] > 500_000 or (previous is not None and frame[4] >
                    max(500_000, frame[2] - previous[2]) * 1.5)
            if bus_changed or (late and (previous is None or previous[3] <= 500_000)):
                self.emit('events', 'cycle_bus_timing_event', {'master': master, **dict(zip(HEADER, map(int, frame[:len(HEADER)])))})
                trigger = True
            for axis in range(7):
                start = len(HEADER) + axis * len(AXIS)
                values = frame[start:start + len(AXIS)]
                if values[7]:
                    name = ('right' if master == 0 else 'left') + f'_joint{axis + 1}'
                    old = previous[start:start + len(AXIS)] if previous is not None else None
                    self.emit('events', 'drive_transition', {
                        'master': master, 'slave': axis, 'joint': name, 'cycle': cycle,
                        'sample_steady_ns': stamp, 'values': dict(zip(AXIS, map(int, values))),
                        'previous': dict(zip(AXIS, map(int, old))) if old is not None else None,
                    })
                    trigger = True
            self.previous[master] = frame

        packet = {'schema': 1, 'master': master, 'width': WIDTH, 'frames': len(data) // WIDTH,
                  'first_cycle': int(data[0]), 'last_cycle': int(data[-WIDTH]),
                  'first_sample_steady_ns': int(data[1]), 'last_sample_steady_ns': int(data[-WIDTH + 1]),
                  'codec': 'zlib-base64-i64le', 'payload': encode(data)}
        now_ns = packet['last_sample_steady_ns']
        history = self.history[master]
        history.append(packet)
        while history and history[0]['last_sample_steady_ns'] < now_ns - self.pre_ns:
            history.popleft()
        if trigger:
            if now_ns > self.until_ns:
                self.emit('events', 'waveform_trigger', {'master': master, 'sample_steady_ns': now_ns,
                          'pre_seconds': self.pre_ns / 1e9, 'post_seconds': self.post_ns / 1e9})
            self.until_ns = max(self.until_ns, now_ns + self.post_ns)
            for other, blocks in self.history.items():
                for block in blocks:
                    if block['last_sample_steady_ns'] >= now_ns - self.pre_ns:
                        self.store(other, block)
        if now_ns <= self.until_ns:
            self.store(master, packet)


def main():
    import argparse
    parser = argparse.ArgumentParser(description='Decode waveform JSONL to CSV without ROS or hardware access')
    parser.add_argument('files', nargs='+')
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    count = 0
    with open(args.output, 'x', newline='') as output:
        writer = csv.writer(output)
        writer.writerow(['master'] + HEADER + [f'axis{a + 1}_{field}' for a in range(7) for field in AXIS])
        for filename in sorted(args.files):
            with open(filename) as handle:
                for number, line in enumerate(handle, 1):
                    try:
                        record = json.loads(line)
                        if record.get('kind') != 'cycle_block':
                            continue
                        packet = record['data']
                        data = decode(packet)
                        for offset in range(0, len(data), WIDTH):
                            writer.writerow([packet['master']] + list(data[offset:offset + WIDTH]))
                            count += 1
                    except (ValueError, KeyError, zlib.error) as exc:
                        print(f'Skipping damaged record {filename}:{number}: {exc}', file=sys.stderr)
    print(f'Decoded {count} cycles -> {args.output}')


if __name__ == '__main__':
    main()
