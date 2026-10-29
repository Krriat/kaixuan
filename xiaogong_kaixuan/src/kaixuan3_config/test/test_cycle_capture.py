import importlib.util
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from cycle_capture import TriggeredCapture, encode, decode, WIDTH, HEADER, AXIS


def batch(first, count=50, fault=False, master=0):
    data = [1, master, WIDTH]
    for i in range(first, first + count):
        stamp = 10_000_000_000 + i * 1_000_000
        frame = [0] * WIDTH
        frame[:len(HEADER)] = [i, stamp, stamp, 0, 1_000_000, 21, 2, 0, 5000, 0, 8, 7, 1, 0]
        for axis in range(7):
            start = len(HEADER) + axis * len(AXIS)
            frame[start:start + len(AXIS)] = [-33849852, -33848495, 0, -232, 0x27, 15, 0, 0, 8]
        if fault and i == first + 20:
            start = len(HEADER) + len(AXIS)
            frame[start + 4:start + 8] = [0x08, 0x80, 400, 7]
        if fault and i == first + 21:
            start = len(HEADER) + len(AXIS)
            frame[start + 7] = 7
        data.extend(frame)
    return data


class CaptureTests(unittest.TestCase):
    def test_normal_traffic_is_memory_only(self):
        events = []
        capture = TriggeredCapture(lambda *e: events.append(e))
        for start in range(0, 5000, 50):
            capture.receive(batch(start))
        self.assertEqual(events, [])
        self.assertLessEqual(len(capture.history[0]), 42)

    def test_single_cycle_fault_pre_post_and_roundtrip(self):
        events = []
        capture = TriggeredCapture(lambda *e: events.append(e))
        for start in range(0, 14500, 50):
            capture.receive(batch(start, fault=start == 3000))
        transitions = [e[2] for e in events if e[1] == 'drive_transition']
        self.assertEqual(len(transitions), 2)
        self.assertEqual(transitions[0]['joint'], 'right_joint2')
        self.assertEqual(transitions[0]['values']['error_code'], 400)
        self.assertEqual(transitions[0]['values']['control_word'], 128)
        self.assertEqual(transitions[1]['values']['error_code'], 0)
        self.assertEqual(transitions[1]['previous']['error_code'], 400)
        packets = [e[2] for e in events if e[1] == 'cycle_block']
        self.assertLessEqual(packets[0]['first_cycle'], 1050)
        self.assertGreaterEqual(packets[0]['first_cycle'], 950)
        self.assertGreaterEqual(packets[-1]['last_cycle'], 13000)
        self.assertLess(packets[-1]['last_cycle'], 13100)
        self.assertEqual(len({p['first_cycle'] for p in packets}), len(packets))
        fault = next(p for p in packets if p['first_cycle'] == 3000)
        self.assertEqual(list(decode(fault)), batch(3000, fault=True)[3:])
        self.assertLess(len(fault['payload']), 10000)

    def test_dropped_batches_and_invalid_input(self):
        events = []
        capture = TriggeredCapture(lambda *e: events.append(e))
        capture.receive(batch(0))
        capture.receive(batch(100))
        self.assertEqual(capture.gaps, 50)
        self.assertTrue(any(e[1] == 'cycle_gap' for e in events))
        with self.assertRaises(ValueError):
            capture.receive([1, 0, WIDTH, 1])
        with self.assertRaises(ValueError):
            capture.receive([9, 0, WIDTH])

    def test_both_arms_saved_on_one_arm_trigger(self):
        events = []
        capture = TriggeredCapture(lambda *e: events.append(e))
        for start in range(0, 3000, 50):
            capture.receive(batch(start, master=0))
            capture.receive(batch(start, master=1))
        capture.receive(batch(3000, fault=True))
        self.assertEqual({e[2]['master'] for e in events if e[1] == 'cycle_block'}, {0, 1})


if __name__ == '__main__':
    unittest.main(verbosity=2)
