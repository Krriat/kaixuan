import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from bus_diagnostics import BusEvidence, decode, WIDTH


def packet(sequence=1, kind=0, index=0x0300, sub=0, raw=None, outcome=0, actual=None):
    raw = bytes(20) if raw is None else raw
    head = [sequence, sequence * 1_000_000_000, sequence * 1_000_000_000 + 10000000,
            1, kind, index, sub, len(raw), len(raw) if actual is None else actual,
            outcome, 2, 0, 0]
    return [1, 0, WIDTH] + head + list(raw) + [0] * (64 - len(raw))


class BusDiagnosticsTests(unittest.TestCase):
    def test_ros_int64_array_payload(self):
        from array import array
        wire = packet(index=0x092C, raw=(0x80000123).to_bytes(4, 'little'))
        self.assertEqual(decode(array('q', wire)), decode(wire))
        self.assertEqual(decode(array('q', wire))[0]['value'], -291)

    def test_counters_and_sign_magnitude(self):
        item = decode(packet(raw=bytes(range(20))))[0]
        self.assertEqual(item['joint'], 'right_joint2')
        self.assertEqual(item['value']['port3_lost_links'], 19)
        self.assertEqual(item['value']['port0_rx_errors'], 1)
        self.assertEqual(item['value']['ecat_processing_errors'], 12)
        for number, expected in [(0, 0), (123, 123), (0x8000007B, -123), (0x80000000, 0)]:
            item = decode(packet(index=0x092C, raw=number.to_bytes(4, 'little')))[0]
            self.assertEqual(item['value'], expected)

    def test_failed_read_never_becomes_zero(self):
        for code in (-1, -2, -3, -5):
            item = decode(packet(outcome=code, actual=65))[0]
            self.assertIsNone(item['value'])
            self.assertIsNone(item['raw_hex'])
        with self.assertRaises(ValueError):
            decode(packet(actual=65))
        with self.assertRaises(ValueError):
            decode(packet()[:-1])
        self.assertEqual(decode([1, 0, WIDTH]), [])  # heartbeat

    def test_counter_change_and_failed_read_preserves_baseline(self):
        events = []
        evidence = BusEvidence(lambda *row: events.append(row))
        evidence.receive(packet(raw=bytes([2]) + bytes(19)))
        evidence.receive(packet(sequence=2, outcome=-1, actual=0))
        evidence.receive(packet(sequence=4, raw=bytes([3]) + bytes(19)))
        change = [e[2] for e in events if e[1] == 'esc_counter_change'][0]
        self.assertEqual(change['counter_changes']['port0_invalid_frames']['before'], 2)
        self.assertEqual(change['counter_changes']['port0_invalid_frames']['after'], 3)
        self.assertEqual(change['elapsed_since_previous_success_s'], 3)
        self.assertEqual(evidence.gaps, 1)
        evidence.receive(packet(sequence=5))
        change = [e[2] for e in events if e[1] == 'esc_counter_change'][-1]
        self.assertEqual(change['counter_changes']['port0_invalid_frames']['direction'], 'decreased_reset_or_wrap')

    def test_parameters_and_optional_firmware(self):
        item = decode(packet(kind=1, index=0x2070, raw=b'\x07\x00'))[0]
        self.assertEqual(item['value'], 7)
        item = decode(packet(kind=1, index=0x100A, raw=b'v1.2\x00'))[0]
        self.assertEqual(item['value'], 'v1.2')
        events = []
        evidence = BusEvidence(lambda *row: events.append(row))
        evidence.receive(packet(kind=1, index=0x2070, raw=b'\x07\x00'))
        evidence.receive(packet(sequence=2, kind=1, index=0x2070, raw=b'\x08\x00'))
        self.assertTrue(any(e[1] == 'drive_object_change' and e[2]['previous_value'] == 7 for e in events))


if __name__ == '__main__':
    unittest.main()
