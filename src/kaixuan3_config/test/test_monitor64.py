import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from monitor64 import sample, Monitor64
import threading
import time
from unittest.mock import Mock, patch


class MonitorTests(unittest.TestCase):
    def execute(self, original=14, fail=None):
        state = [original]
        calls, events = [], []
        def run(master, slave, *args):
            self.assertEqual((master, slave), (0, 1))
            calls.append(args)
            op, _, datatype, index, sub, *value = args
            self.assertEqual(sub, '0')
            if op == 'download':
                self.assertEqual((datatype, index), ('int16', '0x2002'))
                state[0] = int(value[0])
                if fail == 'write' and state[0] == 64:
                    raise TimeoutError('ambiguous write')
                if fail == 'restore' and state[0] == original:
                    raise TimeoutError('restore timeout')
                return b''
            if index == '0x2002':
                return state[0].to_bytes(2, 'little', signed=True)
            self.assertEqual(index, '0x3000')
            if fail == 'read':
                raise TimeoutError('read timeout')
            if fail == 'size':
                return b'\x00\x00'
            return (727).to_bytes(4, 'little', signed=True)
        result = sample(0, 1, lambda kind, data: events.append((kind, data)), run, wait=lambda _: None)
        return result, state[0], calls, events

    def test_success_restores(self):
        result, selector, calls, events = self.execute()
        self.assertEqual(result, (True, 727, True))
        self.assertEqual(selector, 14)
        self.assertEqual(next(data['raw_hex'] for kind, data in events if kind == 'counter_raw'), 'd7020000')

    def test_existing_64_never_writes(self):
        result, selector, calls, _ = self.execute(original=64)
        self.assertEqual(result, (True, 727, True))
        self.assertFalse(any(args[0] == 'download' for args in calls))

    def test_failures_restore_and_do_not_invent_zero(self):
        for fail in ('write', 'read', 'size'):
            result, selector, _, events = self.execute(fail=fail)
            self.assertEqual(result, (False, None, True))
            self.assertEqual(selector, 14)
            self.assertTrue(any(kind == 'selector_restored' for kind, _ in events))

    def test_restore_failure_disables(self):
        result, _, _, events = self.execute(fail='restore')
        self.assertFalse(result[0])
        self.assertFalse(result[2])
        self.assertTrue(any(kind == 'restore_failed' for kind, _ in events))

    def worker(self, fresh=True):
        worker = Monitor64.__new__(Monitor64)
        worker.emit = Mock()
        worker.stop = Mock()
        worker.stop.is_set.return_value = False
        worker.stop.wait.side_effect = [False, True]
        worker.lock = threading.Lock()
        worker.ready = {0: time.monotonic() if fresh else 0}
        worker.priority = {}
        worker.previous = {(0, 1): (727, 1)}
        worker.cursor = 0
        worker.failures = {}
        worker.retry_at = {}
        worker.last_attempt = {(0,0): time.monotonic(), (0,1): time.monotonic()}
        worker.state = {}
        worker.blocked = None
        return worker

    def test_trigger_priority_and_delta(self):
        worker = self.worker()
        worker.trigger({'master': 0, 'slave': 1,
                        'values': {'transition_flags': 8, 'error_code': 0}})
        with patch('monitor64.sample', return_value=(True, 730, True)) as request:
            worker.poll()
        self.assertEqual(request.call_args.args[:2], (0, 1))
        data = worker.emit.call_args.args[2]
        self.assertEqual(data['delta'], 3)
        self.assertEqual(data['reason'], 'trigger')

    def test_stale_feedback_never_starts_transaction(self):
        worker = self.worker(fresh=False)
        with patch('monitor64.sample') as request:
            worker.poll()
        request.assert_not_called()

    def test_decrease_not_negative_delta(self):
        worker = self.worker()
        worker.cursor = 1
        with patch('monitor64.sample', return_value=(True, 2, True)):
            worker.poll()
        data = worker.emit.call_args.args[2]
        self.assertIsNone(data['delta'])
        self.assertTrue(data['discontinuity'])

    def test_failed_left3_does_not_stop_right2_trigger(self):
        worker = self.worker()
        worker.ready[1] = time.monotonic()
        worker.cursor = 9  # left3, matching the actual incident
        worker.stop.wait.side_effect = [False, False, True]
        calls = []
        def request(m, s, emit):
            calls.append((m,s))
            if len(calls)==1:
                worker.trigger({'master':0,'slave':1,
                    'sample_steady_ns':123, 'values':{'transition_flags':8,'error_code':0}})
                return False, None, True
            return True, 730, True
        with patch('monitor64.sample', side_effect=request):
            worker.poll()
        self.assertEqual(calls, [(1,2),(0,1)])
        self.assertEqual(worker.previous[(0,1)][0],730)
        self.assertEqual(worker.state[(1,2)]['status'], 'retry_pending')
        self.assertIsNone(worker.blocked)

    def test_failed_joint_retried_after_backoff(self):
        worker = self.worker()
        worker.cursor = 1
        worker.stop.wait.side_effect = [False, False, True]
        calls=[]
        def request(m,s,emit):
            calls.append((m,s))
            if len(calls)==1:
                return False,None,True
            return True,729,True
        def wait(seconds):
            if len(calls)==1:
                worker.retry_at[(0,1)]=0
                worker.cursor=1
            return len(calls)>=2
        worker.stop.wait.side_effect=wait
        with patch('monitor64.sample', side_effect=request):
            worker.poll()
        self.assertEqual(calls,[(0,1),(0,1)])
        self.assertEqual(worker.failures[(0,1)],0)

    def test_restore_unknown_stops_further_writes(self):
        worker=self.worker()
        with patch('monitor64.sample', return_value=(False,None,False)) as request:
            worker.poll()
        self.assertEqual(request.call_count,1)
        self.assertIsNotNone(worker.blocked)

    def test_trigger_records_previous_baseline(self):
        worker=self.worker()
        worker.trigger({'master':0,'slave':1,'sample_steady_ns':456,
                        'values':{'transition_flags':8,'error_code':0}})
        data=worker.emit.call_args.args[2]
        self.assertEqual(data['previous_value'],727)
        self.assertEqual(data['sample_steady_ns'],456)

    def test_two_hour_run_survives_left_failure_and_later_right_trigger(self):
        worker=self.worker()
        clock=[1000.0]
        failed=[False]
        fired=[False]
        successes=[]
        worker.last_attempt={}
        worker.previous={}
        def wait(seconds):
            clock[0]+=seconds
            worker.ready={0:clock[0],1:clock[0]}
            if clock[0]>9500 and not fired[0]:
                fired[0]=True
                worker.trigger({'master':0,'slave':1,'sample_steady_ns':int(clock[0]*1e9),
                                'values':{'transition_flags':8,'error_code':0}})
            return clock[0]>9535
        def request(m,s,emit):
            if clock[0]>5200 and (m,s)==(1,2) and not failed[0]:
                failed[0]=True
                return False,None,True
            successes.append((clock[0],m,s))
            return True,int(clock[0]),True
        worker.stop.wait.side_effect=wait
        with patch('monitor64.time.monotonic', side_effect=lambda:clock[0]), \
             patch('monitor64.time.monotonic_ns', side_effect=lambda:int(clock[0]*1e9)), \
             patch('monitor64.sample', side_effect=request):
            worker.poll()
        self.assertTrue(failed[0])
        self.assertTrue(fired[0])
        self.assertTrue(any(t>9500 and (m,s)==(0,1) for t,m,s in successes))
        self.assertEqual(len({(m,s) for t,m,s in successes if 9400<t<9500}),14)
        rows=[c.args[2] for c in worker.emit.call_args_list if c.args[1]=='counter_sample'
              and c.args[2]['reason']=='trigger']
        self.assertTrue(any(row['growth_observed_in_sample_interval'] for row in rows))
        self.assertIsNone(worker.blocked)


if __name__ == '__main__':
    unittest.main()
