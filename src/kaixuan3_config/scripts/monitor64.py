"""Bounded background monitor-selector transactions; never commands motion."""
import subprocess
import socket
import threading
import time


def ethercat(master, slave, *args):
    result = subprocess.run(
        ['ethercat', '-m', str(master), '-p', str(slave), *args],
        capture_output=True, timeout=0.5, start_new_session=True, check=False)
    if result.returncode:
        raise RuntimeError(f'ethercat exit {result.returncode}: ' +
                           result.stderr.decode('utf-8', errors='replace')[:2000])
    return result.stdout


def sample(master, slave, emit, run=ethercat, wait=time.sleep):
    """Restore even after an ambiguous download failure. No persistent save."""
    def read(index, size):
        raw = run(master, slave, 'upload', '-t', 'octet_string', index, '0')
        if len(raw) != size:
            raise ValueError(f'{index}: expected {size} bytes, got {len(raw)}')
        return raw, int.from_bytes(raw, 'little', signed=True)

    original = None
    attempted = False
    ok = True
    value = None
    restore_safe = True
    try:
        _, original = read('0x2002', 2)
        emit('selector_before', {'selector': original})
        if original != 64:
            attempted = True
            run(master, slave, 'download', '-t', 'int16', '0x2002', '0', '64')
        if read('0x2002', 2)[1] != 64:
            raise ValueError('selector readback is not 64')
        # Allow settling, but do NOT claim this undocumented delay proves freshness.
        if attempted:
            wait(0.1)
        read_started = time.monotonic_ns()
        raw, value = read('0x3000', 4)
        read_completed = time.monotonic_ns()
        if read('0x2002', 2)[1] != 64:
            raise ValueError('selector changed during sample')
        emit('counter_raw', {'selector': 64, 'raw_hex': raw.hex(), 'value': value,
                            'datatype': 'int32', 'index': '0x3000:00',
                            'read_started_monotonic_ns': read_started,
                            'read_completed_monotonic_ns': read_completed,
                            'selector_settle_s': 0.1 if attempted else 0,
                            'drive_refresh_latency_verified': False})
    except Exception as exc:
        ok = False
        value = None
        emit('read_failed', {'error': str(exc)})
    finally:
        if attempted:
            try:
                run(master, slave, 'download', '-t', 'int16', '0x2002', '0', str(original))
                if read('0x2002', 2)[1] != original:
                    raise ValueError('restore readback mismatch')
                emit('selector_restored', {'selector': original})
            except Exception as exc:
                ok = False
                restore_safe = False
                emit('restore_failed', {'selector': original, 'error': str(exc),
                                       'action': 'all monitor64 sampling disabled; inspect selector manually'})
    return ok, value, restore_safe


class Monitor64:
    def __init__(self, emit):
        self.emit = emit
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.ready = {}
        self.priority = {}
        self.previous = {}
        self.cursor = 0
        self.failures = {}
        self.retry_at = {}
        self.last_attempt = {}
        self.state = {}
        self.blocked = None
        self.started = time.monotonic()
        self.thread = threading.Thread(target=self.work, name='drive-monitor64', daemon=True)
        self.thread.start()

    def observe(self, wire):
        # Only called after the cycle decoder has validated this packet.
        master = int(wire[1])
        frame = wire[-77:]
        healthy = frame[6] == 2 and frame[10] == 8 and frame[11] == 7 and frame[12] == 1
        with self.lock:
            if healthy:
                self.ready[master] = time.monotonic()
            else:
                self.ready.pop(master, None)

    def trigger(self, data):
        values = data['values']
        if values['transition_flags'] & 8 or values['error_code']:
            with self.lock:
                key = (data['master'], data['slave'])
                new = self.priority.get(key, 0) < time.monotonic()
                self.priority[key] = time.monotonic() + 30
                previous = self.previous.get(key)
                # A trigger can shorten backoff, but never causes a retry burst.
                self.retry_at[key] = min(self.retry_at.get(key, 0), time.monotonic()+1)
            if new:
                self.emit('monitor64', 'counter_trigger', {
                    'master': key[0], 'slave': key[1],
                    'sample_steady_ns': data.get('sample_steady_ns'),
                    'previous_value': previous[0] if previous else None,
                    'previous_request_monotonic_ns': previous[1] if previous else None,
                    'baseline_available': previous is not None,
                    'observation_seconds': 30})

    def health(self):
        now = time.monotonic()
        with self.lock:
            return {'worker_alive': self.thread.is_alive(), 'blocked': self.blocked,
                    'age_s': now-self.started,
                    'joints': {f'{m}:{s}': {**v, 'sample_age_s':
                        (time.monotonic_ns()-self.previous[(m,s)][1])/1e9
                        if (m,s) in self.previous else None}
                        for (m,s),v in self.state.items()},
                    'triggered': [f'{m}:{s}' for (m,s), end in self.priority.items() if end>now]}

    def work(self):
        # Linux abstract socket: prevent two recorder instances racing selectors.
        with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as guard:
            try:
                guard.bind('\0kaixuan-monitor64-selector-owner')
            except OSError as exc:
                self.blocked = 'selector ownership unavailable'
                self.emit('monitor64', 'monitor64_disabled', {'reason': 'selector ownership unavailable',
                                                            'error': str(exc)})
                return
            try:
                self.poll()
            except Exception as exc:
                self.blocked = f'worker exception: {exc}'
                self.emit('monitor64', 'monitor64_disabled', {'reason': self.blocked})

    def poll(self):
        if self.stop.wait(5):
            return
        while not self.stop.is_set():
            now = time.monotonic()
            with self.lock:
                eligible = {m for m, stamp in self.ready.items() if now - stamp < 1}
                self.priority = {key: end for key, end in self.priority.items() if end > now}
                keys = [key for key in self.priority if key[0] in eligible
                        and now >= self.retry_at.get(key, 0)]
                keys.sort(key=lambda k: self.last_attempt.get(k, 0))
                advance_cursor = False
                if not keys:
                    advance_cursor = True
                    keys = [(n // 7, n % 7) for n in
                            ((self.cursor + i) % 14 for i in range(14)) if n // 7 in eligible
                            and now >= self.retry_at.get((n//7,n%7), 0)]
                    # Frequently affected axes retain a recent baseline.
                    urgent = [k for k in [(0,0),(0,1)] if k in keys
                              and now-self.last_attempt.get(k, 0)>=5]
                    if urgent:
                        advance_cursor = False
                        keys = sorted(urgent, key=lambda k: self.last_attempt.get(k, 0)) + keys
                key = keys[0] if keys else None
                triggered = key in self.priority
                if key in self.priority:
                    self.priority[key] = self.priority.pop(key)  # rotate triggered axes
            if key is not None:
                master, slave = key
                if advance_cursor:
                    self.cursor = (master * 7 + slave + 1) % 14
                started = time.monotonic_ns()
                with self.lock:
                    self.last_attempt[key] = time.monotonic()
                context = {'master': master, 'slave': slave,
                           'joint': f"{'right' if master == 0 else 'left'}_joint{slave + 1}",
                           'request_monotonic_ns': started,
                           'reason': 'trigger' if triggered else 'baseline'}
                raw_metadata = {}
                def emit(kind, data):
                    if kind == 'counter_raw':
                        raw_metadata.update(data)
                    self.emit('monitor64', kind, {**context, **data})
                ok, value, restore_safe = sample(master, slave, emit)
                if value is not None:
                    previous = self.previous.get(key)
                    valid = value >= 0 and previous is not None and 0 <= previous[0] <= value
                    emit('counter_sample', {'value': value,
                         'previous_value': previous[0] if previous else None,
                         'previous_request_monotonic_ns': previous[1] if previous else None,
                         'delta': value - previous[0] if valid else None,
                         'growth_observed_in_sample_interval': value > previous[0] if valid else None,
                         'read_started_monotonic_ns': raw_metadata.get('read_started_monotonic_ns'),
                         'read_completed_monotonic_ns': raw_metadata.get('read_completed_monotonic_ns'),
                         'interpretation': 'interval evidence, not exact loss time or proof of causality',
                         'discontinuity': value < 0 or (previous is not None and value < previous[0]),
                         'restore_ok': ok})
                    with self.lock:
                        self.previous[key] = (value, started)
                with self.lock:
                    self.failures[key] = 0 if ok else self.failures.get(key, 0)+1
                    delay = 0 if ok else min(10, 2**min(self.failures[key]-1, 4))
                    self.retry_at[key] = time.monotonic()+delay
                    self.state[key] = {'status': 'sampling' if ok else 'retry_pending',
                                       'consecutive_failures': self.failures[key],
                                       'retry_delay_s': delay, 'restore_safe': restore_safe}
                if not ok:
                    if not restore_safe:
                        self.blocked = f'master{master}/slave{slave}: selector restore not confirmed'
                        emit('monitor64_disabled', {'reason': self.blocked})
                        return
                    emit('monitor64_retry', {'retry_delay_s': delay,
                         'reason': 'read failed, selector safe; other joints remain enabled'})
            if self.stop.wait(1):
                return

    def close(self):
        self.stop.set()
        self.thread.join(timeout=4.5)
        if self.thread.is_alive():
            self.emit('monitor64', 'shutdown_incomplete', {'selector_restore_not_confirmed': True})
