"""Bounded nonblocking CSV producer; disk I/O runs in a separate process.

The control process owns a prefaulted ring and only uses nonblocking socket
writes. A full ring or a failed consumer raises an error, so the existing
controller stop path remains responsible for stopping motion. close() drains
and waits, and must only be called after commands have been invalidated.
"""
import mmap
from pathlib import Path
import select
import socket
import subprocess
import sys
import time


class AsyncCsvLog:
    def __init__(self, path, capacity=64*1024*1024, send_buffer=None, mirror_stdout=False):
        if capacity < 1024:
            raise ValueError("CSV ring capacity must be at least 1024 bytes")
        self.capacity = capacity
        self.memory = mmap.mmap(-1, capacity)
        self.memory[:] = b"\0"*capacity  # Touch pages before any drive is enabled.
        self.view = memoryview(self.memory)
        self.read_cursor = self.write_cursor = self.high_water_bytes = 0
        self.closed = False
        self.socket, child = socket.socketpair()
        if send_buffer is not None:
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, send_buffer)
        self.process = subprocess.Popen(
            [sys.executable, str(Path(__file__).resolve()), "--consume", str(child.fileno()), str(path),
             "1" if mirror_stdout else "0"],
            pass_fds=(child.fileno(),), stdin=subprocess.DEVNULL,
            stdout=None if mirror_stdout else subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        child.close()
        self.socket.settimeout(10.)
        try:
            if self.socket.recv(5) != b"READY":
                raise RuntimeError("CSV consumer failed to open the output file")
        except BaseException:
            self.socket.close()
            self.process.kill()
            self.process.wait()
            self.view.release()
            self.memory.close()
            raise
        self.socket.setblocking(False)

    @property
    def buffered_bytes(self):
        return self.write_cursor-self.read_cursor

    def _drain_once(self):
        if self.closed or self.process.poll() is not None:
            raise RuntimeError("CSV consumer is no longer running")
        if not self.buffered_bytes:
            return
        start = self.read_cursor % self.capacity
        count = min(self.buffered_bytes, self.capacity-start, 65536)
        try:
            sent = self.socket.send(self.view[start:start+count])
        except BlockingIOError:
            return
        except OSError as error:
            raise RuntimeError("CSV consumer connection failed") from error
        if sent <= 0:
            raise RuntimeError("CSV consumer connection closed")
        self.read_cursor += sent

    def write(self, text):
        data = text.encode("utf-8")
        if len(data) > self.capacity-self.buffered_bytes:
            raise RuntimeError("CSV logging backlog exhausted its bounded memory ring")
        start = self.write_cursor % self.capacity
        first = min(len(data), self.capacity-start)
        self.memory[start:start+first] = data[:first]
        if first < len(data):
            self.memory[:len(data)-first] = data[first:]
        self.write_cursor += len(data)
        self.high_water_bytes = max(self.high_water_bytes, self.buffered_bytes)
        self._drain_once()
        return len(text)

    def flush(self):
        """Nonblocking during motion; close() performs the final disk drain."""
        self._drain_once()

    def close(self):
        if self.closed:
            return
        try:
            deadline = time.monotonic()+30.
            while self.buffered_bytes:
                self._drain_once()
                if time.monotonic() >= deadline:
                    raise RuntimeError("Timed out draining CSV after stopping the controller")
                if self.buffered_bytes:
                    select.select([], [self.socket], [], .05)
            self.socket.shutdown(socket.SHUT_WR)
            if self.process.wait(timeout=max(.1, deadline-time.monotonic())) != 0:
                raise RuntimeError("CSV consumer failed while writing the output file")
        finally:
            self.closed = True
            self.socket.close()
            if self.process.poll() is None:
                self.process.kill()
                self.process.wait()
            self.view.release()
            self.memory.close()


def consume(fd, path, mirror_stdout=False):
    connection = socket.socket(fileno=fd)
    with connection, open(path, "wb", buffering=0 if mirror_stdout else 1024*1024) as output:
        connection.sendall(b"READY")
        while True:
            data = connection.recv(65536)
            if not data:
                break
            output.write(data)
            if mirror_stdout:
                sys.stdout.buffer.write(data)
                sys.stdout.buffer.flush()


if __name__ == "__main__":
    if len(sys.argv) != 5 or sys.argv[1] != "--consume":
        raise SystemExit("Internal log consumer: --consume FD PATH MIRROR_STDOUT")
    consume(int(sys.argv[2]), sys.argv[3], sys.argv[4] == "1")
