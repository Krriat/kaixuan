"""Disk-consumer stalls must never block the active controller thread."""
from pathlib import Path
import signal
import subprocess
import os
import sys
import tempfile
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"scripts"))
from cst_csv_log import AsyncCsvLog


class CsvLogTests(unittest.TestCase):
    def test_event_worker_mirrors_stdout_and_records_exact_text(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"events.log"
            scripts = str(Path(__file__).resolve().parents[1]/"scripts")
            code = ("import sys; sys.path.insert(0,sys.argv[1]); "
                    "from cst_csv_log import AsyncCsvLog; "
                    "log=AsyncCsvLog(sys.argv[2],capacity=4096,mirror_stdout=True); "
                    "log.write('PHASE move\\nSTOP\\n'); log.close()")
            result = subprocess.run([sys.executable, "-c", code, scripts, str(path)],
                                    capture_output=True, text=True, timeout=5., check=True)
            self.assertEqual(result.stdout, "PHASE move\nSTOP\n")
            self.assertEqual(path.read_text(), result.stdout)

    def test_paused_consumer_does_not_block_and_preserves_wrapped_data(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"samples.csv"
            log = AsyncCsvLog(path, capacity=65536, send_buffer=4096)
            data = "1,2,3,停\n"*4000
            try:
                os.kill(log.process.pid, signal.SIGSTOP)
                start = time.monotonic()
                log.write(data)
                self.assertLess(time.monotonic()-start, 1.)
                self.assertGreater(log.buffered_bytes, 0)
                os.kill(log.process.pid, signal.SIGCONT)
                deadline = time.monotonic()+3.
                while log.buffered_bytes and time.monotonic()<deadline:
                    log.flush(); time.sleep(.001)
                self.assertEqual(log.buffered_bytes, 0)
                log.write(data)  # Wraps around the prefaulted ring.
            finally:
                os.kill(log.process.pid, signal.SIGCONT)
                log.close()
            self.assertEqual(path.read_text(), data*2)

    def test_bounded_backlog_rejects_overflow_without_corrupting_previous_rows(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"samples.csv"
            log = AsyncCsvLog(path, capacity=4096, send_buffer=4096)
            written = ""
            try:
                os.kill(log.process.pid, signal.SIGSTOP)
                for _ in range(100):
                    row = "1,"*512+"\n"
                    try:
                        log.write(row)
                    except RuntimeError as error:
                        self.assertIn("bounded memory", str(error))
                        break
                    written += row
                else:
                    self.fail("A stopped consumer must eventually reach the bounded backlog")
            finally:
                os.kill(log.process.pid, signal.SIGCONT)
                log.close()
            self.assertEqual(path.read_text(), written)

    def test_consumer_failure_is_reported_to_stop_path(self):
        with tempfile.TemporaryDirectory() as directory:
            log = AsyncCsvLog(Path(directory)/"samples.csv", capacity=4096)
            log.process.kill(); log.process.wait()
            with self.assertRaises(RuntimeError):
                log.write("1,2,3\n")
            with self.assertRaises(RuntimeError):
                log.close()


if __name__ == "__main__":
    unittest.main()
