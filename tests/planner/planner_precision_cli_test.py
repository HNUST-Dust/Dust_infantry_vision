#!/usr/bin/env python3
"""CLI regression checks; use temporary files and never start robot hardware."""
from pathlib import Path
import importlib.util
import os
import signal
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest import mock

import numpy as np
import osqp

SCRIPT = Path(__file__).resolve().parents[2] / 'scripts/planner_precision.py'


class PrecisionCliTest(unittest.TestCase):
    def run_cli(self, directory, *arguments):
        return subprocess.run([sys.executable, str(SCRIPT), *arguments], cwd=directory,
                              text=True, capture_output=True)

    def test_missing_trace_does_not_create_output_or_show_traceback(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)/'result'
            result = self.run_cli(directory, 'logs/planner_trace/实际文件名.jsonl',
                                  '--output', str(output))
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('输入文件不存在', result.stderr)
            self.assertNotIn('Traceback', result.stderr)
            self.assertFalse(output.exists())

    def test_latest_without_capture_explains_next_step(self):
        with tempfile.TemporaryDirectory() as directory:
            result = self.run_cli(directory, '--latest')
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('未找到', result.stderr)
            self.assertIn('采集', result.stderr)
            self.assertNotIn('Traceback', result.stderr)
            self.assertFalse((Path(directory)/'logs/planner_precision').exists())

    def test_latest_selects_newest_capture_and_replay_failure_is_concise(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            traces = root/'logs/planner_trace'
            traces.mkdir(parents=True)
            older, newer = traces/'older.jsonl', traces/'newer.jsonl'
            older.write_text('{}\n')
            newer.write_text('{}\n')
            os.utime(older, ns=(1, 1))
            os.utime(newer, ns=(2, 2))
            replay = root/'fake_replay'
            replay.write_text('#!/usr/bin/env python3\nimport sys\n'
                              f'assert sys.argv[1] == {str(newer)!r}\n'
                              'print("EXPECTED_NEWEST_TRACE", file=sys.stderr)\n'
                              'sys.exit(7)\n')
            replay.chmod(0o755)
            result = self.run_cli(directory, '--latest', '--replay', str(replay))
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('EXPECTED_NEWEST_TRACE', result.stderr)
            self.assertIn('回放失败', result.stderr)
            self.assertNotIn('Traceback', result.stderr)

    def test_empty_latest_does_not_silently_use_an_older_capture(self):
        with tempfile.TemporaryDirectory() as directory:
            traces = Path(directory)/'logs/planner_trace'
            traces.mkdir(parents=True)
            older, newer = traces/'older.jsonl', traces/'newer.jsonl'
            older.write_text('{}\n')
            newer.touch()
            os.utime(older, ns=(1, 1))
            os.utime(newer, ns=(2, 2))
            result = self.run_cli(directory, '--latest')
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('输入文件为空', result.stderr)
            self.assertFalse((Path(directory)/'logs/planner_precision').exists())

    def test_osqp_interruption_is_cancellation(self):
        spec = importlib.util.spec_from_file_location('precision_under_test', SCRIPT)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        interrupted = SimpleNamespace(x=None, info=SimpleNamespace(
            status_val=osqp.constant('OSQP_SIGINT'), status='interrupted', run_time=.01))
        with mock.patch.object(module.osqp, 'OSQP') as factory:
            factory.return_value.solve.return_value = interrupted
            with self.assertRaises(KeyboardInterrupt):
                module.oracle(np.eye(1), np.zeros(1), 1.)

    def test_terminal_sigint_cancels_without_json_error_or_final_results(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            trace, replay = root/'trace.jsonl', root/'fake_replay'
            trace.write_text('{}\n')
            ready, pidfile = root/'ready', root/'child_pid'
            replay.write_text('#!/usr/bin/env python3\nimport os,sys,time\nfrom pathlib import Path\n'
                              f'Path({str(pidfile)!r}).write_text(str(os.getpid()))\n'
                              'sys.stdout.write("{\\\"unfinished\\\":")\nsys.stdout.flush()\n'
                              f'Path({str(ready)!r}).touch()\n'
                              'time.sleep(30)\n')
            replay.chmod(0o755)
            output = root/'result'
            process = subprocess.Popen([sys.executable, str(SCRIPT), str(trace),
                                        '--replay', str(replay), '--output', str(output)],
                                       cwd=directory, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                       text=True, start_new_session=True)
            child = None
            try:
                deadline = time.monotonic() + 5
                while not ready.exists() and process.poll() is None and time.monotonic() < deadline:
                    time.sleep(.01)
                self.assertTrue(ready.exists(), 'Fake replay did not start')
                child = int(pidfile.read_text())
                os.killpg(process.pid, signal.SIGINT)
                stdout, stderr = process.communicate(timeout=5)
                self.assertEqual(process.returncode, 130, stderr)
                self.assertIn('已取消', stderr)
                self.assertNotIn('Traceback', stderr)
                self.assertNotIn('delimiter', stderr)
                self.assertFalse((output/'summary.json').exists())
                self.assertFalse((output/'details.csv').exists())
                with self.assertRaises(ProcessLookupError):
                    os.kill(child, 0)
            finally:
                if process.poll() is None:
                    process.kill()
                    process.communicate()
                if child is not None:
                    try:
                        os.kill(child, signal.SIGKILL)
                    except ProcessLookupError:
                        pass


if __name__ == '__main__':
    unittest.main()
