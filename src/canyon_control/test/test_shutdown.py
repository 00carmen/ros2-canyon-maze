# Copyright (c) 2026 00carmen
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

"""Exercise real ROS node shutdown in isolated subprocesses."""

import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest


CHILD = '''
import importlib
import sys

module = importlib.import_module(sys.argv[1])
original_spin = module.rclpy.spin

def ready_spin(node):
    print('CANYON_TEST_READY', flush=True)
    original_spin(node)

module.rclpy.spin = ready_spin
module.main(args=[])
print('CANYON_TEST_EXIT_OK', flush=True)
'''


@pytest.mark.skipif(os.name != 'posix', reason='Requires POSIX signals and ROS 2')
@pytest.mark.parametrize('module', [
    'canyon_control.maze_wall_v3',
    'canyon_control.maze_monitor',
])
@pytest.mark.parametrize('shutdown_signal', [signal.SIGINT, signal.SIGTERM])
def test_node_shutdown(module, shutdown_signal, tmp_path):
    """Exit cleanly on SIGINT and on external ROS context shutdown."""
    pytest.importorskip('rclpy', reason='Run this test inside the ROS 2 environment')
    environment = os.environ.copy()
    environment['ROS_DOMAIN_ID'] = str(90 + os.getpid() % 10)
    environment['ROS_LOCALHOST_ONLY'] = '1'
    environment['ROS_LOG_DIR'] = str(tmp_path / 'ros_logs')
    environment['PYTHONUNBUFFERED'] = '1'
    # Exercise the current source instead of an older installed overlay.
    package_dir = str(Path(__file__).resolve().parents[1])
    environment['PYTHONPATH'] = os.pathsep.join(filter(None, [
        package_dir, environment.get('PYTHONPATH'),
    ]))
    output_file = tmp_path / 'node.log'
    with output_file.open('w', encoding='utf-8') as output:
        process = subprocess.Popen(
            [sys.executable, '-u', '-c', CHILD, module],
            stdout=output,
            stderr=subprocess.STDOUT,
            env=environment,
        )
        try:
            deadline = time.monotonic() + 15.0
            while time.monotonic() < deadline:
                text = output_file.read_text(encoding='utf-8')
                if 'CANYON_TEST_READY' in text:
                    break
                assert process.poll() is None, text
                time.sleep(0.05)
            else:
                pytest.fail('Node did not reach spin(): ' + text)
            process.send_signal(shutdown_signal)
            process.wait(timeout=10)
        finally:
            if process.poll() is None:
                process.kill()
                process.wait(timeout=5)
    text = output_file.read_text(encoding='utf-8')
    assert process.returncode == 0, text
    assert 'CANYON_TEST_EXIT_OK' in text, text
    assert 'Traceback' not in text, text
