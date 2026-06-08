# conftest.py
# Located at: breakabot_ws/src/breakabot_hardware/test/conftest.py
#
# Three jobs:
#   1. Add roboteq_node.py's directory to sys.path so pytest can import it
#      without requiring a colcon build + source cycle on every edit.
#   2. Prevent the ROS2 launch_testing pytest plugin from intercepting
#      collection of plain pytest unit tests.
#   3. Initialize and shut down the ROS2 context once per test session,
#      so that RoboteqNode() can call super().__init__() against the real rclpy.
#
# Usage:
#   cd ~/breakabot_ws
#   pytest src/breakabot_hardware/test/test_roboteq_node.py -v

import sys
import os
import pytest

# ---------------------------------------------------------------------------
# Path setup
# Standard ROS2 Python package layout:
#   breakabot_ws/src/breakabot_hardware/
#     breakabot_hardware/    <- roboteq_node.py lives here
#     test/                  <- this conftest.py lives here
# ---------------------------------------------------------------------------
_test_dir    = os.path.dirname(os.path.abspath(__file__))
_pkg_src_dir = os.path.join(_test_dir, '..', 'breakabot_hardware')

if os.path.isdir(_pkg_src_dir):
    sys.path.insert(0, os.path.abspath(_pkg_src_dir))
else:
    # Fallback: node is directly in the package root
    sys.path.insert(0, os.path.abspath(os.path.join(_test_dir, '..')))

# ---------------------------------------------------------------------------
# Suppress launch_testing plugin interference.
# ---------------------------------------------------------------------------
collect_ignore_glob = []

# ---------------------------------------------------------------------------
# ROS2 context lifecycle
#
# The real rclpy requires rclpy.init() before any Node can be instantiated,
# and rclpy.shutdown() at the end of the session. A session-scoped fixture
# handles this once for the entire test run.
#
# autouse=True — every test gets this automatically, no explicit declaration
# needed in test classes or functions.
#
# scope='session' — init/shutdown happen once per pytest invocation, not
# once per test. This matches how ROS2 expects the context to be managed
# and avoids 'context already initialized' errors between tests.
# ---------------------------------------------------------------------------
import rclpy

@pytest.fixture(scope='session', autouse=True)
def rclpy_session():
    rclpy.init()
    yield
    rclpy.shutdown()
