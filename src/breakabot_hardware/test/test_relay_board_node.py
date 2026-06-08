# File: test_relay_board_node.py
# Author: George Gorospe, ggorospe@scu.edu
# About: Unit tests for the RelayBoardNode hardware interface.
#
# These tests run without ROS2 hardware or a live ROS2 context by mocking
# rclpy, gpiod, and breakabot_interfaces before importing the node module.
#
# Run with:
#   colcon test --packages-select breakabot_hardware
#   colcon test-result --verbose
# Or directly:
#   pytest src/breakabot_hardware/test/test_relay_board_node.py -v

# About the tests - these tests were designed to evaluate this node's contracts,
# the tasks that this node was created to do.
# - On startup, all three MCs start on channel 1 and that state is published
# - A valid command updates only the targeted MC's channel and publishes the new state
# - An invalid MC number is rejected with a warning: state is unchanged
# - An invalid channel number is rejected with a warning: state is unchanged
# - On shutdown, GPIO lines are released if they were acquired



import sys
import types
import unittest
from unittest.mock import MagicMock, patch, call


# ---------------------------------------------------------------------------
# Minimal stubs — must be injected before the module under test is imported
# ---------------------------------------------------------------------------

def _make_ros2_stubs():
    """Inject just enough of rclpy into sys.modules for the node to import."""

    # --- rclpy ---
    rclpy_mod = types.ModuleType('rclpy')
    rclpy_mod.init = MagicMock()
    rclpy_mod.spin = MagicMock()
    rclpy_mod.shutdown = MagicMock()

    # --- rclpy.node ---
    node_mod = types.ModuleType('rclpy.node')

    class FakeNode:
        """Minimal stand-in for rclpy.node.Node."""
        def __init__(self, name):
            self._name = name
            self._params = {}
            self._logger = MagicMock()
            self._logger.warn = MagicMock()
            self._logger.info = MagicMock()
            self._logger.error = MagicMock()

        def get_logger(self):
            return self._logger

        def declare_parameter(self, name, default):
            param = MagicMock()
            param.value = default
            self._params[name] = param

        def get_parameter(self, name):
            return self._params[name]

        def create_subscription(self, *args, **kwargs):
            return MagicMock()

        def create_publisher(self, *args, **kwargs):
            pub = MagicMock()
            pub.publish = MagicMock()
            return pub

        def destroy_node(self):
            pass

    node_mod.Node = FakeNode
    rclpy_mod.node = node_mod

    # --- std_msgs ---
    std_msgs = types.ModuleType('std_msgs')
    std_msgs_msg = types.ModuleType('std_msgs.msg')

    class FakeInt32MultiArray:
        def __init__(self):
            self.data = []

    std_msgs_msg.Int32MultiArray = FakeInt32MultiArray
    std_msgs.msg = std_msgs_msg

    # --- breakabot_interfaces ---
    bk_ifaces = types.ModuleType('breakabot_interfaces')
    bk_ifaces_msg = types.ModuleType('breakabot_interfaces.msg')

    class FakeRelayCommand:
        def __init__(self, motor_controller=1, channel=1):
            self.motor_controller = motor_controller
            self.channel = channel

    bk_ifaces_msg.RelayCommand = FakeRelayCommand
    bk_ifaces.msg = bk_ifaces_msg

    # --- gpiod (stub so HW_AVAILABLE stays False) ---
    # Not injected — ImportError causes the node to set HW_AVAILABLE = False,
    # which is exactly what we want for unit testing.

    for name, mod in [
        ('rclpy', rclpy_mod),
        ('rclpy.node', node_mod),
        ('std_msgs', std_msgs),
        ('std_msgs.msg', std_msgs_msg),
        ('breakabot_interfaces', bk_ifaces),
        ('breakabot_interfaces.msg', bk_ifaces_msg),
    ]:
        sys.modules[name] = mod


_make_ros2_stubs()

# Now it's safe to import the module under test
import importlib, pathlib, types as _types

# Load relay_board_node.py directly by file path so the test file can live
# anywhere in the repository without requiring an installed package.
_src = pathlib.Path(__file__).parent.parent / 'breakabot_hardware' / 'relay_board_node.py'
_spec = importlib.util.spec_from_file_location('relay_board_node', _src)
_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_mod)

RelayBoardNode = _mod.RelayBoardNode


# ---------------------------------------------------------------------------
# Helper
# ---------------------------------------------------------------------------

def make_node():
    """Return a freshly constructed RelayBoardNode in stub (no-HW) mode."""
    return RelayBoardNode()


def make_cmd(motor_controller=1, channel=1):
    from breakabot_interfaces.msg import RelayCommand
    return RelayCommand(motor_controller=motor_controller, channel=channel)


# ---------------------------------------------------------------------------
# Test cases
# ---------------------------------------------------------------------------

class TestRelayBoardNodeInit(unittest.TestCase):
    """Tests covering node initialisation."""

    def test_initial_channel_state(self):
        """All three motor controllers should start on channel 1."""
        node = make_node()
        self.assertEqual(node.current_channel, {1: 1, 2: 1, 3: 1})

    def test_gpio_request_matches_hw_state(self):
        """Ensure that test for gpio avalability,gpio_request, matches hardware (laptop or RPi)."""
        node = make_node()
        if node.hw_ready:
            self.assertIsNotNone(node.gpio_request)
        else:
            self.assertIsNone(node.gpio_request)

    def test_mc_relay_map_default_pins(self):
        """mc_relay_map should reflect the default GPIO pin parameters."""
        node = make_node()
        self.assertEqual(node.mc_relay_map[1], (17, 27))
        self.assertEqual(node.mc_relay_map[2], (22, 23))
        self.assertEqual(node.mc_relay_map[3], (24, 25))

    def test_initial_state_published(self):
        """publish_state() must be called once during __init__."""
        node = make_node()
        # The publisher's publish method should have been called once on init
        node.relay_state_publisher.publish.assert_called_once()
        published_msg = node.relay_state_publisher.publish.call_args[0][0]
        self.assertEqual(list(published_msg.data), [1, 1, 1])


class TestListenerCallbackValidCommands(unittest.TestCase):
    """Tests covering normal (valid) relay commands."""

    def test_set_mc1_to_channel_2(self):
        node = make_node()
        node.listener_callback(make_cmd(motor_controller=1, channel=2))
        self.assertEqual(node.current_channel[1], 2)

    def test_set_mc2_to_channel_2(self):
        node = make_node()
        node.listener_callback(make_cmd(motor_controller=2, channel=2))
        self.assertEqual(node.current_channel[2], 2)

    def test_set_mc3_to_channel_2(self):
        node = make_node()
        node.listener_callback(make_cmd(motor_controller=3, channel=2))
        self.assertEqual(node.current_channel[3], 2)

    def test_set_channel_then_reset_to_1(self):
        """A second command should overwrite the first."""
        node = make_node()
        node.listener_callback(make_cmd(motor_controller=1, channel=2))
        node.listener_callback(make_cmd(motor_controller=1, channel=1))
        self.assertEqual(node.current_channel[1], 1)

    def test_only_targeted_mc_changes(self):
        """Commanding MC1 must not affect MC2 or MC3."""
        node = make_node()
        node.listener_callback(make_cmd(motor_controller=1, channel=2))
        self.assertEqual(node.current_channel[2], 1)
        self.assertEqual(node.current_channel[3], 1)

    def test_state_published_after_valid_command(self):
        """publish_state() must be called after every valid command."""
        node = make_node()
        initial_call_count = node.relay_state_publisher.publish.call_count
        node.listener_callback(make_cmd(motor_controller=2, channel=2))
        self.assertEqual(
            node.relay_state_publisher.publish.call_count,
            initial_call_count + 1
        )

    def test_published_state_reflects_new_channel(self):
        """The published Int32MultiArray must contain the updated channel."""
        node = make_node()
        node.listener_callback(make_cmd(motor_controller=3, channel=2))
        last_msg = node.relay_state_publisher.publish.call_args[0][0]
        self.assertEqual(list(last_msg.data), [1, 1, 2])

    def test_all_three_controllers_to_channel_2(self):
        """Setting all three MCs to channel 2 should update all entries."""
        node = make_node()
        for mc in (1, 2, 3):
            node.listener_callback(make_cmd(motor_controller=mc, channel=2))
        self.assertEqual(node.current_channel, {1: 2, 2: 2, 3: 2})


class TestListenerCallbackInvalidCommands(unittest.TestCase):
    """Tests covering guard-clause rejection of bad commands."""

    def _assert_state_unchanged(self, node):
        self.assertEqual(node.current_channel, {1: 1, 2: 1, 3: 1})

    def test_invalid_mc_number_too_high(self):
        """MC number 5 is out of range — state must not change."""
        node = make_node()
        node.listener_callback(make_cmd(motor_controller=5, channel=1))
        self._assert_state_unchanged(node)

    def test_invalid_mc_number_zero(self):
        """MC number 0 is out of range — state must not change."""
        node = make_node()
        node.listener_callback(make_cmd(motor_controller=0, channel=1))
        self._assert_state_unchanged(node)

    def test_invalid_mc_number_negative(self):
        node = make_node()
        node.listener_callback(make_cmd(motor_controller=-1, channel=1))
        self._assert_state_unchanged(node)

    def test_invalid_channel_too_high(self):
        """Channel 9 is out of range — state must not change."""
        node = make_node()
        node.listener_callback(make_cmd(motor_controller=1, channel=9))
        self._assert_state_unchanged(node)

    def test_invalid_channel_zero(self):
        node = make_node()
        node.listener_callback(make_cmd(motor_controller=1, channel=0))
        self._assert_state_unchanged(node)

    def test_invalid_mc_logs_warning(self):
        """An invalid MC number must trigger a logger.warn call."""
        node = make_node()
        node.listener_callback(make_cmd(motor_controller=5, channel=1))
        node.get_logger().warn.assert_called()

    def test_invalid_channel_logs_warning(self):
        """An invalid channel must trigger a logger.warn call."""
        node = make_node()
        node.listener_callback(make_cmd(motor_controller=1, channel=9))
        node.get_logger().warn.assert_called()

    def test_invalid_command_does_not_publish_state(self):
        """No extra publish_state() call should occur on a rejected command."""
        node = make_node()
        initial_count = node.relay_state_publisher.publish.call_count
        node.listener_callback(make_cmd(motor_controller=5, channel=1))
        self.assertEqual(
            node.relay_state_publisher.publish.call_count,
            initial_count
        )


class TestPublishState(unittest.TestCase):
    """Tests covering the publish_state helper directly."""

    def test_publish_state_data_length(self):
        """Published data must always have exactly 3 elements."""
        node = make_node()
        node.publish_state()
        msg = node.relay_state_publisher.publish.call_args[0][0]
        self.assertEqual(len(msg.data), 3)

    def test_publish_state_reflects_current_channel(self):
        """publish_state must read from current_channel, not a stale copy."""
        node = make_node()
        node.current_channel = {1: 2, 2: 1, 3: 2}
        node.publish_state()
        msg = node.relay_state_publisher.publish.call_args[0][0]
        self.assertEqual(list(msg.data), [2, 1, 2])


class TestDestroyNode(unittest.TestCase):
    """Tests covering cleanup behaviour."""

    def test_destroy_releases_gpio_when_request_exists(self):
        """destroy_node() must call release() on a non-None gpio_request."""
        node = make_node()
        mock_request = MagicMock()
        node.gpio_request = mock_request
        node.destroy_node()
        mock_request.release.assert_called_once()

    def test_destroy_safe_when_no_gpio_request(self):
        """destroy_node() must not raise regardless of gpio_request state."""
        node = make_node()
        try:
            node.destroy_node()
        except Exception as e:
            self.fail(f'destroy_node raised unexpectedly: {e}')


if __name__ == '__main__':
    unittest.main(verbosity=2)
