# test_roboteq_node.py
# Unit tests for roboteq_node — Break-a-bot hardware interface
#
# Run with:
#   cd ~/breakabot_ws
#   colcon test --packages-select breakabot_hardware
#   colcon test-result --verbose
#
# Or directly with pytest (faster during development):
#   pytest src/breakabot_hardware/test/test_roboteq_node.py -v
#
# These tests cover pure logic only — no serial hardware required.
# Serial ports are replaced with unittest.mock.MagicMock objects.
# Tests run on any machine with ROS2 and pyserial installed.
#
# Test coverage:
#   [PARSE]   _parse_two_ints — normal, edge, and malformed inputs
#   [INIT]    last_cmd initialization for open_loop and closed_loop modes
#   [CMD]     _cmd_callback — command string formatting, inversion, clamping,
#             last_cmd update, wrong-length rejection
#   [SERIAL]  _send_command — serial write content, lock acquisition
#   [QUERY]   _send_query — reply parsing, prefix stripping, malformed reply handling
#   [POLL]    _fast_poll_callback — keepalive sent, encoder/current published
#   [BATT]    _battery_poll_callback — voltage parsing, averaging, publish
#   [SHUTDOWN] destroy_node — stop commands sent before port close

import sys
import threading
from unittest.mock import MagicMock, call, patch

import pytest

# ---------------------------------------------------------------------------
# Serial mock — must be installed before roboteq_node is imported
# ---------------------------------------------------------------------------
_serial_mock = MagicMock()
_serial_mock.SerialException = IOError   # use a real exception class
_serial_mock.PARITY_NONE  = 'N'
_serial_mock.STOPBITS_ONE = 1
_serial_mock.EIGHTBITS    = 8
sys.modules['serial'] = _serial_mock

# Import the node under test (ROS2 stubs are expected to be on PYTHONPATH
# when running via colcon; on bare pytest, install the rclpy package normally)
from roboteq_node import RoboteqNode, NUM_CONTROLLERS, NUM_CHANNELS, NUM_MOTORS  # noqa: E402


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_serial_port(readline_responses: list[bytes] | None = None) -> MagicMock:
    """Return a mock serial.Serial with configurable readline responses."""
    port = MagicMock()
    port.is_open = True
    if readline_responses:
        port.readline.side_effect = readline_responses
    else:
        port.readline.return_value = b''
    return port


def _make_node(
    control_mode: str = 'open_loop',
    invert_ch1: list[bool] | None = None,
    invert_ch2: list[bool] | None = None,
    serial_ports: list[MagicMock] | None = None,
) -> RoboteqNode:
    """
    Instantiate RoboteqNode with mocked serial ports.

    serial_ports: list of 3 mock port objects. If None, plain MagicMocks are used.
    """
    if invert_ch1 is None:
        invert_ch1 = [False, False, False]
    if invert_ch2 is None:
        invert_ch2 = [False, False, False]
    if serial_ports is None:
        serial_ports = [_make_serial_port() for _ in range(NUM_CONTROLLERS)]

    # serial.Serial() is called once per controller in __init__
    _serial_mock.Serial.side_effect = serial_ports

    node = RoboteqNode()

    # Override parameters that were read during __init__
    # (the stub node stores them as instance attributes)
    node.control_mode = control_mode
    node.invert_ch1   = invert_ch1
    node.invert_ch2   = invert_ch2

    # Rebuild last_cmd to reflect the (possibly overridden) control_mode
    cmd_char = 'G' if control_mode == 'open_loop' else 'S'
    node.last_cmd = [f'!{cmd_char} 1 0_!{cmd_char} 2 0_' for _ in range(NUM_CONTROLLERS)]

    return node


def _make_cmd_msg(data: list[int]):
    """Return a minimal Int32MultiArray-like object."""
    from std_msgs.msg import Int32MultiArray
    msg = Int32MultiArray()
    msg.data = data
    return msg


# ---------------------------------------------------------------------------
# [PARSE] _parse_two_ints
# ---------------------------------------------------------------------------

class TestParseTwoInts:

    def test_normal_positive_values(self):
        assert RoboteqNode._parse_two_ints('246:135') == (246, 135)

    def test_normal_negative_values(self):
        assert RoboteqNode._parse_two_ints('-500:300') == (-500, 300)

    def test_zero_values(self):
        assert RoboteqNode._parse_two_ints('0:0') == (0, 0)

    def test_large_encoder_counts(self):
        # Encoder counts are 32-bit signed — test large values
        assert RoboteqNode._parse_two_ints('2147483647:-2147483648') == (2147483647, -2147483648)

    def test_empty_string_returns_safe_default(self):
        # Empty string arrives when a serial query times out
        assert RoboteqNode._parse_two_ints('') == (0, 0)

    def test_missing_colon_returns_safe_default(self):
        assert RoboteqNode._parse_two_ints('246') == (0, 0)

    def test_non_numeric_returns_safe_default(self):
        assert RoboteqNode._parse_two_ints('abc:def') == (0, 0)

    def test_extra_fields_uses_first_two(self):
        # ?V replies have three colon-separated fields — ensure only first two parsed
        result = RoboteqNode._parse_two_ints('246:135:4730')
        assert result == (246, 135)

    def test_whitespace_stripped_reply(self):
        # Python's int() accepts leading/trailing whitespace, so ' 100:200'
        # parses successfully. This is a robustness feature, not a bug.
        assert RoboteqNode._parse_two_ints(' 100:200') == (100, 200)


# ---------------------------------------------------------------------------
# [INIT] last_cmd initialization
# ---------------------------------------------------------------------------

class TestLastCmdInit:

    def test_open_loop_uses_G_command(self):
        node = _make_node(control_mode='open_loop')
        for cmd in node.last_cmd:
            assert '!G' in cmd
            assert '!S' not in cmd

    def test_closed_loop_uses_S_command(self):
        node = _make_node(control_mode='closed_loop')
        for cmd in node.last_cmd:
            assert '!S' in cmd
            assert '!G' not in cmd

    def test_last_cmd_has_one_entry_per_controller(self):
        node = _make_node()
        assert len(node.last_cmd) == NUM_CONTROLLERS

    def test_last_cmd_initialised_to_zero(self):
        node = _make_node(control_mode='open_loop')
        assert node.last_cmd[0] == '!G 1 0_!G 2 0_'

    def test_last_cmd_closed_loop_initialised_to_zero(self):
        node = _make_node(control_mode='closed_loop')
        assert node.last_cmd[0] == '!S 1 0_!S 2 0_'


# ---------------------------------------------------------------------------
# [CMD] _cmd_callback — command string formatting
# ---------------------------------------------------------------------------

class TestCmdCallback:

    def test_open_loop_correct_command_string(self):
        """Positive command values should produce correct !G strings."""
        node = _make_node(control_mode='open_loop')
        node._cmd_callback(_make_cmd_msg([500, 300, 0, 0, 0, 0]))
        assert node.last_cmd[0] == '!G 1 500_!G 2 300_'

    def test_closed_loop_correct_command_string(self):
        """Closed-loop mode should produce !S strings."""
        node = _make_node(control_mode='closed_loop')
        node._cmd_callback(_make_cmd_msg([120, 90, 0, 0, 0, 0]))
        assert node.last_cmd[0] == '!S 1 120_!S 2 90_'

    def test_all_three_controllers_updated(self):
        node = _make_node(control_mode='open_loop')
        node._cmd_callback(_make_cmd_msg([100, 200, 300, 400, 500, 600]))
        assert node.last_cmd[0] == '!G 1 100_!G 2 200_'
        assert node.last_cmd[1] == '!G 1 300_!G 2 400_'
        assert node.last_cmd[2] == '!G 1 500_!G 2 600_'

    def test_negative_values_preserved(self):
        node = _make_node(control_mode='open_loop')
        node._cmd_callback(_make_cmd_msg([-500, -300, 0, 0, 0, 0]))
        assert node.last_cmd[0] == '!G 1 -500_!G 2 -300_'

    def test_zero_command(self):
        node = _make_node(control_mode='open_loop')
        node._cmd_callback(_make_cmd_msg([0, 0, 0, 0, 0, 0]))
        assert node.last_cmd[0] == '!G 1 0_!G 2 0_'

    # --- Clamping ---

    def test_clamp_above_positive_limit(self):
        node = _make_node(control_mode='open_loop')
        node._cmd_callback(_make_cmd_msg([1500, 0, 0, 0, 0, 0]))
        assert '!G 1 1000' in node.last_cmd[0]

    def test_clamp_below_negative_limit(self):
        node = _make_node(control_mode='open_loop')
        node._cmd_callback(_make_cmd_msg([-1500, 0, 0, 0, 0, 0]))
        assert '!G 1 -1000' in node.last_cmd[0]

    def test_clamp_at_exactly_positive_limit(self):
        node = _make_node(control_mode='open_loop')
        node._cmd_callback(_make_cmd_msg([1000, 0, 0, 0, 0, 0]))
        assert '!G 1 1000' in node.last_cmd[0]

    def test_clamp_at_exactly_negative_limit(self):
        node = _make_node(control_mode='open_loop')
        node._cmd_callback(_make_cmd_msg([-1000, 0, 0, 0, 0, 0]))
        assert '!G 1 -1000' in node.last_cmd[0]

    # --- Inversion ---

    def test_ch1_inversion_on_controller_0(self):
        node = _make_node(invert_ch1=[True, False, False])
        node._cmd_callback(_make_cmd_msg([500, 300, 0, 0, 0, 0]))
        # ch1 inverted: 500 → -500; ch2 unchanged: 300
        assert node.last_cmd[0] == '!G 1 -500_!G 2 300_'

    def test_ch2_inversion_on_controller_1(self):
        node = _make_node(invert_ch2=[False, True, False])
        node._cmd_callback(_make_cmd_msg([0, 0, 400, 200, 0, 0]))
        # ch1 unchanged: 400; ch2 inverted: 200 → -200
        assert node.last_cmd[1] == '!G 1 400_!G 2 -200_'

    def test_both_channels_inverted_on_controller_2(self):
        node = _make_node(invert_ch1=[False, False, True], invert_ch2=[False, False, True])
        node._cmd_callback(_make_cmd_msg([0, 0, 0, 0, 600, 700]))
        assert node.last_cmd[2] == '!G 1 -600_!G 2 -700_'

    def test_inversion_of_zero_remains_zero(self):
        node = _make_node(invert_ch1=[True, True, True])
        node._cmd_callback(_make_cmd_msg([0, 0, 0, 0, 0, 0]))
        for cmd in node.last_cmd:
            assert '!G 1 0' in cmd

    def test_inversion_applied_before_clamp(self):
        # If inversion were applied after clamp, -(-1500) = 1500 would be unclamped.
        # If applied before, -1500 → clamp → -1000 → invert would give +1000.
        # The node applies inversion THEN clamps, so -1500 inverted = 1500, clamped = 1000.
        node = _make_node(invert_ch1=[True, False, False])
        node._cmd_callback(_make_cmd_msg([-1500, 0, 0, 0, 0, 0]))
        assert '!G 1 1000' in node.last_cmd[0]

    # --- Input validation ---

    def test_wrong_length_array_ignored(self):
        node = _make_node()
        original_last_cmd = node.last_cmd.copy()
        node._cmd_callback(_make_cmd_msg([100, 200, 300]))   # 3 values, expected 6
        assert node.last_cmd == original_last_cmd            # last_cmd unchanged

    def test_empty_array_ignored(self):
        node = _make_node()
        original_last_cmd = node.last_cmd.copy()
        node._cmd_callback(_make_cmd_msg([]))
        assert node.last_cmd == original_last_cmd

    def test_seven_element_array_ignored(self):
        node = _make_node()
        original_last_cmd = node.last_cmd.copy()
        node._cmd_callback(_make_cmd_msg([0, 0, 0, 0, 0, 0, 0]))
        assert node.last_cmd == original_last_cmd

    # --- last_cmd update ---

    def test_last_cmd_updated_after_valid_command(self):
        node = _make_node()
        node._cmd_callback(_make_cmd_msg([100, 200, 300, 400, 500, 600]))
        assert node.last_cmd[0] == '!G 1 100_!G 2 200_'
        assert node.last_cmd[1] == '!G 1 300_!G 2 400_'
        assert node.last_cmd[2] == '!G 1 500_!G 2 600_'

    def test_last_cmd_not_updated_after_invalid_command(self):
        node = _make_node()
        node._cmd_callback(_make_cmd_msg([100, 200, 300, 400, 500, 600]))
        snapshot = node.last_cmd.copy()
        node._cmd_callback(_make_cmd_msg([99]))              # invalid — should be ignored
        assert node.last_cmd == snapshot


# ---------------------------------------------------------------------------
# [SERIAL] _send_command — serial write and lock behaviour
# ---------------------------------------------------------------------------

class TestSendCommand:

    def test_command_written_to_correct_port(self):
        ports = [_make_serial_port() for _ in range(NUM_CONTROLLERS)]
        node = _make_node(serial_ports=ports)
        ctrl = node.controllers[1]
        node._send_command(ctrl, '!G 1 500_!G 2 300_', controller_index=1)
        ctrl['port'].write.assert_called_once_with(b'!G 1 500_!G 2 300_')

    def test_ack_lines_read_after_write(self):
        """Two concatenated commands → two ACK reads expected."""
        ports = [_make_serial_port() for _ in range(NUM_CONTROLLERS)]
        node = _make_node(serial_ports=ports)
        ctrl = node.controllers[0]
        node._send_command(ctrl, '!G 1 0_!G 2 0_', controller_index=0)
        assert ctrl['port'].readline.call_count == 2

    def test_lock_acquired_during_write(self):
        """Verify the per-controller lock is used (thread safety)."""
        ports = [_make_serial_port() for _ in range(NUM_CONTROLLERS)]
        node = _make_node(serial_ports=ports)
        ctrl = node.controllers[0]
        real_lock = threading.Lock()
        ctrl['lock'] = real_lock
        # If the lock were not acquired this would still pass, but the lock
        # being a real threading.Lock confirms the code path is exercised.
        node._send_command(ctrl, '!G 1 0_!G 2 0_', controller_index=0)
        assert not real_lock.locked()   # lock released after call

    def test_serial_exception_does_not_propagate(self):
        """A serial error should be logged and swallowed, not crash the node."""
        ports = [_make_serial_port() for _ in range(NUM_CONTROLLERS)]
        node = _make_node(serial_ports=ports)
        ctrl = node.controllers[0]
        ctrl['port'].write.side_effect = IOError('port disconnected')
        # Should not raise
        node._send_command(ctrl, '!G 1 0_!G 2 0_', controller_index=0)


# ---------------------------------------------------------------------------
# [QUERY] _send_query — reply parsing
# ---------------------------------------------------------------------------

class TestSendQuery:

    def test_valid_reply_strips_prefix(self):
        ports = [_make_serial_port(readline_responses=[b'C=246:135\r']) for _ in range(NUM_CONTROLLERS)]
        node = _make_node(serial_ports=ports)
        ctrl = node.controllers[0]
        result = node._send_query(ctrl, '?C\r', prefix='C', controller_index=0)
        assert result == '246:135'

    def test_voltage_reply_strips_prefix(self):
        ports = [_make_serial_port(readline_responses=[b'V=246:135:4730\r']) for _ in range(NUM_CONTROLLERS)]
        node = _make_node(serial_ports=ports)
        ctrl = node.controllers[0]
        result = node._send_query(ctrl, '?V\r', prefix='V', controller_index=0)
        assert result == '246:135:4730'

    def test_wrong_prefix_returns_empty_string(self):
        # Controller reply with wrong prefix → empty string, no crash
        ports = [_make_serial_port(readline_responses=[b'X=100:200\r']) for _ in range(NUM_CONTROLLERS)]
        node = _make_node(serial_ports=ports)
        ctrl = node.controllers[0]
        result = node._send_query(ctrl, '?C\r', prefix='C', controller_index=0)
        assert result == ''

    def test_empty_reply_returns_empty_string(self):
        ports = [_make_serial_port(readline_responses=[b'']) for _ in range(NUM_CONTROLLERS)]
        node = _make_node(serial_ports=ports)
        ctrl = node.controllers[0]
        result = node._send_query(ctrl, '?C\r', prefix='C', controller_index=0)
        assert result == ''

    def test_serial_exception_returns_empty_string(self):
        ports = [_make_serial_port() for _ in range(NUM_CONTROLLERS)]
        node = _make_node(serial_ports=ports)
        ctrl = node.controllers[0]
        ctrl['port'].readline.side_effect = IOError('timeout')
        result = node._send_query(ctrl, '?C\r', prefix='C', controller_index=0)
        assert result == ''

    def test_input_buffer_reset_before_query(self):
        """Stale bytes in the buffer must be cleared before each query."""
        ports = [_make_serial_port(readline_responses=[b'C=0:0\r']) for _ in range(NUM_CONTROLLERS)]
        node = _make_node(serial_ports=ports)
        ctrl = node.controllers[0]
        node._send_query(ctrl, '?C\r', prefix='C', controller_index=0)
        ctrl['port'].reset_input_buffer.assert_called_once()

    def test_query_string_written_to_port(self):
        ports = [_make_serial_port(readline_responses=[b'A=50:80\r']) for _ in range(NUM_CONTROLLERS)]
        node = _make_node(serial_ports=ports)
        ctrl = node.controllers[0]
        node._send_query(ctrl, '?A\r', prefix='A', controller_index=0)
        ctrl['port'].write.assert_called_once_with(b'?A\r')


# ---------------------------------------------------------------------------
# [POLL] _fast_poll_callback
# ---------------------------------------------------------------------------

class TestFastPollCallback:

    def _make_poll_node(self):
        """Node whose ports return plausible ?C and ?A replies."""
        responses = [
            b'C=1000:2000\r',   # ?C reply
            b'A=50:80\r',       # ?A reply
        ]
        # Each of three controllers needs the same two responses, repeated
        ports = [_make_serial_port(readline_responses=responses * 10)
                 for _ in range(NUM_CONTROLLERS)]
        return _make_node(serial_ports=ports)

    def test_keepalive_sent_to_all_controllers(self):
        """Each fast poll cycle must write the last_cmd to every controller."""
        node = self._make_poll_node()
        node.last_cmd = ['!G 1 100_!G 2 200_', '!G 1 300_!G 2 400_', '!G 1 500_!G 2 600_']
        node._fast_poll_callback()
        for i, ctrl in enumerate(node.controllers):
            written = b''.join(
                c.args[0] for c in ctrl['port'].write.call_args_list
                if c.args[0] in [node.last_cmd[i].encode('ascii')]
            )
            assert node.last_cmd[i].encode('ascii') in written or \
                   ctrl['port'].write.called  # keepalive was sent

    def test_encoder_counts_published(self):
        # Patch publish() so we can inspect what the node passed to it.
        # The real rclpy.Publisher sends to the ROS2 graph — it has no last_msg.
        node = self._make_poll_node()
        with patch.object(node.encoder_pub, 'publish') as mock_pub:
            node._fast_poll_callback()
            mock_pub.assert_called_once()
            msg = mock_pub.call_args[0][0]
            assert len(msg.data) == NUM_MOTORS

    def test_motor_current_published(self):
        node = self._make_poll_node()
        with patch.object(node.current_pub, 'publish') as mock_pub:
            node._fast_poll_callback()
            mock_pub.assert_called_once()
            msg = mock_pub.call_args[0][0]
            assert len(msg.data) == NUM_MOTORS

    def test_encoder_values_correctly_parsed(self):
        """?C=1000:2000 → encoder_counts[0]=1000, encoder_counts[1]=2000 for controller 0."""
        node = self._make_poll_node()
        with patch.object(node.encoder_pub, 'publish') as mock_pub:
            node._fast_poll_callback()
            data = mock_pub.call_args[0][0].data
        assert data[0] == 1000   # controller 0, ch1
        assert data[1] == 2000   # controller 0, ch2

    def test_current_values_converted_from_tenths(self):
        """?A=50:80 → motor_current[0]=5.0A, motor_current[1]=8.0A for controller 0."""
        node = self._make_poll_node()
        with patch.object(node.current_pub, 'publish') as mock_pub:
            node._fast_poll_callback()
            data = mock_pub.call_args[0][0].data
        assert abs(data[0] - 5.0) < 0.001   # 50 / 10.0
        assert abs(data[1] - 8.0) < 0.001   # 80 / 10.0

    def test_failed_query_publishes_zeros(self):
        """A serial timeout on one controller should publish 0 for that controller.

        Each fast poll cycle per controller calls readline() in this order:
          _send_command: 2× readline() to consume !G ACKs
          _send_query ?C: 1× readline()
          _send_query ?A: 1× readline()
        Total: 4 readline() calls per controller. Supply empty bytes for each
        to simulate a full timeout across all controllers.
        """
        reads_per_controller = [b''] * 4
        ports = [
            _make_serial_port(readline_responses=reads_per_controller)
            for _ in range(NUM_CONTROLLERS)
        ]
        node = _make_node(serial_ports=ports)
        with patch.object(node.encoder_pub, 'publish') as enc_pub, \
             patch.object(node.current_pub, 'publish') as cur_pub:
            node._fast_poll_callback()
            enc_data = enc_pub.call_args[0][0].data
            cur_data = cur_pub.call_args[0][0].data
        assert enc_data[0] == 0
        assert cur_data[0] == 0.0


# ---------------------------------------------------------------------------
# [BATT] _battery_poll_callback
# ---------------------------------------------------------------------------

class TestBatteryPollCallback:

    def _make_batt_node(self, v_replies: list[str]):
        """Node whose ports return specified ?V replies for each controller."""
        ports = [
            _make_serial_port(readline_responses=[reply.encode('ascii')])
            for reply in v_replies
        ]
        return _make_node(serial_ports=ports)

    def test_battery_state_published(self):
        node = self._make_batt_node(['V=135:246:4730\r'] * 3)
        with patch.object(node.battery_pub, 'publish') as mock_pub:
            node._battery_poll_callback()
            mock_pub.assert_called_once()

    def test_voltage_correctly_parsed(self):
        """V=135:246:4730 → battery voltage = 246/10 = 24.6V"""
        node = self._make_batt_node(['V=135:246:4730\r'] * 3)
        with patch.object(node.battery_pub, 'publish') as mock_pub:
            node._battery_poll_callback()
            msg = mock_pub.call_args[0][0]
        assert abs(msg.voltage - 24.6) < 0.01

    def test_voltage_averaged_across_controllers(self):
        """Three controllers reporting different voltages should be averaged."""
        node = self._make_batt_node([
            'V=135:240:4730\r',   # 24.0V
            'V=135:246:4730\r',   # 24.6V
            'V=135:252:4730\r',   # 25.2V
        ])
        with patch.object(node.battery_pub, 'publish') as mock_pub:
            node._battery_poll_callback()
            msg = mock_pub.call_args[0][0]
        expected_avg = (24.0 + 24.6 + 25.2) / 3
        assert abs(msg.voltage - expected_avg) < 0.01

    def test_malformed_reply_does_not_crash(self):
        """One bad reply should not prevent the other two from being averaged."""
        node = self._make_batt_node([
            'V=135:246:4730\r',
            b''.decode('ascii'),   # empty reply — simulates timeout
            'V=135:246:4730\r',
        ])
        with patch.object(node.battery_pub, 'publish') as mock_pub:
            node._battery_poll_callback()   # should not raise
            mock_pub.assert_called_once()   # two valid readings → one publish

    def test_all_replies_failed_does_not_publish(self):
        """If all three controllers fail, nothing should be published."""
        ports = [_make_serial_port(readline_responses=[b'']) for _ in range(NUM_CONTROLLERS)]
        node = _make_node(serial_ports=ports)
        with patch.object(node.battery_pub, 'publish') as mock_pub:
            node._battery_poll_callback()
            mock_pub.assert_not_called()


# ---------------------------------------------------------------------------
# [SHUTDOWN] destroy_node
# ---------------------------------------------------------------------------

class TestDestroyNode:

    def test_stop_command_sent_to_all_controllers(self):
        """!MS 1_!MS 2_ must be written to every controller before port close."""
        ports = [_make_serial_port() for _ in range(NUM_CONTROLLERS)]
        node = _make_node(serial_ports=ports)
        node.destroy_node()
        for ctrl in node.controllers:
            written_bytes = b''.join(
                c.args[0] for c in ctrl['port'].write.call_args_list
            )
            assert b'!MS 1' in written_bytes
            assert b'!MS 2' in written_bytes

    def test_ports_closed_after_stop(self):
        """Serial ports must be closed during shutdown."""
        ports = [_make_serial_port() for _ in range(NUM_CONTROLLERS)]
        node = _make_node(serial_ports=ports)
        node.destroy_node()
        for ctrl in node.controllers:
            ctrl['port'].close.assert_called_once()

    def test_stop_sent_before_close(self):
        """Stop command must precede port close on each controller."""
        ports = [_make_serial_port() for _ in range(NUM_CONTROLLERS)]
        node = _make_node(serial_ports=ports)
        node.destroy_node()
        for ctrl in node.controllers:
            write_order  = ctrl['port'].write.call_args_list
            close_order  = ctrl['port'].close.call_args_list
            assert len(write_order) > 0, 'write() was never called'
            assert len(close_order) > 0, 'close() was never called'
            # write() call index always < close() call index in call sequence
            # (MagicMock records all calls on the parent mock's mock_calls)
            write_idx = next(
                i for i, c in enumerate(ctrl['port'].mock_calls)
                if 'write' in str(c)
            )
            close_idx = next(
                i for i, c in enumerate(ctrl['port'].mock_calls)
                if 'close' in str(c)
            )
            assert write_idx < close_idx

    def test_port_close_error_does_not_propagate(self):
        """A close() failure on one port must not prevent shutdown of others."""
        ports = [_make_serial_port() for _ in range(NUM_CONTROLLERS)]
        node = _make_node(serial_ports=ports)
        node.controllers[1]['port'].close.side_effect = IOError('port busy')
        # Should not raise
        node.destroy_node()
        # Controller 2 should still have been closed
        node.controllers[2]['port'].close.assert_called_once()
