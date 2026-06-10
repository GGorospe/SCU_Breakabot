"""
test_rpm_scale_node.py
======================
Unit tests for rpm_scale_node.

Run from the workspace root:
    pytest src/breakabot_hardware/test/test_rpm_scale_node.py -v

Requires the session-scoped rclpy.init() fixture in conftest.py.
No hardware dependencies — pure math.
"""

from unittest.mock import patch, MagicMock, call
import pytest
import rclpy
from std_msgs.msg import Int32MultiArray

from breakabot_hardware.rpm_scale_node import RpmScaleNode, MOTOR_COUNT


# ── Helpers ───────────────────────────────────────────────────────────────────

def make_node(max_rpm: float = 3000.0, control_mode: str = 'open_loop') -> RpmScaleNode:
    """Construct an RpmScaleNode with overridden parameters."""
    node = RpmScaleNode()
    # Override parameters post-construction for test isolation
    node._max_rpm = max_rpm
    node._control_mode = control_mode
    return node


def make_rpm_msg(data: list[int]) -> Int32MultiArray:
    msg = Int32MultiArray()
    msg.data = data
    return msg


def published_msg(mock_publish) -> Int32MultiArray:
    """Extract the message from the most recent publish() call."""
    return mock_publish.call_args[0][0]


# ── Open-loop conversion ──────────────────────────────────────────────────────

class TestOpenLoopConvert:
    """Unit tests for _open_loop_convert() — the core math."""

    def setup_method(self):
        self.node = make_node(max_rpm=1000.0)

    def teardown_method(self):
        self.node.destroy_node()

    def test_zero_rpm_gives_zero_g(self):
        result = self.node._open_loop_convert([0] * MOTOR_COUNT)
        assert result == [0] * MOTOR_COUNT

    def test_full_forward_gives_1000(self):
        result = self.node._open_loop_convert([1000] * MOTOR_COUNT)
        assert result == [1000] * MOTOR_COUNT

    def test_full_reverse_gives_neg_1000(self):
        result = self.node._open_loop_convert([-1000] * MOTOR_COUNT)
        assert result == [-1000] * MOTOR_COUNT

    def test_half_speed_gives_500(self):
        result = self.node._open_loop_convert([500] * MOTOR_COUNT)
        assert result == [500] * MOTOR_COUNT

    def test_over_max_clamps_to_1000(self):
        result = self.node._open_loop_convert([2000] * MOTOR_COUNT)
        assert result == [1000] * MOTOR_COUNT

    def test_under_min_clamps_to_neg_1000(self):
        result = self.node._open_loop_convert([-2000] * MOTOR_COUNT)
        assert result == [-1000] * MOTOR_COUNT

    def test_mixed_channels(self):
        # With max_rpm=1000: 500→500, -250→-250, 1500→1000, 0→0, -1500→-1000, 100→100
        result = self.node._open_loop_convert([500, -250, 1500, 0, -1500, 100])
        assert result == [500, -250, 1000, 0, -1000, 100]

    def test_truncates_to_int(self):
        # 333 / 1000 * 1000 = 333.0 exactly → 333
        result = self.node._open_loop_convert([333, 0, 0, 0, 0, 0])
        assert isinstance(result[0], int)
        assert result[0] == 333

    def test_fractional_truncation(self):
        # 1 / 3000 * 1000 = 0.333... → int → 0
        node = make_node(max_rpm=3000.0)
        result = node._open_loop_convert([1, 0, 0, 0, 0, 0])
        assert result[0] == 0
        node.destroy_node()

    def test_default_max_rpm_midpoint(self):
        # max_rpm=3000: 1500 → 500
        node = make_node(max_rpm=3000.0)
        result = node._open_loop_convert([1500, 0, 0, 0, 0, 0])
        assert result[0] == 500
        node.destroy_node()


# ── RPM cmd callback — publish path ──────────────────────────────────────────

class TestRpmCmdCallback:
    """Tests for the subscriber callback and publish behaviour."""

    def setup_method(self):
        self.node = make_node(max_rpm=1000.0)

    def teardown_method(self):
        self.node.destroy_node()

    def test_valid_message_publishes_motor_cmd(self):
        with patch.object(self.node.pub, 'publish') as mock_pub:
            self.node._rpm_cmd_callback(make_rpm_msg([500, 0, 0, 0, 0, 0]))
            mock_pub.assert_called_once()
            msg = published_msg(mock_pub)
            assert msg.data[0] == 500
            assert len(msg.data) == MOTOR_COUNT

    def test_valid_message_all_zeros(self):
        with patch.object(self.node.pub, 'publish') as mock_pub:
            self.node._rpm_cmd_callback(make_rpm_msg([0] * MOTOR_COUNT))
            mock_pub.assert_called_once()
            assert list(published_msg(mock_pub).data) == [0] * MOTOR_COUNT

    def test_full_forward_all_channels(self):
        with patch.object(self.node.pub, 'publish') as mock_pub:
            self.node._rpm_cmd_callback(make_rpm_msg([1000] * MOTOR_COUNT))
            assert list(published_msg(mock_pub).data) == [1000] * MOTOR_COUNT

    def test_full_reverse_all_channels(self):
        with patch.object(self.node.pub, 'publish') as mock_pub:
            self.node._rpm_cmd_callback(make_rpm_msg([-1000] * MOTOR_COUNT))
            assert list(published_msg(mock_pub).data) == [-1000] * MOTOR_COUNT

    def test_over_limit_clamps(self):
        with patch.object(self.node.pub, 'publish') as mock_pub:
            self.node._rpm_cmd_callback(make_rpm_msg([9999] * MOTOR_COUNT))
            assert list(published_msg(mock_pub).data) == [1000] * MOTOR_COUNT

    def test_output_is_int32multiarray(self):
        with patch.object(self.node.pub, 'publish') as mock_pub:
            self.node._rpm_cmd_callback(make_rpm_msg([100] * MOTOR_COUNT))
            msg = published_msg(mock_pub)
            assert isinstance(msg, Int32MultiArray)

    def test_output_elements_are_ints(self):
        with patch.object(self.node.pub, 'publish') as mock_pub:
            self.node._rpm_cmd_callback(make_rpm_msg([500] * MOTOR_COUNT))
            msg = published_msg(mock_pub)
            for val in msg.data:
                assert isinstance(val, int)


# ── Invalid message handling ───────────────────────────────────────────────────

class TestInvalidMessages:
    """Wrong array lengths should be dropped without publishing."""

    def setup_method(self):
        self.node = make_node()

    def teardown_method(self):
        self.node.destroy_node()

    def test_too_few_elements_drops_message(self):
        with patch.object(self.node.pub, 'publish') as mock_pub:
            self.node._rpm_cmd_callback(make_rpm_msg([0, 0, 0]))
            mock_pub.assert_not_called()

    def test_too_many_elements_drops_message(self):
        with patch.object(self.node.pub, 'publish') as mock_pub:
            self.node._rpm_cmd_callback(make_rpm_msg([0] * 8))
            mock_pub.assert_not_called()

    def test_empty_message_drops(self):
        with patch.object(self.node.pub, 'publish') as mock_pub:
            self.node._rpm_cmd_callback(make_rpm_msg([]))
            mock_pub.assert_not_called()

    def test_encoder_wrong_length_drops(self):
        """Encoder callback with wrong length should not update stored counts."""
        original = list(self.node._latest_encoder_counts)
        encoder_msg = Int32MultiArray()
        encoder_msg.data = [1, 2, 3]  # wrong length
        self.node._encoder_counts_callback(encoder_msg)
        assert self.node._latest_encoder_counts == original


# ── Control mode fallthrough ───────────────────────────────────────────────────

class TestControlMode:
    """Closed-loop mode should warn and fall through to open-loop."""

    def setup_method(self):
        self.node = make_node(max_rpm=1000.0, control_mode='closed_loop')

    def teardown_method(self):
        self.node.destroy_node()

    def test_closed_loop_still_publishes(self):
        """Even with closed_loop set, a valid message should produce output."""
        with patch.object(self.node.pub, 'publish') as mock_pub:
            self.node._rpm_cmd_callback(make_rpm_msg([500] * MOTOR_COUNT))
            mock_pub.assert_called_once()

    def test_closed_loop_fallthrough_matches_open_loop(self):
        """Closed-loop fallthrough should produce the same G-values as open-loop."""
        rpms = [100, -200, 500, 0, -1000, 750]
        with patch.object(self.node.pub, 'publish') as mock_pub:
            self.node._rpm_cmd_callback(make_rpm_msg(rpms))
            closed_result = list(published_msg(mock_pub).data)

        open_node = make_node(max_rpm=1000.0, control_mode='open_loop')
        with patch.object(open_node.pub, 'publish') as mock_pub2:
            open_node._rpm_cmd_callback(make_rpm_msg(rpms))
            open_result = list(published_msg(mock_pub2).data)
        open_node.destroy_node()

        assert closed_result == open_result


# ── Encoder counts storage ─────────────────────────────────────────────────────

class TestEncoderStorage:
    """Encoder counts are stored for future Phase 3 use."""

    def setup_method(self):
        self.node = make_node()

    def teardown_method(self):
        self.node.destroy_node()

    def test_encoder_counts_stored(self):
        counts = [10, 20, 30, 40, 50, 60]
        msg = Int32MultiArray()
        msg.data = counts
        self.node._encoder_counts_callback(msg)
        assert self.node._latest_encoder_counts == counts

    def test_encoder_counts_overwritten_on_new_message(self):
        first = [1, 2, 3, 4, 5, 6]
        second = [7, 8, 9, 10, 11, 12]
        for counts in (first, second):
            msg = Int32MultiArray()
            msg.data = counts
            self.node._encoder_counts_callback(msg)
        assert self.node._latest_encoder_counts == second
