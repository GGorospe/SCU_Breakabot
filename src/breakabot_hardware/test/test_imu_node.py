"""
test_imu_node.py — unit tests for ImuNode

Tests ROS2 plumbing only. No hardware required.
Run from workspace root:
    colcon test --packages-select breakabot_hw
    colcon test-result --verbose
Or directly:
    pytest src/breakabot_hardware/test/test_imu_node.py -v
"""

import time
import unittest

import rclpy
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Imu

from breakabot_hardware.imu_node import ImuNode


class TestImuNodePlumbing(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = ImuNode()
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

    def tearDown(self):
        self.executor.remove_node(self.node)
        self.node.destroy_node()

    # ── Parameter tests ───────────────────────────────────────────────────────

    def test_default_publish_rate(self):
        rate = self.node.get_parameter('publish_rate_hz').value
        self.assertEqual(rate, 50.0)

    def test_default_frame_id(self):
        frame = self.node.get_parameter('frame_id').value
        self.assertEqual(frame, 'imu_link')

    def test_default_i2c_address(self):
        addr = self.node.get_parameter('i2c_address').value
        self.assertEqual(addr, 0x28)

    # ── Node identity tests ───────────────────────────────────────────────────

    def test_node_name(self):
        self.assertEqual(self.node.get_name(), 'imu_node')

    def test_publisher_exists(self):
        """Publisher should exist regardless of hardware availability."""
        self.assertIsNotNone(self.node.imu_publisher)

    def test_timer_exists(self):
        """Timer should be created regardless of hardware availability."""
        self.assertIsNotNone(self.node.timer)

    def test_hw_ready_false_without_hardware(self):
        """On a machine without the Adafruit libs, hw_ready must be False."""
        try:
            import board  # noqa: F401
        except ImportError:
            self.assertFalse(self.node.hw_ready)
        else:
            # Hardware libs present — hw_ready depends on physical sensor
            # Just confirm the attribute exists with a boolean value
            self.assertIsInstance(self.node.hw_ready, bool)

    # ── Topic tests ───────────────────────────────────────────────────────────

    def test_topic_name(self):
        """Node must publish on /imu/data."""
        topic_names = [
            name for name, _ in self.node.get_publisher_names_and_types_by_node(
                'imu_node', ''
            )
        ]
        self.assertIn('/imu/data', topic_names)

    def test_topic_type(self):
        """Topic must carry sensor_msgs/msg/Imu."""
        publishers = self.node.get_publisher_names_and_types_by_node(
            'imu_node', ''
        )
        topic_types = {name: types for name, types in publishers}
        self.assertIn('/imu/data', topic_types)
        self.assertIn('sensor_msgs/msg/Imu', topic_types['/imu/data'])

    # ── Stub-mode publish test ────────────────────────────────────────────────

    def test_no_publish_in_stub_mode(self):
        """Without hardware, timer fires but nothing should be published."""
        if self.node.hw_ready:
            self.skipTest('Hardware present — stub mode test not applicable')

        received = []

        helper = rclpy.create_node('test_subscriber')
        helper.create_subscription(
            Imu, '/imu/data', lambda msg: received.append(msg), 10
        )

        # Spin long enough for several timer callbacks at 50 Hz
        end_time = time.time() + 0.5
        while time.time() < end_time:
            self.executor.spin_once(timeout_sec=0.05)
            rclpy.spin_once(helper, timeout_sec=0.0)

        helper.destroy_node()
        self.assertEqual(
            len(received), 0,
            'Node should not publish messages in stub mode'
        )


class TestImuMessageStructure(unittest.TestCase):
    """
    Tests for the structure of a manually constructed Imu message.
    These run anywhere — no node, no hardware, no ROS2 spin needed.
    """

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_covariance_arrays_are_nine_elements(self):
        msg = Imu()
        msg.orientation_covariance        = [0.0] * 9
        msg.angular_velocity_covariance   = [0.0] * 9
        msg.linear_acceleration_covariance = [0.0] * 9
        self.assertEqual(len(msg.orientation_covariance), 9)
        self.assertEqual(len(msg.angular_velocity_covariance), 9)
        self.assertEqual(len(msg.linear_acceleration_covariance), 9)

    def test_quaternion_field_assignment(self):
        """Verify BNO055 (w,x,y,z) → Imu (x,y,z,w) remapping."""
        msg = Imu()
        bno055_quat = (1.0, 0.0, 0.0, 0.0)   # identity: w=1, x=y=z=0
        msg.orientation.w = bno055_quat[0]
        msg.orientation.x = bno055_quat[1]
        msg.orientation.y = bno055_quat[2]
        msg.orientation.z = bno055_quat[3]
        self.assertEqual(msg.orientation.w, 1.0)
        self.assertEqual(msg.orientation.x, 0.0)

    def test_identity_quaternion_norm(self):
        import math
        w, x, y, z = 1.0, 0.0, 0.0, 0.0
        norm = math.sqrt(w**2 + x**2 + y**2 + z**2)
        self.assertAlmostEqual(norm, 1.0, places=5)


if __name__ == '__main__':
    unittest.main()
