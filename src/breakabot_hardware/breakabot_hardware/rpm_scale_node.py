"""
rpm_scale_node.py
=================
Translates wheel RPM targets from kinematics_node into native G-unit commands
for the RoboteQ SDC2130 motor controllers (±1000 scale).

Subscriptions
-------------
  /roboteq/rpm_cmd      std_msgs/Int32MultiArray  6 RPM targets
  /roboteq/encoder_counts  std_msgs/Int32MultiArray  6 encoder counts (Phase 3)

Publications
------------
  /roboteq/motor_cmd    std_msgs/Int32MultiArray  6 G-unit commands (±1000)

Array layout (all 6-element arrays):
  [c0_ch1, c0_ch2, c1_ch1, c1_ch2, c2_ch1, c2_ch2]

Parameters
----------
  control_mode   (str)   : 'open_loop' | 'closed_loop'  default: 'open_loop'
  max_rpm        (float) : No-load RPM of Pittman LO-COG F5019.
                           *** PLACEHOLDER — update from motor label in lab ***
                           Pittman F5019 is an OEM winding; no public datasheet
                           exists. 3000.0 is a conservative estimate for a 24 V
                           Pittman 5000-series winding; under-estimating max_rpm
                           produces slower-than-commanded motion (safe failure).
  kp             (float) : PID proportional gain (Phase 3 only)
  ki             (float) : PID integral gain     (Phase 3 only)
  kd             (float) : PID derivative gain   (Phase 3 only)
  encoder_ppr    (int)   : Encoder pulses per revolution (Phase 3 only)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

MOTOR_COUNT = 6


class RpmScaleNode(Node):
    """Open-loop RPM → G-unit converter (Phase 2).

    Designed so Phase 2 → Phase 3 requires only:
      1. Setting control_mode: closed_loop in the params yaml.
      2. Filling in _closed_loop_update() with PID logic.
    Topic interfaces remain unchanged between phases.
    """

    def __init__(self):
        super().__init__('rpm_scale_node')

        # ── Parameter declarations ──────────────────────────────────────────
        # Follow the same declare_parameters style as roboteq_node.
        self.declare_parameters(
            namespace='',
            parameters=[
                ('control_mode', 'open_loop'),
                # *** PLACEHOLDER — read from motor label in lab and update ***
                ('max_rpm', 3000.0),
                # Phase 3 PID parameters (declared now, unused in Phase 2)
                ('kp', 1.0),
                ('ki', 0.0),
                ('kd', 0.0),
                ('encoder_ppr', 512),
            ]
        )

        # ── Read parameters ─────────────────────────────────────────────────
        self._control_mode = self.get_parameter('control_mode').value
        self._max_rpm = self.get_parameter('max_rpm').value

        # Phase 3 parameters (stored but not used until Phase 3 is implemented)
        self._kp = self.get_parameter('kp').value
        self._ki = self.get_parameter('ki').value
        self._kd = self.get_parameter('kd').value
        self._encoder_ppr = self.get_parameter('encoder_ppr').value

        # ── Validate control_mode ────────────────────────────────────────────
        valid_modes = ('open_loop', 'closed_loop')
        if self._control_mode not in valid_modes:
            self.get_logger().error(
                f"Unknown control_mode '{self._control_mode}'. "
                f"Must be one of {valid_modes}. Defaulting to 'open_loop'."
            )
            self._control_mode = 'open_loop'

        if self._control_mode == 'closed_loop':
            self.get_logger().warn(
                "control_mode is 'closed_loop' but Phase 3 is not yet "
                "implemented. Falling through to open-loop control."
            )

        # ── Log startup info ────────────────────────────────────────────────
        self.get_logger().info(
            f"rpm_scale_node starting | control_mode={self._control_mode} | "
            f"max_rpm={self._max_rpm}"
        )
        if self._max_rpm == 3000.0:
            self.get_logger().warn(
                "max_rpm is using the default placeholder (3000.0). "
                "Update from the Pittman F5019 motor label in lab before "
                "running speed-critical tests."
            )

        # ── Publisher ────────────────────────────────────────────────────────
        self.pub = self.create_publisher(
            Int32MultiArray,
            '/roboteq/motor_cmd',
            10
        )

        # ── Subscribers ──────────────────────────────────────────────────────
        self.create_subscription(
            Int32MultiArray,
            '/roboteq/rpm_cmd',
            self._rpm_cmd_callback,
            10
        )

        # Phase 3: encoder feedback subscription
        # Declared here so the interface exists; callback is a no-op in Phase 2
        self._latest_encoder_counts = [0] * MOTOR_COUNT
        self.create_subscription(
            Int32MultiArray,
            '/roboteq/encoder_counts',
            self._encoder_counts_callback,
            10
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _rpm_cmd_callback(self, msg: Int32MultiArray):
        """Convert RPM targets to G-unit motor commands and publish."""
        if len(msg.data) != MOTOR_COUNT:
            self.get_logger().error(
                f"Received /roboteq/rpm_cmd with {len(msg.data)} element(s); "
                f"expected {MOTOR_COUNT}. Dropping message."
            )
            return

        if self._control_mode == 'closed_loop':
            # Phase 3 hook — falls through to open-loop until implemented
            g_values = self._closed_loop_update(list(msg.data))
        else:
            g_values = self._open_loop_convert(list(msg.data))

        out = Int32MultiArray()
        out.data = g_values
        self.pub.publish(out)

    def _encoder_counts_callback(self, msg: Int32MultiArray):
        """Store the latest encoder counts for use in Phase 3 PID."""
        if len(msg.data) != MOTOR_COUNT:
            self.get_logger().error(
                f"Received /roboteq/encoder_counts with {len(msg.data)} "
                f"element(s); expected {MOTOR_COUNT}. Dropping message."
            )
            return
        self._latest_encoder_counts = list(msg.data)

    # ── Conversion logic ──────────────────────────────────────────────────────

    def _open_loop_convert(self, rpm_targets: list[int]) -> list[int]:
        """Phase 2: pure open-loop RPM → G-unit conversion.

        Formula per channel:
            g_value = int((rpm_target / max_rpm) * 1000)
            g_value = clamp(g_value, -1000, 1000)
        """
        g_values = []
        for rpm in rpm_targets:
            g = int((rpm / self._max_rpm) * 1000)
            g = max(-1000, min(1000, g))
            g_values.append(g)
        return g_values

    def _closed_loop_update(self, rpm_targets: list[int]) -> list[int]:
        """Phase 3: closed-loop PID speed control using encoder feedback.

        NOT YET IMPLEMENTED — falls through to open-loop with a warning.

        When implementing Phase 3:
          - Compute actual RPM from self._latest_encoder_counts and
            self._encoder_ppr using the time delta since last callback.
          - Run per-channel PID using self._kp, self._ki, self._kd.
          - Return a list of 6 clamped G-unit commands.
        """
        self.get_logger().warn(
            "Closed-loop control not yet implemented. "
            "Falling through to open-loop.",
            throttle_duration_sec=5.0
        )
        return self._open_loop_convert(rpm_targets)


def main(args=None):
    rclpy.init(args=args)
    node = RpmScaleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
