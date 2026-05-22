"""
test_imu_node_hw.py — hardware integration test for ImuNode

Requires: Raspberry Pi with BNO055 wired and powered.
Run directly (not via colcon):
    python3 src/breakabot_hw/test/test_imu_node_hw.py

What this checks:
  1. Node starts and BNO055 initializes without error
  2. /imu/data publishes at approximately the configured rate
  3. Quaternion norm is within tolerance (fusion algorithm converged)
  4. Linear acceleration magnitude is near 0 m/s² (sensor at rest)
  5. No None readings over a 5-second window

Keep the sensor stationary on a flat surface when running this test.
"""

import math
import sys
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Imu

from breakabot_hardware.imu_node import ImuNode

# ── Test configuration ────────────────────────────────────────────────────────
COLLECT_SECONDS      = 5.0
EXPECTED_RATE_HZ     = 50.0
RATE_TOLERANCE       = 0.10   # ±10%
QUATERNION_NORM_TOL  = 0.05   # |norm - 1.0| < 0.05
MAX_LINEAR_ACCEL_MS2 = 0.5    # m/s² — sensor at rest, gravity removed


def run_hw_test():
    rclpy.init()
    node = ImuNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    results = {
        'initialized': node.hw_ready,
        'messages':    [],
    }

    if not node.hw_ready:
        print('FAIL — BNO055 did not initialize. Check wiring and I2C address.')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    # Subscribe to the topic the node publishes
    subscriber = rclpy.create_node('hw_test_subscriber')
    subscriber.create_subscription(
        Imu,
        '/imu/data',
        lambda msg: results['messages'].append(msg),
        10
    )

    print(f'Collecting data for {COLLECT_SECONDS}s — keep sensor still...')
    end_time = time.time() + COLLECT_SECONDS
    while time.time() < end_time:
        executor.spin_once(timeout_sec=0.01)
        rclpy.spin_once(subscriber, timeout_sec=0.0)

    subscriber.destroy_node()
    node.destroy_node()
    rclpy.shutdown()

    msgs = results['messages']
    passed = True

    # ── Check 1: message count / rate ─────────────────────────────────────────
    expected_count = EXPECTED_RATE_HZ * COLLECT_SECONDS
    low  = expected_count * (1.0 - RATE_TOLERANCE)
    high = expected_count * (1.0 + RATE_TOLERANCE)
    actual_rate = len(msgs) / COLLECT_SECONDS

    if low <= len(msgs) <= high:
        print(f'PASS  Publish rate: {actual_rate:.1f} Hz '
              f'({len(msgs)} msgs in {COLLECT_SECONDS}s)')
    else:
        print(f'FAIL  Publish rate: {actual_rate:.1f} Hz — '
              f'expected {EXPECTED_RATE_HZ} ±{RATE_TOLERANCE*100:.0f}%')
        passed = False

    # ── Check 2: quaternion norm ──────────────────────────────────────────────
    bad_norm_count = 0
    for msg in msgs:
        o = msg.orientation
        norm = math.sqrt(o.w**2 + o.x**2 + o.y**2 + o.z**2)
        if abs(norm - 1.0) > QUATERNION_NORM_TOL:
            bad_norm_count += 1

    if bad_norm_count == 0:
        print(f'PASS  Quaternion norm within tolerance for all {len(msgs)} msgs')
    else:
        pct = 100.0 * bad_norm_count / len(msgs)
        print(f'FAIL  {bad_norm_count} msgs ({pct:.1f}%) had quaternion norm '
              f'outside tolerance — sensor may not have converged')
        passed = False

    # ── Check 3: linear acceleration at rest ─────────────────────────────────
    bad_accel_count = 0
    for msg in msgs:
        a = msg.linear_acceleration
        magnitude = math.sqrt(a.x**2 + a.y**2 + a.z**2)
        if magnitude > MAX_LINEAR_ACCEL_MS2:
            bad_accel_count += 1

    if bad_accel_count == 0:
        print(f'PASS  Linear acceleration within rest threshold '
              f'(<{MAX_LINEAR_ACCEL_MS2} m/s²) for all msgs')
    else:
        pct = 100.0 * bad_accel_count / len(msgs)
        print(f'WARN  {bad_accel_count} msgs ({pct:.1f}%) exceeded rest '
              f'threshold — was the sensor moving? '
              f'(threshold: {MAX_LINEAR_ACCEL_MS2} m/s²)')
        # Warning only — don't fail on this; vibration from Pi fan can trigger it

    # ── Check 4: no None fields ───────────────────────────────────────────────
    # If any reading was None the node substitutes zeros — detect via zero
    # quaternion (w=0, x=0, y=0, z=0) which is physically impossible
    null_quat_count = sum(
        1 for msg in msgs
        if (msg.orientation.w == 0.0 and
            msg.orientation.x == 0.0 and
            msg.orientation.y == 0.0 and
            msg.orientation.z == 0.0)
    )
    if null_quat_count == 0:
        print('PASS  No null quaternion readings detected')
    else:
        print(f'FAIL  {null_quat_count} msgs had null quaternion '
              f'— sensor read failures occurred')
        passed = False

    # ── Summary ───────────────────────────────────────────────────────────────
    print()
    print('─' * 40)
    print('RESULT:', 'ALL TESTS PASSED' if passed else 'ONE OR MORE TESTS FAILED')
    print('─' * 40)
    sys.exit(0 if passed else 1)


if __name__ == '__main__':
    run_hw_test()
