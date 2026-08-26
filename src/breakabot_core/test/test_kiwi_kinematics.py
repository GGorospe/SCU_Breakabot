# test_kiwi_kinematics.py
# Unit tests for kiwi_kinematics.py — Break-a-bot Kiwi-drive inverse kinematics
#
# kiwi_kinematics.py is a pure Python module with no ROS dependencies.
# No mocking, no rclpy.init(), no conftest.py fixture required.
#
# Run with:
#   pytest src/breakabot_core/test/test_kiwi_kinematics.py -v
#
# Test coverage:
#   [INIT]     KiwiKinematics constructor — valid construction, guard clauses
#   [PURE]     Pure body-frame motions — forward, strafe, spin, all-stop
#   [SCALE]    Linear scaling with vx, wheel_radius, robot_radius
#   [SYM]      Geometric symmetry properties of the Kiwi matrix
#   [STUB]     forward_kinematics raises NotImplementedError

import math
import sys
import os
import pytest

# ---------------------------------------------------------------------------
# Path setup — locate kiwi_kinematics.py relative to this test file.
# Supports running pytest from the workspace root or from within the package.
# ---------------------------------------------------------------------------
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'breakabot_core'))

from kiwi_kinematics import KiwiKinematics, J1F

# ---------------------------------------------------------------------------
# Shared tolerance for floating-point comparisons
# ---------------------------------------------------------------------------
TOL = 1e-9

# ---------------------------------------------------------------------------
# Shared fixture — a standard robot with round numbers for easy hand-checking
#   wheel_radius = 0.05 m  (5 cm)
#   robot_radius = 0.15 m  (15 cm, center to wheel)
# ---------------------------------------------------------------------------
@pytest.fixture
def robot():
    return KiwiKinematics(wheel_radius=0.05, robot_radius=0.15)


# ===========================================================================
# [INIT] Constructor and guard clauses
# ===========================================================================

class TestKiwiKinematicsInit:

    def test_valid_construction_succeeds(self):
        k = KiwiKinematics(wheel_radius=0.05, robot_radius=0.15)
        assert k.wheel_radius == 0.05
        assert k.robot_radius == 0.15

    def test_wheel_radius_zero_raises(self):
        with pytest.raises(ValueError, match='wheel_radius'):
            KiwiKinematics(wheel_radius=0.0, robot_radius=0.15)

    def test_wheel_radius_negative_raises(self):
        with pytest.raises(ValueError, match='wheel_radius'):
            KiwiKinematics(wheel_radius=-0.05, robot_radius=0.15)

    def test_robot_radius_zero_raises(self):
        # A zero robot_radius silently makes all rotation commands have no
        # effect. Caught at construction time rather than silently wrong.
        with pytest.raises(ValueError, match='robot_radius'):
            KiwiKinematics(wheel_radius=0.05, robot_radius=0.0)

    def test_robot_radius_negative_raises(self):
        with pytest.raises(ValueError, match='robot_radius'):
            KiwiKinematics(wheel_radius=0.05, robot_radius=-0.15)

    def test_very_small_positive_radii_accepted(self):
        # Guard is strictly > 0; any positive value is valid
        k = KiwiKinematics(wheel_radius=1e-9, robot_radius=1e-9)
        assert k.wheel_radius == pytest.approx(1e-9)


# ===========================================================================
# [PURE] Pure body-frame motions
#
# These tests verify physically meaningful motion primitives.
# Each motion is chosen so the expected wheel-speed relationships follow
# directly from the Kiwi geometry, without needing to compute by hand.
# ===========================================================================

class TestPureMotions:

    def test_all_stop_returns_zeros(self, robot):
        w = robot.inverse_kinematics(0.0, 0.0, 0.0)
        assert w[0] == pytest.approx(0.0, abs=TOL)
        assert w[1] == pytest.approx(0.0, abs=TOL)
        assert w[2] == pytest.approx(0.0, abs=TOL)

    def test_pure_forward_w0_and_w2_equal_magnitude(self, robot):
        # Pure +vx: the chassis is symmetric about the x-axis, so wheel 0
        # (at 60°) and wheel 2 (at 300° = -60°) must have equal magnitude.
        w = robot.inverse_kinematics(1.0, 0.0, 0.0)
        assert abs(w[0]) == pytest.approx(abs(w[2]), rel=1e-9)

    def test_pure_forward_w0_and_w2_opposite_sign(self, robot):
        # The vx column of J1F is [+√3/2, 0, -√3/2], so w0 and w2 are
        # equal in magnitude but opposite in sign for pure forward motion.
        w = robot.inverse_kinematics(1.0, 0.0, 0.0)
        assert w[0] == pytest.approx(-w[2], rel=1e-9)

    def test_pure_forward_w1_is_zero(self, robot):
        # J1F[1][0] = 0.0 — wheel 1 has no vx contribution.
        w = robot.inverse_kinematics(1.0, 0.0, 0.0)
        assert w[1] == pytest.approx(0.0, abs=TOL)

    def test_pure_strafe_w0_and_w2_same_sign(self, robot):
        # Pure +vy: the vy column is [-1/2, +1, -1/2].
        # w0 and w2 both get the -1/2 coefficient → same sign.
        w = robot.inverse_kinematics(0.0, 1.0, 0.0)
        assert math.copysign(1, w[0]) == math.copysign(1, w[2])

    def test_pure_strafe_w0_and_w2_equal_magnitude(self, robot):
        # Both get identical -1/2 coefficient → equal magnitude.
        w = robot.inverse_kinematics(0.0, 1.0, 0.0)
        assert abs(w[0]) == pytest.approx(abs(w[2]), rel=1e-9)

    def test_pure_strafe_w1_opposite_sign_to_w0(self, robot):
        # vy column: w1 gets +1, w0 and w2 get -1/2 → opposite sign.
        w = robot.inverse_kinematics(0.0, 1.0, 0.0)
        assert math.copysign(1, w[1]) != math.copysign(1, w[0])

    def test_pure_spin_all_equal_magnitude(self, robot):
        # The omega column of J1F is [-l, -l, -l] — identical for all wheels.
        # Pure rotation must produce equal wheel speeds (magnitude) on all three.
        w = robot.inverse_kinematics(0.0, 0.0, 1.0)
        assert abs(w[0]) == pytest.approx(abs(w[1]), rel=1e-9)
        assert abs(w[1]) == pytest.approx(abs(w[2]), rel=1e-9)

    def test_pure_spin_all_same_sign(self, robot):
        # All three omega coefficients are identical (-l), so all wheel
        # speeds must have the same sign for a pure spin command.
        w = robot.inverse_kinematics(0.0, 0.0, 1.0)
        assert math.copysign(1, w[0]) == math.copysign(1, w[1])
        assert math.copysign(1, w[1]) == math.copysign(1, w[2])

    def test_pure_spin_numerical_value(self):
        # For omega=1 rad/s, wheel_radius=r, robot_radius=l:
        #   w_i = (1/r) * (-l * 1.0) = -l/r
        r, l = 0.05, 0.15
        robot = KiwiKinematics(wheel_radius=r, robot_radius=l)
        expected = -l / r  # = -3.0
        w = robot.inverse_kinematics(0.0, 0.0, 1.0)
        assert w[0] == pytest.approx(expected, rel=1e-9)
        assert w[1] == pytest.approx(expected, rel=1e-9)
        assert w[2] == pytest.approx(expected, rel=1e-9)

    def test_pure_forward_numerical_value(self):
        # For vx=1, wheel_radius=r:
        #   w0 = (1/r) * (√3/2 * 1) = √3/(2r)
        #   w1 = 0
        #   w2 = (1/r) * (-√3/2 * 1) = -√3/(2r)
        r = 0.05
        robot = KiwiKinematics(wheel_radius=r, robot_radius=0.15)
        w = robot.inverse_kinematics(1.0, 0.0, 0.0)
        assert w[0] == pytest.approx(math.sqrt(3) / (2 * r), rel=1e-9)
        assert w[1] == pytest.approx(0.0, abs=TOL)
        assert w[2] == pytest.approx(-math.sqrt(3) / (2 * r), rel=1e-9)


# ===========================================================================
# [SCALE] Linear scaling properties
#
# The IK is a linear map: scaling any input must scale the output
# proportionally. These tests verify that the implementation is truly linear
# and that the physical parameters (r, l) scale as expected.
# ===========================================================================

class TestScaling:

    def test_doubling_vx_doubles_wheel_speeds(self, robot):
        w1x = robot.inverse_kinematics(1.0, 0.0, 0.0)
        w2x = robot.inverse_kinematics(2.0, 0.0, 0.0)
        for i in range(3):
            assert w2x[i] == pytest.approx(2.0 * w1x[i], rel=1e-9)

    def test_doubling_vy_doubles_wheel_speeds(self, robot):
        w1y = robot.inverse_kinematics(0.0, 1.0, 0.0)
        w2y = robot.inverse_kinematics(0.0, 2.0, 0.0)
        for i in range(3):
            assert w2y[i] == pytest.approx(2.0 * w1y[i], rel=1e-9)

    def test_doubling_omega_doubles_wheel_speeds(self, robot):
        w1o = robot.inverse_kinematics(0.0, 0.0, 1.0)
        w2o = robot.inverse_kinematics(0.0, 0.0, 2.0)
        for i in range(3):
            assert w2o[i] == pytest.approx(2.0 * w1o[i], rel=1e-9)

    def test_doubling_wheel_radius_halves_wheel_speeds(self):
        # Wheel speed = (1/r) * geometry — doubling r halves the result.
        k1 = KiwiKinematics(wheel_radius=0.05, robot_radius=0.15)
        k2 = KiwiKinematics(wheel_radius=0.10, robot_radius=0.15)
        w1 = k1.inverse_kinematics(1.0, 0.5, 0.3)
        w2 = k2.inverse_kinematics(1.0, 0.5, 0.3)
        for i in range(3):
            assert w2[i] == pytest.approx(0.5 * w1[i], rel=1e-9)

    def test_doubling_robot_radius_doubles_omega_contribution_only(self):
        # robot_radius only scales the omega term.
        # For a pure translation (omega=0) the wheel speeds must be identical.
        k1 = KiwiKinematics(wheel_radius=0.05, robot_radius=0.15)
        k2 = KiwiKinematics(wheel_radius=0.05, robot_radius=0.30)
        w1 = k1.inverse_kinematics(1.0, 0.5, 0.0)
        w2 = k2.inverse_kinematics(1.0, 0.5, 0.0)
        for i in range(3):
            assert w2[i] == pytest.approx(w1[i], rel=1e-9)

    def test_doubling_robot_radius_doubles_spin_speed(self):
        # For pure omega, wheel speed = (-l/r) * omega — doubling l doubles result.
        k1 = KiwiKinematics(wheel_radius=0.05, robot_radius=0.15)
        k2 = KiwiKinematics(wheel_radius=0.05, robot_radius=0.30)
        w1 = k1.inverse_kinematics(0.0, 0.0, 1.0)
        w2 = k2.inverse_kinematics(0.0, 0.0, 1.0)
        for i in range(3):
            assert w2[i] == pytest.approx(2.0 * w1[i], rel=1e-9)

    def test_superposition_vx_plus_omega(self, robot):
        # Linearity: result of combined input must equal sum of individual results.
        w_vx    = robot.inverse_kinematics(1.0, 0.0, 0.0)
        w_omega = robot.inverse_kinematics(0.0, 0.0, 1.0)
        w_both  = robot.inverse_kinematics(1.0, 0.0, 1.0)
        for i in range(3):
            assert w_both[i] == pytest.approx(w_vx[i] + w_omega[i], rel=1e-9)


# ===========================================================================
# [SYM] Geometric symmetry properties
#
# The Kiwi geometry has specific symmetries that must hold regardless of
# parameter values. These catch matrix transcription errors that numerical
# tests might miss.
# ===========================================================================

class TestSymmetry:

    def test_forward_motion_w0_equals_negative_w2(self, robot):
        # Pure +vx: J1F column 0 is [+√3/2, 0, -√3/2] — antisymmetric.
        w = robot.inverse_kinematics(1.0, 0.0, 0.0)
        assert w[0] == pytest.approx(-w[2], rel=1e-9)

    def test_strafe_motion_w0_equals_w2(self, robot):
        # Pure +vy: J1F column 1 is [-1/2, +1, -1/2] — w0 and w2 identical.
        w = robot.inverse_kinematics(0.0, 1.0, 0.0)
        assert w[0] == pytest.approx(w[2], rel=1e-9)

    def test_reversing_vx_negates_all_outputs(self, robot):
        w_pos = robot.inverse_kinematics( 1.0, 0.0, 0.0)
        w_neg = robot.inverse_kinematics(-1.0, 0.0, 0.0)
        for i in range(3):
            assert w_neg[i] == pytest.approx(-w_pos[i], rel=1e-9)

    def test_reversing_vy_negates_all_outputs(self, robot):
        w_pos = robot.inverse_kinematics(0.0,  1.0, 0.0)
        w_neg = robot.inverse_kinematics(0.0, -1.0, 0.0)
        for i in range(3):
            assert w_neg[i] == pytest.approx(-w_pos[i], rel=1e-9)

    def test_reversing_omega_negates_all_outputs(self, robot):
        w_pos = robot.inverse_kinematics(0.0, 0.0,  1.0)
        w_neg = robot.inverse_kinematics(0.0, 0.0, -1.0)
        for i in range(3):
            assert w_neg[i] == pytest.approx(-w_pos[i], rel=1e-9)

    def test_j1f_omega_column_all_equal(self):
        # The omega column of J1F must be identical for all three wheels.
        # Any deviation means the robot would translate during a pure spin.
        assert J1F[0][2] == J1F[1][2] == J1F[2][2]

    def test_j1f_vx_column_antisymmetric(self):
        # vx column: J1F[0][0] must equal -J1F[2][0]; J1F[1][0] must be 0.
        assert J1F[0][0] == pytest.approx(-J1F[2][0], rel=1e-9)
        assert J1F[1][0] == pytest.approx(0.0, abs=TOL)

    def test_j1f_vy_column_w0_equals_w2(self):
        # vy column: J1F[0][1] must equal J1F[2][1] (both -1/2).
        assert J1F[0][1] == pytest.approx(J1F[2][1], rel=1e-9)

    def test_sqrt3_over_2_not_sqrt_of_3_over_2(self):
        # Guard against the sqrt(3/2) vs sqrt(3)/2 typo that was caught
        # during development. sqrt(3)/2 ≈ 0.866; sqrt(3/2) ≈ 0.707.
        # The correct value is sqrt(3)/2.
        assert J1F[0][0] == pytest.approx(math.sqrt(3) / 2, rel=1e-9)
        assert J1F[2][0] == pytest.approx(-math.sqrt(3) / 2, rel=1e-9)


# ===========================================================================
# [STUB] forward_kinematics raises NotImplementedError
# ===========================================================================

class TestForwardKinematicsStub:

    def test_forward_kinematics_raises_not_implemented(self, robot):
        with pytest.raises(NotImplementedError):
            robot.forward_kinematics(1.0, 1.0, 1.0)

    def test_forward_kinematics_raises_for_all_zero(self, robot):
        # Ensure the stub raises even for a trivial all-zero input —
        # no shortcut path that might accidentally return a value.
        with pytest.raises(NotImplementedError):
            robot.forward_kinematics(0.0, 0.0, 0.0)
