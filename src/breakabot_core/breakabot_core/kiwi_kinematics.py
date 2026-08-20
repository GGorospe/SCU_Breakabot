# File: kiwi_kinematics.py
# Author: George Gorospe, ggorospe@scu.edu
# About: this file defines a class and functions useful for computing the inverse kinematics for a kiwi-drive robot, the breakabot
#
# -----------------------------------------------------------------------------
# Wheel layout and conventions
# -----------------------------------------------------------------------------
# Break-a-bot is a Kiwi drive: a triangular chassis with one omniwheel/motor
# pair at each corner. A FLAT EDGE points forward, so the three wheels sit at
# the following angular positions, measured from +x (forward), CCW positive
# per ROS REP-103 (x forward, y left, z up, +omega counter-clockwise):
#
#       Wheel 0 -> 60 deg   (front-left)
#       Wheel 1 -> 180 deg  (rear)
#       Wheel 2 -> 300 deg  (front-right, = -60 deg)
#
# The inverse-kinematics geometry matrix J1f (below) follows the Siegwart
# rolling-constraint derivation documented in the project slides
# (Siegwart, Nourbakhsh & Scaramuzza, "Introduction to Autonomous Mobile
# Robots", MIT Press, 2011). Each ROW corresponds to one wheel, in the order
# [wheel 0, wheel 1, wheel 2]. Each COLUMN is the contribution of one body
# velocity component, in the order [vx, vy, omega]:
#
#                 vx        vy      omega
#   wheel 0 [  +sqrt(3)/2  -1/2   -l ]
#   wheel 1 [      0       +1     -l ]
#   wheel 2 [  -sqrt(3)/2  -1/2   -l ]
#
# Design decision: the inverse kinematics operates in the robot BODY frame, so
# the global->local rotation R(theta) collapses to the identity matrix and is
# omitted. This means inverse_kinematics() does NOT need the robot's current
# heading and stays stateless. It also aligns with ROS2's Twist message, which
# is conventionally a body-frame velocity command.
#
# Note on 'l': the omega column carries a unit coefficient (-1.0) in the shared
# matrix constant below; the actual robot_radius (l) is applied per-instance at
# call time, since it is a physical dimension, not part of the wheel geometry.
# -----------------------------------------------------------------------------


# Importing Required Libraries
import math

# Shared geometry matrix J1f (dimensionless wheel layout).
# Rows = wheels [0, 1, 2]; columns = body velocity components [vx, vy, omega].
# The omega column uses -1.0 as a unit coefficient; the actual robot_radius (l)
# is multiplied in at call time. Defined once here so both inverse_kinematics
# and forward_kinematics share a single source of truth for the geometry.
J1F = (
    ( math.sqrt(3) / 2, -1.0 / 2, -1.0),   # wheel 0
    ( 0.0,              1.0,      -1.0),   # wheel 1
    (-math.sqrt(3) / 2, -1.0 / 2, -1.0),   # wheel 2
)


class KiwiKinematics:
    def __init__(self, wheel_radius, robot_radius):
        """
        wheel_radius - wheel radius in meters
        robot_radius - center-to-wheel distance in meters
        """
        if wheel_radius <= 0:
            raise ValueError(f"wheel_radius must be positive, got {wheel_radius}")
        if robot_radius <= 0:
            raise ValueError(f"robot_radius must be positive, got {robot_radius}")
        self.wheel_radius = wheel_radius
        self.robot_radius = robot_radius

    # A function for computing the inverse kinematics for the kiwi-drive breakabot
    def inverse_kinematics(self, vx, vy, omega):
        """
        Inputs:
        vx - desired velocity in the robot's x coordinate frame in meters/second
        vy - desired velocity in the robot's y coordinate frame in meters/second
        omega - desired angular velocity in the robots coordinate frame in radians/second

        Returns:
        [w0, w1, w2] - wheel angular velocities in radians/second
        """

        l = self.robot_radius
        inv_r = 1.0 / self.wheel_radius

        # Each wheel speed is the dot product of its J1F row with [vx, vy, omega],
        # with the omega column's unit coefficient scaled by l, all divided by r.
        w0 = inv_r * (J1F[0][0] * vx + J1F[0][1] * vy + J1F[0][2] * l * omega)
        w1 = inv_r * (J1F[1][0] * vx + J1F[1][1] * vy + J1F[1][2] * l * omega)
        w2 = inv_r * (J1F[2][0] * vx + J1F[2][1] * vy + J1F[2][2] * l * omega)
        return [w0, w1, w2]

    # A function for computing the forward kinematics
    def forward_kinematics(self, w0, w1, w2):
        raise NotImplementedError("Phase 3 — forward kinematics for state_vector_node")
