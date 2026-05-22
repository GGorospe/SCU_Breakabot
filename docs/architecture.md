# Break-a-bot software architecture

## Overview

The Break-a-bot (BB) is a Kiwi-drive mobile robot used for developing and
validating diagnostics and prognostics algorithms. The software is built on
ROS2 Jazzy and organized into three packages:

| Package | Role |
|---|---|
| `breakabot_hw` | Hardware interface nodes (sensors, actuators) |
| `breakabot_core` | Control, planning, and analysis nodes |
| `breakabot_bringup` | Launch files and configuration |

The system is designed around two test scenarios:
1. **Teleoperated fault injection** — operator drives via Xbox controller;
   Test Manager injects a motor fault at a preset time.
2. **Autonomous fault injection** — robot traces a 1.5 m square repeatedly;
   Test Manager injects a motor fault at a preset time.

Sensor data is recorded via `ros2 bag` for offline diagnostic prototyping.

---

## ROS2 node reference

---

### `imu_node`

**Package:** `breakabot_hw`
**File:** `breakabot_hw/imu_node.py`
**Hardware required:** Adafruit BNO055 IMU via I2C

#### Purpose

Reads orientation, angular velocity, and linear acceleration from the BNO055
IMU and publishes them as a standard `sensor_msgs/Imu` message. This data
feeds the Position Estimation Node (dead reckoning) and the State Vector Node
(diagnostic input).

#### Published topics

| Topic | Type | Rate | Notes |
|---|---|---|---|
| `/imu/data` | `sensor_msgs/Imu` | 50 Hz (default) | QoS: BEST_EFFORT |

#### Subscribed topics

None.

#### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `publish_rate_hz` | `double` | `50.0` | Timer callback rate |
| `frame_id` | `string` | `'imu_link'` | Coordinate frame for message header |
| `i2c_address` | `int` | `0x28` | BNO055 I2C address (alt: `0x29`) |

#### Coordinate frame

This node publishes in the `imu_link` frame. A static transform from
`base_link` → `imu_link` must be broadcast by `breakabot_bringup` to make
IMU data usable in the robot body frame. See `bringup/launch/all_nodes.launch.py`.

#### Degraded operation

If hardware libraries are unavailable (laptop development) or the BNO055
fails to initialize, the node starts in stub mode: the timer fires but
publishes nothing. This allows ROS2 graph and parameter testing without
hardware. Check the startup log for `hw: UNAVAILABLE` to confirm stub mode.

#### BNO055 field mapping

The BNO055 quaternion output order is `(w, x, y, z)`.
`sensor_msgs/Imu` stores `(x, y, z, w)`. The node remaps these explicitly.

`linear_acceleration` (gravity-compensated) is used rather than `acceleration`
(raw). The BNO055 fusion algorithm removes the gravity vector onboard.

Covariance matrices are published as all-zeros, indicating unknown covariance
per ROS2 convention. Update when sensor characterization data is available.

#### Verification

```bash
# Run the node
ros2 run breakabot_hw imu_node

# In a second terminal — confirm topic is publishing
ros2 topic echo /imu/data

# Check publish rate
ros2 topic hz /imu/data

# Confirm parameters loaded
ros2 param list /imu_node
ros2 param get /imu_node publish_rate_hz

# Override rate at launch (example)
ros2 run breakabot_hw imu_node --ros-args -p publish_rate_hz:=10.0
```

#### Known limitations / future work

- Covariance matrices are unpopulated. Characterization data needed.
- Static transform values (`base_link` → `imu_link`) are placeholder zeros
  pending physical measurement on the robot.
- Calibration status from the BNO055 (system, gyro, accel, mag) is not
  currently published. Consider adding a `/imu/calibration_status` topic.

---

<!-- ── Node entries below this line are stubs — fill in as implemented ── -->

### `relay_board_node`

**Package:** `breakabot_hw`
**File:** `breakabot_hw/relay_board_node.py`
**Status:** STUB node

---

### `roboteq_node`

**Package:** `breakabot_hw`
**File:** `breakabot_hw/roboteq_node.py`
**Status:** STUB node

---

### `kinematics_node`

**Package:** `breakabot_core`
**Status:** Not yet implemented

---

### `test_manager_node`

**Package:** `breakabot_core`
**Status:** Not yet implemented

---

### `trajectory_node`

**Package:** `breakabot_core`
**Status:** Not yet implemented

---

### `state_vector_node`

**Package:** `breakabot_core`
**Status:** Not yet implemented

---

## tf2 frame tree

```
odom
 └── base_link
      └── imu_link
```

Static transforms are broadcast by `breakabot_bringup/launch/all_nodes.launch.py`.
Measured offsets (base_link → imu_link) to be updated after lab measurement.

---

## Topic map

*To be filled in as nodes are implemented.*

---

## Test scenarios

### Scenario 1 — Teleoperated fault injection

*To be documented.*

### Scenario 2 — Autonomous square with fault injection

*To be documented.*

---

## Hardware dependencies by node

| Node | Hardware | Available at home |
|---|---|---|
| `imu_node` | BNO055 via I2C | Yes |
| `relay_board_node` | 8-ch relay board via GPIO | Yes |
| `roboteq_node` | Roboteq SDC2130 via serial | Lab only |
| `kinematics_node` | None | Yes |
| `test_manager_node` | None | Yes |
| `trajectory_node` | None (sim: turtlesim) | Yes |
| `state_vector_node` | None | Yes |
