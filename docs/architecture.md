# Break-a-bot software architecture

## Overview

The Break-a-bot (BB) is a Kiwi-drive mobile robot used for developing and
validating diagnostics and prognostics algorithms. The software is built on
ROS2 Jazzy and organized into three packages:

| Package | Role |
|---|---|
| `breakabot_hardware` | Hardware interface nodes (sensors, actuators) |
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

**Package:** `breakabot_hardware`
**File:** `breakabot_hardware/imu_node.py`
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
ros2 run breakabot_hardware imu_node

# In a second terminal — confirm topic is publishing
ros2 topic echo /imu/data

# Check publish rate
ros2 topic hz /imu/data

# Confirm parameters loaded
ros2 param list /imu_node
ros2 param get /imu_node publish_rate_hz

# Override rate at launch (example)
ros2 run breakabot_hardware imu_node --ros-args -p publish_rate_hz:=10.0
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

**Package:** `breakabot_hardware`
**File:** `breakabot_hardware/relay_board_node.py`
**Hardware required:** SunFounder 5V 8-Channel Relay Board via GPIO

#### Purpose

Hardware subscriber node that controls the SunFounder 8-channel relay board.
The relay board physically switches which channel of each RoboteQ SDC2130 motor
controller is connected to its motor. Each motor controller has two channels;
two relays (positive and negative lines) are toggled together to connect the
motor to either channel 1 (default, NO path) or channel 2 (NC path). This
switching capability is the core fault-injection mechanism of the Break-a-Bot
platform — it allows the Test Manager to abruptly reroute or isolate motor
drive circuits in a controlled way.

The node is a pure subscriber: it actuates hardware in response to incoming
commands and publishes its resulting state. It does not poll or generate data
on a timer.

#### Relay-to-motor-controller mapping

| Motor Controller | Positive relay pin | Negative relay pin |
|---|---|---|
| MC1 | GPIO 17 (relay 2) | GPIO 27 (relay 3) |
| MC2 | GPIO 22 (relay 4) | GPIO 23 (relay 5) |
| MC3 | GPIO 24 (relay 6) | GPIO 25 (relay 7) |

Both relays for a given MC are always actuated together to prevent a
positive/negative mismatch.

#### Active-low relay logic

The SunFounder relay board is active-low: driving a GPIO pin LOW energizes the
relay coil. The node uses `gpiod` with `active_low=True` so that the logical
values map cleanly:

| `gpiod` value | Electrical signal | Relay state | Motor connected to |
|---|---|---|---|
| `Value.INACTIVE` | HIGH | De-energized (NO path) | Channel 1 |
| `Value.ACTIVE` | LOW | Energized (NC path) | Channel 2 |

#### Published topics

| Topic | Type | Notes |
|---|---|---|
| `/relay_board/state` | `std_msgs/Int32MultiArray` | QoS: RELIABLE. Three-element array: `[MC1_channel, MC2_channel, MC3_channel]`. Published once on startup and after every accepted command. |

#### Subscribed topics

| Topic | Type | Notes |
|---|---|---|
| `/relay_board/command` | `breakabot_interfaces/msg/RelayCommand` | QoS: RELIABLE. Fields: `motor_controller` (int, 1–3), `channel` (int, 1–2). |

#### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `gpio_pin_relay_2` | `int` | `17` | GPIO pin for MC1 positive relay |
| `gpio_pin_relay_3` | `int` | `27` | GPIO pin for MC1 negative relay |
| `gpio_pin_relay_4` | `int` | `22` | GPIO pin for MC2 positive relay |
| `gpio_pin_relay_5` | `int` | `23` | GPIO pin for MC2 negative relay |
| `gpio_pin_relay_6` | `int` | `24` | GPIO pin for MC3 positive relay |
| `gpio_pin_relay_7` | `int` | `25` | GPIO pin for MC3 negative relay |
| `gpiochip` | `string` | `'gpiochip4'` | gpiod chip device name (RPi 5 default) |
| `require_safe_state` | `bool` | `True` | Reserved for Phase 3 Test Manager interlock |

#### Degraded operation

If `gpiod` is unavailable (laptop development), the node starts in stub mode:
`hw_ready` is set to `False` and `gpio_request` remains `None`. Incoming
commands still update `current_channel` and publish state, but no GPIO lines
are driven. This allows logic and topic testing without hardware. Check the
startup log for `gpiod not available - running in stub mode` to confirm.

#### Guard clauses

The `listener_callback` rejects commands silently (with a logged warning) if:
- `motor_controller` is not in `{1, 2, 3}`
- `channel` is not in `{1, 2}`

Rejected commands leave `current_channel` and published state unchanged.

#### Verification

```bash
# Run the node
ros2 run breakabot_hardware relay_board_node

# In a second terminal — monitor relay state
ros2 topic echo /relay_board/state

# Send a valid command: set MC1 to channel 2
ros2 topic pub --once /relay_board/command breakabot_interfaces/msg/RelayCommand \
  "{motor_controller: 1, channel: 2}"

# Test guard clause: invalid motor controller (should warn, state unchanged)
ros2 topic pub --once /relay_board/command breakabot_interfaces/msg/RelayCommand \
  "{motor_controller: 5, channel: 1}"

# Test guard clause: invalid channel (should warn, state unchanged)
ros2 topic pub --once /relay_board/command breakabot_interfaces/msg/RelayCommand \
  "{motor_controller: 1, channel: 9}"

# Inspect QoS and subscriber count
ros2 topic info /relay_board/command --verbose
ros2 topic info /relay_board/state --verbose
```

#### Unit tests

**File:** `src/breakabot_hardware/test/test_relay_board_node.py`

Tests run without a live ROS2 context or hardware by injecting stubs for
`rclpy`, `gpiod`, and `breakabot_interfaces` before the node module is
imported. The node is loaded directly from its source file path, so no
installed package is required.

Run directly:

```bash
pytest src/breakabot_hardware/test/test_relay_board_node.py -v
```

Or via colcon:

```bash
colcon test --packages-select breakabot_hardware
colcon test-result --verbose
```

| Test class | What is covered |
|---|---|
| `TestRelayBoardNodeInit` | Startup state: `current_channel` defaults, `hw_ready`/`gpio_request` consistency, default pin mapping, initial state publish |
| `TestListenerCallbackValidCommands` | Valid commands update the correct MC, leave others unchanged, and publish updated state |
| `TestListenerCallbackInvalidCommands` | Out-of-range MC numbers and channels are rejected with a warning; state and publish count are unchanged |
| `TestPublishState` | `publish_state()` always emits a 3-element array reflecting the current `current_channel` dict |
| `TestDestroyNode` | `gpio_request.release()` is called on shutdown when lines are held; no exception when `gpio_request` is `None` |

Two tests are written to pass in both environments:

- `test_gpio_request_matches_hw_state` — asserts `gpio_request is not None` on
  the RPi (where `gpiod` succeeds) and `gpio_request is None` on a laptop
  (where `gpiod` is absent), branching on `node.hw_ready`.
- `test_destroy_safe_when_no_gpio_request` — asserts no exception is raised
  regardless of whether `gpio_request` holds a real `LineRequest` or `None`.

#### Known limitations / future work

- The safety interlock (`require_safe_state`) is declared as a parameter but
  not yet enforced. Connection to the Test Manager state is planned for Phase 3.
- Relay 1 (index 0) on the 8-channel board is unused; only relays 2–7 are
  wired. Document physical connector pinout when hardware is finalized.
- `destroy_node()` releases GPIO lines but does not explicitly set them to a
  safe state (all de-energized) before release. Consider driving all pins to
  `Value.INACTIVE` before calling `release()`.

---

### `roboteq_node`

**Package:** `breakabot_hardware`
**File:** `breakabot_hardware/roboteq_node.py`
**Hardware required:** 3× RoboteQ SDC2130 motor controllers via USB

#### Purpose

Hardware interface node for the three RoboteQ SDC2130 dual-channel brushed DC
motor controllers that drive the Kiwi-drive platform. Each controller connects
to the Raspberry Pi 5 via a dedicated USB cable. The node manages all three
controllers as a unified group: it receives a single 6-element motor command
array, fans commands out to each controller over serial, and polls each
controller periodically for encoder counts, motor current, and battery voltage.

The node operates in one of two control modes selected by parameter:

- **open_loop** (default, Phase 2): sends `!G` commands (power level ±1000).
  The upstream `rpm_scale_node` outputs G-units derived from body velocity.
- **closed_loop** (Phase 3+): sends `!S` commands (RPM setpoint). Requires
  encoder wiring complete and SDC2130 firmware configured for closed-loop speed
  mode via the Roborun+ PC utility. The SDC2130 runs its own internal PID loop
  at 1 kHz — no PID logic exists in this node.

Switching modes requires only a parameter change in `roboteq_params.yaml` and
a node restart. No code changes are needed.

#### Watchdog keepalive

The SDC2130 serial watchdog (default timeout: 1 second) stops motors if no
motor command is received within the timeout window. Critically, query commands
(`?C`, `?V`, `?A`) do **not** reset the watchdog — only motor commands (`!G`,
`!S`) do. The fast poll callback therefore re-sends the last known command to
each controller every cycle, keeping the watchdog satisfied regardless of
whether new `motor_cmd` messages are arriving upstream. The last known command
is initialised to all-zeros at startup so the first keepalive is always safe.

#### USB port identification

RoboteQ controllers appear as `/dev/ttyACMx` by default, but that index shifts
on reboot. The node uses stable `/dev/serial/by-id/` symlinks instead, which
encode each controller's USB serial number and never change.

To discover port paths in the lab:

```bash
ls -l /dev/serial/by-id/ | grep -i roboteq
```

Unplug one controller at a time to map each serial number to a physical
controller. Record the mapping in the table in `roboteq_params.yaml` and in
the Phase 5 sign-off row of `docs/roboteq_node_test_plan.md`.

#### Command layout convention

The 6-element `Int32MultiArray` on `/roboteq/motor_cmd` maps to controllers
and channels as follows:

```
[c0_ch1, c0_ch2, c1_ch1, c1_ch2, c2_ch1, c2_ch2]
```

Controller numbering (0–2) is assigned by physical position on the robot and
recorded during Phase 1 of the lab test plan. Commands arriving with any length
other than 6 are logged as warnings and discarded.

#### Thread safety

`cmd_callback` (subscriber) and `_fast_poll_callback` (timer) run in separate
threads and both write to the same serial ports. Each controller has a dedicated
`threading.Lock()` that is acquired before any read or write and released
immediately after, preventing interleaved bytes on the wire.

#### Published topics

| Topic | Type | Rate | Notes |
|---|---|---|---|
| `/roboteq/encoder_counts` | `std_msgs/Int32MultiArray` | `fast_poll_rate_hz` | 6-element array: `[c0_ch1, c0_ch2, c1_ch1, c1_ch2, c2_ch1, c2_ch2]`. Absolute 32-bit signed counts from `?C` query. |
| `/roboteq/motor_current` | `std_msgs/Float32MultiArray` | `fast_poll_rate_hz` | 6-element array, same layout. Motor amps converted from `?A` reply (Amps×10 → Amps). |
| `/roboteq/battery_state` | `sensor_msgs/BatteryState` | `battery_poll_rate_hz` | Battery voltage averaged across all three controllers from `?V` reply. |

#### Subscribed topics

| Topic | Type | Notes |
|---|---|---|
| `/roboteq/motor_cmd` | `std_msgs/Int32MultiArray` | 6-element array. G-units (±1000) in open_loop mode; RPM targets in closed_loop mode. |

#### Parameters

**File:** `breakabot_bringup/config/roboteq_params.yaml`

| Parameter | Type | Default | Description |
|---|---|---|---|
| `port_0` | `string` | placeholder | USB by-id path for controller 0 |
| `port_1` | `string` | placeholder | USB by-id path for controller 1 |
| `port_2` | `string` | placeholder | USB by-id path for controller 2 |
| `baud_rate` | `int` | `115200` | Serial baud rate (SDC2130 maximum) |
| `control_mode` | `string` | `'open_loop'` | `'open_loop'` or `'closed_loop'` |
| `invert_ch1` | `bool[]` | `[false, false, false]` | Per-controller channel 1 sign inversion |
| `invert_ch2` | `bool[]` | `[false, false, false]` | Per-controller channel 2 sign inversion |
| `fast_poll_rate_hz` | `double` | `10.0` | Rate for encoder, current, and watchdog keepalive. Must exceed 1 Hz (watchdog timeout). Raise to ~50 Hz for diagnostic routines. |
| `battery_poll_rate_hz` | `double` | `1.0` | Rate for battery voltage polling |
| `serial_timeout` | `double` | `0.1` | Serial read timeout in seconds |

#### Serial protocol reference

| Command | Direction | Format | Description |
|---|---|---|---|
| `!G cc nn` | Write | `!G 1 500_!G 2 -300_` | Open-loop power, cc=channel, nn=±1000 |
| `!S cc nn` | Write | `!S 1 120_!S 2 90_` | Closed-loop RPM setpoint |
| `!MS cc` | Write | `!MS 1_!MS 2_` | Stop motor on channel cc (mode-independent) |
| `?C` | Query | Reply: `C=ch1:ch2` | Absolute encoder counts (32-bit signed) |
| `?A` | Query | Reply: `A=ch1×10:ch2×10` | Motor amps ×10 |
| `?V` | Query | Reply: `V=internal×10:battery×10:5Vout_mV` | Voltages |

Commands use `_` as a carriage-return shorthand, allowing two commands to be
concatenated in one serial write. The controller ACKs each command with `+\r`.

#### Verification

```bash
# Run the node
ros2 run breakabot_hardware roboteq_node \
  --ros-args --params-file ~/breakabot_ws/src/breakabot_bringup/config/roboteq_params.yaml

# Monitor encoder counts
ros2 topic echo /roboteq/encoder_counts

# Monitor motor current
ros2 topic echo /roboteq/motor_current

# Monitor battery voltage
ros2 topic echo /roboteq/battery_state

# Confirm publish rate
ros2 topic hz /roboteq/encoder_counts

# Send a test command — controller 0 ch1 at 20% power, all others stopped
ros2 topic pub --once /roboteq/motor_cmd std_msgs/msg/Int32MultiArray \
  "data: [200, 0, 0, 0, 0, 0]"

# Stop all motors
ros2 topic pub --once /roboteq/motor_cmd std_msgs/msg/Int32MultiArray \
  "data: [0, 0, 0, 0, 0, 0]"
```

See `docs/roboteq_node_test_plan.md` for the full 5-phase lab test procedure
covering USB identification, serial verification, sensor polling, single-channel
motor tests, and channel inversion determination.

#### Unit tests

**File:** `src/breakabot_hardware/test/test_roboteq_node.py`
**Runner:** `src/breakabot_hardware/test/conftest.py`

Tests use the real `rclpy` (requiring `rclpy.init()` via a session-scoped
pytest fixture in `conftest.py`) and mock `serial.Serial` with
`unittest.mock.MagicMock`. Publisher assertions use `patch.object` on the
`publish` method rather than inspecting a `last_msg` attribute, since the real
`rclpy.Publisher` sends to the ROS2 graph rather than storing messages locally.

Run directly:

```bash
cd ~/breakabot_ws
pytest src/breakabot_hardware/test/test_roboteq_node.py -v
```

Or via colcon:

```bash
colcon test --packages-select breakabot_hardware
colcon test-result --verbose
```

| Test class | What is covered |
|---|---|
| `TestParseTwoInts` | Normal values, negative values, zero, large 32-bit counts, empty string, missing colon, non-numeric, extra fields, whitespace tolerance |
| `TestLastCmdInit` | Correct command character (`G`/`S`) per mode, one entry per controller, zero initialisation |
| `TestCmdCallback` | Correct command string formatting for open/closed loop; all three controllers updated; negative values; zero; clamping above and below ±1000; boundary values; ch1 and ch2 inversion per controller; both channels inverted; inversion of zero; inversion applied before clamp; wrong-length array ignored (3, 0, 7 elements); `last_cmd` updated on valid command; `last_cmd` unchanged on invalid command |
| `TestSendCommand` | Correct bytes written to correct port; two ACK reads consumed after write; lock acquired and released; `SerialException` does not propagate |
| `TestSendQuery` | Valid reply prefix stripped; voltage multi-field reply; wrong prefix returns empty string; empty reply returns empty string; `SerialException` returns empty string; input buffer reset before query; query bytes written to port |
| `TestFastPollCallback` | Keepalive sent to all controllers; encoder counts published; motor current published; encoder values correctly parsed from `?C` reply; current converted from ×10 units; failed query (all-timeout) publishes zeros |
| `TestBatteryPollCallback` | `BatteryState` published on valid replies; voltage parsed correctly from `?V`; voltage averaged across three controllers; one malformed reply does not crash, others averaged; all replies failed — nothing published |
| `TestDestroyNode` | `!MS` stop command sent to all controllers; ports closed after stop; stop precedes close on each controller; port close error does not prevent shutdown of remaining controllers |

59 tests total. Runtime: ~2.5 seconds.

#### Design decisions

| Decision | Choice | Rationale |
|---|---|---|
| Single node for 3 controllers | One `roboteq_node` manages all three | Simpler launch, atomic shutdown, single serial lock policy |
| Controller storage | `self.controllers` list of dicts `{port, lock}` | Loop-based operations; adding/removing a controller is a yaml change, not a code change |
| Command topic layout | `[c0_ch1, c0_ch2, c1_ch1, c1_ch2, c2_ch1, c2_ch2]` | Controller-pairs make fan-out indexing `base = i * 2` straightforward |
| USB port paths | `/dev/serial/by-id/` symlinks | Stable across reboots; `ttyACMx` index shifts when controllers are plugged in different order |
| Watchdog keepalive | Re-send `last_cmd` every fast poll cycle | Queries do not reset the SDC2130 watchdog; keepalive is explicit and visible rather than an emergent property of upstream publish rate |
| `last_cmd` initialised to zero | `!G 1 0_!G 2 0_` per controller | First keepalive before any upstream command is always safe |
| Two poll timers | `fast_poll_rate_hz` for encoder/current/keepalive; `battery_poll_rate_hz` for voltage | Different diagnostic needs; decoupling prevents a slow voltage read from blocking encoder data |
| Control mode as parameter | `control_mode: open_loop / closed_loop` | Switching from open to closed loop requires a yaml edit and restart — no code change |
| Inversion as parameter lists | `invert_ch1: [false, false, false]` | Determined in lab during Phase 5; survivable across code updates |
| Inversion before clamp | Sign applied, then `max(-1000, min(1000, val))` | Prevents an inverted large value from escaping the valid range |
| `!MS` on shutdown | `!MS 1_!MS 2_` to each controller | Mode-independent stop; works even if the watchdog has already fired |
| `rpm_scale_node` separation | Conversion from RPM → G-units in a separate node | `roboteq_node` speaks the controller's native language only; closed-loop upgrade swaps `rpm_scale_node` without touching this node |

#### Known limitations / future work

- USB port placeholder paths in `roboteq_params.yaml` must be replaced with
  real by-id paths after lab identification (Phase 1 of test plan).
- Channel inversion parameters are all `false` pending lab verification
  (Phase 5 of test plan).
- Motor max RPM (`max_rpm`) for the Pittman F5019 is undetermined (OEM winding,
  no public datasheet). Measure unloaded RPM from motor label or via encoder
  count in the lab and update `max_rpm` in `rpm_scale_params.yaml`.
- In closed-loop mode, the SDC2130 must be configured via the Roborun+ PC
  utility before switching `control_mode`. See Phase 3 checklist in
  `docs/progress.md`.
- The `require_safe_state` interlock (connection to `test_manager_node`) is
  not yet implemented. Planned for Phase 3.
- Individual per-controller battery voltages are not published separately.
  The `BatteryState` message contains only the average. Individual readings
  are available via the Roborun+ PC utility or by temporarily enabling debug
  logging.
- SDC2130 stream mode (`# C` command) could replace the polled `?C` approach
  for encoder data above ~100 Hz. Relevant if diagnostic routines require
  very high encoder sample rates.

---

### `rpm_scale_node`

**Package:** `breakabot_hardware`
**File:** `breakabot_hardware/rpm_scale_node.py`
**Hardware required:** None

#### Purpose

Translates wheel speed targets in RPM (published by `kinematics_node`) into
native G-unit commands (±1000 scale) consumed by `roboteq_node`. The RoboteQ
SDC2130 firmware accepts power levels as integers in the range −1000 to +1000;
this node owns the conversion so neither neighbor has to know about the other's
units.

In Phase 2 (open-loop) the conversion is:

```
g_value = int((rpm_target / max_rpm) * 1000)
g_value = clamp(g_value, -1000, 1000)
```

In Phase 3 (closed-loop) the node will use encoder feedback from
`roboteq_node` to run a per-channel PID controller and output corrected G-unit
commands. The Phase 3 code path is stubbed out — switching to it requires only
setting `control_mode: closed_loop` in the params yaml and filling in
`_closed_loop_update()`. No topic interface changes are needed.

#### Subscribed topics

| Topic | Type | Notes |
|---|---|---|
| `/roboteq/rpm_cmd` | `std_msgs/Int32MultiArray` | 6-element RPM targets: `[c0_ch1, c0_ch2, c1_ch1, c1_ch2, c2_ch1, c2_ch2]`. Messages with wrong array length are logged as errors and dropped. |
| `/roboteq/encoder_counts` | `std_msgs/Int32MultiArray` | 6-element encoder counts. Stored for Phase 3 PID use. Ignored in Phase 2. |

#### Published topics

| Topic | Type | Notes |
|---|---|---|
| `/roboteq/motor_cmd` | `std_msgs/Int32MultiArray` | 6-element G-unit commands (±1000), same array layout as input. |

#### Parameters

**File:** `breakabot_bringup/config/rpm_scale_params.yaml`

| Parameter | Type | Default | Description |
|---|---|---|---|
| `control_mode` | `string` | `'open_loop'` | `'open_loop'` (Phase 2) or `'closed_loop'` (Phase 3). If `closed_loop` is set before Phase 3 is implemented, the node logs a warning and falls through to open-loop. |
| `max_rpm` | `double` | `3000.0` | No-load RPM of the Pittman LO-COG F5019. **Placeholder** — update from motor label in lab. 3000.0 is a conservative estimate for a 24 V Pittman 5000-series winding; under-estimating produces slower-than-commanded motion (safe failure mode). A startup warning fires whenever the placeholder is still in use. |
| `kp` | `double` | `1.0` | PID proportional gain. Phase 3 only. |
| `ki` | `double` | `0.0` | PID integral gain. Phase 3 only. |
| `kd` | `double` | `0.0` | PID derivative gain. Phase 3 only. |
| `encoder_ppr` | `int` | `512` | Encoder pulses per revolution. Phase 3 only. Update from encoder spec or measured count. |

#### Verification

```bash
# Run the node
ros2 run breakabot_hardware rpm_scale_node \
  --ros-args --params-file ~/breakabot_ws/src/breakabot_bringup/config/rpm_scale_params.yaml

# In a second terminal — confirm motor_cmd is publishing
ros2 topic echo /roboteq/motor_cmd

# Inject a test RPM command (max_rpm=3000 default → 500 RPM = G-unit 166)
ros2 topic pub --once /roboteq/rpm_cmd std_msgs/msg/Int32MultiArray \
  "data: [500, 0, 0, 0, 0, 0]"

# Verify clamping — value above max_rpm should saturate at 1000
ros2 topic pub --once /roboteq/rpm_cmd std_msgs/msg/Int32MultiArray \
  "data: [9999, 0, 0, 0, 0, 0]"

# Confirm parameters loaded
ros2 param list /rpm_scale_node
ros2 param get /rpm_scale_node max_rpm
```

#### Unit tests

**File:** `src/breakabot_hardware/test/test_rpm_scale_node.py`
**Runner:** `src/breakabot_hardware/test/conftest.py`

Tests use the real `rclpy` (requiring `rclpy.init()` via a session-scoped
pytest fixture in `conftest.py`) and `patch.object` for publisher assertions.
No hardware dependencies — the node is pure math with no serial or GPIO calls.

Run directly:

```bash
cd ~/breakabot_ws
pytest src/breakabot_hardware/test/test_rpm_scale_node.py -v
```

| Test class | What is covered |
|---|---|
| `TestOpenLoopConvert` | Zero RPM, full forward/reverse, half speed, over/under limit clamping, mixed channels, integer truncation, fractional truncation, midpoint at default `max_rpm` |
| `TestRpmCmdCallback` | Valid message publishes `Int32MultiArray`; all-zeros, full forward, full reverse; over-limit clamping; output type and element types verified |
| `TestInvalidMessages` | Too-few elements dropped; too-many elements dropped; empty array dropped; encoder callback with wrong length does not update stored counts |
| `TestControlMode` | `closed_loop` still publishes; closed-loop fallthrough produces same G-values as open-loop |
| `TestEncoderStorage` | Encoder counts stored on valid message; overwritten on subsequent message |

25 tests total.

#### Design decisions

| Decision | Choice | Rationale |
|---|---|---|
| Dedicated node for conversion | `rpm_scale_node` separate from `kinematics_node` and `roboteq_node` | Neither neighbor needs to know the other's unit system; closed-loop upgrade replaces only this node's internal logic |
| Phase 2 → Phase 3 path | Parameter change + fill in one function; no interface changes | `_closed_loop_update()` stub is wired into the callback today; topic layout is frozen |
| `max_rpm` default of 3000.0 | Conservative estimate for 24 V Pittman 5000-series | Under-estimating produces slower motion (safe); over-estimating risks over-driving motors |
| Clamp after conversion | `max(-1000, min(1000, g))` applied to result | Hardware safety gate independent of upstream RPM range |
| Log-and-drop on wrong array length | Error logged; message discarded; no crash | Preserves last valid motor state; avoids silent zero-commands to unintended channels |
| `_open_loop_convert()` as pure function | Takes list, returns list; no ROS types | Core math is independently unit-testable without any ROS mocking |
| Closed-loop fallthrough | Warns every 5 s; passes through to open-loop | Safe default for Phase 2; makes the gap visible in logs without crashing |

#### Known limitations / future work

- `max_rpm` placeholder (3000.0) must be replaced with the measured no-load RPM
  of the Pittman F5019 motors after lab visit. A startup warning fires as a
  reminder.
- Phase 3 `_closed_loop_update()` is a stub. Implement PID logic using
  `self._latest_encoder_counts`, `self._encoder_ppr`, and per-channel error
  integration when Phase 3 begins.
- PID gains (`kp`, `ki`, `kd`) are placeholder defaults. Tuning procedure:
  raise `kp` until oscillation, then add `ki` to eliminate steady-state error.

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
| `roboteq_node` | 3× RoboteQ SDC2130 via USB serial | Lab only |
| `rpm_scale_node` | None | Yes |
| `kinematics_node` | None | Yes |
| `test_manager_node` | None | Yes |
| `trajectory_node` | None (sim: turtlesim) | Yes |
| `state_vector_node` | None | Yes |
