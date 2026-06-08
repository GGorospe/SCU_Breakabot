# Break-a-bot Development Progress

**Project:** Break-a-bot — Fault-tolerant mobile robotics platform  
**Researcher:** George Gorospe, ggorospe@scu.edu  
**Advisor:** Dr. Chris Kitts, Santa Clara University Robotic Systems Lab  
**Repo:** GGorospe/SCU_Breakabot  
**ROS2 distro:** Jazzy on Ubuntu 24.04, Raspberry Pi 5  
**Tagline:** "Demonstrating Diagnostics for Mobile Robotics"

---

## Overall project phase

Currently in **Phase 2 — Hardware interfaces**.
Phase 1 (environment and skeleton packages) is complete.

---

## Package structure

```
breakabot_ws/
├── docs/
│   ├── architecture.md       # Node reference, topic map, tf2 tree
│   ├── progress.md           # This file
│   ├── quickstart.md         # Not yet written
│   └── roboteq_node_test_plan.md  # 5-phase lab test procedure for roboteq_node
└── src/
    ├── breakabot_interfaces/  # Custom message definitions (ament_cmake)
    ├── breakabot_hardware/    # Hardware interface nodes (ament_python)
    ├── breakabot_core/        # Control, planning, analysis (ament_python)
    └── breakabot_bringup/     # Launch files and config (ament_python)
```

---

## Node status

| Node | Package | Status | Hardware available at home |
|---|---|---|---|
| `imu_node` | `breakabot_hardware` | ✅ Complete, tested | Yes (BNO055) |
| `relay_board_node` | `breakabot_hardware` | ✅ Complete, tested | Yes (relay board) |
| `roboteq_node` | `breakabot_hardware` | 🔶 Code complete, untested (lab only) | Lab only |
| `rpm_scale_node` | `breakabot_hardware` | ⬜ Not started | Yes (no hardware needed) |
| `kinematics_node` | `breakabot_core` | ⬜ Not started | Yes (no hardware needed) |
| `test_manager_node` | `breakabot_core` | ⬜ Not started | Yes (no hardware needed) |
| `trajectory_node` | `breakabot_core` | ⬜ Not started | Yes (turtlesim) |
| `state_vector_node` | `breakabot_core` | ⬜ Not started | Yes (no hardware needed) |
| `data_logger_node` | `breakabot_core` | ⬜ Not started | Yes (no hardware needed) |

---

## Completed this session

### Phase 1
- ROS2 workspace created at `~/breakabot_ws/`
- Three packages scaffolded: `breakabot_hardware`, `breakabot_core`,
  `breakabot_bringup`
- Stub nodes confirmed building and appearing in `ros2 node list`
- `.gitignore` configured (excludes `build/`, `install/`, `log/`)
- `docs/architecture.md` created with node reference template

### imu_node (complete)
- Publishes `sensor_msgs/Imu` on `/imu/data` at 50 Hz (configurable)
- Parameters: `publish_rate_hz`, `frame_id`, `i2c_address`
- Graceful stub mode when Adafruit libraries unavailable (laptop dev)
- BNO055 quaternion order remapped: sensor returns `(w,x,y,z)`,
  `sensor_msgs/Imu` expects `(x,y,z,w)` — handled explicitly
- Uses `linear_acceleration` (gravity-compensated) not raw `acceleration`
- Quaternion norm sanity check in callback
- 12 unit tests passing, 1 skipped (stub mode test, runs on laptop)
- Hardware verified on Raspberry Pi 5 with BNO055 via I2C

### breakabot_interfaces package
- Created as `ament_cmake` package (required for message generation)
- `RelayCommand.msg` defined with two `uint8` fields:
  - `motor_controller` (1, 2, or 3)
  - `channel` (1 = normally open, 2 = normally closed)
- Built and verified with `ros2 interface show`

### relay_board_node (complete)
- Debugged and fixed misindented `return` statements in `listener_callback` —
  both guard clauses were exiting unconditionally, silently dropping every command
- Verified fix on Raspberry Pi 5: all three MCs successfully commanded to channel 2;
  invalid MC number (5) correctly logged a warning and left state unchanged
- Confirmed `/relay_board/state` updates correctly via `ros2 topic echo`
- QoS mismatch identified: `ros2 topic pub` defaulted to `BEST_EFFORT` while
  node subscriber uses `RELIABLE`; resolved by using `--once` flag with correct YAML syntax
- 24 unit tests written and passing (`test_relay_board_node.py`):
  - Tests mock `rclpy`, `gpiod`, and `breakabot_interfaces` — no hardware or
    live ROS2 context required
  - Node loaded directly from source file path via `importlib` — no installed
    package required
  - Two tests updated to be environment-aware, branching on `node.hw_ready`:
    - `test_gpio_request_matches_hw_state` — asserts `gpio_request is not None`
      on Pi (gpiod available), `None` on laptop
    - `test_destroy_safe_when_no_gpio_request` — asserts no exception regardless
      of gpio_request state
  - `test_hw_not_ready_without_gpiod` removed — redundant with
    `test_gpio_request_matches_hw_state` after environment-aware refactor
- Test file placed at `src/breakabot_hardware/test/test_relay_board_node.py`;
  loader path updated from `__file__.parent` to `__file__.parent.parent / 'breakabot_hardware'`
- `docs/architecture.md` relay_board_node entry fully written (was stub):
  - Purpose, relay-to-MC mapping table, active-low logic table
  - Published/subscribed topics with QoS notes
  - Full parameter table
  - Degraded operation and guard clause behavior
  - Verification commands
  - Unit test summary table with environment-aware test callouts
  - Known limitations / future work

### roboteq_node (code complete, lab testing pending)

#### Architecture decisions made
- **Single node manages all three SDC2130 controllers** — unified startup,
  shutdown, and lock policy; adding/removing a controller is a yaml change
- **Command topic layout** — `[c0_ch1, c0_ch2, c1_ch1, c1_ch2, c2_ch1, c2_ch2]`,
  fan-out indexing via `base = i * 2`
- **USB port paths** — `/dev/serial/by-id/` symlinks, stable across reboots;
  `ttyACMx` indices shift when controllers plug order changes
- **Two poll timers** — `fast_poll_rate_hz` (encoder/current/watchdog keepalive,
  default 10 Hz, raisable to ~50 Hz for diagnostics) and `battery_poll_rate_hz`
  (voltage only, default 1 Hz); decoupled so voltage reads don't block encoder data
- **Watchdog keepalive** — `last_cmd` re-sent every fast poll cycle; SDC2130
  watchdog resets only on motor commands, not queries; keepalive is explicit
  rather than an emergent property of upstream publish rate
- **Control mode as parameter** — `open_loop` sends `!G` (power ±1000);
  `closed_loop` sends `!S` (RPM setpoint); switching requires yaml edit +
  restart, no code change
- **Inversion before clamp** — channel sign inversion applied before
  `max(-1000, min(1000, val))` to prevent out-of-range escape
- **`!MS` on shutdown** — mode-independent stop command; works even if
  watchdog has already fired
- **`rpm_scale_node` as separate node** — `roboteq_node` speaks the
  controller's native language only; RPM→G-unit conversion and future
  closed-loop PID live upstream; closed-loop upgrade swaps `rpm_scale_node`
  without touching `roboteq_node`
- **Control mode selection owned by `test_manager_node`** — `kinematics_node`
  will gate between teleoperation and autonomous command sources based on a
  mode topic from `test_manager_node`; `roboteq_node` is unaware of mode

#### Node design
- **Subscribed:** `/roboteq/motor_cmd` (`Int32MultiArray`, 6 elements)
- **Published:** `/roboteq/encoder_counts` (`Int32MultiArray`),
  `/roboteq/motor_current` (`Float32MultiArray`),
  `/roboteq/battery_state` (`BatteryState`)
- **Serial protocol:** `!G`/`!S` commands, `?C`/`?A`/`?V` queries; `_`
  used as carriage-return shorthand to concatenate two commands per serial write
- **Thread safety:** one `threading.Lock()` per controller, acquired for all
  serial reads and writes
- **Startup failure:** raises immediately if any serial port fails to open;
  placeholder paths in yaml cause an immediate `SerialException` with a
  helpful hint logged

#### Files created
- `breakabot_hardware/roboteq_node.py` — full node implementation
- `breakabot_bringup/config/roboteq_params.yaml` — parameter file with
  placeholder port paths, inversion flags, poll rates, and inline documentation
- `breakabot_bringup/setup.py` — updated with `import os` and
  `from glob import glob` (were missing; caused `NameError` at build time);
  `config/*.yaml` glob already present
- `docs/roboteq_node_test_plan.md` — 5-phase lab test plan:
  - Phase 1: USB identification and port mapping
  - Phase 2: Serial connection verification
  - Phase 3: Sensor polling verification (no motors powered)
  - Phase 4: Single-channel motor tests (one channel at a time)
  - Phase 5: Full 6-channel test and inversion record
- `docs/architecture.md` — `roboteq_node` entry fully written (was stub):
  purpose, watchdog section, USB identification, command layout, thread safety,
  topics, parameters, serial protocol reference, verification commands,
  unit test summary, design decisions table, known limitations

#### Unit tests
- 59 tests written in `src/breakabot_hardware/test/test_roboteq_node.py`
- `src/breakabot_hardware/test/conftest.py` created with:
  - `sys.path` injection so pytest finds `roboteq_node.py` without a build step
  - `launch_testing` plugin interference suppressed via `collect_ignore_glob`
  - Session-scoped `rclpy_session` fixture (`rclpy.init()` / `rclpy.shutdown()`)
    required because the real rclpy is present on the Pi (unlike the sandbox
    where stub rclpy was used during development)
- Publisher assertions use `patch.object(node.encoder_pub, 'publish')` rather
  than `last_msg` attribute — real `rclpy.Publisher` sends to the ROS2 graph
  and has no local storage
- All 59 tests passing on Raspberry Pi 5; runtime ~2.5 seconds

---

## Open issues / current blockers

### 1. GPIO pins not yet assigned to permanent values
Relay board wiring is complete and functional, but pin assignments are
currently using the default parameter values. Once finalized, document
real pin assignments in `test_params.yaml`.

### 2. `destroy_node()` does not set safe state before GPIO release
`relay_board_node.destroy_node()` calls `gpio_request.release()` without
first driving all pins to `Value.INACTIVE`. If the node crashes while relays
are energized, they will remain energized until the OS releases the lines.
Fix planned before Phase 3 integration.

### 3. `require_safe_state` interlock not yet enforced
The parameter is declared and logged but the guard clause is commented out
in `relay_board_node`. Connection to Test Manager state is planned for Phase 3.
The same interlock is planned but not yet implemented in `roboteq_node`.

### 4. `roboteq_node` USB port paths are placeholders
`roboteq_params.yaml` contains placeholder by-id paths. Real paths must be
determined in the lab (Phase 1 of `docs/roboteq_node_test_plan.md`) and
committed to the repo before the node can be launched.

### 5. Channel inversion parameters undetermined
`invert_ch1` and `invert_ch2` in `roboteq_params.yaml` are all `false`.
Correct values depend on motor mounting and wiring direction, determined
during Phase 5 of the lab test plan.

### 6. Motor max RPM undetermined
The Pittman F5019 motor max RPM must be read from the motor label in the lab.
This value is required before `rpm_scale_node` can be written and before
closed-loop mode can be tuned.

---

## Next steps (ordered)

1. **Lab session — `roboteq_node` testing** — follow `docs/roboteq_node_test_plan.md`:
   identify USB port paths, verify serial connections, test sensor polling,
   run single-channel motor tests, determine and record channel inversions
2. **Update `roboteq_params.yaml`** — replace placeholder port paths and
   inversion flags with values determined in lab; commit
3. **Read F5019 motor label** — record max RPM; required for `rpm_scale_node`
4. **Fix `relay_board_node.destroy_node()` safe state** — drive all relay
   pins to `Value.INACTIVE` before calling `gpio_request.release()`
5. **Finalize GPIO pin assignments** — document real pin values in
   `test_params.yaml` once wiring is permanent
6. **Write `rpm_scale_node`** — RPM → G-unit conversion (open loop);
   `max_rpm` parameter; testable at home without hardware
7. **Add `imu_node`, `relay_board_node`, and `roboteq_node` to
   `hw_only.launch.py`** in `breakabot_bringup`
8. **Start `kinematics_node`** — Kiwi-drive forward/inverse kinematics;
   subscribes to `/cmd_vel`, publishes to `/roboteq/motor_cmd` via
   `rpm_scale_node`; mode gating for teleop vs autonomous (controlled by
   `test_manager_node`)
9. **Implement `require_safe_state` interlock** in `relay_board_node` and
   `roboteq_node` once `test_manager_node` state is defined (Phase 3)

---

## Key technical decisions log

| Decision | Choice | Rationale |
|---|---|---|
| GPIO library | `gpiod` v2.4.2 | Only library with full Pi 5 / RP1 support |
| Relay command interface | Custom `RelayCommand.msg` | Named fields enforce structure; eliminates length checks |
| Relay command granularity | Motor-controller level, not individual relay | Prevents mismatched relay pairs by design |
| State topic type | `Int32MultiArray` | Simple 3-element array; custom type not needed for state reporting |
| Python venv | None — global install | ROS2 overlay mechanism incompatible with venv |
| Repo structure | Monorepo (`breakabot_ws/`) | Single clone, atomic commits, simpler handoff |
| Package split | `_hardware` / `_core` / `_bringup` / `_interfaces` | Standard ROS2 pattern; hardware swap doesn't touch core logic |
| IMU data field | `linear_acceleration` | Gravity-compensated; `acceleration` includes ~9.8 m/s² gravity vector |
| IMU publish QoS | `BEST_EFFORT` | Sensor streams favor latency over guaranteed delivery |
| roboteq_node controller storage | `self.controllers` list of dicts `{port, lock}` | Loop-based operations; controller count is a yaml change, not a code change |
| roboteq_node USB paths | `/dev/serial/by-id/` symlinks | Stable across reboots; `ttyACMx` index shifts with plug order |
| roboteq_node watchdog keepalive | Re-send `last_cmd` every fast poll cycle | SDC2130 watchdog resets only on motor commands, not queries |
| roboteq_node control mode | Parameter `open_loop` / `closed_loop` | Mode switch requires yaml edit + restart; no code change |
| roboteq_node inversion order | Inversion applied before clamp | Prevents large inverted values escaping ±1000 range |
| roboteq_node shutdown command | `!MS` (stop all modes) | Works regardless of control mode and watchdog state |
| Motor command pipeline | `kinematics_node` → `rpm_scale_node` → `roboteq_node` | Separation of geometry, scaling, and hardware; closed-loop upgrade swaps only `rpm_scale_node` |
| Control mode selection | `test_manager_node` publishes mode; `kinematics_node` gates inputs | Centralizes authority; hardware and command-source nodes are mode-unaware |
| pytest with real rclpy | Session-scoped `rclpy_session` fixture in `conftest.py` | `rclpy.init()` required once per session; `scope='function'` causes re-init errors |
| Publisher test assertions | `patch.object(pub, 'publish')` + `call_args` | Real `rclpy.Publisher` has no `last_msg`; `patch.object` intercepts the call cleanly |

---

## Hardware notes

| Hardware | Interface | Available | Notes |
|---|---|---|---|
| BNO055 IMU | I2C (`board.SCL`, `board.SDA`) | Home + lab | I2C address `0x28` default, `0x29` alt |
| Sunfounder relay board | GPIO via `gpiod` | Home (unwired) | Active-low; `gpiochip4` on Pi 5 |
| Roboteq SDC2130 (×3) | Serial (USB) | Lab only | ASCII protocol; reports encoder, current, battery |
| Pittman LO-COG F5019 motors (×6) | — | Lab only | Max RPM to be read from motor label |
| Xbox controller | USB/Bluetooth | Lab | `joy` + `teleop_twist_joy` packages |
| Raspberry Pi 5 | — | Home + lab | Ubuntu 24.04, ROS2 Jazzy |

---

## Test scenarios (reminder)

**Scenario 1 — Teleoperated fault injection**
Operator drives via Xbox controller. Test Manager triggers relay board
to open motor 1 circuit at a preset time. IMU records locomotion change.

**Scenario 2 — Autonomous square with fault injection**
Robot traces 1.5 m square four times. Test Manager triggers fault at
preset time. Dead-reckoning expected to drift after fault — this is
the diagnostic signal of interest.

---

## Environment

```
OS:         Ubuntu 24.04
ROS2:       Jazzy (system install)
Python:     3.12 (system)
gpiod:      2.4.2 (pip, --break-system-packages)
pyserial:   (pip, --break-system-packages)
adafruit-blinka + adafruit-circuitpython-bno055: (pip)
Key tools:  colcon, pytest, ros2bag, rqt, rviz2, turtlesim
```

---

*Last updated: session covering roboteq_node design, implementation, unit
tests (59 passing), lab test plan, architecture.md update, and progress.md
update. Node code complete; lab testing scheduled for next session.*
