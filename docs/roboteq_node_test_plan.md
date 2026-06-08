# roboteq_node Lab Test Plan

**Node:** `roboteq_node`  
**Package:** `breakabot_hardware`  
**Hardware required:** Raspberry Pi 5, 3× RoboteQ SDC2130, 3× USB cables, main battery  
**Prerequisites:** `roboteq_node.py` and `roboteq_params.yaml` built and installed in workspace

---

## Overview

This plan progresses through five phases. Each phase must pass before proceeding to the next.
A phase failure is a stop condition — diagnose and resolve before continuing.

```
Phase 1 — USB identification and port mapping
Phase 2 — Serial connection verification
Phase 3 — Sensor polling verification (no motors powered)
Phase 4 — Single-channel motor tests (motors powered, one channel at a time)
Phase 5 — Full 6-channel test and inversion record
```

---

## Phase 1 — USB identification and port mapping

**Goal:** Identify which by-id symlink corresponds to which physical controller,
and populate `roboteq_params.yaml` with the correct port paths.

**Motors should be unpowered for this phase. USB only.**

### Steps

**1.1** Connect all three USB cables from the SDC2130 controllers to the Pi.
Power on the Pi. Do not yet connect motor battery power.

**1.2** List the detected USB serial devices:

```bash
ls -l /dev/serial/by-id/ | grep -i roboteq
```

You should see three entries. Example output:

```
lrwxrwxrwx 1 root root 13 ... usb-Roboteq_SDC2130_AB12CD34-if00 -> ../../ttyACM0
lrwxrwxrwx 1 root root 13 ... usb-Roboteq_SDC2130_EF56GH78-if00 -> ../../ttyACM1
lrwxrwxrwx 1 root root 13 ... usb-Roboteq_SDC2130_IJ90KL12-if00 -> ../../ttyACM2
```

**Pass condition:** Exactly three entries appear.  
**Fail condition:** Fewer than three — check cables and USB power. Re-seat connections.

**1.3** Map each symlink to a physical controller. The most reliable method is to
unplug one controller at a time and re-run the `ls` command to see which entry disappears.
Label each controller (tape or marker) with its serial number fragment (e.g. `AB12CD34`).

**1.4** Decide on a permanent controller numbering convention and record it below.
Suggested convention: number controllers 0–2 by their physical position on the robot
(e.g. front-left = 0, front-right = 1, rear = 2 for Kiwi-drive).

| Controller | Physical position | Serial number | by-id path |
|---|---|---|---|
| 0 | | | |
| 1 | | | |
| 2 | | | |

**1.5** Update `roboteq_params.yaml` with the three by-id paths:

```yaml
port_0: /dev/serial/by-id/usb-Roboteq_SDC2130_<serial_0>-if00
port_1: /dev/serial/by-id/usb-Roboteq_SDC2130_<serial_1>-if00
port_2: /dev/serial/by-id/usb-Roboteq_SDC2130_<serial_2>-if00
```

**1.6** Rebuild the workspace so the updated params file is installed:

```bash
cd ~/breakabot_ws
colcon build --packages-select breakabot_bringup
source install/setup.bash
```

---

## Phase 2 — Serial connection verification

**Goal:** Confirm the node opens all three serial ports successfully at startup.

**Motors still unpowered.**

### Steps

**2.1** Launch the node with the params file:

```bash
ros2 run breakabot_hardware roboteq_node \
  --ros-args --params-file ~/breakabot_ws/src/breakabot_bringup/config/roboteq_params.yaml
```

**2.2** Observe startup log output. You should see:

```
[INFO] [roboteq_node]: RoboteqNode starting — control_mode: open_loop, fast_poll: 10.0 Hz, battery_poll: 1.0 Hz
[INFO] [roboteq_node]: Controller 0: connected on /dev/serial/by-id/usb-Roboteq_SDC2130_...-if00
[INFO] [roboteq_node]: Controller 1: connected on /dev/serial/by-id/usb-Roboteq_SDC2130_...-if00
[INFO] [roboteq_node]: Controller 2: connected on /dev/serial/by-id/usb-Roboteq_SDC2130_...-if00
[INFO] [roboteq_node]: RoboteqNode ready.
```

**Pass condition:** All three "connected" lines appear, followed by "ready".  
**Fail condition:** Any `[FATAL]` line — the port path in the params file is wrong or
the controller is not powered/connected via USB. Check the by-id path matches exactly.

**2.3** Confirm the node's topics are visible:

```bash
ros2 topic list | grep roboteq
```

Expected output:

```
/roboteq/battery_state
/roboteq/encoder_counts
/roboteq/motor_cmd
/roboteq/motor_current
```

**Pass condition:** All four topics listed.

---

## Phase 3 — Sensor polling verification

**Goal:** Confirm that `?V`, `?C`, and `?A` queries are working and publishing
correctly before any motor commands are sent. USB connected, controllers powered
(motor driver stage on, but no load — motors can remain disconnected from drive train
if preferred for safety).

### Steps

**3.1** With the node running from Phase 2, monitor the battery state topic:

```bash
ros2 topic echo /roboteq/battery_state
```

Observe the `voltage` field. With a healthy charged battery you should see a
voltage in the expected range for your battery pack.

**Pass condition:** `voltage` field is non-zero and plausible (e.g. 22–29V for a 24V pack).  
**Fail condition:** Zero or NaN — check that motor battery power is connected to the
controllers and that `?V` queries are not timing out (check node log for warnings).

**Record voltage readings:**

| Controller | Battery voltage (V) |
|---|---|
| 0 (averaged in message) | |

> Note: the published `BatteryState` is an average across all three controllers.
> For individual readings, temporarily add debug logging or use the Roborun+ PC utility.

**3.2** Monitor the encoder counts topic:

```bash
ros2 topic echo /roboteq/encoder_counts
```

With motors stationary, all six values should be zero or a small fixed count (depending
on whether encoders have been zeroed via firmware). The values should be stable
(not incrementing) while the motors are not moving.

**Pass condition:** Topic is publishing at ~10 Hz and values are stable.  
**Fail condition:** Topic not publishing, or values incrementing with no motor movement
(suggests encoder noise or wiring issue — not a node problem).

**3.3** Verify the encoder publish rate:

```bash
ros2 topic hz /roboteq/encoder_counts
```

**Pass condition:** Reported rate is close to `fast_poll_rate_hz` (default 10.0 Hz).

**3.4** Monitor motor current:

```bash
ros2 topic echo /roboteq/motor_current
```

With motors stationary and no commands sent, all six values should be 0.0 A
(the SDC2130 reports 0 A at 0% PWM).

**Pass condition:** All six values are 0.0 A.

---

## Phase 4 — Single-channel motor tests

**Goal:** Confirm that `!G` commands reach each controller correctly and that motors
respond, using the minimum command exposure needed to observe direction.

**Safety checklist before powering motors:**
- [ ] Robot is elevated so wheels can spin freely, or wheels are clear of ground
- [ ] E-stop contactor is accessible
- [ ] No personnel near the wheels
- [ ] Battery fuse is installed

### Test command helper

Open a second terminal. The following command publishes a single motor command
message. Adjust the array values as directed in each step below.

```bash
ros2 topic pub --once /roboteq/motor_cmd std_msgs/msg/Int32MultiArray \
  "data: [0, 0, 0, 0, 0, 0]"
```

Array layout: `[c0_ch1, c0_ch2, c1_ch1, c1_ch2, c2_ch1, c2_ch2]`

> The node's watchdog keepalive will hold the last command after `--once` publishes it.
> To stop motors between tests, publish all zeros:
> ```bash
> ros2 topic pub --once /roboteq/motor_cmd std_msgs/msg/Int32MultiArray "data: [0, 0, 0, 0, 0, 0]"
> ```

### Steps

Work through each channel in order. Use a small command value (200 = 20% power)
to minimise motor speed during direction testing. Observe wheel rotation direction
and record below.

**4.1 — Controller 0, channel 1**

```bash
ros2 topic pub --once /roboteq/motor_cmd std_msgs/msg/Int32MultiArray \
  "data: [200, 0, 0, 0, 0, 0]"
```

Observe the motor connected to controller 0, channel 1.
Stop: publish all zeros.

**4.2 — Controller 0, channel 2**

```bash
ros2 topic pub --once /roboteq/motor_cmd std_msgs/msg/Int32MultiArray \
  "data: [0, 200, 0, 0, 0, 0]"
```

**4.3 — Controller 1, channel 1**

```bash
ros2 topic pub --once /roboteq/motor_cmd std_msgs/msg/Int32MultiArray \
  "data: [0, 0, 200, 0, 0, 0]"
```

**4.4 — Controller 1, channel 2**

```bash
ros2 topic pub --once /roboteq/motor_cmd std_msgs/msg/Int32MultiArray \
  "data: [0, 0, 0, 200, 0, 0]"
```

**4.5 — Controller 2, channel 1**

```bash
ros2 topic pub --once /roboteq/motor_cmd std_msgs/msg/Int32MultiArray \
  "data: [0, 0, 0, 0, 200, 0]"
```

**4.6 — Controller 2, channel 2**

```bash
ros2 topic pub --once /roboteq/motor_cmd std_msgs/msg/Int32MultiArray \
  "data: [0, 0, 0, 0, 0, 200]"
```

**Pass condition for each step:** The correct motor spins, current rises above 0 A
on the correct channel in `/roboteq/motor_current`, and no other motors move.  
**Fail condition:** Wrong motor moves (wiring mapping error), no motor moves
(serial command not reaching controller), or current stays at 0 A (motor not connected).

---

## Phase 5 — Full 6-channel test and inversion record

**Goal:** Determine correct rotation direction for each channel relative to the
Kiwi-drive geometry, record any inversions needed, and confirm current sensing
across all channels simultaneously.

### 5.1 — Determine expected directions

Before running motors, document the expected positive-command spin direction
for each wheel based on your Kiwi-drive geometry. A positive `!G` command
applies positive voltage to the motor leads. Whether this produces
clockwise or counter-clockwise wheel rotation depends on how each motor
is physically mounted and wired.

Sketch or reference your robot's wheel layout and fill in the expected column:

| Channel | Wheel | Expected direction (positive cmd) | Observed direction | Inverted? |
|---|---|---|---|---|
| c0_ch1 | | | | |
| c0_ch2 | | | | |
| c1_ch1 | | | | |
| c1_ch2 | | | | |
| c2_ch1 | | | | |
| c2_ch2 | | | | |

### 5.2 — Run all six channels simultaneously

```bash
ros2 topic pub --once /roboteq/motor_cmd std_msgs/msg/Int32MultiArray \
  "data: [200, 200, 200, 200, 200, 200]"
```

Observe all six wheels. Record actual spin direction in the table above.
Stop:

```bash
ros2 topic pub --once /roboteq/motor_cmd std_msgs/msg/Int32MultiArray \
  "data: [0, 0, 0, 0, 0, 0]"
```

### 5.3 — Verify current sensing under load

While motors are running, in a separate terminal:

```bash
ros2 topic echo /roboteq/motor_current
```

**Pass condition:** All six values are non-zero and broadly similar (expect some
variation due to individual motor load). Zero on any channel while that motor is
visibly spinning indicates a current sensing or wiring issue.

### 5.4 — Update inversion parameters

For each channel marked `Inverted? = Yes` in the table above, update
`roboteq_params.yaml`. The inversion lists are indexed by controller (0, 1, 2):

```yaml
# Example: if c0_ch2 and c2_ch1 are inverted
invert_ch1: [false, false, true]   # controller 0, 1, 2 — channel 1
invert_ch2: [true,  false, false]  # controller 0, 1, 2 — channel 2
```

Rebuild and re-source after editing:

```bash
colcon build --packages-select breakabot_bringup && source install/setup.bash
```

### 5.5 — Verify inversions

Restart the node and repeat step 5.2. All wheels should now spin in the
expected direction for a positive command.

**Pass condition:** All six wheels spin in expected direction with no further
inversions needed.

---

## Phase completion sign-off

| Phase | Pass | Date | Notes |
|---|---|---|---|
| 1 — USB identification | ☐ | | |
| 2 — Serial connection | ☐ | | |
| 3 — Sensor polling | ☐ | | |
| 4 — Single-channel motor | ☐ | | |
| 5 — Full 6-channel + inversion | ☐ | | |

---

## Decisions to record after testing

Once all phases are complete, add the following entries to the decisions table
in `docs/progress.md`:

| Decision | Choice | Rationale |
|---|---|---|
| Controller numbering | TBD from Phase 1 | Physical position → controller index mapping |
| Channel inversions | TBD from Phase 5 | Record which channels required inversion and why |
| Motor max RPM | TBD — read from F5019 label | Required for rpm_scale_node in Phase 3 |
