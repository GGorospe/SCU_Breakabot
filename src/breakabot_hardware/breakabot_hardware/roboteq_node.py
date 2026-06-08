# File: roboteq_node.py
# Author: George Gorospe, ggorospe@scu.edu
# About: Hardware interface node for three RoboteQ SDC2130 motor controllers.
#        Each controller drives two motors on the Break-a-bot Kiwi-drive platform.
#        Controllers connect to the Raspberry Pi 5 via USB (3 separate cables).
#
# Topics subscribed:
#   /roboteq/motor_cmd  (std_msgs/Int32MultiArray) - commands for ch1 and ch2 for all three controllers.
#       6-element array: [c0_ch1, c0_ch2, c1_ch1, c1_ch2, c2_ch1, c2_ch2]
#       In open_loop mode:  values are G-units  (-1000 to +1000)
#       In closed_loop mode: values are RPM targets (converted by rpm_scale_node)
#
# Topics published:
#   /roboteq/encoder_counts  (std_msgs/Int32MultiArray) - encoder information from all channels/controllers.
#       6-element array: [c0_ch1, c0_ch2, c1_ch1, c1_ch2, c2_ch1, c2_ch2]
#       Absolute encoder counts from ?C query. Published at encoder_poll_rate_hz.
#
#   /roboteq/motor_current   (std_msgs/Float32MultiArray) - current information from all channels/controlelrs.
#       6-element array: [c0_ch1, c0_ch2, c1_ch1, c1_ch2, c2_ch1, c2_ch2]
#       Motor amps (Amps, converted from Amps*10 reply). Published at encoder_poll_rate_hz.
#
#   /roboteq/battery_state   (sensor_msgs/BatteryState)
#       Battery voltage averaged across all three controllers. Published at battery_poll_rate_hz.
#
# Serial protocol notes (from RoboteQ User Manual v2.1a):
#   - Commands are ASCII, terminated by carriage return '\r' (or '_' as shorthand)
#   - Controller echoes every valid character received
#   - Commands with no reply issue '+\r' as ACK; bad commands issue '-'
#   - Queries reply with 'KEY=val1:val2\r'
#   - !G cc nn  : Go (open-loop power), cc=channel (1 or 2), nn=-1000..+1000
#   - !S cc nn  : Set speed (closed-loop RPM setpoint), cc=channel, nn=RPM
#   - !MS cc    : Stop motor on channel cc
#   - !EX       : Emergency stop (requires !MG to release)
#   - ?A        : Read motor amps  -> A=ch1*10:ch2*10
#   - ?C        : Read encoder counts -> C=ch1:ch2  (absolute, 32-bit signed)
#   - ?V        : Read voltages -> V=internal*10:battery*10:5Vout_mV
#   - Watchdog: controller stops motors if no command received within timeout (default 1s)
#     The poll_callback keeps the watchdog alive by sending commands regularly.
#
# Discovering USB port paths for each motor controller:
# About: each motor controller is connected to the RPi 5 via USB
# The following command will determine the symlink path for each controller.
# Once the paths are determined they should be copied into the roboteq_params.yaml configuration document
#   ls -l /dev/serial/by-id/ | grep -i roboteq
#   Copy the full symlink path (e.g. /dev/serial/by-id/usb-Roboteq_SDC2130_XXXXXXXX-if00)
#   into roboteq_params.yaml for port_0, port_1, port_2.

# Import ROS2 libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32MultiArray
from sensor_msgs.msg import BatteryState

# Import general purpose python libraries
import serial
import threading
import time

# ---------------------------------------------------------------------------
# Number of controllers and channels — do not change without updating params
# ---------------------------------------------------------------------------
NUM_CONTROLLERS = 3
NUM_CHANNELS    = 2          # channels per controller
NUM_MOTORS      = NUM_CONTROLLERS * NUM_CHANNELS   # 6 total


class RoboteqNode(Node):

    def __init__(self):
        super().__init__('roboteq_node')

        # ------------------------------------------------------------------ #
        # 1. Declare parameters
        # ------------------------------------------------------------------ #
        self.declare_parameters(
            namespace='',
            parameters=[
                # These are backup parameters, just in case the actual parameters from
                # the roboteq_params.yaml file are not found.
                # USB port paths — replace placeholder values with real by-id paths.
                # Find them in the lab with:
                #   ls -l /dev/serial/by-id/ | grep -i roboteq
                ('port_0', '/dev/serial/by-id/usb-Roboteq_SDC2130_PLACEHOLDER_0-if00'),
                ('port_1', '/dev/serial/by-id/usb-Roboteq_SDC2130_PLACEHOLDER_1-if00'),
                ('port_2', '/dev/serial/by-id/usb-Roboteq_SDC2130_PLACEHOLDER_2-if00'),
                ('baud_rate', 115200),

                # Control mode: 'open_loop'  -> sends !G (power level ±1000)
                #               'closed_loop' -> sends !S (RPM setpoint)
                # Switch by editing roboteq_params.yaml — no code changes needed.
                ('control_mode', 'open_loop'),

                # Channel inversion — one boolean per controller per channel.
                # Set to true for any channel where the motor spins the wrong direction.
                # Confirm in the lab before enabling closed-loop.
                ('invert_ch1', [False, False, False]),
                ('invert_ch2', [False, False, False]),

                # Poll rates (Hz)
                # fast_poll_rate_hz: controls encoder counts, motor current, AND the
                # watchdog keepalive command. Must stay below the controller's watchdog
                # timeout (default 1s) — 10 Hz provides a comfortable 10x margin.
                # Raise to ~50 Hz for diagnostic routines.
                # Practical ceiling ~50-100 Hz (3 serial round-trips per cycle).
                ('fast_poll_rate_hz', 10.0),
                ('battery_poll_rate_hz', 1.0),

                # Serial read timeout (seconds). Should be short relative to poll period.
                ('serial_timeout', 0.1),
            ]
        )

        # ------------------------------------------------------------------ #
        # 2. Read parameters
        # ------------------------------------------------------------------ #
        port_paths = [
            self.get_parameter('port_0').value,
            self.get_parameter('port_1').value,
            self.get_parameter('port_2').value,
        ]
        baud_rate          = self.get_parameter('baud_rate').value
        self.control_mode  = self.get_parameter('control_mode').value
        self.invert_ch1    = self.get_parameter('invert_ch1').value   # list[bool] len 3
        self.invert_ch2    = self.get_parameter('invert_ch2').value   # list[bool] len 3
        fast_poll_rate     = self.get_parameter('fast_poll_rate_hz').value
        battery_poll_rate  = self.get_parameter('battery_poll_rate_hz').value
        serial_timeout     = self.get_parameter('serial_timeout').value

        self.get_logger().info(
            f'RoboteqNode starting — control_mode: {self.control_mode}, '
            f'fast_poll: {fast_poll_rate} Hz, battery_poll: {battery_poll_rate} Hz'
        )

        # ------------------------------------------------------------------ #
        # 3. Open serial connections — one per controller
        #
        # self.controllers is a list of dicts, one per physical controller:
        #   {
        #     'port': serial.Serial,   # the open serial connection
        #     'lock': threading.Lock() # guards all read/write on this port
        #   }
        #
        # Using a list lets every operation (command, poll) be a simple loop:
        #   for i, ctrl in enumerate(self.controllers): ...
        # ------------------------------------------------------------------ #
        self.controllers = []
        for i, path in enumerate(port_paths):
            try:
                port = serial.Serial(
                    port=path,
                    baudrate=baud_rate,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    bytesize=serial.EIGHTBITS,
                    timeout=serial_timeout,
                )
                self.controllers.append({'port': port, 'lock': threading.Lock()})
                self.get_logger().info(f'Controller {i}: connected on {path}')
            except serial.SerialException as e:
                # Raise immediately — a missing controller is a startup error,
                # not something to silently ignore. Fix the port path and retry.
                self.get_logger().fatal(
                    f'Controller {i}: failed to open {path} — {e}\n'
                    f'  Hint: ls -l /dev/serial/by-id/ | grep -i roboteq'
                )
                raise

        # ------------------------------------------------------------------ #
        # 4. Last-command state — used by the watchdog keepalive
        #
        # Stores the most recently applied command for each controller as a
        # pre-formatted serial string. Initialised to stop commands so that
        # the very first keepalive transmission is safe.
        # ------------------------------------------------------------------ #
        cmd_char = 'G' if self.control_mode == 'open_loop' else 'S'
        self.last_cmd = [
            f'!{cmd_char} 1 0_!{cmd_char} 2 0_'
            for _ in range(NUM_CONTROLLERS)
        ]

        # ------------------------------------------------------------------ #
        # 5. Subscriber — motor commands in
        # ------------------------------------------------------------------ #
        self.cmd_sub = self.create_subscription(
            Int32MultiArray,
            '/roboteq/motor_cmd',
            self._cmd_callback,
            qos_profile=5,
        )

        # ------------------------------------------------------------------ #
        # 6. Publishers — feedback out
        # ------------------------------------------------------------------ #
        self.encoder_pub = self.create_publisher(
            Int32MultiArray,
            '/roboteq/encoder_counts',
            qos_profile=10,
        )
        self.current_pub = self.create_publisher(
            Float32MultiArray,
            '/roboteq/motor_current',
            qos_profile=10,
        )
        self.battery_pub = self.create_publisher(
            BatteryState,
            '/roboteq/battery_state',
            qos_profile=5,
        )

        # ------------------------------------------------------------------ #
        # 7. Timers — two independent poll loops
        # ------------------------------------------------------------------ #
        # Fast loop: encoder counts, motor current, and watchdog keepalive
        self.fast_timer = self.create_timer(
            1.0 / fast_poll_rate,
            self._fast_poll_callback,
        )
        # Battery voltage: slower, changes on the timescale of minutes
        self.battery_timer = self.create_timer(
            1.0 / battery_poll_rate,
            self._battery_poll_callback,
        )

        self.get_logger().info('RoboteqNode ready.')

    # ======================================================================= #
    # Command callback — fires when a motor command arrives
    # ======================================================================= #
    def _cmd_callback(self, msg: Int32MultiArray):
        """
        Receives a 6-element Int32MultiArray and writes !G (open-loop) or !S
        (closed-loop) commands to the three controllers.

        Expected layout: [c0_ch1, c0_ch2, c1_ch1, c1_ch2, c2_ch1, c2_ch2]
        """
        if len(msg.data) != NUM_MOTORS:
            self.get_logger().warn(
                f'motor_cmd has {len(msg.data)} values, expected {NUM_MOTORS}. Ignoring.',
                throttle_duration_sec=5.0,
            )
            return

        cmd_char = 'G' if self.control_mode == 'open_loop' else 'S'

        for i, ctrl in enumerate(self.controllers):
            base = i * NUM_CHANNELS
            raw_ch1 = int(msg.data[base])
            raw_ch2 = int(msg.data[base + 1])

            # Apply channel inversion if configured
            val_ch1 = -raw_ch1 if self.invert_ch1[i] else raw_ch1
            val_ch2 = -raw_ch2 if self.invert_ch2[i] else raw_ch2

            # Clamp to valid range (open-loop: ±1000; closed-loop: firmware enforces)
            val_ch1 = max(-1000, min(1000, val_ch1))
            val_ch2 = max(-1000, min(1000, val_ch2))

            # Build command string. The '_' character is treated as '\r' by the
            # controller (see manual §14), which lets us concatenate two commands
            # in one serial write — fewer round-trips per cycle.
            command = f'!{cmd_char} 1 {val_ch1}_!{cmd_char} 2 {val_ch2}_'

            # Store for watchdog keepalive — fast poll will resend this every cycle
            self.last_cmd[i] = command

            self._send_command(ctrl, command, controller_index=i)

    # ======================================================================= #
    # Fast poll callback — fires at fast_poll_rate_hz
    # ======================================================================= #
    def _fast_poll_callback(self):
        """
        Three jobs per cycle, in order:

        1. WATCHDOG KEEPALIVE — resend the last known command to each controller.
           The SDC2130 watchdog (default 1s timeout) resets only on motor commands
           (!G, !S), not on queries. Without this, the controller stops the motors
           ~1 second after the last motor_cmd message arrives. By resending
           self.last_cmd[] every fast poll cycle we keep the watchdog alive
           regardless of whether new commands are arriving upstream.

        2. ENCODER COUNTS — query ?C and publish /roboteq/encoder_counts.
           ?C reply format: C=ch1:ch2  (absolute 32-bit signed counts)

        3. MOTOR CURRENT — query ?A and publish /roboteq/motor_current.
           ?A reply format: A=ch1*10:ch2*10  (motor amps * 10, convert to amps)
        """
        encoder_vals = []
        current_vals = []

        for i, ctrl in enumerate(self.controllers):

            # 1. Watchdog keepalive — resend last command
            self._send_command(ctrl, self.last_cmd[i], controller_index=i)

            # 2. Encoder counts
            c_reply = self._send_query(ctrl, '?C\r', prefix='C', controller_index=i)
            ch1_enc, ch2_enc = self._parse_two_ints(c_reply)
            encoder_vals.extend([ch1_enc, ch2_enc])

            # 3. Motor amps (returned as Amps*10, convert to Amps)
            a_reply = self._send_query(ctrl, '?A\r', prefix='A', controller_index=i)
            ch1_raw, ch2_raw = self._parse_two_ints(a_reply)
            current_vals.extend([ch1_raw / 10.0, ch2_raw / 10.0])

        # Publish encoder counts
        enc_msg = Int32MultiArray()
        enc_msg.data = encoder_vals
        self.encoder_pub.publish(enc_msg)

        # Publish motor current
        cur_msg = Float32MultiArray()
        cur_msg.data = [float(v) for v in current_vals]
        self.current_pub.publish(cur_msg)

    # ======================================================================= #
    # Battery poll callback — fires at battery_poll_rate_hz
    # ======================================================================= #
    def _battery_poll_callback(self):
        """
        Queries ?V from all three controllers and publishes an averaged BatteryState.

        ?V reply format: V=internal*10:battery*10:5Vout_mV
        Battery voltage is index 1 (colon-separated), in Volts*10.
        """
        voltages = []

        for i, ctrl in enumerate(self.controllers):
            v_reply = self._send_query(ctrl, '?V\r', prefix='V', controller_index=i)
            if v_reply:
                parts = v_reply.split(':')
                try:
                    # Index 1 = battery voltage * 10
                    battery_v = int(parts[1]) / 10.0
                    voltages.append(battery_v)
                    self.get_logger().debug(
                        f'Controller {i}: battery {battery_v:.1f}V, '
                        f'internal {int(parts[0])/10.0:.1f}V, '
                        f'5V rail {int(parts[2])/1000.0:.2f}V'
                    )
                except (IndexError, ValueError) as e:
                    self.get_logger().warn(
                        f'Controller {i}: could not parse ?V reply "{v_reply}": {e}'
                    )

        if not voltages:
            self.get_logger().warn('Battery poll: no valid readings from any controller.')
            return

        avg_voltage = sum(voltages) / len(voltages)

        msg = BatteryState()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'roboteq_battery'
        msg.voltage         = float(avg_voltage)
        msg.present         = True
        msg.current         = float('nan')
        msg.charge          = float('nan')
        msg.capacity        = float('nan')
        msg.design_capacity = float('nan')
        msg.percentage      = float('nan')
        msg.power_supply_health     = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        msg.power_supply_status     = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN

        self.battery_pub.publish(msg)
        self.get_logger().debug(f'Battery: {avg_voltage:.1f}V (avg of {len(voltages)} controllers)')

    # ======================================================================= #
    # Serial helpers
    # ======================================================================= #
    def _send_command(self, ctrl: dict, command: str, controller_index: int):
        """
        Write a command string to a controller. Acquires the controller's lock
        to prevent interleaving with the poll callbacks.

        The controller responds to non-query commands with '+\r' (ACK) or '-' (error).
        We read and discard the ACK — we don't need it and leaving it in the buffer
        would corrupt subsequent query reads.
        """
        with ctrl['lock']:
            try:
                ctrl['port'].write(command.encode('ascii'))
                # Read and discard ACK lines (one '+\r' per command sent).
                # Two commands were concatenated, so read two ACKs.
                ctrl['port'].readline()
                ctrl['port'].readline()
            except serial.SerialException as e:
                self.get_logger().error(
                    f'Controller {controller_index}: serial write error — {e}',
                    throttle_duration_sec=2.0,
                )

    def _send_query(self, ctrl: dict, query: str, prefix: str,
                    controller_index: int) -> str:
        """
        Send a query command and return the value portion of the reply.

        The controller replies with 'PREFIX=value\r'. This method strips the
        prefix and returns the raw value string (e.g. '246:135:4730' for ?V).
        Returns an empty string on error so callers can handle gracefully.
        """
        with ctrl['lock']:
            try:
                ctrl['port'].reset_input_buffer()
                ctrl['port'].write(query.encode('ascii'))
                reply = ctrl['port'].readline().decode('ascii', errors='replace').strip()

                expected = f'{prefix}='
                if reply.startswith(expected):
                    return reply[len(expected):]
                else:
                    self.get_logger().warn(
                        f'Controller {controller_index}: unexpected reply to '
                        f'"{query.strip()}": "{reply}"',
                        throttle_duration_sec=5.0,
                    )
                    return ''
            except serial.SerialException as e:
                self.get_logger().error(
                    f'Controller {controller_index}: serial read error — {e}',
                    throttle_duration_sec=2.0,
                )
                return ''

    @staticmethod
    def _parse_two_ints(reply: str) -> tuple[int, int]:
        """
        Parse a 'val1:val2' reply string into a pair of ints.
        Returns (0, 0) if parsing fails, so callers always get a safe value.
        """
        try:
            parts = reply.split(':')
            return int(parts[0]), int(parts[1])
        except (IndexError, ValueError):
            return 0, 0

    # ======================================================================= #
    # Shutdown
    # ======================================================================= #
    def destroy_node(self):
        """
        Safe shutdown: zero all motors on all controllers before closing ports.

        Sends !MS (stop in all modes) to each channel on each controller.
        !MS is preferred over !G 0 because it works regardless of control mode
        and does not require the watchdog to still be alive.
        """
        self.get_logger().info('RoboteqNode shutting down — stopping all motors.')
        for i, ctrl in enumerate(self.controllers):
            try:
                # !MS 1 stops channel 1, !MS 2 stops channel 2
                stop_cmd = '!MS 1_!MS 2_'
                with ctrl['lock']:
                    ctrl['port'].write(stop_cmd.encode('ascii'))
                    time.sleep(0.05)   # brief pause to let command land
                self.get_logger().debug(f'Controller {i}: motors stopped.')
            except Exception as e:
                self.get_logger().warn(f'Controller {i}: stop command failed — {e}')

            try:
                if ctrl['port'].is_open:
                    ctrl['port'].close()
                    self.get_logger().debug(f'Controller {i}: port closed.')
            except Exception as e:
                self.get_logger().warn(f'Controller {i}: port close failed — {e}')

        super().destroy_node()


# =========================================================================== #
# Entry point
# =========================================================================== #
def main(args=None):
    rclpy.init(args=args)
    node = RoboteqNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
