# File: relay_node.py
# Author: George Gorospe, ggorospe@scu.edu
# About: this is the hardware interface node for the Sunfounder 5V 8-Channel Relay Board
# Notes: 
#   This node communicates with the relay board via TTL
#   This node is a "hardware subscriber" that means that as a HW interface it subscribes to messages to actuate the hardware


# Initial Implementation: STUB

# Importing Required Libraries
# ROS2 libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from breakabot_interfaces.msg import RelayCommand # Custom message for the Break-a-bot project, located in breakabot_interfaces package

# The following try/except structure enables development and testing of the code on laptop vs RPi
try: 
    import gpiod
    from gpiod.line import Direction, Value
    HW_AVAILABLE = True
except ImportError:
    HW_AVAILABLE = False


# RelayBoardNode class
class RelayBoardNode(Node):
    def __init__(self): 
        super().__init__('relay_board_node')
        self.hw_ready = False
        self.gpio_request = None
        self.current_channel = {1: 1, 2: 1, 3: 1}
                
        # Relay board node parmeters (declared and set)
        self.declare_parameter('gpio_pin_relay_2', 17)
        self.declare_parameter('gpio_pin_relay_3', 27)
        self.declare_parameter('gpio_pin_relay_4', 22)
        self.declare_parameter('gpio_pin_relay_5', 23)
        self.declare_parameter('gpio_pin_relay_6', 24)
        self.declare_parameter('gpio_pin_relay_7', 25)
        self.declare_parameter('require_safe_state', True)
        self.declare_parameter('gpiochip', 'gpiochip4')
        

        # Read them back: pin_r2, pin_r3, pin_r4, pin_r5, pin_r6, pin_r7
        self.pin_r2 = self.get_parameter('gpio_pin_relay_2').value
        self.pin_r3 = self.get_parameter('gpio_pin_relay_3').value
        self.pin_r4 = self.get_parameter('gpio_pin_relay_4').value
        self.pin_r5 = self.get_parameter('gpio_pin_relay_5').value
        self.pin_r6 = self.get_parameter('gpio_pin_relay_6').value
        self.pin_r7 = self.get_parameter('gpio_pin_relay_7').value
        self.gpiochip = self.get_parameter('gpiochip').value
        self.require_safe_state = self.get_parameter('require_safe_state').value

        # The internal relay mapping tying together sets of relays
        self.mc_relay_map = {
            1: (self.pin_r2, self.pin_r3),
            2: (self.pin_r4, self.pin_r5),
            3: (self.pin_r6, self.pin_r7),
        }

        # Request lines and set initial valuess
        if not HW_AVAILABLE:
            self.get_logger().warn('gpiod not available - running in stub mode.')
        else:
            try:
                # Common line setting for all gpio pins tied to a relay
                # *** IMPORTANT ***
                # Sunfounder relay board is active-low: driving a pin LOW energizes the relay.
                # active_low=True tells gpiod that Value.ACTIVE maps to a LOW electrical signal.
                # Result: Value.ACTIVE = relay energized (NC path / channel 2)
                #         Value.INACTIVE = relay de-energized (NO path / channel 1)
                relay_settings = gpiod.LineSettings(
                    direction=Direction.OUTPUT,
                    active_low=True,
                    output_value=Value.INACTIVE # All relays start de-energized
                )

                # Request the lines
                self.gpio_request = gpiod.request_lines(
                    f'/dev/{self.gpiochip}',
                    consumer='relay_board_node',
                    config={
                        self.pin_r2: relay_settings,
                        self.pin_r3: relay_settings,
                        self.pin_r4: relay_settings,
                        self.pin_r5: relay_settings,
                        self.pin_r6: relay_settings,
                        self.pin_r7: relay_settings,
                    }
                )

                self.hw_ready = True
                self.get_logger().info('GPIO lines acquired successfully!')
            
            except Exception as e:
                self.get_logger().error(f'GPIO initialization failed: {e}')

        # Create the subscription
        self.subscription = self.create_subscription(
            RelayCommand,
            'relay_board/command',
            self.listener_callback,
            10
        )

        # Create the publisher
        self.relay_state_publisher = self.create_publisher(
            Int32MultiArray, '/relay_board/state', 10
        )

        # publish initial state
        self.publish_state()
        self.get_logger().info('Relay board node ready')



    
    # Callback function - set the status of the relay board
    def listener_callback(self, msg):

        # The motor controller who's channels are being addressed
        mc_number = msg.motor_controller # Named field, no indexing required
        # The channel (1 or 2) which the relays will transmit to the motor
        channel = msg.channel # Named field, no indexing required

        if mc_number not in self.mc_relay_map:
            self.get_logger().warn(
                f'[relay_board_node] Invalid motor controller: {mc_number}'
            )
            return

        if channel not in (1, 2):
            self.get_logger().warn(
                f'[relay_board_node] Invalid channel: {channel}'
            )
            return

        
        # Safety interlock — will connect to Test Manager state in Phase 3
        # if self.require_safe_state and not self.safe_to_command:
        #     self.get_logger().warn('Command rejected — system not in safe state')
        #     return

        # Get the pins from the map
        pin_positive, pin_negative = self.mc_relay_map[mc_number]

        # Set the pins: pins are Value.INACTIVE if channel = 1 or pins are Value.ACTIVE if the channel = 2
        if self.hw_ready:
            target = Value.ACTIVE if channel == 2 else Value.INACTIVE
            # Request the change in the status for each pin, this will actuate the relays
            # They're always actuated together so that there is never a mismatch
            self.gpio_request.set_values({ 
                pin_positive: target,
                pin_negative: target,
            })

        # Update listing of motor controller channels
        self.current_channel[mc_number] = channel
        self.get_logger().info(
            f'MC{mc_number} set to channel {channel} '
            f'(pins {pin_positive}, {pin_negative})'
        )
        self.publish_state()
    
    # Publish the state of the relay board
    def publish_state(self):
        msg = Int32MultiArray()
        msg.data = [
            self.current_channel[1],
            self.current_channel[2],
            self.current_channel[3],
        ]
        self.relay_state_publisher.publish(msg)

    # Clean up
    def destroy_node(self):
        if self.gpio_request is not None:
            # Maybe set all lines to low here
            self.gpio_request.release()
            self.get_logger().info('GPIO lines released')
        super().destroy_node()


# Main function
def main(args=None):
    rclpy.init(args=args)
    node = RelayBoardNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
