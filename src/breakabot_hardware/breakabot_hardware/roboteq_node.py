# File: roboteq__node.py
# Author: George Gorospe, ggorospe@scu.edu
# About: this is the hardware interface node for the RoboteQ SDC2130 motor controller
# Notes: 
#   The break-a-bot features 3 SDC2130 motor controllers, each will be connected to the RPi via USB.
#   The full implementation of the node is modeled on the original by Manoj Sharma, 
#       https://github.com/irahulone/multi_robots/tree/main/pioneer_ws/locomotion_core

# Initial Implementation: STUB

# Importing ROS libraries:
import rclpy
from rclpy.node import Node

# Define the custom class for the roboteq node
class RoboteqNode(Node):

    def __init__(self):
        super().__init__('roboteq_node')

        # Declare parameters (node settings)
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('frame_id', 'roboteq_link')

        # Get/Set parameters
        rate = self.get_parameter('publish_rate_hz').value
        self.frame_id = self.get_parameter('frame_id').value

        # Create timer for callback
        period = 1.0 / rate
        self.timer = self.create_timer(period, self.timer_callback)

        # Log activation 
        self.get_logger().info(
            f'Roboteq Node started - rate: {rate}, frame: {self.frame_id}'
        )

    def timer_callback(self):
        self.get_logger().info('timer_callback fired', throttle_duration_sec=1.0)


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