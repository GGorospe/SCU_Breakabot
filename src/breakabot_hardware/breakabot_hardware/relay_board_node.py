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



# RelayBoardNode class
class RelayBoardNode(Node):

    def __init__(self):
        super().__init__('relay_board_node')
        
        # Relay board node parmeters (declared and set)
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('frame_id', 'imu_link')

        rate = self.get_parameter('publish_rate_hz').value
        self.frame_id = self.get_parameter('frame_id').value

        period = 1.0 / rate
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(
            f'Relay board node started - rate: {rate} Hz, frame: {self.frame_id}'
        )
    
    # Callback function 
    def timer_callback(self):
        self.get_logger().info('timer_callback fired', throttle_duration_sec=1.0)


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
