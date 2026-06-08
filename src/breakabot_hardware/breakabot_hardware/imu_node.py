# File: imu_node.py
# Author: George Gorospe, ggorospe@scu.edu
# About: This node is the interfaces with the IMU hardware and publishes information from polling the IMU.
# Hardware: Adafruit 9-DOF Absolute Orientation IMU Fusion Breakout - BNO055


# Importing required libraries 
# Standard library
import math
import time
# ROS libraries:
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu # Standard ROS message for IMUs: orientation quaternio, angular velocity, & linear acceleration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy # Used to specify the communication behavior in this node
from std_msgs.msg import Header

# The following try/except structure enables development and testing of the code on laptop vs RPi
try: 
    # Hardware (IMU) program libraries
    import board
    import busio
    import adafruit_bno055
except ImportError:
        HW_AVAILABLE = False

if not HW_AVAILABLE:
    self.get_logger().warn(
        'Hardware libraries not found — running in stub mode'
    )

# Class definition
class ImuNode(Node): # This class inherits from the ROS2 Node class
    def __init__(self):
        super().__init__('imu_node') # Registers the node with the ROS2 Runtime, MANDITORY.

        # Declare parameters (values can be overridden from YAML or command line)
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('i2c_address', 0x28)

        # Read them back
        self.rate = self.get_parameter('publish_rate_hz').value
        self.frame_id = self.get_parameter('frame_id').value

        # Set up the hardware (stage 2 of 2 hardware check)
        try: # Implementing a try/except structure to catch hardware/wiring errors
            i2c = busio.I2C(board.SCL, board.SDA)
            self.sensor = adafruit_bno055.BNO055_I2C(i2c)
            self.hw_ready = True
            self.get_logger().info('BNO055 initialized successfully')
        except Exception as e:
            self.hw_ready = False
            self.sensor = None
            self.get_logger().error(f'BNO055 initialization failed: {e}')

        # Create the publisher (ROS2 Primitive)
        # First define the quality of service settings/policy for tuning communcation between nodes
        # "BEST_EFFORT" is a lossy
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, # attempt to deliver, but may loose some 
            history=HistoryPolicy.KEEP_LAST, # Only store up to N samples (depth)
            depth=10
        )
        self.imu_publisher = self.create_publisher(Imu, '/imu/data', qos)

        # Create the timer — this drives the publish loop (ROS2 Primitive)
        period = 1.0 / self.rate
        self.timer = self.create_timer(period, self.timer_callback)

        
        # Startup confirmation
        self.get_logger().info(
            f'IMU node ready — '
            f'rate: {self.rate} Hz  '
            f'frame: {self.frame_id}  '
            f'hw: {"ok" if self.hw_ready else "UNAVAILABLE"}'
        )

    # Callback function definition
    def timer_callback(self):
        # If hardware was not found, maybe a wiring issue?
        if not self.hw_ready: # node runs, publishes nothing, logs the issue - clean no-op
            self.get_logger().info(
                'timer_callback fired — no hardware',
                throttle_duration_sec=1.0
            )
            return 
        
        # Otherwise continue as normal if hardware is ready
        msg = Imu() # Standard message format for IMU within ROS. (empty)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # Read the orientation quaternion from BNO055 (9-DoF Accelerometer)
        # BNO055 returns (w, x, y, z); sensor_msgs/Imu uses (x, y, z, w)
        quat = self.sensor.quaternion
        if quat is not None: # Load the Imu message with values from the sensor
            msg.orientation.w = quat[0]
            msg.orientation.x = quat[1]
            msg.orientation.y = quat[2]
            msg.orientation.z = quat[3]

            # Sanity check: a valid quaternion has unit norm
            norm = math.sqrt(
                quat[0]**2 + quat[1]**2 + quat[2]**2 + quat[3]**2
            )
            if abs(norm - 1.0) > 0.05:
                self.get_logger().warn(
                    f'Quaternion norm out of range: {norm:.3f}',
                    throttle_duration_sec=5.0
                )
        
        # Covariance unknown — zeros by ROS2 convention
        msg.orientation_covariance = [0.0] * 9

        # Read the Angular velocity (rad/s) from the BNO055
        gyro = self.sensor.gyro
        if gyro is not None: # Load the Imu message with values from the sensor
            msg.angular_velocity.x = gyro[0]
            msg.angular_velocity.y = gyro[1]
            msg.angular_velocity.z = gyro[2]
        msg.angular_velocity_covariance = [0.0] * 9

        # Read the Linear acceleration (m/s²) from the BNO055
        # Gravity removed, best to use this for vehicle motion analysis.
        accel = self.sensor.linear_acceleration
        if accel is not None: # Load the Imu message with the values from the sensor
            msg.linear_acceleration.x = accel[0]
            msg.linear_acceleration.y = accel[1]
            msg.linear_acceleration.z = accel[2]
        msg.linear_acceleration_covariance = [0.0] * 9

        # Publish the message
        self.imu_publisher.publish(msg)

# Main Function:
def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()