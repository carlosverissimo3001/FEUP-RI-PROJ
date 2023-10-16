#!/usr/bin/env python3
#
# This is the script that subscribes to /scan and publishes the data to /cmd_vel
# /cmd_vel's type is Twist, which has two fields: linear and angular
# /scan's type is LaserScan, which has one field: ranges

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class ScanToVelocityNode(Node):
    def __init__(self):
        super().__init__('scan_to_velocity_node')
        
        # Subscribe to the /scan topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10  # QoS profile, adjust as needed
        )
        self.subscription  # Prevent unused variable warning

        # Create a publisher for the /cmd_vel topic
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def scan_callback(self, scan_msg):
        cmd_vel = Twist()

        # Process the LiDAR scan data and generate velocity commands
        # For example, simple logic to stop if an obstacle is too close
        """ if min(scan_msg.ranges) < 0.5:  # Change threshold as needed
            cmd_vel.linear.x = 0.0  # Stop forward motion
            cmd_vel.angular.z = 0.2  # Turn left (adjust as needed)
            self.publisher.publish(cmd_vel) """

        cmd_vel.linear.x = 1.0  # Stop forward motion
        cmd_vel.angular.z = 0.2  # Turn left (adjust as needed)
        self.publisher.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    node = ScanToVelocityNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

