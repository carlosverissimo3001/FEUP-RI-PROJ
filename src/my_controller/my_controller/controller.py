#!/usr/bin/env python3
#
# This is the script that subscribes to /scan and publishes the data to /cmd_vel
# /cmd_vel's type is Twist, which has two fields: linear and angular
# /scan's type is LaserScan, which has one field: ranges
# one way to solve this, is too first wander until you get close to an obstacle

import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# Constants
APPROACH_THRESHOLD = 0.5
MAX_LINEAR_VELOCITY = 0.22
MAX_ANGULAR_VELOCITY = 2.84
NORMAL_LINEAR_VELOCITY = 0.1
NORMAL_ANGULAR_VELOCITY = 0.5

SCAN_LEFT = 90
ERROR_OFFSET = 0.01

DEBUG_MODE = True

wall_kp = 0.1
approach_multiplier = 2.0

def debug_print(string):
    if DEBUG_MODE:
        print(string)

class ScanToVelocityNode(Node):
    def __init__(self):
        super().__init__('scan_to_velocity_node')

        # create a dictionair, if the value is from 0 to 60, then it's on the left
        # if the value is from 60 to 120, then it's in the front, you get it
        self.directions = {
            "front" : list(range(0, 22)) + list(range(337, 360)),
            "front-top" : range(22, 67),
            "top" : range(67, 112),
            "top-back" : range(112, 157),
            "back" : range(157, 202),
            "back-bottom" : range(202, 247),
            "bottom" : range(247, 292),
            "bottom-front" : range(292, 337),
        } 

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

    # If there are no walls around, assume we're lost
    def am_i_lost(self, scan_data):
        # Check if the robot stopped sensing obstacles

        # If the minimum and maximum range are both infinity, then we're lost
        if (math.isinf(min(scan_data.ranges)) and math.isinf(max(scan_data.ranges))):
            debug_print("I'm lost")
            return True
        
        return False
    
    
    def get_angular_speed_for_turn(self, direction):
        # Calculate the angular speed based on the direction of the obstacle
        # If an object is to the left, turn right, and so on
        # TODO: implement this
        
        return 0.0
        

    def get_closest_wall_direction(self, scan_data):
        # get the index of the closest obstacle
        min_range_index = scan_data.ranges.index(min(scan_data.ranges))

        # get the cardinal direction of the closest obstacle
        for direction in self.directions:
            if min_range_index in self.directions[direction]:
                return direction

    def get_closest_wall_degree(self, scan_data):
        # get the index of the closest obstacle
        return scan_data.ranges.index(min(scan_data.ranges))
    
    def follow_wall(self, scan_data):
        debug_print("Inside follow_wall")

        if math.isinf(scan_data[SCAN_LEFT]):
            # Robot positioning is bad
            # should rotate in order for the left side to face the wall 
            debug_print("rotate right")
            return 0, NORMAL_ANGULAR_VELOCITY

        left_dist = scan_data[SCAN_LEFT] - APPROACH_THRESHOLD

        if abs(left_dist) > ERROR_OFFSET:
            # Robot positioning is bad
            # should go foward and rotate in order to move closer to the wall

            if left_dist < 0:
                # too close to the wall 
                # rotate right
                debug_print("rotate right")
                return NORMAL_LINEAR_VELOCITY, NORMAL_ANGULAR_VELOCITY
            
            # too far from the wall 
            # rotate left
            debug_print("rotate left")
            return NORMAL_LINEAR_VELOCITY, -1 * NORMAL_ANGULAR_VELOCITY
            
        parallel_to_wall = scan_data[SCAN_LEFT - 1] - scan_data[SCAN_LEFT + 1] < ERROR_OFFSET  

        if parallel_to_wall:
            # all good, keep going
            debug_print("all good, keep going")
            return NORMAL_LINEAR_VELOCITY, 0
        
        # Robot positioning is bad
        # should go foward and rotate in order to move closer to the wall

        if scan_data[SCAN_LEFT - 1] > scan_data[SCAN_LEFT + 1]:
            # rotate right
            debug_print("rotate right")
            return NORMAL_LINEAR_VELOCITY, NORMAL_ANGULAR_VELOCITY
        
        # rotate left
        debug_print("rotate left")
        return NORMAL_LINEAR_VELOCITY, -1 * NORMAL_ANGULAR_VELOCITY
        



    def scan_callback(self, scan_msg: LaserScan):
        cmd_vel = Twist()
        cmd_vel.linear.x, cmd_vel.angular.z = 0.0, 0.0

        # lost?
        if self.am_i_lost(scan_msg):
            debug_print("I'm lost")
            cmd_vel.linear.x = NORMAL_LINEAR_VELOCITY
            self.publisher.publish(cmd_vel)
            return

        # Still not close enough to an obstacle, so keep wandering
        if min(scan_msg.ranges) > APPROACH_THRESHOLD:
            # find out where the closest obstacle is, in terms of cardinal direction
            obstacle_direction = self.get_closest_wall_direction(scan_msg)

            debug_print("The closest obstacle is to the", obstacle_direction, 
                  " at ", round(min(scan_msg.ranges),2), "meters", 
                  "degree: ", self.get_closest_wall_degree(scan_msg))
            
            debug_print("Approaching it slowly")

            # Go straight, but realllly slow
            cmd_vel.linear.x = 0.1


        # ok, we're close to an obstacle, so let's follow it
        else:
            debug_print ("Wall detected, trying to follow it")

            cmd_vel.linear.x, cmd_vel.angular.z = self.follow_wall(scan_msg)

            """
            Rafa - Imo apagava isto mas pode dar jeito idk

            # get the direction of the closest obstacle
            obstacle_direction = self.get_closest_wall_direction(scan_msg)

            # calculate the angle needed to avoid the obstacle and keep a safe distance
            cmd_vel.angular.z = wall_kp * (approach_multiplier * min(scan_msg.ranges) - 0.5)

            # safe to go straight
            cmd_vel.linear.x = 0.05
            """

        # Apply safety limits to linear and angular velocities
        cmd_vel.linear.x = min(MAX_LINEAR_VELOCITY, cmd_vel.linear.x)
        cmd_vel.angular.z = min(MAX_ANGULAR_VELOCITY, cmd_vel.angular.z)    

        self.publisher.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    node = ScanToVelocityNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

