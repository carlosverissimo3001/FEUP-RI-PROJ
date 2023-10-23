#!/usr/bin/env python3
#
# This is the script that subscribes to /scan and publishes the data to /cmd_vel
# /cmd_vel's type is Twist, which has two fields: linear and angular
# /scan's type is LaserScan, which has one field: ranges
# one way to solve this, is too first wander until you get close to an obstacle

import rclpy
import time
import math
import random
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

# Constants
APPROACH_THRESHOLD = 1.0
MAX_LINEAR_VELOCITY = 0.22
MAX_ANGULAR_VELOCITY = 2.84
NORMAL_LINEAR_VELOCITY = 0.1
NORMAL_ANGULAR_VELOCITY = 0.75

WALL_LENGTH = 2.25
WALL_LENGTH_ERROR = 0.1 * WALL_LENGTH

SCAN_FRONT = 0
SCAN_FRONT_LEFT = 45
SCAN_LEFT = 90
ERROR_OFFSET = 0.01

DEBUG_MODE = True

def debug_print(string):
    if DEBUG_MODE:
        print(string)

class GetInitialYawNode(Node):
    def __init__(self):
        super().__init__('get_initial_yaw_node')

        # Subscrive to /odom topic to get the initial yaw
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10  # QoS profile, adjust as needed
        )

        self.subscription  # Prevent unused variable warning

    def odom_callback(self, odom_msg: Odometry):
        # get the initial yaw -> pose.orientation.z
        initial_yaw = odom_msg.pose.pose.orientation.z

        print("Initial yaw: ", initial_yaw)
        return initial_yaw

class SpinRandomlyNode(Node):
    def __init__(self):
        super().__init__('spin_randomly_node')

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

    def scan_callback(self, scan_msg: LaserScan):
        cmd_vel = Twist()
        
        # just spin randomly
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = float(random.uniform(-1, 1))
        
        # Publish the message
        self.publisher.publish(cmd_vel)

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

        self.start_time = 0
        self.end_time = 0 

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
            print("I'm lost")
            return True
        
        return False
    
    def check_final_position(self, ranges):
        # Get the indexes of the sensors with data
        indexes = [x for x in range(len(ranges)) if not math.isinf(ranges[x])]
        
        # get indexes
        leftest_sensor = min(indexes)    
        rightest_sensor = max(indexes)
    
        # get distances
        leftest_sensor_dist = ranges[leftest_sensor]
        rightest_sensor_dist = ranges[rightest_sensor]

        if abs(leftest_sensor_dist - rightest_sensor_dist) > 10 * ERROR_OFFSET:
            return False
        

        middle_angle = abs(leftest_sensor - rightest_sensor)
        left_angle = (180 - middle_angle) / 2
        right_angle = (180 - middle_angle) / 2


        if (middle_angle > 180):
            return False

        wall_length = leftest_sensor_dist * math.sin(math.radians(middle_angle)) / math.sin(math.radians(left_angle))

        print("Wall length: ", wall_length)

        if abs(wall_length - WALL_LENGTH) < WALL_LENGTH_ERROR:
            print("Found the ending spot")
            
            # stop counting time
            if (self.end_time == 0):
                self.end_time = time.time()

            return True

        return False

        #print("Min index: ", min_index)
        #print("Max index: ", max_index)

        
    def get_closest_wall_direction(self, scan_data):
        # get the index of the closest obstacle
        min_range_index = scan_data.ranges.index(min(scan_data.ranges))

        # get the cardinal direction of the closest obstacle
        for direction in self.directions:
            if min_range_index in self.directions[direction]:
                return direction

    def get_closest_wall_degree(self, ranges):
        # get the index of the closest obstacle
        return ranges.index(min(ranges))
    
    def follow_wall(self, ranges):
        print("\nInside follow_wall")

        if math.isinf(ranges[SCAN_LEFT]):
            closest_sensor = self.get_closest_wall_degree(ranges)
            print("Closest wall index: ", closest_sensor)
            print("Left sensor data is inf")

            if closest_sensor > 270 or closest_sensor < 90: 
                return 0.0, -1 * NORMAL_ANGULAR_VELOCITY
            
            return 0.0, NORMAL_ANGULAR_VELOCITY            
                            
        # if the left sensor is too close to the wall
        if ranges[SCAN_LEFT] < 0.5 * APPROACH_THRESHOLD:
            # rotate left
            print("rotate left")
            return NORMAL_LINEAR_VELOCITY, -1 * NORMAL_ANGULAR_VELOCITY


        parallel_to_wall = abs(ranges[SCAN_LEFT - 1] - ranges[SCAN_LEFT + 1]) < ERROR_OFFSET  

        if parallel_to_wall:
            # all good, keep going
            print("all good, keep going")
            return 2 * NORMAL_LINEAR_VELOCITY, 0.0
        
        # Robot positioning is bad
        # should go foward and rotate in order to move closer to the wall

        if ranges[SCAN_LEFT - 1] > ranges[SCAN_LEFT + 1]:
            # rotate right
            print("rotate right")
            return NORMAL_LINEAR_VELOCITY, NORMAL_ANGULAR_VELOCITY
        
        # rotate left
        print("rotate left")
        return NORMAL_LINEAR_VELOCITY, -1 * NORMAL_ANGULAR_VELOCITY
    


    def scan_callback(self, scan_msg: LaserScan):
        cmd_vel = Twist()
        cmd_vel.linear.x, cmd_vel.angular.z = 0.0, 0.0

        # am i at the end?
        if self.check_final_position(scan_msg.ranges):
            
            print("Time elapsed: ", self.end_time - self.start_time)
            
            # do nothing
            self.publisher.publish(cmd_vel)
            return

        # lost?
        if self.am_i_lost(scan_msg):
            print("I'm lost")
            cmd_vel.linear.x = NORMAL_LINEAR_VELOCITY
            self.publisher.publish(cmd_vel)
            return

        # Still not close enough to an obstacle, so keep wandering
        if min(scan_msg.ranges) > APPROACH_THRESHOLD:
            print("\nGoing straight")
            # find out where the closest obstacle is, in terms of cardinal direction
            obstacle_direction = self.get_closest_wall_direction(scan_msg)

            closest_sensor = self.get_closest_wall_degree(scan_msg.ranges)
            print("Closest wall index: ", closest_sensor)

            # Closest wall is in front of us, so go straight
            if closest_sensor in self.directions["front"]:
                cmd_vel.linear.x = 2 * NORMAL_LINEAR_VELOCITY
                self.publisher.publish(cmd_vel)
                return
        
            # Closest wall is to the left, so turn right
            if closest_sensor > 180:
                cmd_vel.angular.z = -1 * NORMAL_ANGULAR_VELOCITY
            
            # Closest wall is to the right, so turn left
            else:
                cmd_vel.angular.z = NORMAL_ANGULAR_VELOCITY
            
            # Go straight, but realllly slow
            cmd_vel.linear.x = NORMAL_LINEAR_VELOCITY

        # ok, we're close to an obstacle, so let's follow it
        else:
            print ("Close wall detected, trying to follow it")

            cmd_vel.linear.x, cmd_vel.angular.z = self.follow_wall(scan_msg.ranges)
            self.publisher.publish(cmd_vel)
            return

        # Apply safety limits to linear and angular velocities
        cmd_vel.linear.x = min(MAX_LINEAR_VELOCITY, cmd_vel.linear.x)
        cmd_vel.angular.z = min(MAX_ANGULAR_VELOCITY, cmd_vel.angular.z)    

        self.publisher.publish(cmd_vel)

def main(args=None):
    """ # spin randomly first,
    rclpy.init(args=args)
    node = SpinRandomlyNode()
    rclpy.spin_once(node)
    rclpy.shutdown()

    # for 5 seconds
    time.sleep(5) """

    # get the initial yaw
    """ rclpy.init(args=args)
    node = GetInitialYawNode()
    rclpy.spin_once(node)
    rclpy.shutdown() """
    
    # then proceed normal operation
    rclpy.init(args=args)
    node = ScanToVelocityNode()
    node.start_time = time.time()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

