#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Joep Tool

import os
import random
import math

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_random_coordinates(center_x, center_y, radius):
    # Generate random angles for polar coordinates
    angle = random.uniform(0, 2 * math.pi)
    # Generate a random value between 0 and the radius
    r = random.uniform(0, radius)
    
    # Calculate x and y coordinates based on polar coordinates
    x = round(center_x + r * math.cos(angle), 1)
    y = round(center_y + r * math.sin(angle), 1)
    
    return x, y


def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')


    x_pose = LaunchConfiguration('x_pose', default='2')
    y_pose = LaunchConfiguration('y_pose', default='2.5')


    #print("Spawning turtlebot at x: {}, y: {}".format(x_pose, y_pose))

    # Get the path to the world file
    world = os.path.join(
        get_package_share_directory('my_reactive_robot'),
        'worlds',
        'my_world.world'
    )

    # Start the Gazebo server
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    # Start the Gazebo client
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # Publish the robot state
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Spawn the turtlebot at the generated coordinates
    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    # Start the controller node (my_reactive_robot_controller package)
    # ros2 run my_controller controller_
    controller_cmd = Node(
        package='my_controller',
        executable='controller',
        name='controller_node',
        output='screen'
    )


    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    # ld.add_action(controller_cmd)

    print(spawn_turtlebot_cmd)

    return ld