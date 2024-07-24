#!/usr/bin/python3

import rospy
import subprocess
import time
import signal
import os
import math

# List to keep track of subprocesses
processes = []

def run_command(command, delay):
    process = subprocess.Popen(command, shell=True)
    processes.append(process)
    time.sleep(delay)

def main():
    rospy.init_node('simulation1')

    # Define the initial positions and rotation
    robot_init_x = 21.5
    robot_init_y = -9.0
    robot_init_rot = 1.57

    # Define the final positions and rotation
    robot_end_x = 21.5
    robot_end_y = 7.0

    covariance = '[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]'
    qz = math.sin(robot_init_rot / 2.0)
    qw = math.cos(robot_init_rot / 2.0)

    # static_human = '18.50,-1.00;22.0,-4.00'
    static_human =''
    scene_file = 'Scene15'

    # List of commands with their respective delays in seconds
    commands = [
        # ("roslaunch pedsim_gazebo_plugin shopping_mall.launch &", 3),
        (f"roslaunch mars_lite_description spawn_mars.launch robot_init_x:={robot_init_x} robot_init_y:={robot_init_y} robot_init_rot:={robot_init_rot} &", 3),
        ("roslaunch cohan_navigation nav_cohan.launch &", 3),
        (f"rostopic pub /initialpose geometry_msgs/PoseWithCovarianceStamped '{{header: {{stamp: now, frame_id: \"map\"}}, pose: {{pose: {{position: {{x: {robot_init_x}, y: {robot_init_y}, z: 0.0}}, orientation: {{x: 0.0, y: 0.0, z: {qz}, w: {qw}}}}}, covariance: {covariance}}}}}'", 0),
        ("rosrun hyper_system navigation_state_monitor.py", 0),
        ("roslaunch hyper_system hyper_controller.launch", 1),
        (f"roslaunch pedsim_simulator shopping_mall.launch scene_file:={scene_file} &", 3),
        (f"rosrun stage_ros human_pub.py --static_human='{static_human}' &", 1),
        (f"rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '{{header: {{stamp: now, frame_id: \"map\"}}, pose: {{position: {{x: {robot_end_x}, y: {robot_end_y}, z: 0.0}}, orientation: {{x: 0.0, y: 0.0, z: {qz}, w: {qw}}}}}}}' &", 0),
        ("rosservice call /pedsim_simulator/unpause_simulation '{}'", 5),
    ]

    for command, delay in commands:
        run_command(command, delay)

if __name__ == '__main__':
    try:
        main()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Simulation1 terminated.")
