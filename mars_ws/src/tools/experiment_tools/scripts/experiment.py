#!/usr/bin/python3

import rospy
import subprocess
import time
import signal
import os
import math

# List to keep track of subprocesses
processes = []

def run_command(command, delay, suppress_output=False):
    if suppress_output:
        process = subprocess.Popen(command, shell=True, executable='/bin/bash', stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    else:
        process = subprocess.Popen(command, shell=True, executable='/bin/bash')
    processes.append(process)
    time.sleep(delay)

def main():
    rospy.init_node('simulation')

    use_yolo = 0

    # Experiment parameters
    robot_mid_x = -5.09434127808
    robot_mid_y = -13.2973918915
    robot_mid_rot_z = -0.373237511573
    robot_mid_rot_w = 0.927735824443

    robot_end_x = 15.721524238586426
    robot_end_y = -41.202877044677734
    robot_end_rot_z = 0.3748413235623322
    robot_end_rot_w = 0.9270889828652042

    robot_init_pose_x = -5.19388580322
    robot_init_pose_y = -13.4218225479
    robot_init_pose_rot_z = -0.649630225132
    robot_init_pose_rot_w = 0.760250334163
    covariance = '[0.5, 0.0, 0.0, 0.0, 0.0, 0.0, \
               0.0, 0.5, 0.0, 0.0, 0.0, 0.0, \
               0.0, 0.0, 0.5, 0.0, 0.0, 0.0, \
               0.0, 0.0, 0.0, 1.0, 0.0, 0.0, \
               0.0, 0.0, 0.0, 0.0, 1.0, 0.0, \
               0.0, 0.0, 0.0, 0.0, 0.0, 1.0]'

    # List of commands with their respective delays in seconds
    commands = [
        (f"roslaunch cohan_navigation nav.launch &", 1, True),
        (f"rosrun experiment_tools data_collection.py", 1, False),
        (f"rosrun experiment_tools main.py", 1, False),
        (f"roslaunch hyper_system hyper_controller.launch", 1, False)
    ]

    for command, delay, suppress_output in commands:
        run_command(command, delay, suppress_output)

    goal_command = f"""
    rostopic pub /move_base/goal move_base_msgs/MoveBaseActionGoal \\
    '{{header: {{stamp: now, frame_id: "map"}}, goal: {{target_pose: {{header: {{frame_id: "map"}}, pose: {{position: {{x: {robot_end_x}, y: {robot_end_y}, z: 0.0}}, orientation: {{x: 0.0, y: 0.0, z: {robot_end_rot_z}, w: {robot_end_rot_w}}}}}}}}}}}' &
    """

    while True:
        user_input = input("Enter '1' to publish goal and /nav_state_gt true, '2' to publish /door_crossing true, '3' to publish /door_crossing false, '4' to publish /nav_state_gt false,, or 'q' to quit: ")

        if user_input == '1':
            run_command("rosbag record -a  -x \"/camera1.*\"", 0)
            run_command(goal_command, 0)
            run_command("rostopic pub /nav_state_gt std_msgs/Bool \"data: true\"", 0)
        elif user_input == '2':
            run_command("rostopic pub /door_crossing std_msgs/Bool \"data: true\"", 0)
        elif user_input == '3':
            run_command("rostopic pub /door_crossing std_msgs/Bool \"data: false\"", 0)
        elif user_input == '4':
            run_command("rostopic pub /nav_state_gt std_msgs/Bool \"data: false\"", 0)
        elif user_input.lower() == 'q':
            print("Exiting...")
            break
        else:
            print("Invalid input. Please enter '1', '2', '3', '4', or 'q'.")

if __name__ == '__main__':
    try:
        main()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Simulation terminated.")
