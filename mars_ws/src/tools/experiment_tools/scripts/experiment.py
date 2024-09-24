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
    # --------------------------------------------

    robot_mid_x_1 = -4.97444105148
    robot_mid_y_1 = -4.13838386536
    robot_mid_rot_z_1 = -0.981322880131
    robot_mid_rot_w_1 = 0.192367889556

    # --------------------------------------------
    ## 1
    # robot_mid_x_2 = -6.46571063995
    # robot_mid_y_2 = -5.17753410339
    ## 2
    # robot_mid_x_2 = -6.24689626694
    # robot_mid_y_2 = -5.50535202026
    ## 3
    # robot_mid_x_2 = -5.05376911163
    # robot_mid_y_2 = -6.67416858673
    
    # robot_mid_rot_z_2 = -0.338229025333
    # robot_mid_rot_w_2 = 0.941063826965

    ## 4
    robot_mid_x_2 = -3.66008400917
    robot_mid_y_2 = -8.99586009979
    robot_mid_rot_z_2 = -0.817666294174
    robot_mid_rot_w_2 = 0.575692479863

    # --------------------------------------------

    robot_end_x = 16.70
    robot_end_y = -41.90
    robot_end_rot_z = 0.3748413235623322
    robot_end_rot_w = 0.9270889828652042

    # --------------------------------------------

    # robot_init_pose_x = -5.19388580322
    # robot_init_pose_y = -13.4218225479
    # robot_init_pose_rot_z = -0.649630225132
    # robot_init_pose_rot_w = 0.760250334163
    # covariance = '[0.5, 0.0, 0.0, 0.0, 0.0, 0.0, \
    #            0.0, 0.5, 0.0, 0.0, 0.0, 0.0, \
    #            0.0, 0.0, 0.5, 0.0, 0.0, 0.0, \
    #            0.0, 0.0, 0.0, 1.0, 0.0, 0.0, \
    #            0.0, 0.0, 0.0, 0.0, 1.0, 0.0, \
    #            0.0, 0.0, 0.0, 0.0, 0.0, 1.0]'

    # static_human ='-4.5549082756,-4.36297702789;-5.67179727554,-3.30626940727;-6.06984233856,-1.8177126646'
    static_human ='-5.16965198517,-5.17823123932;-4.38030338287,-6.03064537048;-3.56143164635,-6.81289148331;-2.6241736412, -7.0585641861'
    use_yolo = 0
    # time_bias = 61.125231981277466
    time_bias = 0.0

    # List of commands with their respective delays in seconds
    commands = [
        # (f"roslaunch cohan_navigation nav.launch &", 1, True),
        # (f"roslaunch hyper_system hyper_controller.launch", 1, False),
        (f"rosrun experiment_tools data_collection.py {time_bias}", 0, False),
        (f"rosrun experiment_tools main.py", 0, False),
        # (f"rosrun stage_ros human_pub.py --static_human='{static_human}' --pub_tracked_human={int(not use_yolo)} &", 1, False),
        # (f"rosrun stage_ros coatmap_clear.py", 1, True)
    ]

    for command, delay, suppress_output in commands:
        run_command(command, delay, suppress_output)

    mid_1_command = f"""
    rostopic pub /move_base/goal move_base_msgs/MoveBaseActionGoal \\
    '{{header: {{stamp: now, frame_id: "map"}}, goal: {{target_pose: {{header: {{frame_id: "map"}}, pose: {{position: {{x: {robot_mid_x_1}, y: {robot_mid_y_1}, z: 0.0}}, orientation: {{x: 0.0, y: 0.0, z: {robot_mid_rot_z_1}, w: {robot_mid_rot_w_1}}}}}}}}}}}' &
    """

    mid_2_command = f"""
    rostopic pub /move_base/goal move_base_msgs/MoveBaseActionGoal \\
    '{{header: {{stamp: now, frame_id: "map"}}, goal: {{target_pose: {{header: {{frame_id: "map"}}, pose: {{position: {{x: {robot_mid_x_2}, y: {robot_mid_y_2}, z: 0.0}}, orientation: {{x: 0.0, y: 0.0, z: {robot_mid_rot_z_2}, w: {robot_mid_rot_w_2}}}}}}}}}}}' &
    """

    goal_command = f"""
    rostopic pub /move_base/goal move_base_msgs/MoveBaseActionGoal \\
    '{{header: {{stamp: now, frame_id: "map"}}, goal: {{target_pose: {{header: {{frame_id: "map"}}, pose: {{position: {{x: {robot_end_x}, y: {robot_end_y}, z: 0.0}}, orientation: {{x: 0.0, y: 0.0, z: {robot_end_rot_z}, w: {robot_end_rot_w}}}}}}}}}}}' &
    """

    # bag_command = "rosbag record /camera1/color/image_raw /detection_image /map /move_base/HATebLocalPlannerROS/global_plan /move_base/HATebLocalPlannerROS/local_plan /robot_path /move_base/global_costmap/footprint /move_base/navigability_costmap/costmap /move_base/displayed_costmap/costmap /move_base/local_costmap/costmap /move_base/global_costmap/costmap /move_base/HATebLocalPlannerROS/human_arrow /tracked_humans"
    bag_command = "rosbag record /camera1/color/image_raw /detection_image /map /move_base/HATebLocalPlannerROS/global_plan /move_base/HATebLocalPlannerROS/local_plan /robot_path /move_base/global_costmap/footprint  /move_base/displayed_costmap/costmap   /move_base/HATebLocalPlannerROS/human_arrow"

    while True:
        user_input = input("")

        if user_input == '1':
            # run_command("rosbag record -a  -x \"/camera1.*\"", 4.5)
            # run_command("rosbag record -a ", 4.5)
            # run_command(bag_command, 4)
            run_command("rostopic pub -1 /nav_state_gt std_msgs/Bool \"data: true\"", 0)
            run_command(goal_command, 0)
        elif user_input == '2':
            run_command("rostopic pub -1 /door_crossing std_msgs/Bool \"data: true\"", 0)
        elif user_input == '3':
            run_command("rostopic pub -1 /door_crossing std_msgs/Bool \"data: false\"", 0)
        elif user_input == '4':
            run_command("rostopic pub -1 /nav_state_gt std_msgs/Bool \"data: false\"", 0)
            run_command('rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID "{}"', 0)
        elif user_input == '5':
            run_command(bag_command, 0)
        elif user_input == '6':
            run_command("rostopic pub -1 /nav_state_gt std_msgs/Bool \"data: true\"", 0)
            run_command(mid_2_command, 0)
        elif user_input == '7':
            run_command("rosservice call /move_base/clear_costmaps", 0)
        elif user_input == '8':
            run_command('rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID "{}"', 0)
        elif user_input == '9':
            run_command(mid_1_command, 0)
        elif user_input == 'z':
            run_command("rostopic pub -1 /human_crossing std_msgs/Bool \"data: true\"", 0)
        elif user_input == 'x':
            run_command("rostopic pub -1 /human_crossing std_msgs/Bool \"data: false\"", 0)
        elif user_input.lower() == 'q':
            print("Exiting...")
            break
        else:
            print("Invalid input. Please enter '1', '2', '3', '4', or 'q'.")
        time.sleep(0.1)

if __name__ == '__main__':
    try:
        main()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Simulation terminated.")