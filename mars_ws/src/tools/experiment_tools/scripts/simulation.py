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
    process = subprocess.Popen(command, shell=True, executable='/bin/bash')
    processes.append(process)
    time.sleep(delay)

def main():
    rospy.init_node('simulation')

    use_yolo = 0

    # Simulation1
    robot_init_x = 21.5
    robot_init_y = -9.0
    robot_init_rot = 1.57
    robot_end_x = 21.5
    robot_end_y = 7.5
    robot_end_rot = 1.57
    static_human ='20.5,5.0'
    scene_file = 'Simulation1'

    # Simulation2
    # robot_init_x = 0.0
    # robot_init_y = -15.5
    # robot_init_rot = 1.57
    # robot_end_x = -2.3
    # robot_end_y = 0.6
    # robot_end_rot = 3.14
    # # static_human ='-4.70,-2.60;-2.70,-2.60;1.40,-2.60;3.40,-2.60;-1,1.55;0.25,1.55;-2.2,-0.4;1.42,-0.4;-4.6,-0.4;3.7,-0.4;-3,1.55;2.2,1.55'
    # static_human ='-4.70,-2.60;-2.70,-2.60;1.40,-2.60;3.40,-2.60;-4.6,-0.4;-2.2,-0.4;1.42,-0.4;3.7,-0.4;-3,1.55;-1,1.55;0.25,1.55;2.2,1.55'
    # # static_human = '-2.97,1.68;-1.04,1.68;0.387,1.742;2.217,1.658;-4.588,-0.346;-2.487,-0.315;1.55,-0.27;3.734,-0.33;-4.426,-2.612;-2.568,-2.584;1.674,-2.533;3.712,-2.550'
    # scene_file = 'Simulation2'

    # Simulation3
    #####
    # result : 
    # our | path -> our_117 data -> 65
    #####
    # robot_init_x = -4.5
    # robot_init_y = -3.0
    # robot_init_rot = 0.0
    # robot_end_x = 12.7
    # robot_end_y = -34.0
    # robot_end_rot = 4.71
    # static_human ='-1.892,-5.294;1.733,-8.925'
    # scene_file = 'Simulation3'

    # Simulation4
    #####
    # result : 
    # our | path -> our_119 data -> 67
    #####
    # robot_init_x = 1.0
    # robot_init_y = 7.45
    # robot_init_rot = 3.14
    # robot_end_x = -27
    # robot_end_y = -17
    # robot_end_rot = 3.14
    # static_human ='-4.70,-2.60;-2.70,-2.60;1.40,-2.60;3.40,-2.60;-4.6,-0.4;-2.2,-0.4;1.42,-0.4;3.7,-0.4;-3,1.55;-1.2,1.55;0.7,1.55;2.2,1.55;-7.75,3.15;-7.75,0.85;-7.75,-1.3;5.8,3.60;5.8,1.00;5.8,-1.3;-6.0,2.0;5.0,2.0;8.38,11.1;7.71,11.1;8.0,10.5;9.6,12.3'
    # # static_human =''
    # scene_file = 'Scene8'
    
    covariance = '[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]'
    qz_i = math.sin(robot_init_rot / 2.0)
    qw_i = math.cos(robot_init_rot / 2.0)
    qz_e = math.sin(robot_end_rot / 2.0)
    qw_e = math.cos(robot_end_rot / 2.0)


    # List of commands with their respective delays in seconds
    commands = [
        # ("roslaunch pedsim_gazebo_plugin shopping_mall.launch world:=simulation3 &", 3),
        (f"roslaunch mars_lite_description spawn_mars.launch robot_init_x:={robot_init_x} robot_init_y:={robot_init_y} robot_init_rot:={robot_init_rot} &", 3),
        ("roslaunch cohan_navigation nav_cohan.launch &", 3),
        (f"rostopic pub /initialpose geometry_msgs/PoseWithCovarianceStamped '{{header: {{stamp: now, frame_id: \"map\"}}, pose: {{pose: {{position: {{x: {robot_init_x}, y: {robot_init_y}, z: 0.0}}, orientation: {{x: 0.0, y: 0.0, z: {qz_i}, w: {qw_i}}}}}, covariance: {covariance}}}}}'", 1),
        ("roslaunch hyper_system hyper_controller.launch", 1),
        (f"roslaunch pedsim_simulator shopping_mall.launch scene_file:={scene_file} &", 3),
        (f"rosrun stage_ros human_pub.py --static_human='{static_human}' --pub_tracked_human={int(not use_yolo)} &", 1),
        # (f"rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '{{header: {{stamp: now, frame_id: \"map\"}}, pose: {{position: {{x: {robot_end_x}, y: {robot_end_y}, z: 0.0}}, orientation: {{x: 0.0, y: 0.0, z: {qz_e}, w: {qw_e}}}}}}}' &", 0),
        # ("rosservice call /pedsim_simulator/unpause_simulation '{}'", 5),
    ]

    if use_yolo:
        commands.insert(4, ("source /home/developer/lab/socially-store-robot/yolo_ws/devel/setup.bash && roslaunch scan yolodetect.launch &", 0))
    
    for command, delay in commands:
        run_command(command, delay)

    input("Press 'y' to proceed with goal publishing and simulation unpause... ")
    
    goal_command = f"rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '{{header: {{stamp: now, frame_id: \"map\"}}, pose: {{position: {{x: {robot_end_x}, y: {robot_end_y}, z: 0.0}}, orientation: {{x: 0.0, y: 0.0, z: {qz_e}, w: {qw_e}}}}}}}' &"
    unpause_command = "rosservice call /pedsim_simulator/unpause_simulation '{}'"

    run_command(goal_command, 0)
    run_command(unpause_command, 0)

if __name__ == '__main__':
    try:
        main()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Simulation1 terminated.")
