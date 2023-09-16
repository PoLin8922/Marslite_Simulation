#!/bin/bash

# Function to clean up on Ctrl+C
cleanup() {
    echo "Received Ctrl+C, cleaning up..."
    killall gzserver
    exit
}
trap cleanup INT

# Start the first ROS launch command
# roslaunch pedsim_simulator gym_crowd_environment.launch scene_file:=crossing_corridor.xml &

sleep 3
roslaunch pedsim_gazebo_plugin crossing_corridor.launch &

sleep 3
roslaunch mars_lite_description spawn_mars.launch &

sleep 3
rosservice call /pedsim_simulator/unpause_simulation "{}"

# sleep 3
# roslaunch turtlebot3_navigation nav_origin.launch map_file:=crossing_corrider.yaml &

# Keep the script running to maintain the ROS processes
wait
