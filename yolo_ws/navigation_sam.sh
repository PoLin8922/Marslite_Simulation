#!/bin/bash

# Function to clean up on Ctrl+C
cleanup() {
    echo "Received Ctrl+C, cleaning up..."
    killall gzserver
    exit
}
trap cleanup INT

# Start the first ROS launch command
roslaunch path_finding astar_path_finding_with_social_proxemics.launch

sleep 3
roslaunch path_tracking path_tracking_autonomous.launch

# Keep the script running to maintain the ROS processes
wait
