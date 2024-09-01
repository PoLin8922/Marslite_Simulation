#!/usr/bin/env python3

import rospy
import time
import json
import math
from std_msgs.msg import Bool
from min_dist_calculator import DistanceCalculator
from path_recorder import PositionRecorder
from nav_msgs.msg import Odometry, Path

class MainNode:
    def __init__(self):
        rospy.init_node('main_node', anonymous=True)

        self.is_navigating = False
        self.start_time = None
        self.end_time = None
        self.base_name = 'our'
        self.nav_state_sub = rospy.Subscriber('nav_state_gt', Bool, self.nav_state_callback)

        self.distance_calculator = DistanceCalculator()
        self.position_recorder = PositionRecorder()

        rospy.loginfo("Main node started")

    def nav_state_callback(self, msg):
        if msg.data:  # If navigation is active
            if not self.is_navigating:
                print("Navigation started.")
                self.start_time = time.time()
                self.is_navigating = True
                # self.distance_calculator.is_navigating = True
                self.position_recorder.is_navigating = True
                # self.distance_calculator.min_distance = float('inf')
                # self.distance_calculator.current_human_positions = []  
                self.position_recorder.positions = []  # Start a new path
                self.position_recorder.base_name = self.base_name
                self.position_recorder.file_name = self.position_recorder.get_next_file_name()  # Get new file name
                self.position_recorder.path_msg = Path()  # Reset the path
                self.position_recorder.path_msg.header.frame_id = "map"
        else:  # If navigation is not active
            if self.is_navigating:
                print("Navigation ended.")
                self.end_time = time.time()
                navigation_time = self.end_time - self.start_time
                self.is_navigating = False
                # self.distance_calculator.is_navigating = False
                self.position_recorder.is_navigating = False
                self.position_recorder.save_to_json()
                print("-----------------------------------------")
                print(f"{self.base_name} path planning result:")
                print(f"time taken: {navigation_time:.2f} seconds")
                # print(f"path minimum distance to humans: {self.distance_calculator.min_distance:.2f} meters")
                path_data = self.load_path_from_file(self.position_recorder.file_name)
                print("path length:", self.calculate_path_length(path_data))
                print("-----------------------------------------")
                
    def load_path_from_file(self, file_name):
        with open(file_name, 'r') as f:
            path_data = json.load(f)

        return path_data
    
    def calculate_path_length(self, path_data):
        path_length = 0.0
        for i in range(1, len(path_data)):
            x1, y1 = path_data[i-1]['x'], path_data[i-1]['y']
            x2, y2 = path_data[i]['x'], path_data[i]['y']
            distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            path_length += distance
        return path_length

if __name__ == '__main__':
    try:
        MainNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Main node terminated.")
