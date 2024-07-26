#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist
import json
import time
import os

class DataCollector:
    def __init__(self):
        rospy.init_node('data_collector', anonymous=True)
        
        self.data = {
            "time": [],
            "navigability": [],
            "speed_up_level": [],
            "robot_invisiable_level": [],
            "right_side_level": [],
            "pspace_level": [],
            "weight_optimaltime": [],
            "weight_cc": [],
            "pspace_cov": [],
            "pspace_r_ratio": [],
            "use_external_prediction": [],
            # "cmd_vel_linear": [],
            # "cmd_vel_angular": []
        }

        self.recording = False

        self.nav_state_sub = rospy.Subscriber('nav_state', Bool, self.nav_state_callback)
        rospy.Subscriber("navigability", Float32, self.navigability_callback)
        rospy.Subscriber("/speed_up_level", Float32, self.speed_up_level_callback)
        rospy.Subscriber("/robot_invisiable_level", Float32, self.robot_invisiable_level_callback)
        rospy.Subscriber("/right_side_level", Float32, self.right_side_level_callback)
        rospy.Subscriber("/pspace_level", Float32, self.pspace_level_callback)
        rospy.Subscriber("/weight_optimaltime", Float32, self.weight_optimaltime_callback)
        rospy.Subscriber("/weight_cc", Float32, self.weight_cc_callback)
        rospy.Subscriber("/pspace_cov", Float32, self.pspace_cov_callback)
        rospy.Subscriber("/pspace_r_ratio", Float32, self.pspace_r_ratio_callback)
        rospy.Subscriber("/use_external_prediction", Float32, self.use_external_prediction_callback)
        # rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        
        self.start_time = time.time()
        self.base_name = "navigation_data"

        self.current_data = {
            "navigability": None,
            "speed_up_level": None,
            "robot_invisiable_level": None,
            "right_side_level": None,
            "pspace_level": None,
            "weight_optimaltime": None,
            "weight_cc": None,
            "pspace_cov": None,
            "pspace_r_ratio": None,
            "use_external_prediction": None,
            # "cmd_vel_linear": None,
            # "cmd_vel_angular": None
        }

        self.timer = rospy.Timer(rospy.Duration(0.1), self.collect_data)  # Adjust the duration as needed

        rospy.spin()

    def get_next_file_name(self):
        i = 1
        directory = '/home/developer/lab/socially-store-robot/mars_ws/src/tools/experiment_tools/files' # docker
        while os.path.isfile(os.path.join(directory, f"{self.base_name}_{i}.json")):
            i += 1
        return os.path.join(directory, f"{self.base_name}_{i}.json")

    def save_data(self):
        filename = self.get_next_file_name()
        with open(filename, 'w') as f:
            json.dump(self.data, f, indent=4)
        rospy.loginfo(f"Data saved to {filename}")

    def nav_state_callback(self, msg):
        if msg.data and not self.recording:  # Navigation started
            rospy.loginfo("Navigation started")
            self.recording = True
            self.start_time = time.time()
            self.data = {key: [] for key in self.data}  # Reset data
        elif not msg.data and self.recording:  # Navigation ended
            rospy.loginfo("Navigation ended")
            self.recording = False
            self.save_data()

    def collect_data(self, event):
        if self.recording:
            self.data["time"].append(time.time() - self.start_time)
            self.data["navigability"].append(self.current_data["navigability"])
            self.data["speed_up_level"].append(self.current_data["speed_up_level"])
            self.data["robot_invisiable_level"].append(self.current_data["robot_invisiable_level"])
            self.data["right_side_level"].append(self.current_data["right_side_level"])
            self.data["pspace_level"].append(self.current_data["pspace_level"])
            self.data["weight_optimaltime"].append(self.current_data["weight_optimaltime"])
            self.data["weight_cc"].append(self.current_data["weight_cc"])
            self.data["pspace_cov"].append(self.current_data["pspace_cov"])
            self.data["pspace_r_ratio"].append(self.current_data["pspace_r_ratio"])
            self.data["use_external_prediction"].append(self.current_data["use_external_prediction"])
            # self.data["cmd_vel_linear"].append(self.current_data["cmd_vel_linear"])
            # self.data["cmd_vel_angular"].append(self.current_data["cmd_vel_angular"])

    def navigability_callback(self, msg):
        self.current_data["navigability"] = msg.data

    def speed_up_level_callback(self, msg):
        self.current_data["speed_up_level"] = msg.data

    def robot_invisiable_level_callback(self, msg):
        self.current_data["robot_invisiable_level"] = msg.data

    def right_side_level_callback(self, msg):
        self.current_data["right_side_level"] = msg.data

    def pspace_level_callback(self, msg):
        self.current_data["pspace_level"] = msg.data

    def weight_optimaltime_callback(self, msg):
        self.current_data["weight_optimaltime"] = msg.data

    def weight_cc_callback(self, msg):
        self.current_data["weight_cc"] = msg.data

    def pspace_cov_callback(self, msg):
        self.current_data["pspace_cov"] = msg.data

    def pspace_r_ratio_callback(self, msg):
        self.current_data["pspace_r_ratio"] = msg.data

    def use_external_prediction_callback(self, msg):
        self.current_data["use_external_prediction"] = msg.data

    # def cmd_vel_callback(self, msg):
    #     self.current_data["cmd_vel_linear"] = msg.linear.x
    #     self.current_data["cmd_vel_angular"] = msg.angular.z

if __name__ == '__main__':
    try:
        DataCollector()
    except rospy.ROSInterruptException:
        pass
