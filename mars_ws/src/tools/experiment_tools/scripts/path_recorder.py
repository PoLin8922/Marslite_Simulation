#!/usr/bin/env python3

import rospy
import json
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
import os
import tf2_ros
import tf2_geometry_msgs

class PositionRecorder:
    def __init__(self):

        self.path_pub = rospy.Publisher('/robot_path', Path, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(0.3), self.timer_callback)
        
        self.base_name = 'simulation1'
        self.positions = []
        self.is_navigating = False
        self.file_name = self.get_next_file_name()
        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        rospy.loginfo("Position Recorder node started")
    
    def timer_callback(self, event):
        robot_px, robot_py = self.get_robot_position_in_map()

        if self.is_navigating:
            current_time = rospy.Time.now()
            
            position = {
                'x': robot_px,
                'y': robot_py,
            }
            self.positions.append(position)

            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "map"
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.pose.position.x = robot_px
            pose_stamped.pose.position.y = robot_py
            pose_stamped.pose.position.z = 0.0
            self.path_msg.poses.append(pose_stamped)
            self.path_pub.publish(self.path_msg)
    
    def get_robot_position_in_map(self):
        robot_pose_base = PoseStamped()
        robot_pose_base.header.frame_id = "base_link"
        robot_pose_base.header.stamp = rospy.Time(0)

        try:
            robot_pose_map = self.tf_buffer.transform(robot_pose_base, "map", rospy.Duration(1.0))
            robot_px = robot_pose_map.pose.position.x
            robot_py = robot_pose_map.pose.position.y
            return robot_px, robot_py
        except :
            return None, None
        # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
        #     # rospy.logwarn("Could not transform robot's pose: %s", ex)
        #     return None, None
    
    def save_to_json(self):
        with open(self.file_name, 'w') as f:
            json.dump(self.positions, f, indent=4)
        rospy.loginfo(f"Positions saved to {self.file_name}")

    def get_next_file_name(self):
        i = 1
        directory = '/home/developer/berlin/Marslite_Simulation/mars_ws/src/tools/experiment_tools/files/path' # docker
        while os.path.isfile(os.path.join(directory, f"{self.base_name}_{i}.json")):
            i += 1
        return os.path.join(directory, f"{self.base_name}_{i}.json")


