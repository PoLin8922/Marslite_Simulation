#!/usr/bin/env python3

import rospy
import json
import math
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from scipy.interpolate import UnivariateSpline

# teb_file_name = '/home/developer/lab/socially-store-robot/mars_ws/src/tools/experiment_tools/files/path/our_178.json'
# hateb_file_name = '/home/developer/lab/socially-store-robot/mars_ws/src/tools/experiment_tools/files/path/our_177.json'
our_file_name = '/home/developer/berlin/Marslite_Simulation/mars_ws/src/tools/experiment_tools/files/path/our_14.json'

class PathVisualizer:
    def __init__(self):
        rospy.init_node('path_visualizer')

        # self.teb_path_pub = rospy.Publisher('/teb_path', Path, queue_size=10)
        # self.hateb_path_pub = rospy.Publisher('/hateb_path', Path, queue_size=10)
        self.our_path_pub = rospy.Publisher('/our_path', Path, queue_size=10)

        # self.teb_path_data, self.teb_path = self.load_and_smooth_path(teb_file_name)
        # self.hateb_path_data,self.hateb_path = self.load_path_from_file(hateb_file_name)
        self.our_path_data,self.our_path = self.load_and_smooth_path(our_file_name)

        rospy.loginfo("Path Visualizer node started and publishing path.")

        # print("teb path length:", self.calculate_path_length(self.teb_path_data))
        # print("hateb path length:", self.calculate_path_length(self.hateb_path_data))
        print("our path length:", self.calculate_path_length(self.our_path_data))

        # rospy.Timer(rospy.Duration(1.0), self.publish_teb_path)
        # rospy.Timer(rospy.Duration(1.0), self.publish_hateb_path)
        rospy.Timer(rospy.Duration(1.0), self.publish_our_path)

    
    def load_path_from_file(self, file_name):
        with open(file_name, 'r') as f:
            path_data = json.load(f)
        
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.poses = []
        for point in path_data:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "map"
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.pose.position.x = point['x']
            pose_stamped.pose.position.y = point['y']
            pose_stamped.pose.position.z = 0.0  
            
            path_msg.poses.append(pose_stamped)
        
        return path_data, path_msg
    
    def load_and_smooth_path(self, file_name):
        with open(file_name, 'r') as f:
            path_data = json.load(f)
        
        x = [point['x'] for point in path_data]
        y = [point['y'] for point in path_data]

        x = np.array(x)
        y = np.array(y)

        indices = np.arange(len(x))
        s = 0.1  # Small smoothing factor
        spline_x = UnivariateSpline(indices, x, s=s)
        spline_y = UnivariateSpline(indices, y, s=s)

        x_smooth = spline_x(indices)
        y_smooth = spline_y(indices)

        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.poses = []
        for i in range(len(x_smooth)):
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "map"
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.pose.position.x = x_smooth[i]
            pose_stamped.pose.position.y = y_smooth[i]
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation.w = 1.0  
            
            path_msg.poses.append(pose_stamped)
        
        return path_data, path_msg
            
    def calculate_path_length(self, path_data):
        path_length = 0.0
        for i in range(1, len(path_data)):
            x1, y1 = path_data[i-1]['x'], path_data[i-1]['y']
            x2, y2 = path_data[i]['x'], path_data[i]['y']
            distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            path_length += distance
        return path_length
    
    # def publish_teb_path(self, event):
    #     self.teb_path.header.stamp = rospy.Time.now()  
    #     self.teb_path_pub.publish(self.teb_path)

    # def publish_hateb_path(self, event):
    #     self.hateb_path.header.stamp = rospy.Time.now()  
    #     self.hateb_path_pub.publish(self.hateb_path)

    def publish_our_path(self, event):
        self.our_path.header.stamp = rospy.Time.now()  
        self.our_path_pub.publish(self.our_path)

if __name__ == '__main__':
    try:
        path_visualizer = PathVisualizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Path Visualizer node terminated.")
