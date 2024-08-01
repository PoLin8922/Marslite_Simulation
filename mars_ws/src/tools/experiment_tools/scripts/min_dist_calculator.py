#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from human_msgs.msg import TrackedHumans
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from math import sqrt

class DistanceCalculator:
    def __init__(self):

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)

        self.human_sub = rospy.Subscriber('/ground_truth_humans', TrackedHumans, self.human_callback)

        self.min_distance = float('inf')
        self.current_human_positions = []
        self.is_navigating = False

        rospy.Timer(rospy.Duration(0.3), self.update_and_calculate_distance)

        rospy.loginfo("Distance Calculator node started")

    def get_robot_position_in_map(self):
        robot_pose_base = PoseStamped()
        robot_pose_base.header.frame_id = "base_link"
        robot_pose_base.header.stamp = rospy.Time(0)

        try:
            robot_pose_map = self.tf_buffer.transform(robot_pose_base, "map", rospy.Duration(1.0))
            return robot_pose_map.pose.position
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
            rospy.logwarn("Could not transform robot's pose: %s", ex)
            return None

    def human_callback(self, msg):
        if self.is_navigating:
            self.current_human_positions = [human.segments[-1].pose.pose.position for human in msg.humans if human.segments]

        # print(self.current_human_positions)

    def update_and_calculate_distance(self, event):
        if self.is_navigating:
            robot_position = self.get_robot_position_in_map()
            if robot_position and self.current_human_positions:
                for human_pos in self.current_human_positions:
                    distance = self.calculate_distance(robot_position, human_pos)
                    if distance < self.min_distance:
                        self.min_distance = distance

                print(f"Current minimum distance to humans: {self.min_distance:.2f} meters")

    @staticmethod
    def calculate_distance(p1, p2):
        return sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)

