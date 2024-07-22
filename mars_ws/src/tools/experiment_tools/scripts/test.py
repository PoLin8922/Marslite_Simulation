#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Point

class PositionRecorder:
    def __init__(self):
        rospy.init_node('position_recorder')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Publisher to publish robot position
        self.position_pub = rospy.Publisher('/robot_position', PoseStamped, queue_size=10)
        
        # Set the timer to call get_robot_position_in_map periodically
        self.timer = rospy.Timer(rospy.Duration(1.0), self.timer_callback)
        
        rospy.loginfo("Position Recorder node started")

    def timer_callback(self, event):
        self.get_robot_position_in_map()

    def get_robot_position_in_map(self):
        robot_pose_base = PoseStamped()
        robot_pose_base.header.frame_id = "base_link"
        robot_pose_base.header.stamp = rospy.Time(0)

        try:
            robot_pose_map = self.tf_buffer.transform(robot_pose_base, "map", rospy.Duration(1.0))
            robot_px = robot_pose_map.pose.position.x
            robot_py = robot_pose_map.pose.position.y

            rospy.loginfo(f"Robot position in map: x={robot_px}, y={robot_py}")

            # Publish the position
            self.position_pub.publish(robot_pose_map)

            return robot_px, robot_py
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
            rospy.logwarn("Could not transform robot's pose: %s", ex)
            return None, None

if __name__ == '__main__':
    try:
        position_recorder = PositionRecorder()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Position Recorder node terminated.")
