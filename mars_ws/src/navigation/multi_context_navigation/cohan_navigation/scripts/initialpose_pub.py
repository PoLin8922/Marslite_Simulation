#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Header

# Global variables to store robot position
robot_position_x = 0.0
robot_position_y = 0.0

def robot_position_callback(msg):
    global robot_position_x, robot_position_y
    robot_position_x = msg.x
    robot_position_y = msg.y

def publish_initial_pose():
    initial_pose_msg = PoseWithCovarianceStamped()

    initial_pose_msg.header = Header()
    initial_pose_msg.header.stamp = rospy.Time.now()
    initial_pose_msg.header.frame_id = "map"

    initial_pose_msg.pose.pose.position.x = robot_position_x
    initial_pose_msg.pose.pose.position.y = robot_position_y
    initial_pose_msg.pose.pose.position.z = 0.0

    initial_pose_msg.pose.pose.orientation.w = 1.0  # Assuming no rotation for simplicity

    initial_pose_msg.pose.covariance = [
        0.2, 0,   0,   0,   0,   0,
        0,   0.2, 0,   0,   0,   0,
        0,   0,   0.1, 0,   0,   0,
        0,   0,   0,   0.1, 0,   0,
        0,   0,   0,   0,   0.1, 0,
        0,   0,   0,   0,   0,   1.0
    ]

    initial_pose_pub.publish(initial_pose_msg)
    print("Published initial pose with position x: {}, y: {}".format(robot_position_x, robot_position_y))

def main():
    rospy.init_node('initialpose_publisher')

    global initial_pose_pub

    initial_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
    rospy.Subscriber('/robot_position_map', Point, robot_position_callback)

    print("Press Enter to publish the initial pose.")
    while not rospy.is_shutdown():
        input()  # Wait for Enter key press
        publish_initial_pose()

if __name__ == '__main__':
    main()
