#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, Point
from std_msgs.msg import Header

MAX_INPUT_VEL = 1.1
MAX_OUTPUT_VEL = 0.60

angular_z_threshold = 0.035
angular_z_count = 0
angular_z_count_limit = 5

linear_x_threshold = 0.09

robot_position_x = 0.0
robot_position_y = 0.0

def map_velocity(value, input_min, input_max, output_min, output_max):
    value = max(min(value, input_max), input_min)
    return output_min + (value - input_min) * (output_max - output_min) / (input_max - input_min)

def cmd_vel_callback(msg):
    global angular_z_count
    new_msg = Twist()

    new_msg.linear.x = map_velocity(msg.linear.x, 0, MAX_INPUT_VEL, 0, MAX_OUTPUT_VEL)
    new_msg.linear.y = msg.linear.y
    new_msg.linear.z = msg.linear.z
    new_msg.angular.x = msg.angular.x
    new_msg.angular.y = msg.angular.y
    new_msg.angular.z = msg.angular.z

    # if abs(msg.linear.x) < linear_x_threshold and abs(msg.angular.z) != 0.0:
    #     publish_initial_pose()

    if abs(msg.angular.z) < angular_z_threshold:
        angular_z_count += 1
    else:
        angular_z_count = 0

    if angular_z_count > angular_z_count_limit:
        new_msg.angular.z = 0

    pub.publish(new_msg)
    print("transformed vel_x: ", new_msg.linear.x)

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

    initial_pose_msg.pose.pose.orientation.w = 1.0

    initial_pose_msg.pose.covariance = [
        0.2, 0,   0,   0,   0,   0,
        0,   0.2, 0,   0,   0,   0,
        0,   0,   0.1, 0,   0,   0,
        0,   0,   0,   0.1, 0,   0,
        0,   0,   0,   0,   0.1, 0,
        0,   0,   0,   0,   0,   1.0
    ]

    initial_pose_pub.publish(initial_pose_msg)
    print("Robot is rotating in place.")
    print("Published initial pose with position x: {}, y: {}".format(robot_position_x, robot_position_y))

def main():
    rospy.init_node('velocity_mapper')

    global pub, initial_pose_pub
    pub = rospy.Publisher('/mob_plat/cmd_vel', Twist, queue_size=10)
    initial_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)

    rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)
    rospy.Subscriber('/robot_position_map', Point, robot_position_callback)

    rospy.spin()

if __name__ == '__main__':
    main()
