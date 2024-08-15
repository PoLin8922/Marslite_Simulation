#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

MAX_INPUT_VEL = 1.1
MAX_OUTPUT_VEL = 0.60

angular_z_threshold = 0.035
angular_z_count = 0
angular_z_count_limit = 7

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

    if abs(msg.angular.z) < angular_z_threshold:
        angular_z_count += 1
    else:
        angular_z_count = 0

    if angular_z_count > angular_z_count_limit:
        new_msg.angular.z = 0

    pub.publish(new_msg)
    print("transformed vel_x: ", new_msg.linear.x)

def main():
    rospy.init_node('velocity_mapper')

    global pub
    pub = rospy.Publisher('/mob_plat/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)

    rospy.spin()

if __name__ == '__main__':
    main()
