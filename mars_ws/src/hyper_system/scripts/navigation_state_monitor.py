#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class NavigationStateMonitor:
    def __init__(self):
        rospy.init_node('navigation_state_monitor')

        self.velocity_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.navigation_done_sub = rospy.Subscriber('/navigation_done', Bool, self.navigation_done_callback)
        self.nav_state_pub = rospy.Publisher('/nav_state', Bool, queue_size=1)
        
        self.non_zero_velocity_count = 0
        self.zero_velocity_count = 0
        self.low_velocity_count = 0
        self.start_threshold = 5
        self.end_threshold = 2
        self.low_velocity_threshold = 30
        self.low_velocity_limit = 0.001
        self.is_navigating = False

        rospy.loginfo("Navigation State Monitor node started")

    def navigation_done_callback(self, msg):
        if msg.data:
            if self.is_navigating:
                self.is_navigating = False
                rospy.loginfo("Navigation ended due to /navigation_done signal.")
                self.publish_nav_state()
    
    def cmd_vel_callback(self, msg):
        linear_velocity = msg.linear
        angular_velocity = msg.angular
        
        if linear_velocity.x == 0 and linear_velocity.y == 0 and linear_velocity.z == 0 and \
           angular_velocity.x == 0 and angular_velocity.y == 0 and angular_velocity.z == 0:
            self.zero_velocity_count += 1
            self.non_zero_velocity_count = 0
            self.low_velocity_count = 0
        else:
            self.non_zero_velocity_count += 1
            self.zero_velocity_count = 0
            
            if abs(linear_velocity.x) < self.low_velocity_limit:
                self.low_velocity_count += 1
            else:
                self.low_velocity_count = 0

        if self.non_zero_velocity_count >= self.start_threshold and not self.is_navigating:
            self.is_navigating = True
            rospy.loginfo("Navigation started.")
            self.publish_nav_state()

        if (self.zero_velocity_count >= self.end_threshold or self.low_velocity_count >= self.low_velocity_threshold) and self.is_navigating:
            self.is_navigating = False
            rospy.loginfo("Navigation ended.")
            self.publish_nav_state()

    def publish_nav_state(self):
        nav_state_msg = Bool()
        nav_state_msg.data = self.is_navigating
        self.nav_state_pub.publish(nav_state_msg)
        rospy.loginfo(f"Published nav_state: {self.is_navigating}")

if __name__ == '__main__':
    try:
        NavigationStateMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation State Monitor node terminated.")
