import rospy
from std_msgs.msg import Bool
import time

class NavigationTimeCalculator:
    def __init__(self):
        rospy.init_node('navigation_time_calculator')

        self.nav_state_sub = rospy.Subscriber('nav_state', Bool, self.nav_state_callback)
        
        self.start_time = None
        self.end_time = None
        self.is_navigating = False
        
        rospy.loginfo("Navigation Time Calculator node started")
    
    def nav_state_callback(self, msg):
        if msg.data:  
            if not self.is_navigating:
                self.start_time = time.time()
                self.is_navigating = True
                rospy.loginfo("Navigation started.")
        else:  
            if self.is_navigating:
                self.end_time = time.time()
                navigation_time = self.end_time - self.start_time
                rospy.loginfo(f"Navigation ended. Time taken: {navigation_time:.2f} seconds")
                self.is_navigating = False

if __name__ == '__main__':
    try:
        NavigationTimeCalculator()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation Time Calculator node terminated.")
