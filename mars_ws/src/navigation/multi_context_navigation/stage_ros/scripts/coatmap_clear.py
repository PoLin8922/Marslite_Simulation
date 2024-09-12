#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Pose, Twist
from std_srvs.srv import Empty

class StageHumans():

    def HumansPub(self):
        rate = rospy.Rate(0.8)
        clear_costmaps_service = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        while not rospy.is_shutdown():
            try:
                response = clear_costmaps_service()
                rospy.loginfo("Costmaps cleared successfully.")
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s" % e)
            rate.sleep()

if __name__ == '__main__':

    rospy.init_node('Stage_Humans', anonymous=True)
    humans = StageHumans()
    humans.HumansPub()