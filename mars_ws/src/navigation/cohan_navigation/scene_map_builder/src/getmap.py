#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid

def map_callback(msg):
    data_size = len(msg.data)
    rospy.loginfo(f"Received OccupancyGrid message with data size: {data_size}")

def map_subscriber():
    rospy.init_node('map_subscriber', anonymous=True)
    rospy.Subscriber('/map', OccupancyGrid, map_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        map_subscriber()
    except rospy.ROSInterruptException:
        pass
