#!/usr/bin/python3

import sys
import rospy
from geometry_msgs.msg import Pose, Twist
from human_msgs.msg import TrackedHumans
from human_msgs.msg import TrackedHuman
from human_msgs.msg import TrackedSegmentType
from human_msgs.msg import TrackedSegment
from nav_msgs.msg import Odometry
from pedsim_msgs.msg import AgentStates, AgentState  # Import AgentStates and AgentState messages
from std_srvs.srv import Empty   # Import Empty service message

class StageHumans(object):

    def __init__(self):
        self.tracked_humans_pub = rospy.Publisher("/tracked_humans", TrackedHumans, queue_size=1)
        self.Segment_Type = TrackedSegmentType.TORSO

        rospy.Subscriber("/pedsim_simulator/simulated_agents", AgentStates, self.agent_states_callback)

    def agent_states_callback(self, agent_states):
        tracked_humans = TrackedHumans()

        for i, agent_state in enumerate(agent_states.agent_states):
            human_segment = TrackedSegment()
            human_segment.type = self.Segment_Type

            # Extract pose information from the AgentState message
            pose = Pose()
            pose.position.x = agent_state.pose.position.x
            pose.position.y = agent_state.pose.position.y
            pose.position.z = agent_state.pose.position.z
            pose.orientation = agent_state.pose.orientation

            twist = Twist()
            twist.linear.x = agent_state.twist.linear.x
            twist.linear.y = agent_state.twist.linear.y
            twist.linear.z = agent_state.twist.linear.z

            human_segment.pose.pose = pose
            human_segment.twist.twist = twist

            tracked_human = TrackedHuman()
            tracked_human.track_id = i + 1  
            tracked_human.segments.append(human_segment)
            tracked_humans.humans.append(tracked_human)

        if tracked_humans.humans:
            tracked_humans.header.stamp = rospy.Time.now()
            tracked_humans.header.frame_id = 'map'
            self.tracked_humans_pub.publish(tracked_humans)

    def HumansPub(self):
        rate = rospy.Rate(10) 
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
