#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Pose, Twist
from human_msgs.msg import TrackedHumans, TrackedHuman, TrackedSegmentType, TrackedSegment
from pedsim_msgs.msg import AgentStates
from std_srvs.srv import Empty
import argparse

def parse_static_humans(static_humans_str):
    """
    Parse the input static_human string into a list of positions.
    Example input: "15.90,-3.88;25.4,-6.15"
    Output: [[15.90, -3.88], [25.4, -6.15]]
    """
    if static_humans_str == '':
        return []  # Return an empty list if no static humans are provided
        
    humans = []
    for human in static_humans_str.split(';'):
        pos = list(map(float, human.split(',')))
        humans.append(pos)
    return humans

class StageHumans(object):
    def __init__(self, static_human_positions, pub_tracked_human):
        self.tracked_humans_pub = rospy.Publisher("/tracked_humans", TrackedHumans, queue_size=1)
        self.ground_truth__humans_pub = rospy.Publisher("/ground_truth_humans", TrackedHumans, queue_size=1)
        self.Segment_Type = TrackedSegmentType.TORSO
        self.static_human_positions = static_human_positions
        self.pub_tracked_human = pub_tracked_human

        self.agent_states_timer = rospy.Timer(rospy.Duration(0.1), self.agent_states_callback)
        
    def agent_states_callback(self, pub_tracked_human):
        tracked_humans = TrackedHumans()

        # Add static humans first
        for j, static_pos in enumerate(self.static_human_positions, start=1):
            human_segment = TrackedSegment()
            human_segment.type = self.Segment_Type

            pose = Pose()
            pose.position.x = static_pos[0]
            pose.position.y = static_pos[1]
            pose.position.z = 0.0  # Assuming static humans are on the ground
            pose.orientation.w = 1.0  # Neutral orientation

            twist = Twist()  # Static humans have no velocity

            human_segment.pose.pose = pose
            human_segment.twist.twist = twist

            tracked_human = TrackedHuman()
            tracked_human.track_id = j
            tracked_human.segments.append(human_segment)
            tracked_humans.humans.append(tracked_human)

        if tracked_humans.humans:
            tracked_humans.header.stamp = rospy.Time.now()
            tracked_humans.header.frame_id = 'map'
            if self.pub_tracked_human:
                self.tracked_humans_pub.publish(tracked_humans)
            self.ground_truth__humans_pub.publish(tracked_humans)

    def HumansPub(self):
        rate = rospy.Rate(0.1)
        clear_costmaps_service = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        while not rospy.is_shutdown():
            try:
                response = clear_costmaps_service()
                # rospy.loginfo("Costmaps cleared successfully.")
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s" % e)
            rate.sleep()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Stage Humans Node")
    parser.add_argument(
        "--static_human", type=str, required=True,
        help="Static human positions in the format 'x1,y1;x2,y2;...'"
    )
    parser.add_argument(
        "--pub_tracked_human", type=int, choices=[0, 1], required=True,
        help="Boolean flag to publish tracked humans (1 for True, 0 for False)"
    )
    args = parser.parse_args()

    static_human_positions = parse_static_humans(args.static_human)
    pub_tracked_human = bool(args.pub_tracked_human)

    rospy.init_node('Stage_Humans', anonymous=True)
    humans = StageHumans(static_human_positions, pub_tracked_human)
    humans.HumansPub()