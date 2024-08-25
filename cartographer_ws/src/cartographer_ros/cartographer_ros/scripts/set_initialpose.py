#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from cartographer_ros_msgs.srv import FinishTrajectory, FinishTrajectoryRequest, GetTrajectoryStates, GetTrajectoryStatesRequest
from cartographer_ros_msgs.srv import StartTrajectory, StartTrajectoryRequest

class TrajectoryManager:
    def __init__(self):
        rospy.init_node('trajectory_manager')

        # Subscribe to the /initialpose topic
        self.pose_subscriber = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.pose_callback)

        # Service clients for finishing and starting trajectories
        self.finish_trajectory_client = rospy.ServiceProxy('/finish_trajectory', FinishTrajectory)
        self.start_trajectory_client = rospy.ServiceProxy('/start_trajectory', StartTrajectory)
        self.get_trajectory_states_client = rospy.ServiceProxy('/get_trajectory_states', GetTrajectoryStates)

        # Keep track of the last trajectory ID
        self.current_trajectory_id = None

    def get_latest_trajectory_id(self):
        try:
            request = GetTrajectoryStatesRequest()
            response = self.get_trajectory_states_client(request)
            if response.trajectory_states.trajectory_id:
                # Get the maximum trajectory_id which is the latest one
                return max(response.trajectory_states.trajectory_id)
            else:
                return None
        except rospy.ServiceException as e:
            rospy.logerr("Failed to call service /get_trajectory_states: %s", e)
            return None

    def pose_callback(self, msg):
        # Find the latest trajectory ID
        self.current_trajectory_id = self.get_latest_trajectory_id()

        # End the previous trajectory if one exists
        if self.current_trajectory_id is not None:
            try:
                finish_request = FinishTrajectoryRequest()
                finish_request.trajectory_id = self.current_trajectory_id

                finish_response = self.finish_trajectory_client(finish_request)
                if finish_response.status.code == 0:
                    rospy.loginfo("Successfully finished trajectory with ID: %d", self.current_trajectory_id)
                else:
                    rospy.logwarn("Failed to finish trajectory with ID: %d", self.current_trajectory_id)
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s", e)

        # Start a new trajectory with the pose received from RViz
        start_request = StartTrajectoryRequest()

        # Adjust these to your configuration
        start_request.configuration_directory = "/home/developer/berlin/Marslite_Simulation/cartographer_ws/install_isolated/share/cartographer_ros/configuration_files"
        start_request.configuration_basename = "mars_localization.lua"
        start_request.initial_pose = msg.pose.pose
        start_request.relative_to_trajectory_id = 0

        try:
            start_response = self.start_trajectory_client(start_request)
            if start_response.trajectory_id >= 0:
                self.current_trajectory_id = start_response.trajectory_id
                rospy.loginfo("Successfully started a new trajectory with ID: %d", self.current_trajectory_id)
            else:
                rospy.logwarn("Failed to start a new trajectory.")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    manager = TrajectoryManager()
    manager.run()
