#!/usr/bin/env python3

# Brief: This node subscribes to /tracked_humans and publishes the predicted goal to humans based on their trajectory
# Author: Berlin JIang

import numpy as np
import rospy
import tf
from geometry_msgs.msg import Point, PoseStamped
from human_msgs.msg import TrackedHumans, TrackedHuman, TrackedSegmentType
from human_path_prediction.msg import PredictedGoal, PredictedGoals
from human_path_prediction.srv import HumanGoalPredict, HumanGoalPredictResponse
from scipy.stats import multivariate_normal
from std_srvs.srv import SetBool, Trigger, TriggerResponse
import json
import math
import time

EPS = 1e-12

class PredictGoal(object):
    def __init__(self, human_num=1):
        self.human_num = human_num

        self.goals_x = []
        self.goals_y = []
        self.resolution = 0.5  
        self.json_file = "/home/developer/lab/socially-store-robot/mars_ws/src/navigation/cohan_navigation/human_path_prediction/scripts/crossing_corrider_outlines.json"
        self.read_json(self.json_file, self.resolution)
        self.goal_num = len(self.goals_x)

        self.last_idx = 0
        self.changed = False
        self.current_poses = [[] for i in range(self.human_num)]
        self.prev_poses = [[] for i in range(self.human_num)]
        # self.mean = np.array([0, 0])
        # self.cov = np.array([[0.07, 0.0], [0.0, 120.0]])
        # self.mv_nd = multivariate_normal(mean=self.mean, cov=self.cov)
        self.mv_nd = multivariate_normal(mean=0,cov=0.1)
        self.theta_phi = [[0]*self.goal_num for i in range(self.human_num)]
        self.window_size = 10
        self.probability_goal = [np.array([1.0/self.goal_num]*self.goal_num) for i in range(self.human_num)]
        self.probability_goal_window = [np.array([[1.0/self.goal_num]*self.goal_num]*self.window_size) for i in range(self.human_num)]
        self.done = False
        self.itr = np.zeros(self.human_num, dtype=int)

        NODE_NAME = "human_goals_publisher"
        rospy.init_node(NODE_NAME)
        self.humans_sub_ = rospy.Subscriber("/tracked_humans",TrackedHumans,self.tracked_humansCB)
        self.goal_srv_ = rospy.Service("goal_changed_2", Trigger, self.goal_changed)
        self.goal_pub_ = rospy.Publisher(NODE_NAME+"/predicted_goals",PredictedGoals, queue_size=2)
        rospy.spin()
    
    def read_json(self, json_file, resolution):
        with open(json_file, 'r') as file:
            data = json.load(file)
            if "rooms" in data:
                rooms = data["rooms"]
                for room_key, room_data in rooms.items():
                    if "outline" in room_data:
                        outline = room_data["outline"]
                        if len(outline[0]) > 0:
                            x, y, _ = outline[0][0]
                            self.goals_x.append(x)
                            self.goals_y.append(y)

                            for point in outline[0][1:]:
                                x, y, _ = point
                                prev_x, prev_y = self.goals_x[-1], self.goals_y[-1]

                                # Check the distance to the previous point
                                distance = math.sqrt((x - prev_x)**2 + (y - prev_y)**2)

                                # Add points along the outline if distance exceeds resolution
                                while distance > resolution:
                                    angle = math.atan2(y - prev_y, x - prev_x)
                                    new_x = prev_x + resolution * math.cos(angle)
                                    new_y = prev_y + resolution * math.sin(angle)
                                    self.goals_x.append(new_x)
                                    self.goals_y.append(new_y)
                                    prev_x, prev_y = new_x, new_y
                                    distance = math.sqrt((x - prev_x)**2 + (y - prev_y)**2)

                                self.goals_x.append(x)
                                self.goals_y.append(y)

    def tracked_humansCB(self,msg):
        self.prev_poses = self.current_poses
        self.current_poses = [[] for i in range(self.human_num)]

        for human in msg.humans:
            for segment in human.segments:
                if segment.type == TrackedSegmentType.TORSO:
                    self.current_poses[human.track_id-1].append(segment.pose.pose)
        if not self.done:
            self.prev_poses = self.current_poses         

        for human in msg.humans:
            track_id = human.track_id -1
            for i in range(0,len(self.current_poses[track_id])):  # i is 0 
                diff = np.linalg.norm([self.current_poses[track_id][i].position.x - self.prev_poses[track_id][i].position.x, self.current_poses[track_id][i].position.y - self.prev_poses[track_id][i].position.y])

                if diff > EPS or not self.done:
                    # calculate human - hot spots distance and theta
                    dist = []
                    for j in range(0,len(self.goals_x)):
                        vec1 = np.array([self.goals_x[j],self.goals_y[j],0.0]) - np.array([self.current_poses[track_id][i].position.x,self.current_poses[track_id][i].position.y,0.0])  #Vector from current position to a goal
                        rotation = (self.current_poses[track_id][i].orientation.x,self.current_poses[track_id][i].orientation.y,self.current_poses[track_id][i].orientation.z,self.current_poses[track_id][i].orientation.w)
                        roll,pitch,yaw = tf.transformations.euler_from_quaternion(rotation)
                        unit_vec = np.array([np.cos(yaw), np.sin(yaw),0.0])
                        self.theta_phi[track_id][j] = (np.arccos(np.dot(vec1,unit_vec)/np.linalg.norm(vec1)))
                        dist.append(np.linalg.norm([self.current_poses[track_id][i].position.x - self.goals_x[j],self.current_poses[track_id][i].position.y - self.goals_y[j]]))

                    # self.probability_goal_window[i][self.itr] = self.mv_nd.pdf(np.column_stack((np.array(self.theta_phi[i]), np.array(dist))))
                    self.probability_goal_window[track_id][self.itr[track_id]] = self.mv_nd.pdf(np.array(self.theta_phi[track_id]))

                    self.probability_goal[track_id] = np.array([1.0]*self.goal_num)
                    for k in range(0,len(self.probability_goal_window[track_id])):
                        gf = np.exp((k-self.window_size)/5)
                        
                        self.probability_goal[track_id] =  np.power(self.probability_goal_window[track_id][k],gf)* np.array(self.probability_goal[track_id]) # Linear prediction of goal

                    for ln in range(0,len(self.goals_x)):
                        self.probability_goal[track_id][ln] = (1/dist[ln])*self.probability_goal[track_id][ln]
                        # self.probability_goal[track_id][ln] = 0.5*(1/dist[ln] + 1/self.theta_phi[track_id][ln])*self.probability_goal[track_id][ln]

                    self.probability_goal[track_id] = (self.probability_goal[track_id]-np.min(self.probability_goal[track_id]))/(np.max(self.probability_goal[track_id])-np.min(self.probability_goal[track_id]))

                    self.itr[track_id] = self.itr[track_id] + 1
                    if self.itr[track_id] == self.window_size:
                        self.itr[track_id] = 0

    
        # publish goals
        p_goals = PredictedGoals()
        predicted_goals = []
        

        initialized_goal = PredictedGoal()
        predicted_goal = PoseStamped()
        initialized_goal.goal = predicted_goal 
        initialized_goal.changed = False
        initialized_goal.id = -1   ### note for none
        while len(predicted_goals) <= self.human_num:
            predicted_goals.append(initialized_goal)

        for human in msg.humans:
            idx = 0
            max_prob = 0.0
            track_id = human.track_id -1
            p_goal = PredictedGoal()
            for j in range(0,len(self.goals_x)):
                if(max_prob<self.probability_goal[track_id][j]):
                    idx = j
                    max_prob = self.probability_goal[track_id][j]
            
            predicted_goal = PoseStamped()
            predicted_goal.header.stamp = rospy.Time.now()
            predicted_goal.header.frame_id = 'map'
            predicted_goal.pose.position.x = self.goals_x[idx]
            predicted_goal.pose.position.y = self.goals_y[idx]
            predicted_goal.pose.position.z = 0.0
            predicted_goal.pose.orientation = self.current_poses[track_id][0].orientation

            if self.last_idx != idx:
                p_goal.changed = True
                self.changed = True

            self.last_idx = idx
            p_goal.id = human.track_id
            p_goal.goal = predicted_goal   
            predicted_goals[track_id] = p_goal

        p_goals.goals = predicted_goals
        self.goal_pub_.publish(p_goals)


    def goal_changed(self,req):
        if self.changed:
            self.changed = False
            return TriggerResponse(True,"Goal Changed")
        return TriggerResponse(False, "Goal not changed")

if __name__ == '__main__':
    predict_srv = PredictGoal(60)