#!/usr/bin/env python3

# Brief: This node subscribes to /tracked_humans and publishes the predicted goal to humans based on their trajectory
# Author: Berlin JIang

import numpy as np
import rospy
import tf
from geometry_msgs.msg import Point, PoseStamped
from human_msgs.msg import TrackedHumans, TrackedHuman, TrackedSegmentType
from human_path_prediction.msg import PredictedGoal
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

        # # crossing corrider
        # self.goals_x = [6.0, -6.0, 6.0, -6.0, 6.0, -6.0, 0.0, 0.0, 1.0, -1.0, 2.0, -2.0]
        # self.goals_y = [0.0, 0.0, 1.0, -1.0, 2.0, -2.0, 6.0, -6.0, 6.0, -6.0, 6.0, -6.0]

        # self.goal_num = 12

        self.goals_x = []
        self.goals_y = []
        self.resolution = 0.5  
        self.json_file = "/home/developer/lab/socially-store-robot/mars_ws/src/navigation/cohan_navigation/human_path_prediction/scripts/crossing_corrider_outlines.json"
        self.read_json(self.json_file, self.resolution)
        self.goal_num = len(self.goals_x)
        # print("Goals X:", self.goals_x)
        # print("Goals Y:", self.goals_y)
        # print("goal_num:", self.goal_num)

        self.predicted_goal = PoseStamped()
        self.last_idx = 0
        self.changed = False
        self.poses_data = [[] for i in range(self.human_num)]
        self.mean = np.array([0, 0])
        self.cov = np.array([[0.07, 0.0], [0.0, 120.0]])
        # self.mv_nd = multivariate_normal(mean=self.mean, cov=self.cov)
        self.mv_nd = multivariate_normal(mean=0,cov=0.1)
        self.theta_phi = [[0]*self.goal_num for i in range(self.human_num)]
        self.window_size = 10
        self.probability_goal = [np.array([1.0/self.goal_num]*self.goal_num) for i in range(self.human_num)]
        self.probability_goal_window = [np.array([[1.0/self.goal_num]*self.goal_num]*self.window_size) for i in range(self.human_num)]
        # self.done = False
        self.itr = 0

        NODE_NAME = "human_goal_predict_srv"
        rospy.init_node(NODE_NAME)
        self.humans_sub_ = rospy.Subscriber("/tracked_humans",TrackedHumans,self.tracked_humansCB)
        self.goal_srv_ = rospy.Service("goal_changed_2", Trigger, self.goal_changed)
        self.predict_srv_ = rospy.Service("human_goal_predict", HumanGoalPredict, self.predict_goal)
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
        for human in msg.humans:
            for segment in human.segments:
                if segment.type == TrackedSegmentType.TORSO:
                    self.poses_data[human.track_id-1].append(segment.pose.pose)
                    if(len(self.poses_data[human.track_id-1]) > self.window_size+1):
                        self.poses_data[human.track_id-1] = self.poses_data[human.track_id-1][1:]

    def predict_goal(self, req):
        time_start = time.time()
        cur_poses_data = self.poses_data
        # print("human_num", self.human_num)
        # print(cur_poses_data)
        track_id = req.human_id -1
        for i in range(0,len(cur_poses_data[track_id])-1):
            diff = np.linalg.norm([cur_poses_data[track_id][i+1].position.x - cur_poses_data[track_id][i].position.x, cur_poses_data[track_id][i+1].position.y - cur_poses_data[track_id][i].position.y])

            if diff > EPS:
                dist = []
                for j in range(0,len(self.goals_x)):
                    vec1 = np.array([self.goals_x[j],self.goals_y[j],0.0]) - np.array([cur_poses_data[track_id][i].position.x,cur_poses_data[track_id][i].position.y,0.0])  #Vector from current position to a goal
                    rotation = (cur_poses_data[track_id][i].orientation.x,cur_poses_data[track_id][i].orientation.y,cur_poses_data[track_id][i].orientation.z,cur_poses_data[track_id][i].orientation.w)
                    roll,pitch,yaw = tf.transformations.euler_from_quaternion(rotation)
                    unit_vec = np.array([np.cos(yaw), np.sin(yaw),0.0])
                    self.theta_phi[i][j] = (np.arccos(np.dot(vec1,unit_vec)/np.linalg.norm(vec1)))
                    dist.append(np.linalg.norm([cur_poses_data[track_id][i].position.x - self.goals_x[j],cur_poses_data[track_id][i].position.y - self.goals_y[j]]))

                # self.probability_goal_window[i][self.itr] = self.mv_nd.pdf(np.column_stack((np.array(self.theta_phi[i]), np.array(dist))))
                self.probability_goal_window[i][self.itr] = self.mv_nd.pdf(np.array(self.theta_phi[i]))

                self.probability_goal[i] = np.array([1.0]*self.goal_num)

                for k in range(0,len(self.probability_goal_window[i])):
                    gf = np.exp((k-self.window_size)/5)
                    
                    self.probability_goal[i] =  np.power(self.probability_goal_window[i][k],gf)* np.array(self.probability_goal[i]) # Linear prediction of goal

                for ln in range(0,len(self.goals_x)):
                    # self.probability_goal[i][ln] = (1/dist[ln])*self.probability_goal[i][ln]
                    self.probability_goal[i][ln] = 0.5*(1/dist[ln] + 1/self.theta_phi[i][ln])*self.probability_goal[i][ln]

                self.probability_goal[i] = (self.probability_goal[i]-np.min(self.probability_goal[i]))/(np.max(self.probability_goal[i])-np.min(self.probability_goal[i]))

                self.itr = self.itr + 1
                if self.itr == self.window_size:
                    self.itr = 0


        # response goal
        idx = 0
        max_prob = 0.0
        p_goal = PredictedGoal()

        for i in range(0,len(cur_poses_data[track_id])):
            for j in range(0,len(self.goals_x)):
                if(max_prob<self.probability_goal[i][j]):
                    idx = j
                    max_prob = self.probability_goal[i][j]

            self.predicted_goal.header.stamp = rospy.Time.now()
            self.predicted_goal.header.frame_id = 'map'
            self.predicted_goal.pose.position.x = self.goals_x[idx]
            self.predicted_goal.pose.position.y = self.goals_y[idx]
            self.predicted_goal.pose.position.z = 0.0
            self.predicted_goal.pose.orientation = cur_poses_data[track_id][i].orientation

            if self.last_idx != idx:
                p_goal.changed = True
                self.changed = True

        self.last_idx = idx
        p_goal.goal = self.predicted_goal   
        time_end = time.time()
        print("used time : ", time_end-time_start)
        return HumanGoalPredictResponse(p_goal)


    def goal_changed(self,req):
        if self.changed:
            self.changed = False
            return TriggerResponse(True,"Goal Changed")
        return TriggerResponse(False, "Goal not changed")

if __name__ == '__main__':
    predict_srv = PredictGoal(60)