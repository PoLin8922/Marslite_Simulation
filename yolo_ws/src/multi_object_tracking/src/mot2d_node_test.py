#!/usr/bin/env python3

# -*- coding: utf-8 -*-
'''
@Time          : 20/04 / 25 15:49
@Author        : huguanghao
@File          : demo.py
@Noice         :
@Modificattion :
    @Author    :
    @Time      :
    @Detail    :

'''

import argparse
import os
import sys
import math
import tf
import time
import copy
import numpy as np
from AB3DMOT_libs.model import AB3DMOT
import message_filters 

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from visualization_msgs.msg import Marker, MarkerArray
from walker_msgs.msg import Det3D, Det3DArray
from walker_msgs.msg import Trk3D, Trk3DArray
from sensor_msgs.msg import LaserScan

from geometry_msgs.msg import Pose, Twist, PoseStamped, TwistStamped, TransformStamped
from human_msgs.msg import TrackedHumans
from human_msgs.msg import TrackedHuman
from human_msgs.msg import TrackedSegmentType
from human_msgs.msg import TrackedSegment
import tf2_ros
# import tf2_geometry_msgs
import tf.transformations as tft

INTEREST_CLASSES = ["person"]
MARKER_LIFETIME = 0.1


def euler_to_quaternion(roll, pitch, yaw):
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    return [qx, qy, qz, qw]


class MultiObjectTrackingNode(object):
    def __init__(self):
        rospy.on_shutdown(self.shutdown_cb)
        self.last_time = None
        
        
        # Tracker
        self.mot_tracker = AB3DMOT(max_age=6, min_hits=3)


        self.pub_trk3d_vis = rospy.Publisher('trk3d_vis', MarkerArray, queue_size=1)
        self.pub_trk3d_result = rospy.Publisher('trk3d_result', Trk3DArray, queue_size=1)

        self.tracked_humans_pub = rospy.Publisher("/tracked_humans", TrackedHumans, queue_size=1)
        self.Segment_Type = TrackedSegmentType.TORSO
        
        self.odom_sub = rospy.Subscriber('/odom_filtered', Odometry, self.odom_cb)
        self.sub_det3d = rospy.Subscriber("det3d_result", Det3DArray,self.det_result_cb)
        
        # TF Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Ego velocity init
        self.ego_velocity = Vector3()
        self.ego_theta = Vector3()

        #param
        self.trk3d_array = Trk3DArray()
        
        rospy.loginfo(rospy.get_name() + ' is ready.')

    # def scan_cb(self,scan_msg):
    #     print("scan ")
    #     self.trk3d_array.scan = scan_msg.ranges
    def odom_cb(self, odom_msg):
        self.ego_velocity = odom_msg.twist.twist.linear
        self.ego_theta = odom_msg.twist.twist.angular
        # print("vx",self.ego_velocity.x)
        # print("vy",self.ego_velocity.y)


    def det_result_cb(self, msg): #msg for person position and scan_msg for laserscanning
        #print("det")
        dets_list = None
        info_list = None
        
        for idx, det in enumerate(msg.dets_list):
            if dets_list is None:
                dets_list = np.array([det.x, det.y, det.radius], dtype=np.float32)
                info_list = np.array([det.confidence, det.class_id], dtype=np.float32)
            else:
                dets_list = np.vstack([dets_list, [det.x, det.y, det.radius]])
                info_list = np.vstack([info_list, [det.confidence, det.class_id]])

        
        # if len(dets_list.shape) == 1: dets_list = np.expand_dims(dets_list, axis=0)
        # if len(info_list.shape) == 1: info_list = np.expand_dims(info_list, axis=0)msg
        # if dets_list.shape[1] == 0:
        #     # If there is no detection, just pack the laserscan topic for localmap creation
        #     trk3d_array = Trk3DArray()
        #     trk3d_array.header.frame_id = msg.header.frame_id
        #     trk3d_array.header.stamp = rospy.Time.now()
        #     trk3d_array.scan = msg.scan
        #     self.pub_trk3d_result.publish(trk3d_array)
        #     return

        # rospy.loginfo('number of alive trks: {}'.format(len(self.mot_tracker.trackers)))

        if dets_list is not None:
            if len(dets_list.shape) == 1: dets_list = np.expand_dims(dets_list, axis=0)
            if len(info_list.shape) == 1: info_list = np.expand_dims(info_list, axis=0)
            dets_all = {'dets': dets_list, 'info': info_list}
            print("O")
            trackers = self.mot_tracker.update(dets_all)
        else:
            # print("X")
            # if no detection in a sequence
            trackers = self.mot_tracker.update_with_no_dets()
        
        # Skip the visualization at first callback
        if self.last_time is None:
            self.last_time = rospy.Time.now()
            # self.last_time = time.time()
            return

        # saving results, loop over each tracklet           
        marker_array = MarkerArray()

        time_now = rospy.Time.now()
        delta_t = (time_now - self.last_time).to_sec()
        if delta_t < 0.1:
            delta_t = 0.1
            # small_time = (0.1 - delta_t) * 0.92
            # delta_t += small_time
            # rospy.sleep(small_time)

        # sys.stdout.write("{:.4f} s \r".format(delta_t))
        # sys.stdout.flush()
        self.last_time = time_now
        self.trk3d_array.trks_list = []
        self.tracked_humans = TrackedHumans()
        for idx, d in enumerate(trackers):
            '''
                x, y, r, vx, vy, id, confidence, class_id
            '''
            d_now = np.sqrt(d[0]**2 + d[1]**2)
            #print(d[0])
            #print(d[1])
            # vx, vy = np.array([d[3], d[4]]) / (time.time() - self.last_time)
            vx, vy = np.array([d[3], d[4]]) / delta_t #- np.array([self.ego_velocity.x, self.ego_velocity.y])-np.array([d_now*math.sin(self.ego_theta.z),d_now*math.cos(self.ego_theta.z)])-
            speed = np.sqrt(vx**2 + vy**2)       # Note: reconstruct speed by multipling the sampling rate 
            yaw = np.arctan2(vy, vx) 
            if(speed>2):
                speed = 2
            if(speed>0.5):
                dangerous=2*speed/(1+np.exp(0.3*np.sqrt(d[0]**2 + d[1]**2)))
                #print("peolple dangerous:",dangerous)
            else:
                dangerous=2*0.5/(1+np.exp(0.3*np.sqrt(d[0]**2 + d[1]**2)))
                #print("peolple dangerous:",dangerous)
            #print(speed)
            # Custom ROS message
            trk3d_msg = Trk3D()
            trk3d_msg.x, trk3d_msg.y = d[0], d[1]
            # print("peolple x:",trk3d_msg.x)
            # print("peolple y:",trk3d_msg.y)
            trk3d_msg.radius = d[2]
            trk3d_msg.vx, trk3d_msg.vy = vx, vy
            trk3d_msg.yaw = yaw
            trk3d_msg.confidence = d[6]
            trk3d_msg.class_id = int(d[7])
            trk3d_msg.dangerous = dangerous
            self.trk3d_array.trks_list.append(trk3d_msg)
            # print("------------------------------------")
            # print("x,y = ",trk3d_msg.x, trk3d_msg.y)
            # print("trk3d_msg.yaw =", trk3d_msg.yaw*180/math.pi)
            # print("peolple speed:",speed)

            
            # Visualization
            marker = Marker()
            marker.header.frame_id ='odom' #msg.header.frame_id #'odom'
            marker.header.stamp = rospy.Time().now()
            marker.ns = 'object'
            marker.id = idx
            marker.lifetime = rospy.Duration(MARKER_LIFETIME)#The lifetime of the bounding-box, you can modify it according to the power of your machine.
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.scale.x = d[2]*2
            marker.scale.y = d[2]*2
            marker.scale.z = 1.0
            marker.color.b = 1.0
            marker.color.a = 0.5 #The alpha of the bounding-box
            marker.pose.position.x = d[0]
            marker.pose.position.y = d[1]
            marker.pose.position.z = 0.5
            q = euler_to_quaternion(0, 0, yaw)
            marker.pose.orientation.x = q[0]
            marker.pose.orientation.y = q[1]
            marker.pose.orientation.z = q[2]
            marker.pose.orientation.w = q[3]
            marker_array.markers.append(marker)


            # Show tracking ID
            str_marker = Marker()
            str_marker.header.frame_id = 'odom' #msg.header.frame_id #'odom'
            str_marker.header.stamp = rospy.Time().now()
            str_marker.ns = 'text'
            str_marker.id = idx
            str_marker.scale.z = 0.4 #The size of the text
            str_marker.color.b = 1.0
            str_marker.color.g = 1.0
            str_marker.color.r = 1.0
            str_marker.color.a = 1.0
            str_marker.pose.position.x = d[0]
            str_marker.pose.position.y = d[1]
            str_marker.pose.position.z = 0.5
            str_marker.lifetime = rospy.Duration(MARKER_LIFETIME)
            str_marker.type = Marker.TEXT_VIEW_FACING
            str_marker.action = Marker.ADD
            str_marker.text = "{}".format(int(d[5])) # str(d[5])
            marker_array.markers.append(str_marker)
            
            # Show direction 
            arrow_marker = copy.deepcopy(marker)
            arrow_marker.type = Marker.ARROW
            arrow_marker.ns = 'direction'
            arrow_marker.scale.x = 2 * (speed/1.5)
            arrow_marker.scale.y = 0.2
            arrow_marker.scale.z = 0.2 
            marker_array.markers.append(arrow_marker)

            # Pub humans
            human_segment = TrackedSegment()
            human_segment.type = self.Segment_Type
            
            pose = Pose()
            pose.position.x = d[0]
            pose.position.y = d[1]
            pose.position.z = 0.0
            pose.orientation.x = q[0]
            pose.orientation.y = q[1]
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]
            # transformed_pose = self.transform_pose(pose, 'map')

            
            if abs(vx) < 0.2:
                vx = 0
            if abs(vy) < 0.2:
                vy = 0
            twist = Twist()
            twist.linear.x = vx
            twist.linear.y = vy
            twist.linear.z = 0.0
            # transformed_twist = self.transform_twist(twist, 'map')

            # human_segment.pose.pose = transformed_pose
            # human_segment.twist.twist = transformed_twist
            human_segment.pose.pose = pose
            human_segment.twist.twist = twist

            tracked_human = TrackedHuman()
            tracked_human.track_id = idx + 1  
            tracked_human.segments.append(human_segment)
            self.tracked_humans.humans.append(tracked_human)

        if self.tracked_humans.humans:
            self.tracked_humans.header.stamp = rospy.Time.now()
            # self.tracked_humans.header.frame_id = 'map'
            self.tracked_humans.header.frame_id = 'odom'
            self.tracked_humans_pub.publish(self.tracked_humans)

        self.pub_trk3d_vis.publish(marker_array)

        #self.trk3d_array.scan = scan_msg
        self.trk3d_array.header.frame_id = 'odom' #msg.header.frame_id
        self.trk3d_array.header.stamp = rospy.Time().now()
        #trk3d_array.pointcloud = msg.pointcloud
        self.pub_trk3d_result.publish(self.trk3d_array)


        # self.last_time = time.time()

        # print("elapsed time: {}".format(cycle_time))        

    # def transform_pose(self, pose, target_frame):
    #     """
    #     Transforms a Pose from its current frame to the target frame.
    #     """
    #     pose_stamped = PoseStamped()
    #     pose_stamped.pose = pose
    #     pose_stamped.header.frame_id = 'odom'
    #     pose_stamped.header.stamp = rospy.Time.now()

    #     try:
    #         transform = self.tf_buffer.lookup_transform(target_frame, 'odom', rospy.Time(0))
    #         transformed_pose = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
    #         return transformed_pose.pose
    #     except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
    #         rospy.logerr(e)
    #         return None

    def transform_pose(self, pose, target_frame):
        """
        Transforms a Pose from its current frame to the target frame without using tf2_geometry_msgs.
        """
        pose_stamped = PoseStamped()
        pose_stamped.pose = pose
        pose_stamped.header.frame_id = 'odom'
        pose_stamped.header.stamp = rospy.Time.now()

        try:
            transform = self.tf_buffer.lookup_transform(target_frame, 'odom', rospy.Time(0))

            # Extract translation and rotation from the transform
            translation = transform.transform.translation
            rotation = transform.transform.rotation

            # Create transformation matrix from translation and rotation
            transform_matrix = tft.concatenate_matrices(
                tft.translation_matrix([translation.x, translation.y, translation.z]),
                tft.quaternion_matrix([rotation.x, rotation.y, rotation.z, rotation.w])
            )

            # Extract the pose components
            pose_position = [pose.position.x, pose.position.y, pose.position.z, 1.0]
            pose_orientation = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]

            # Apply the transform
            transformed_position = tft.translation_from_matrix(tft.concatenate_matrices(transform_matrix, tft.translation_matrix(pose_position)))
            transformed_orientation = tft.quaternion_from_matrix(tft.concatenate_matrices(transform_matrix, tft.quaternion_matrix(pose_orientation)))

            # Construct the transformed pose
            transformed_pose = PoseStamped()
            transformed_pose.pose.position.x = transformed_position[0]
            transformed_pose.pose.position.y = transformed_position[1]
            transformed_pose.pose.position.z = transformed_position[2]
            transformed_pose.pose.orientation.x = transformed_orientation[0]
            transformed_pose.pose.orientation.y = transformed_orientation[1]
            transformed_pose.pose.orientation.z = transformed_orientation[2]
            transformed_pose.pose.orientation.w = transformed_orientation[3]
            transformed_pose.header.frame_id = target_frame
            transformed_pose.header.stamp = rospy.Time.now()

            return transformed_pose.pose
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(e)
            return None
    
    def transform_twist(self, twist, target_frame):
        """
        Transforms a Twist (linear velocity) from its current frame to the target frame.
        """
        try:
            transform = self.tf_buffer.lookup_transform(target_frame, 'odom', rospy.Time(0))
            rotation = transform.transform.rotation
            rotation_matrix = tf.transformations.quaternion_matrix([rotation.x, rotation.y, rotation.z, rotation.w])
            rotation_matrix = rotation_matrix[:3, :3]

            lin_vel = np.array([twist.linear.x, twist.linear.y, twist.linear.z])
            transformed_lin_vel = np.dot(rotation_matrix, lin_vel)

            transformed_twist = Twist()
            transformed_twist.linear.x = transformed_lin_vel[0]
            transformed_twist.linear.y = transformed_lin_vel[1]
            transformed_twist.linear.z = transformed_lin_vel[2]
            transformed_twist.angular = twist.angular  
            return transformed_twist
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(e)
            return None

    
    def shutdown_cb(self):
        rospy.loginfo("Shutdown " + rospy.get_name())


if __name__ == '__main__':
    rospy.init_node('mot_node', anonymous=False)

    
    node = MultiObjectTrackingNode()
    rospy.spin()
