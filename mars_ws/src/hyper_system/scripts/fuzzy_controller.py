#!/usr/bin/env python3

# Brief: ...
# Author: Po Lin Jiang

import rospy
import numpy as np
import skfuzzy as fuzz
import time
from fuzzy_definition import FyzzyDefinition
from std_msgs.msg import Bool
from skfuzzy import control as ctrl
from std_msgs.msg import String
from hyper_system.srv import Navigability
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import Float32
from dynamic_reconfigure.client import Client
from collections import deque

class FyzzyController:
    def __init__(self):
        rospy.init_node('fuzzy_controller', anonymous=True)

        rospy.Subscriber("/scenario", String, self.scenario_callback)
        rospy.Subscriber("/navigability", Float32, self.navigability_callback)
        rospy.Subscriber('nav_state', Bool, self.nav_state_callback)

        self.speed_up_level_pub = rospy.Publisher("/speed_up_level", Float32, queue_size=10)
        self.robot_invisiable_level_pub = rospy.Publisher("/robot_invisiable_level", Float32, queue_size=10)
        self.right_side_level_pub = rospy.Publisher("/right_side_level", Float32, queue_size=10)
        self.pspace_level_pub = rospy.Publisher("/pspace_level", Float32, queue_size=10)
        self.weight_optimaltime_pub = rospy.Publisher("/weight_optimaltime", Float32, queue_size=10)
        self.weight_cc_pub = rospy.Publisher("/weight_cc", Float32, queue_size=10)
        self.pspace_cov_pub = rospy.Publisher("/pspace_cov", Float32, queue_size=10)
        self.pspace_r_ratio_pub = rospy.Publisher("/pspace_r_ratio", Float32, queue_size=10)
        self.use_external_prediction_pub = rospy.Publisher("/use_external_prediction", Float32, queue_size=10)

        # parameter
        self.navigability = -1
        self.robot_move = False
        self.command = "rosrun dynamic_reconfigure dynparam set " 

        # Initialize the moving averages
        self.moving_average_window = 5
        self.weight_optimaltime_deque = deque(maxlen=self.moving_average_window)
        self.weight_cc_deque = deque(maxlen=self.moving_average_window)
        self.pspace_cov_deque = deque(maxlen=self.moving_average_window)
        self.pspace_r_ratio_deque = deque(maxlen=self.moving_average_window)
        self.external_predict_deque = deque(maxlen=self.moving_average_window)

        self.pa_weight_optimaltime = "/move_base/HATebLocalPlannerROS weight_optimaltime "
        self.pa_weight_cc = "/move_base/HATebLocalPlannerROS weight_cc "
        self.pa_pspace_cov_global= "/move_base/global_costmap/human_layer_static radius "
        self.pa_pspace_cov_local= "/move_base/local_costmap/human_layer_static radius "
        self.pa_pspace_r_ratio_global= "/move_base/global_costmap/human_layer_static right_cov_ratio "
        self.pa_pspace_r_ratio_local= "/move_base/local_costmap/human_layer_static right_cov_ratio "
        self.pa_use_external_prediction = "/move_base/HATebLocalPlannerROS use_external_prediction "  ## 0 or 1
        # self.pa_hr_safety = "/move_base/HATebLocalPlannerROS weight_human_robot_safety "

        # fuzzy controller
        self.fuzzy_df = FyzzyDefinition()
        self.optimaltime_controller = self.fuzzy_df.optimaltime_controller
        self.critical_corner_controller = self.fuzzy_df.critical_corner_controller
        self.pspace_cov_controller = self.fuzzy_df.pspace_cov_controller
        self.pspace_r_ratio_controller = self.fuzzy_df.pspace_r_ratio_controller
        self.human_path_prediction_controller = self.fuzzy_df.human_path_prediction_controller

        # scene semantics levels
        self.speed_up_level = -1    
        self.robot_invisiable_level = -1
        self.right_side_level = -1 
        self.pspace_level = -1 

        # output
        self.weight_optimaltime = 1
        self.pspace_cov = 0.7
        self.pspace_r_ratio = 1
        self.weight_cc = 0
        
        rate = rospy.Rate(5) # unit : HZ
        while not rospy.is_shutdown():
            if self.navigability != -1 and self.robot_move: 
                self.main_control()
            rate.sleep()
            
        rospy.spin()


    def scenario_callback(self, data):
        # rospy.loginfo("Received scenario data: %s", data.data)
        if data.data == "mall":
            self.speed_up_level = 5
            self.robot_invisiable_level = 10
            self.right_side_level = 6
            self.pspace_level = 10
        elif data.data == "corrider":
            self.speed_up_level = 7
            self.robot_invisiable_level = 5
            self.right_side_level = 10
            self.pspace_level = 10
        elif data.data == "warehouse":
            self.speed_up_level = 10
            self.robot_invisiable_level = 0
            self.right_side_level = 0
            self.pspace_level = 0
        else:
            self.speed_up_level = -1
            self.robot_invisiable_level = -1
            self.right_side_level = -1
            self.pspace_level = -1


    def navigability_callback(self, data):
        self.navigability = data.data
        # print("Received navigability value: ", self.navigability)
        
    
    def nav_state_callback(self, msg):
        if msg.data: 
            if not self.robot_move:
                self.robot_move = True
                rospy.loginfo("Navigation started.")
        else:  # If navigation is not active
            if self.robot_move:
                self.robot_move = False
                rospy.loginfo("Navigation ended.")
    
    def smooth_output(self, deque, value):
        deque.append(value)
        return sum(deque) / len(deque)

    def update_optimaltime(self):
        simulation = ctrl.ControlSystemSimulation(self.optimaltime_controller)
        simulation.input['navigability'] = self.navigability
        simulation.input['speed_up_level'] = self.speed_up_level
        simulation.compute()

        raw_output = simulation.output['weight_optimaltime']
        smoothed_output = self.smooth_output(self.weight_optimaltime_deque, raw_output)

        msg = Float32()
        msg.data = self.speed_up_level
        self.speed_up_level_pub.publish(msg)
        msg.data = smoothed_output
        self.weight_optimaltime_pub.publish(msg)
        print("updated optimaltime: ", msg.data)

        return smoothed_output
  

    def update_weight_cc(self):
        simulation = ctrl.ControlSystemSimulation(self.critical_corner_controller)
        simulation.input['navigability'] = self.navigability
        simulation.input['robot_invisiable_level'] = self.robot_invisiable_level
        simulation.compute()

        raw_output = simulation.output['weight_cc']
        smoothed_output = self.smooth_output(self.weight_cc_deque, raw_output)

        msg = Float32()
        msg.data = self.robot_invisiable_level
        self.robot_invisiable_level_pub.publish(msg)
        msg.data = smoothed_output
        self.weight_cc_pub.publish(msg)
        print("updated weight_cc: ", msg.data)

        return smoothed_output
    

    def update_pspace_cov(self):
        simulation = ctrl.ControlSystemSimulation(self.pspace_cov_controller)
        simulation.input['navigability'] = self.navigability
        simulation.input['pspace_level'] = self.pspace_level
        simulation.compute()

        raw_output = simulation.output['pspace_cov']
        smoothed_output = self.smooth_output(self.pspace_cov_deque, raw_output)

        msg = Float32()
        msg.data = self.pspace_level
        self.pspace_level_pub.publish(msg)
        msg.data = smoothed_output
        self.pspace_cov_pub.publish(msg)
        print("updated pspace_cov: ", msg.data)

        return smoothed_output
    

    def update_pspace_r_ratio(self):
        simulation = ctrl.ControlSystemSimulation(self.pspace_r_ratio_controller)
        simulation.input['navigability'] = self.navigability
        simulation.input['right_side_level'] = self.right_side_level
        simulation.compute()

        raw_output = simulation.output['pspace_r_ratio']
        smoothed_output = self.smooth_output(self.pspace_r_ratio_deque, raw_output)

        msg = Float32()
        msg.data = self.right_side_level
        self.right_side_level_pub.publish(msg)
        # msg.data = smoothed_output
        # self.pspace_r_ratio_pub.publish(msg)
        # print("updated pspace_r_ratio: ", msg.data)

        return smoothed_output

    
    def update_human_path_predict(self):
            simulation = ctrl.ControlSystemSimulation(self.human_path_prediction_controller)

            simulation.input['navigability'] = self.navigability
            simulation.compute()

            predict_thredshold = 0.6
            if simulation.output['use_external_prediction'] > predict_thredshold:
                external_predict = 1
            else:
                external_predict = 0

            msg = Float32()
            msg.data = external_predict
            self.use_external_prediction_pub.publish(msg)
            print("updated external_predict: ", msg.data)

            return external_predict
    
    def update_update_pspace_r_ratio_definitions(self, pspace_cov):
        L = 0.35 / pspace_cov
        cov = (1 - L) / 10

        self.pspace_r_ratio_controller = None

        self.fz_pspace_r_ratio = ctrl.Consequent(np.arange(L, 1.01, (1 - L) / 100), 'pspace_r_ratio')
        self.fz_pspace_r_ratio['VL'] = fuzz.gaussmf(self.fz_pspace_r_ratio.universe, L, 0.05)
        self.fz_pspace_r_ratio['L'] = fuzz.gaussmf(self.fz_pspace_r_ratio.universe, 0.75 * L + 0.25, cov)
        self.fz_pspace_r_ratio['M'] = fuzz.gaussmf(self.fz_pspace_r_ratio.universe, 0.5 * L + 0.5, cov)
        self.fz_pspace_r_ratio['H'] = fuzz.gaussmf(self.fz_pspace_r_ratio.universe, 0.25 * L + 0.75, cov)
        self.fz_pspace_r_ratio['VH'] = fuzz.gaussmf(self.fz_pspace_r_ratio.universe, 1.0, cov)

        new_rule46 = ctrl.Rule(self.fuzzy_df.fz_navigability['VL'] & self.fuzzy_df.fz_right_side_level['L'], self.fz_pspace_r_ratio['M'])
        new_rule47 = ctrl.Rule(self.fuzzy_df.fz_navigability['VL'] & self.fuzzy_df.fz_right_side_level['M'], self.fz_pspace_r_ratio['L'])
        new_rule48 = ctrl.Rule(self.fuzzy_df.fz_navigability['VL'] & self.fuzzy_df.fz_right_side_level['H'], self.fz_pspace_r_ratio['VL'])
        new_rule49 = ctrl.Rule(self.fuzzy_df.fz_navigability['L'] & self.fuzzy_df.fz_right_side_level['L'], self.fz_pspace_r_ratio['H'])
        new_rule50 = ctrl.Rule(self.fuzzy_df.fz_navigability['L'] & self.fuzzy_df.fz_right_side_level['M'], self.fz_pspace_r_ratio['M'])
        new_rule51 = ctrl.Rule(self.fuzzy_df.fz_navigability['L'] & self.fuzzy_df.fz_right_side_level['H'], self.fz_pspace_r_ratio['L'])
        new_rule52 = ctrl.Rule(self.fuzzy_df.fz_navigability['M'] & self.fuzzy_df.fz_right_side_level['L'], self.fz_pspace_r_ratio['H'])
        new_rule53 = ctrl.Rule(self.fuzzy_df.fz_navigability['M'] & self.fuzzy_df.fz_right_side_level['M'], self.fz_pspace_r_ratio['M'])
        new_rule54 = ctrl.Rule(self.fuzzy_df.fz_navigability['M'] & self.fuzzy_df.fz_right_side_level['H'], self.fz_pspace_r_ratio['L'])
        new_rule55 = ctrl.Rule(self.fuzzy_df.fz_navigability['H'] & self.fuzzy_df.fz_right_side_level['L'], self.fz_pspace_r_ratio['VH'])
        new_rule56 = ctrl.Rule(self.fuzzy_df.fz_navigability['H'] & self.fuzzy_df.fz_right_side_level['M'], self.fz_pspace_r_ratio['H'])
        new_rule57 = ctrl.Rule(self.fuzzy_df.fz_navigability['H'] & self.fuzzy_df.fz_right_side_level['H'], self.fz_pspace_r_ratio['M'])
        new_rule58 = ctrl.Rule(self.fuzzy_df.fz_navigability['VH'] & self.fuzzy_df.fz_right_side_level['L'], self.fz_pspace_r_ratio['VH'])
        new_rule59 = ctrl.Rule(self.fuzzy_df.fz_navigability['VH'] & self.fuzzy_df.fz_right_side_level['M'], self.fz_pspace_r_ratio['H'])
        new_rule60 = ctrl.Rule(self.fuzzy_df.fz_navigability['VH'] & self.fuzzy_df.fz_right_side_level['H'], self.fz_pspace_r_ratio['M'])
        self.pspace_r_ratio_controller = ctrl.ControlSystem(
            [
                new_rule46, new_rule47, new_rule48, new_rule49, new_rule50, 
                new_rule51, new_rule52, new_rule53, new_rule54, new_rule55, 
                new_rule56, new_rule57, new_rule58, new_rule59, new_rule60
            ]
        )
            

    def main_control(self):

        print("--------  navigability = ", self.navigability, "  --------")

        if  self.speed_up_level != -1: 
            self.weight_optimaltime = self.update_optimaltime()
        
        if self.pspace_level != -1: 
            self.pspace_cov = self.update_pspace_cov()

        if self.right_side_level != -1: 
            self.pspace_r_ratio = self.update_pspace_r_ratio()
        msg = Float32()
        # msg.data = self.pspace_r_ratio * self.pspace_cov
        msg.data = self.pspace_r_ratio
        self.pspace_r_ratio_pub.publish(msg)
        print("updated pspace_r_ratio: ", self.pspace_r_ratio)
        print("updated pspace_r_cov: ", msg.data)

        
        if self.robot_invisiable_level != -1: 
            self.weight_cc = self.update_weight_cc()

        self.external_predict = self.update_human_path_predict()

        # if pspace_r_ratio and pspace_cov and weight_optimaltime and weight_cc and external_predict:
        
        hateb_client = Client("/move_base/HATebLocalPlannerROS", timeout=30)
        human_layer_global_client = Client("/move_base/global_costmap/human_layer_static", timeout=30)
        human_layer_local_client = Client("/move_base/local_costmap/human_layer_static", timeout=30)

        hateb_config = {
            "weight_optimaltime": "{:.3f}".format(self.weight_optimaltime),
            "weight_cc": "{:.3f}".format(self.weight_cc),
            # "use_external_prediction": "{:.3f}".format(self.external_predict),
        }
        human_layer_global_config = {
            "radius": "{:.3f}".format(self.pspace_cov),
            "right_cov_ratio": "{:.3f}".format(self.pspace_r_ratio)
        }
        human_layer_local_config = {
            "radius": "{:.3f}".format(self.pspace_cov),
            "right_cov_ratio": "{:.3f}".format(self.pspace_r_ratio)
        }
        
        hateb_client.update_configuration(hateb_config)
        human_layer_global_client.update_configuration(human_layer_global_config)
        human_layer_local_client.update_configuration(human_layer_local_config)
        self.update_update_pspace_r_ratio_definitions(self.pspace_cov)


if __name__ == '__main__':
    try:
        FyzzyController()
    except rospy.ROSInterruptException:
        pass