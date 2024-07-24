#!/usr/bin/env python3

# Brief: ...
# Author: Po Lin Jiang

import rospy
import numpy as np
import skfuzzy as fuzz
import time
import fuzzy_definition
from std_msgs.msg import Bool
from skfuzzy import control as ctrl
from std_msgs.msg import String
from hyper_system.srv import Navigability
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import Float32
from dynamic_reconfigure.client import Client

class FyzzyController:
    def __init__(self):
        rospy.init_node('fuzzy_controller', anonymous=True)

        rospy.Subscriber("/scenario", String, self.scenario_callback)
        rospy.Subscriber("/navigability", Float32, self.navigability_callback)
        # rospy.Subscriber("/move_base/status", GoalStatusArray, self.robot_status_callback)
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
        self.pa_weight_optimaltime = "/move_base/HATebLocalPlannerROS weight_optimaltime "
        self.pa_weight_cc = "/move_base/HATebLocalPlannerROS weight_cc "
        self.pa_pspace_cov_global= "/move_base/global_costmap/human_layer_static radius "
        self.pa_pspace_cov_local= "/move_base/local_costmap/human_layer_static radius "
        self.pa_pspace_r_ratio_global= "/move_base/global_costmap/human_layer_static right_cov_ratio "
        self.pa_pspace_r_ratio_local= "/move_base/local_costmap/human_layer_static right_cov_ratio "
        self.pa_use_external_prediction = "/move_base/HATebLocalPlannerROS use_external_prediction "  ## 0 or 1
        # self.pa_hr_safety = "/move_base/HATebLocalPlannerROS weight_human_robot_safety "

        # fuzzy controller
        self.fuzzy_df = fuzzy_definition.FyzzyDefinition()
        # self.fuzzy_df = FyzzyDefinition()
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
        
        rate = rospy.Rate(5) # unit : HZ
        while not rospy.is_shutdown():
            if  self.navigability != -1 and self.robot_move: 
                self.main_control()
            rate.sleep()
            
        rospy.spin()


    def scenario_callback(self, data):
        # rospy.loginfo("Received scenario data: %s", data.data)
        if data.data == "mall":
            self.speed_up_level = 5
            self.robot_invisiable_level =10
            self.right_side_level = 5
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
        else :
            self.speed_up_level = -1
            self.robot_invisiable_level = -1
            self.right_side_level = -1
            self.pspace_level = -1


    def navigability_callback(self, data):
        self.navigability = data.data
        # print("Received navigability value: ", self.navigability)
        

    # def robot_status_callback(self, data):
    #     if not data.status_list or data.status_list[-1].text == "Goal reached.":
    #         self.robot_move = False
    #     else:
    #         self.robot_move = True
    
    def nav_state_callback(self, msg):
        if msg.data: 
            if not self.robot_move:
                self.robot_move = True
                rospy.loginfo("Navigation started.")
        else:  # If navigation is not active
            if self.robot_move:
                self.robot_move = False
                rospy.loginfo("Navigation ended.")
    
    
    def update_optimaltime(self):
        simulation = ctrl.ControlSystemSimulation(self.optimaltime_controller)

        simulation.input['navigability'] = self.navigability
        simulation.input['speed_up_level'] = self.speed_up_level
        simulation.compute()

        msg = Float32()
        msg.data = self.speed_up_level
        self.speed_up_level_pub.publish(msg)
        msg.data = simulation.output['weight_optimaltime']
        self.weight_optimaltime_pub.publish(msg)
        print("updated optimaltime: ", msg.data)

        return simulation.output['weight_optimaltime']
  

    def update_weight_cc(self):
            simulation = ctrl.ControlSystemSimulation(self.critical_corner_controller)

            simulation.input['navigability'] = self.navigability
            simulation.input['robot_invisiable_level'] = self.robot_invisiable_level
            simulation.compute()

            msg = Float32()
            msg.data = self.robot_invisiable_level
            self.robot_invisiable_level_pub.publish(msg)
            msg.data = simulation.output['weight_cc']
            self.weight_cc_pub.publish(msg)
            print("updated weight_cc: ", msg.data)

            return simulation.output['weight_cc']
    

    def update_pspace_cov(self):
            simulation = ctrl.ControlSystemSimulation(self.pspace_cov_controller)

            simulation.input['navigability'] = self.navigability
            simulation.input['pspace_level'] = self.pspace_level
            simulation.compute()

            msg = Float32()
            msg.data = self.pspace_level
            self.pspace_level_pub.publish(msg)
            msg.data = simulation.output['pspace_cov']
            self.pspace_cov_pub.publish(msg)
            print("updated pspace_cov: ", msg.data)

            return simulation.output['pspace_cov']
    

    def update_pspace_r_ratio(self):
            simulation = ctrl.ControlSystemSimulation(self.pspace_r_ratio_controller)

            simulation.input['navigability'] = self.navigability
            simulation.input['right_side_level'] = self.right_side_level
            simulation.compute()
        
            msg = Float32()
            msg.data = self.right_side_level
            self.right_side_level_pub.publish(msg)
            msg.data = simulation.output['pspace_r_ratio']
            self.pspace_r_ratio_pub.publish(msg)
            print("updated pspace_r_ratio: ", msg.data)

            return simulation.output['pspace_r_ratio']

    
    def update_human_path_predict(self):
            simulation = ctrl.ControlSystemSimulation(self.human_path_prediction_controller)

            simulation.input['navigability'] = self.navigability
            simulation.compute()

            predict_thredshold = 0.7
            if simulation.output['use_external_prediction'] > predict_thredshold:
                external_predict = 1
            else:
                external_predict = 0

            msg = Float32()
            msg.data = external_predict
            self.use_external_prediction_pub.publish(msg)
            print("updated external_predict: ", msg.data)

            return external_predict
            

    def main_control(self):
        print("--------  navigability = ", self.navigability, "  --------")
        if  self.speed_up_level != -1: 
            weight_optimaltime = self.update_optimaltime()
        
        if self.pspace_level != -1: 
            pspace_cov = self.update_pspace_cov()

        if self.right_side_level != -1: 
            pspace_r_ratio = self.update_pspace_r_ratio()
        
        if self.robot_invisiable_level != -1: 
            weight_cc = self.update_weight_cc()

        external_predict = self.update_human_path_predict()

        hateb_client = Client("/move_base/HATebLocalPlannerROS", timeout=30)
        human_layer_global_client = Client("/move_base/global_costmap/human_layer_static", timeout=30)
        human_layer_local_client = Client("/move_base/local_costmap/human_layer_static", timeout=30)

        hateb_config = {
            "weight_optimaltime": "{:.3f}".format(weight_optimaltime),
            "weight_cc": "{:.3f}".format(weight_cc),
            # "use_external_prediction": "{:.3f}".format(external_predict),
        }
        human_layer_global_config = {
            "radius": "{:.3f}".format(pspace_cov),
            "right_cov_ratio": "{:.3f}".format(pspace_r_ratio)
        }
        human_layer_local_config = {
            "radius": "{:.3f}".format(pspace_cov),
            "right_cov_ratio": "{:.3f}".format(pspace_r_ratio)
        }
        
        hateb_client.update_configuration(hateb_config)
        human_layer_global_client.update_configuration(human_layer_global_config)
        human_layer_local_client.update_configuration(human_layer_local_config)


if __name__ == '__main__':
    try:
        FyzzyController()
    except rospy.ROSInterruptException:
        pass