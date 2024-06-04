#!/usr/bin/env python3

# Brief: ...
# Author: Po Lin Jiang

import rospy
import numpy as np
import skfuzzy as fuzz
import subprocess
import time
import fuzzy_definition
from skfuzzy import control as ctrl
from std_msgs.msg import String
from hyper_system.srv import Navigability
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import Float32


class FyzzyController:
    def __init__(self):
        rospy.init_node('fuzzy_controller', anonymous=True)

        # fuzzy controller
        self.fuzzy_df = fuzzy_definition.FyzzyDefinition()
        self.optimaltime_controller = self.fuzzy_df.optimaltime_controller


        # scene semantics levels
        self.speed_up_level = -1    
        self.robot_invisiable_level = -1
        self.right_side_level = -1 
        self.pspace_level = -1 

        # parameter
        self.navigability = -1
        self.robot_move = False
        self.command = "rosrun dynamic_reconfigure dynparam set " 
        self.pa_weight_optimaltime = "/move_base/HATebLocalPlannerROS weight_optimaltime "
        self.pa_weight_cc = "/move_base/HATebLocalPlannerROS weight_cc "
        self.pa_hr_safety = "/move_base/HATebLocalPlannerROS weight_human_robot_safety "
        self.pa_pspace_cov_global= "/move_base/global_costmap/human_layer_static radius "
        self.pa_pspace_cov_local= "/move_base/local_costmap/human_layer_static radius "
        self.pa_pspace_r_ratio_global= "/move_base/global_costmap/human_layer_static right_cov_ratio "
        self.pa_pspace_r_ratio_local= "/move_base/local_costmap/human_layer_static right_cov_ratio "
        self.pa_use_external_prediction = "/move_base/HATebLocalPlannerROS use_external_prediction "  ## + "true" or "false"

        rospy.Subscriber("/scenario", String, self.scenario_callback)
        rospy.Subscriber("/navigability", Float32, self.navigability_callback)
        rospy.Subscriber("/move_base/status", GoalStatusArray, self.robot_status_callback)

        self.speed_up_level_pub = rospy.Publisher("/speed_up_level", Float32, queue_size=10)
        self.weight_optimaltime_pub = rospy.Publisher("/weight_optimaltime", Float32, queue_size=10)
        
        rate = rospy.Rate(5) # unit : HZ
        while not rospy.is_shutdown():
            print(self.speed_up_level, self.navigability, self.robot_move)
            if self.speed_up_level != -1 and self.navigability != -1 and self.robot_move: 
                self.update_optimaltime()
                pass
            else:
                pass
                # print("not updated")
            rate.sleep()
        rospy.spin()


    def scenario_callback(self, data):
        # rospy.loginfo("Received scenario data: %s", data.data)
        if data.data == "mall":
            self.speed_up_level = 5
            self.robot_invisiable_level =10
        elif data.data == "corrider":
            self.speed_up_level = 7
            self.robot_invisiable_level = 5
        elif data.data == "warehouse":
            self.speed_up_level = 10
            self.robot_invisiable_level = 0
        else :
            self.speed_up_level = -1
            self.robot_invisiable_level = -1


    def navigability_callback(self, data):
        self.navigability = data.data
        # print("Received navigability value: ", self.navigability)
        

    def robot_status_callback(self, data):
        if data.status_list[-1].text == "Goal reached.":
            self.robot_move = False
        else:
            self.robot_move = True
    
    
    def update_optimaltime(self):
        simulation = ctrl.ControlSystemSimulation(self.optimaltime_controller)

        simulation.input['navigability'] = self.navigability
        simulation.input['speed_up_level'] = self.speed_up_level
        simulation.compute()
        
        command = self.command + self.pa_weight_optimaltime + "{:.3f}".format(simulation.output['weight_optimaltime'])
        process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        msg = Float32()
        msg.data = self.speed_up_level
        self.speed_up_level_pub.publish(msg)
        msg.data = simulation.output['weight_optimaltime']
        self.weight_optimaltime_pub.publish(msg)
        # print(command)
        # print('navigability:', self.navigability, 'speed_up_level:', self.speed_up_level, "Weight Optimal Time:", simulation.output['weight_optimaltime'])
  

if __name__ == '__main__':
    try:
        FyzzyController()
    except rospy.ROSInterruptException:
        pass