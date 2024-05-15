#!/usr/bin/env python3

# Brief: ...
# Author: Po Lin Jiang

import rospy
import numpy as np
import skfuzzy as fuzz
import subprocess
import time
import matplotlib.pyplot as plt
from skfuzzy import control as ctrl
from std_msgs.msg import String
from hyper_system.srv import Navigability
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import Float32

# define fuzzy input
fz_navigability = ctrl.Antecedent(np.arange(0, 1.01, 0.01), 'navigability')
fz_speed_up_level = ctrl.Antecedent(np.arange(0, 11, 0.1), 'speed_up_level')

# define fuzzy output
fz_weight_optimaltime = ctrl.Consequent(np.arange(5, 31, 0.1), 'weight_optimaltime')

# define membership function
fz_navigability['VL'] = fuzz.gaussmf(fz_navigability.universe, 0, 0.2)
fz_navigability['L'] = fuzz.gaussmf(fz_navigability.universe, 0.25, 0.2)
fz_navigability['M'] = fuzz.gaussmf(fz_navigability.universe, 0.5, 0.2)
fz_navigability['H'] = fuzz.gaussmf(fz_navigability.universe, 0.75, 0.2)
fz_navigability['VH'] = fuzz.gaussmf(fz_navigability.universe, 1, 0.2)

fz_speed_up_level['L'] = fuzz.gaussmf(fz_speed_up_level.universe, 0, 2)
fz_speed_up_level['M'] = fuzz.gaussmf(fz_speed_up_level.universe, 5, 2)
fz_speed_up_level['H'] = fuzz.gaussmf(fz_speed_up_level.universe, 10, 2)

fz_weight_optimaltime['VL'] = fuzz.gaussmf(fz_weight_optimaltime.universe, 5, 3)
fz_weight_optimaltime['L'] = fuzz.gaussmf(fz_weight_optimaltime.universe, 10, 3)
fz_weight_optimaltime['M'] = fuzz.gaussmf(fz_weight_optimaltime.universe, 15, 3)
fz_weight_optimaltime['H'] = fuzz.gaussmf(fz_weight_optimaltime.universe, 20, 3)
fz_weight_optimaltime['VH'] = fuzz.gaussmf(fz_weight_optimaltime.universe, 25, 3)

# define rules
## fz_navigability, fz_speed_up_level | fz_weight_optimaltime
rule1 = ctrl.Rule(fz_navigability['VL'] & fz_speed_up_level['L'], fz_weight_optimaltime['VL'])
rule2 = ctrl.Rule(fz_navigability['VL'] & fz_speed_up_level['M'], fz_weight_optimaltime['L'])
rule3 = ctrl.Rule(fz_navigability['VL'] & fz_speed_up_level['H'], fz_weight_optimaltime['L'])
rule4 = ctrl.Rule(fz_navigability['L'] & fz_speed_up_level['L'], fz_weight_optimaltime['L'])
rule5 = ctrl.Rule(fz_navigability['L'] & fz_speed_up_level['M'], fz_weight_optimaltime['M'])
rule6 = ctrl.Rule(fz_navigability['L'] & fz_speed_up_level['H'], fz_weight_optimaltime['M'])
rule7 = ctrl.Rule(fz_navigability['M'] & fz_speed_up_level['L'], fz_weight_optimaltime['L'])
rule8 = ctrl.Rule(fz_navigability['M'] & fz_speed_up_level['M'], fz_weight_optimaltime['M'])
rule9 = ctrl.Rule(fz_navigability['M'] & fz_speed_up_level['H'], fz_weight_optimaltime['H'])
rule10 = ctrl.Rule(fz_navigability['H'] & fz_speed_up_level['L'], fz_weight_optimaltime['M'])
rule11 = ctrl.Rule(fz_navigability['H'] & fz_speed_up_level['M'], fz_weight_optimaltime['M'])
rule12 = ctrl.Rule(fz_navigability['H'] & fz_speed_up_level['H'], fz_weight_optimaltime['H'])
rule13 = ctrl.Rule(fz_navigability['VH'] & fz_speed_up_level['L'], fz_weight_optimaltime['H'])
rule14 = ctrl.Rule(fz_navigability['VH'] & fz_speed_up_level['M'], fz_weight_optimaltime['H'])
rule15 = ctrl.Rule(fz_navigability['VH'] & fz_speed_up_level['H'], fz_weight_optimaltime['VH']) 

# tuned parameter
pa_weight_optimaltime = "/move_base/HATebLocalPlannerROS weight_optimaltime "


class FyzzyController:
    def __init__(self):
        rospy.init_node('fuzzy_controller', anonymous=True)

        fz_navigability.view()
        fz_speed_up_level.view()
        fz_weight_optimaltime.view()
        plt.show()

        # scene semantics levels
        self.speed_up_level = -1  

        # navigability
        self.navigability = -1

        # parameter
        self.robot_move = False
        self.command = "rosrun dynamic_reconfigure dynparam set " 

        # fuzzy controller
        self.optimaltime_controller = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8,
                                  rule9, rule10, rule11, rule12, rule13, rule14, rule15])


        rospy.Subscriber("/scenario", String, self.scenario_callback)
        rospy.Subscriber("/navigability", Float32, self.navigability_callback)
        rospy.Subscriber("/move_base/status", GoalStatusArray, self.status_callback)

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
        elif data.data == "corrider":
            self.speed_up_level = 7
        elif data.data == "warehouse":
            self.speed_up_level = 10
        else :
            self.speed_up_level = -1


    def navigability_callback(self, data):
        self.navigability = data.data
        # print("Received navigability value: ", self.navigability)
        
    def status_callback(self, data):
        if data.status_list[-1].text == "Goal reached.":
            self.robot_move = False
        else:
            self.robot_move = True
    
    def update_optimaltime(self):
        simulation = ctrl.ControlSystemSimulation(self.optimaltime_controller)

        simulation.input['navigability'] = self.navigability
        simulation.input['speed_up_level'] = self.speed_up_level
        simulation.compute()
        
        command = self.command + pa_weight_optimaltime + "{:.3f}".format(simulation.output['weight_optimaltime'])
        process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        msg = Float32()
        msg.data = self.speed_up_level
        self.speed_up_level_pub.publish(msg)
        msg.data = simulation.output['weight_optimaltime']
        self.weight_optimaltime_pub.publish(msg)

        # print('navigability:', self.navigability, 'speed_up_level:', self.speed_up_level, "Weight Optimal Time:", simulation.output['weight_optimaltime'])
        # print(command)
        # fz_navigability.view()
        # fz_speed_up_level.view()
        # fz_weight_optimaltime.view()
        # plt.show()

if __name__ == '__main__':
    try:
        FyzzyController()
    except rospy.ROSInterruptException:
        pass