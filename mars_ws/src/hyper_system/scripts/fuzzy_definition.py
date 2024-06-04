#!/usr/bin/env python3

# Brief: ...
# Author: Po Lin Jiang

import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import matplotlib.pyplot as plt

class FyzzyDefinition:
    def __init__(self):
        # define fuzzy input
        self.fz_navigability = ctrl.Antecedent(np.arange(0, 1.01, 0.01), 'navigability')
        self.fz_speed_up_level = ctrl.Antecedent(np.arange(0, 11, 0.1), 'speed_up_level')
        self.fz_pspace_level = ctrl.Antecedent(np.arange(0, 11, 0.1), 'pspace_level')                                        
        self.fz_right_side_level = ctrl.Antecedent(np.arange(0, 11, 0.1), 'right_side_level')                             
        self.fz_robot_invisible_level = ctrl.Antecedent(np.arange(0, 11, 0.1), 'robot_invisible_level')                   


        # define fuzzy output
        self.fz_weight_optimaltime = ctrl.Consequent(np.arange(5, 31, 0.1), 'weight_optimaltime')
        self.fz_weight_hr_safety = ctrl.Consequent(np.arange(5, 31, 0.1), 'weight_hr_safety')                  #### range not yet 
        self.fz_weight_cc = ctrl.Consequent(np.arange(0, 21, 0.1), 'weight_cc')                               
        self.fz_pspace_cov = ctrl.Consequent(np.arange(5, 31, 0.1), 'pspace_cov')                              #### range not yet 
        self.fz_pspace_r_ratio = ctrl.Consequent(np.arange(5, 31, 0.1), 'pspace_r_ratio')                      #### range not yet 
        self.fz_use_external_prediction = ctrl.Consequent(np.arange(5, 31, 0.1), 'use_external_prediction')    #### range not yet 


        # define membership function
        ## input
        self.fz_navigability['VL'] = fuzz.gaussmf(self.fz_navigability.universe, 0, 0.2)
        self.fz_navigability['L'] = fuzz.gaussmf(self.fz_navigability.universe, 0.25, 0.2)
        self.fz_navigability['M'] = fuzz.gaussmf(self.fz_navigability.universe, 0.5, 0.2)
        self.fz_navigability['H'] = fuzz.gaussmf(self.fz_navigability.universe, 0.75, 0.2)
        self.fz_navigability['VH'] = fuzz.gaussmf(self.fz_navigability.universe, 1, 0.2)

        self.fz_speed_up_level['L'] = fuzz.gaussmf(self.fz_speed_up_level.universe, 0, 2)
        self.fz_speed_up_level['M'] = fuzz.gaussmf(self.fz_speed_up_level.universe, 5, 2)
        self.fz_speed_up_level['H'] = fuzz.gaussmf(self.fz_speed_up_level.universe, 10, 2)

        self.fz_pspace_level['L'] = fuzz.gaussmf(self.fz_pspace_level.universe, 0, 2)
        self.fz_pspace_level['M'] = fuzz.gaussmf(self.fz_pspace_level.universe, 5, 2)
        self.fz_pspace_level['H'] = fuzz.gaussmf(self.fz_pspace_level.universe, 10, 2)

        self.fz_right_side_level['L'] = fuzz.gaussmf(self.fz_right_side_level.universe, 0, 2)
        self.fz_right_side_level['M'] = fuzz.gaussmf(self.fz_right_side_level.universe, 5, 2)
        self.fz_right_side_level['H'] = fuzz.gaussmf(self.fz_right_side_level.universe, 10, 2)

        self.fz_robot_invisible_level['L'] = fuzz.gaussmf(self.fz_robot_invisible_level.universe, 0, 2)
        self.fz_robot_invisible_level['M'] = fuzz.gaussmf(self.fz_robot_invisible_level.universe, 5, 2)
        self.fz_robot_invisible_level['H'] = fuzz.gaussmf(self.fz_robot_invisible_level.universe, 10, 2)


        # output
        self.fz_weight_optimaltime['VL'] = fuzz.gaussmf(self.fz_weight_optimaltime.universe, 5, 3)
        self.fz_weight_optimaltime['L'] = fuzz.gaussmf(self.fz_weight_optimaltime.universe, 10, 3)
        self.fz_weight_optimaltime['M'] = fuzz.gaussmf(self.fz_weight_optimaltime.universe, 15, 3)
        self.fz_weight_optimaltime['H'] = fuzz.gaussmf(self.fz_weight_optimaltime.universe, 20, 3)
        self.fz_weight_optimaltime['VH'] = fuzz.gaussmf(self.fz_weight_optimaltime.universe, 25, 3)

        self.fz_weight_cc['VL'] = fuzz.gaussmf(self.fz_weight_cc.universe, 0, 2)
        self.fz_weight_cc['L'] = fuzz.gaussmf(self.fz_weight_cc.universe, 5, 2)
        self.fz_weight_cc['M'] = fuzz.gaussmf(self.fz_weight_cc.universe, 10, 2)
        self.fz_weight_cc['H'] = fuzz.gaussmf(self.fz_weight_cc.universe, 15, 2)
        self.fz_weight_cc['VH'] = fuzz.gaussmf(self.fz_weight_cc.universe, 20, 2)


        # define rules
        ## fz_navigability, fz_speed_up_level | fz_weight_optimaltime
        self.rule1 = ctrl.Rule(self.fz_navigability['VL'] & self.fz_speed_up_level['L'], self.fz_weight_optimaltime['VL'])
        self.rule2 = ctrl.Rule(self.fz_navigability['VL'] & self.fz_speed_up_level['M'], self.fz_weight_optimaltime['L'])
        self.rule3 = ctrl.Rule(self.fz_navigability['VL'] & self.fz_speed_up_level['H'], self.fz_weight_optimaltime['L'])
        self.rule4 = ctrl.Rule(self.fz_navigability['L'] & self.fz_speed_up_level['L'], self.fz_weight_optimaltime['L'])
        self.rule5 = ctrl.Rule(self.fz_navigability['L'] & self.fz_speed_up_level['M'], self.fz_weight_optimaltime['M'])
        self.rule6 = ctrl.Rule(self.fz_navigability['L'] & self.fz_speed_up_level['H'], self.fz_weight_optimaltime['M'])
        self.rule7 = ctrl.Rule(self.fz_navigability['M'] & self.fz_speed_up_level['L'], self.fz_weight_optimaltime['L'])
        self.rule8 = ctrl.Rule(self.fz_navigability['M'] & self.fz_speed_up_level['M'], self.fz_weight_optimaltime['M'])
        self.rule9 = ctrl.Rule(self.fz_navigability['M'] & self.fz_speed_up_level['H'], self.fz_weight_optimaltime['H'])
        self.rule10 = ctrl.Rule(self.fz_navigability['H'] & self.fz_speed_up_level['L'], self.fz_weight_optimaltime['M'])
        self.rule11 = ctrl.Rule(self.fz_navigability['H'] & self.fz_speed_up_level['M'], self.fz_weight_optimaltime['M'])
        self.rule12 = ctrl.Rule(self.fz_navigability['H'] & self.fz_speed_up_level['H'], self.fz_weight_optimaltime['H'])
        self.rule13 = ctrl.Rule(self.fz_navigability['VH'] & self.fz_speed_up_level['L'], self.fz_weight_optimaltime['H'])
        self.rule14 = ctrl.Rule(self.fz_navigability['VH'] & self.fz_speed_up_level['M'], self.fz_weight_optimaltime['H'])
        self.rule15 = ctrl.Rule(self.fz_navigability['VH'] & self.fz_speed_up_level['H'], self.fz_weight_optimaltime['VH']) 
        self.optimaltime_controller = ctrl.ControlSystem([self.rule1, self.rule2, self.rule3, self.rule4, self.rule5, self.rule6, self.rule7, self.rule8,
                                  self.rule9, self.rule10, self.rule11, self.rule12, self.rule13, self.rule14, self.rule15])

        ## fz_navigability, fz_robot_invisible_level | fz_weight_cc



        # visulization
        # self.fz_navigability.view()
        # self.fz_speed_up_level.view()
        self.fz_weight_optimaltime.view()
        self.fz_weight_cc.view()
        plt.show()
