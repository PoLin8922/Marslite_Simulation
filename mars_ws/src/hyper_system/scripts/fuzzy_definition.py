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
        self.fz_robot_invisible_level = ctrl.Antecedent(np.arange(0, 11, 0.1), 'robot_invisiable_level')                   


        # define fuzzy output
        # self.fz_weight_optimaltime = ctrl.Consequent(np.arange(5, 31, 0.1), 'weight_optimaltime')
        self.fz_weight_optimaltime = ctrl.Consequent(np.arange(0, 21, 0.1), 'weight_optimaltime')
        self.fz_weight_cc = ctrl.Consequent(np.arange(0, 21, 0.1), 'weight_cc')                               
        self.fz_pspace_cov = ctrl.Consequent(np.arange(0.4, 1.21, 0.005), 'pspace_cov')                             
        self.fz_pspace_r_ratio = ctrl.Consequent(np.arange(0.6, 1.01, 0.005), 'pspace_r_ratio')                     
        self.fz_use_external_prediction = ctrl.Consequent(np.arange(0, 1.01, 0.01), 'use_external_prediction')    #### range not yet 
        # self.fz_inflation_rate_global = ctrl.Consequent(np.arange(0, 1.51, 0.015), 'use_external_prediction')
        # self.fz_inflation_rate_local = ctrl.Consequent(np.arange(0, 1.01, 0.01), 'use_external_prediction')


        # define membership function
        ## input
        self.fz_navigability['VL'] = fuzz.gaussmf(self.fz_navigability.universe, 0, 0.1)
        self.fz_navigability['L'] = fuzz.gaussmf(self.fz_navigability.universe, 0.25, 0.1)
        self.fz_navigability['M'] = fuzz.gaussmf(self.fz_navigability.universe, 0.5, 0.1)
        self.fz_navigability['H'] = fuzz.gaussmf(self.fz_navigability.universe, 0.75, 0.1)
        self.fz_navigability['VH'] = fuzz.gaussmf(self.fz_navigability.universe, 1, 0.1)

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
        self.fz_weight_optimaltime['VL'] = fuzz.gaussmf(self.fz_weight_optimaltime.universe, 0, 2)
        self.fz_weight_optimaltime['L'] = fuzz.gaussmf(self.fz_weight_optimaltime.universe, 4, 2)
        self.fz_weight_optimaltime['M'] = fuzz.gaussmf(self.fz_weight_optimaltime.universe, 9, 2)
        self.fz_weight_optimaltime['H'] = fuzz.gaussmf(self.fz_weight_optimaltime.universe, 17, 2)
        self.fz_weight_optimaltime['VH'] = fuzz.gaussmf(self.fz_weight_optimaltime.universe, 20, 2)

        self.fz_weight_cc['VL'] = fuzz.gaussmf(self.fz_weight_cc.universe, 0, 2)
        self.fz_weight_cc['L'] = fuzz.gaussmf(self.fz_weight_cc.universe, 3.0, 2)
        self.fz_weight_cc['M'] = fuzz.gaussmf(self.fz_weight_cc.universe, 9, 2)
        self.fz_weight_cc['H'] = fuzz.gaussmf(self.fz_weight_cc.universe, 15, 2)
        self.fz_weight_cc['VH'] = fuzz.gaussmf(self.fz_weight_cc.universe, 20, 2)

        self.fz_pspace_cov['VL'] = fuzz.gaussmf(self.fz_pspace_cov.universe, 0.4, 0.1)
        self.fz_pspace_cov['L'] = fuzz.gaussmf(self.fz_pspace_cov.universe, 0.55, 0.1)
        self.fz_pspace_cov['M'] = fuzz.gaussmf(self.fz_pspace_cov.universe, 0.8, 0.1)
        self.fz_pspace_cov['H'] = fuzz.gaussmf(self.fz_pspace_cov.universe, 1.05, 0.1)
        self.fz_pspace_cov['VH'] = fuzz.gaussmf(self.fz_pspace_cov.universe, 1.2, 0.1)

        self.fz_pspace_r_ratio['VL'] = fuzz.gaussmf(self.fz_pspace_r_ratio.universe, 0.6, 0.05)
        self.fz_pspace_r_ratio['L'] = fuzz.gaussmf(self.fz_pspace_r_ratio.universe, 0.7, 0.05)
        self.fz_pspace_r_ratio['M'] = fuzz.gaussmf(self.fz_pspace_r_ratio.universe, 0.8, 0.05)
        self.fz_pspace_r_ratio['H'] = fuzz.gaussmf(self.fz_pspace_r_ratio.universe, 0.9, 0.05)
        self.fz_pspace_r_ratio['VH'] = fuzz.gaussmf(self.fz_pspace_r_ratio.universe, 1.0, 0.05)

        self.fz_use_external_prediction['S'] = fuzz.trimf(self.fz_use_external_prediction.universe, [0, 0.4, 0.8])
        self.fz_use_external_prediction['L'] = fuzz.trimf(self.fz_use_external_prediction.universe, [0.6, 0.8, 1.0])

        # self.fz_inflation_rate_global['VL'] = fuzz.gaussmf(self.fz_inflation_rate_global.universe, 0.0, 0.15)
        # self.fz_inflation_rate_global['L'] = fuzz.gaussmf(self.fz_inflation_rate_global.universe, 0.38, 0.15)
        # self.fz_inflation_rate_global['M'] = fuzz.gaussmf(self.fz_inflation_rate_global.universe, 0.75, 0.15)
        # self.fz_inflation_rate_global['H'] = fuzz.gaussmf(self.fz_inflation_rate_global.universe, 1.22, 0.15)
        # self.fz_inflation_rate_global['VH'] = fuzz.gaussmf(self.fz_inflation_rate_global.universe, 1.5, 0.15)


        # define rules
        ## fz_navigability, fz_speed_up_level | fz_weight_optimaltime
        self.rule1 = ctrl.Rule(self.fz_navigability['VL'] & self.fz_speed_up_level['L'], self.fz_weight_optimaltime['VL'])
        # self.rule2 = ctrl.Rule(self.fz_navigability['VL'] & self.fz_speed_up_level['M'], self.fz_weight_optimaltime['L'])
        self.rule2 = ctrl.Rule(self.fz_navigability['VL'] & self.fz_speed_up_level['M'], self.fz_weight_optimaltime['VL'])
        # self.rule3 = ctrl.Rule(self.fz_navigability['VL'] & self.fz_speed_up_level['H'], self.fz_weight_optimaltime['M'])
        self.rule3 = ctrl.Rule(self.fz_navigability['VL'] & self.fz_speed_up_level['H'], self.fz_weight_optimaltime['L'])
        # self.rule4 = ctrl.Rule(self.fz_navigability['L'] & self.fz_speed_up_level['L'], self.fz_weight_optimaltime['L'])
        self.rule4 = ctrl.Rule(self.fz_navigability['L'] & self.fz_speed_up_level['L'], self.fz_weight_optimaltime['VL'])
        self.rule5 = ctrl.Rule(self.fz_navigability['L'] & self.fz_speed_up_level['M'], self.fz_weight_optimaltime['L'])
        self.rule6 = ctrl.Rule(self.fz_navigability['L'] & self.fz_speed_up_level['H'], self.fz_weight_optimaltime['H'])
        self.rule7 = ctrl.Rule(self.fz_navigability['M'] & self.fz_speed_up_level['L'], self.fz_weight_optimaltime['L'])
        self.rule8 = ctrl.Rule(self.fz_navigability['M'] & self.fz_speed_up_level['M'], self.fz_weight_optimaltime['M'])
        # self.rule8 = ctrl.Rule(self.fz_navigability['M'] & self.fz_speed_up_level['M'], self.fz_weight_optimaltime['L'])
        # self.rule9 = ctrl.Rule(self.fz_navigability['M'] & self.fz_speed_up_level['H'], self.fz_weight_optimaltime['VH'])
        self.rule9 = ctrl.Rule(self.fz_navigability['M'] & self.fz_speed_up_level['H'], self.fz_weight_optimaltime['H'])
        # self.rule10 = ctrl.Rule(self.fz_navigability['H'] & self.fz_speed_up_level['L'], self.fz_weight_optimaltime['M'])
        self.rule10 = ctrl.Rule(self.fz_navigability['H'] & self.fz_speed_up_level['L'], self.fz_weight_optimaltime['L'])
        # self.rule11 = ctrl.Rule(self.fz_navigability['H'] & self.fz_speed_up_level['M'], self.fz_weight_optimaltime['M'])
        self.rule11 = ctrl.Rule(self.fz_navigability['H'] & self.fz_speed_up_level['M'], self.fz_weight_optimaltime['H'])
        self.rule12 = ctrl.Rule(self.fz_navigability['H'] & self.fz_speed_up_level['H'], self.fz_weight_optimaltime['VH'])
        # self.rule13 = ctrl.Rule(self.fz_navigability['VH'] & self.fz_speed_up_level['L'], self.fz_weight_optimaltime['H'])
        self.rule13 = ctrl.Rule(self.fz_navigability['VH'] & self.fz_speed_up_level['L'], self.fz_weight_optimaltime['M'])
        self.rule14 = ctrl.Rule(self.fz_navigability['VH'] & self.fz_speed_up_level['M'], self.fz_weight_optimaltime['H'])
        self.rule15 = ctrl.Rule(self.fz_navigability['VH'] & self.fz_speed_up_level['H'], self.fz_weight_optimaltime['VH']) 
        self.optimaltime_controller = ctrl.ControlSystem([self.rule1, self.rule2, self.rule3, self.rule4, self.rule5, self.rule6, self.rule7, self.rule8,
                                  self.rule9, self.rule10, self.rule11, self.rule12, self.rule13, self.rule14, self.rule15])

        ## fz_navigability, fz_robot_invisible_level | fz_weight_cc
        self.rule16 = ctrl.Rule(self.fz_navigability['VL'] & self.fz_robot_invisible_level['L'], self.fz_weight_cc['VL'])
        self.rule17 = ctrl.Rule(self.fz_navigability['VL'] & self.fz_robot_invisible_level['M'], self.fz_weight_cc['L'])
        self.rule18 = ctrl.Rule(self.fz_navigability['VL'] & self.fz_robot_invisible_level['H'], self.fz_weight_cc['L'])
        self.rule19 = ctrl.Rule(self.fz_navigability['L'] & self.fz_robot_invisible_level['L'], self.fz_weight_cc['VL'])
        # self.rule20 = ctrl.Rule(self.fz_navigability['L'] & self.fz_robot_invisible_level['M'], self.fz_weight_cc['M'])
        self.rule20 = ctrl.Rule(self.fz_navigability['L'] & self.fz_robot_invisible_level['M'], self.fz_weight_cc['L'])
        self.rule21 = ctrl.Rule(self.fz_navigability['L'] & self.fz_robot_invisible_level['H'], self.fz_weight_cc['M'])
        self.rule22 = ctrl.Rule(self.fz_navigability['M'] & self.fz_robot_invisible_level['L'], self.fz_weight_cc['VL'])
        self.rule23 = ctrl.Rule(self.fz_navigability['M'] & self.fz_robot_invisible_level['M'], self.fz_weight_cc['M'])
        self.rule24 = ctrl.Rule(self.fz_navigability['M'] & self.fz_robot_invisible_level['H'], self.fz_weight_cc['H'])
        self.rule25 = ctrl.Rule(self.fz_navigability['H'] & self.fz_robot_invisible_level['L'], self.fz_weight_cc['L'])
        self.rule26 = ctrl.Rule(self.fz_navigability['H'] & self.fz_robot_invisible_level['M'], self.fz_weight_cc['M'])
        self.rule27 = ctrl.Rule(self.fz_navigability['H'] & self.fz_robot_invisible_level['H'], self.fz_weight_cc['VH'])
        self.rule28 = ctrl.Rule(self.fz_navigability['VH'] & self.fz_robot_invisible_level['L'], self.fz_weight_cc['L'])
        self.rule29 = ctrl.Rule(self.fz_navigability['VH'] & self.fz_robot_invisible_level['M'], self.fz_weight_cc['H'])
        self.rule30 = ctrl.Rule(self.fz_navigability['VH'] & self.fz_robot_invisible_level['H'], self.fz_weight_cc['VH'])
        self.critical_corner_controller = ctrl.ControlSystem([self.rule16, self.rule17, self.rule18, self.rule19, self.rule20, self.rule21, self.rule22, self.rule23,
                                                        self.rule24, self.rule25, self.rule26, self.rule27, self.rule28, self.rule29, self.rule30])

        ## fz_navigability, fz_pspace_level | fz_pspace_cov
        self.rule31 = ctrl.Rule(self.fz_navigability['VL'] & self.fz_pspace_level['L'], self.fz_pspace_cov['VL'])
        self.rule32 = ctrl.Rule(self.fz_navigability['VL'] & self.fz_pspace_level['M'], self.fz_pspace_cov['VL'])
        self.rule33 = ctrl.Rule(self.fz_navigability['VL'] & self.fz_pspace_level['H'], self.fz_pspace_cov['VL'])
        self.rule34 = ctrl.Rule(self.fz_navigability['L'] & self.fz_pspace_level['L'], self.fz_pspace_cov['VL'])
        self.rule35 = ctrl.Rule(self.fz_navigability['L'] & self.fz_pspace_level['M'], self.fz_pspace_cov['L'])
        self.rule36 = ctrl.Rule(self.fz_navigability['L'] & self.fz_pspace_level['H'], self.fz_pspace_cov['L'])
        self.rule37 = ctrl.Rule(self.fz_navigability['M'] & self.fz_pspace_level['L'], self.fz_pspace_cov['VL'])
        self.rule38 = ctrl.Rule(self.fz_navigability['M'] & self.fz_pspace_level['M'], self.fz_pspace_cov['L'])
        self.rule39 = ctrl.Rule(self.fz_navigability['M'] & self.fz_pspace_level['H'], self.fz_pspace_cov['M'])
        self.rule40 = ctrl.Rule(self.fz_navigability['H'] & self.fz_pspace_level['L'], self.fz_pspace_cov['L'])
        self.rule41 = ctrl.Rule(self.fz_navigability['H'] & self.fz_pspace_level['M'], self.fz_pspace_cov['M'])
        self.rule42 = ctrl.Rule(self.fz_navigability['H'] & self.fz_pspace_level['H'], self.fz_pspace_cov['H'])
        self.rule43 = ctrl.Rule(self.fz_navigability['VH'] & self.fz_pspace_level['L'], self.fz_pspace_cov['L'])
        self.rule44 = ctrl.Rule(self.fz_navigability['VH'] & self.fz_pspace_level['M'], self.fz_pspace_cov['H'])
        self.rule45 = ctrl.Rule(self.fz_navigability['VH'] & self.fz_pspace_level['H'], self.fz_pspace_cov['VH'])
        self.pspace_cov_controller = ctrl.ControlSystem([self.rule31, self.rule32, self.rule33, self.rule34, self.rule35, self.rule36, self.rule37, self.rule38,
                                                            self.rule39, self.rule40, self.rule41, self.rule42, self.rule43, self.rule44, self.rule45])

        ## fz_navigability, fz_right_side_level | fz_pspace_r_ratio
        self.rule46 = ctrl.Rule(self.fz_navigability['VL'] & self.fz_right_side_level['L'], self.fz_pspace_r_ratio['M'])
        self.rule47 = ctrl.Rule(self.fz_navigability['VL'] & self.fz_right_side_level['M'], self.fz_pspace_r_ratio['L'])
        self.rule48 = ctrl.Rule(self.fz_navigability['VL'] & self.fz_right_side_level['H'], self.fz_pspace_r_ratio['VL'])
        self.rule49 = ctrl.Rule(self.fz_navigability['L'] & self.fz_right_side_level['L'], self.fz_pspace_r_ratio['H'])
        self.rule50 = ctrl.Rule(self.fz_navigability['L'] & self.fz_right_side_level['M'], self.fz_pspace_r_ratio['M'])
        self.rule51 = ctrl.Rule(self.fz_navigability['L'] & self.fz_right_side_level['H'], self.fz_pspace_r_ratio['L'])
        self.rule52 = ctrl.Rule(self.fz_navigability['M'] & self.fz_right_side_level['L'], self.fz_pspace_r_ratio['H'])
        self.rule53 = ctrl.Rule(self.fz_navigability['M'] & self.fz_right_side_level['M'], self.fz_pspace_r_ratio['M'])
        self.rule54 = ctrl.Rule(self.fz_navigability['M'] & self.fz_right_side_level['H'], self.fz_pspace_r_ratio['L'])
        self.rule55 = ctrl.Rule(self.fz_navigability['H'] & self.fz_right_side_level['L'], self.fz_pspace_r_ratio['VH'])
        self.rule56 = ctrl.Rule(self.fz_navigability['H'] & self.fz_right_side_level['M'], self.fz_pspace_r_ratio['H'])
        self.rule57 = ctrl.Rule(self.fz_navigability['H'] & self.fz_right_side_level['H'], self.fz_pspace_r_ratio['M'])
        self.rule58 = ctrl.Rule(self.fz_navigability['VH'] & self.fz_right_side_level['L'], self.fz_pspace_r_ratio['VH'])
        self.rule59 = ctrl.Rule(self.fz_navigability['VH'] & self.fz_right_side_level['M'], self.fz_pspace_r_ratio['H'])
        self.rule60 = ctrl.Rule(self.fz_navigability['VH'] & self.fz_right_side_level['H'], self.fz_pspace_r_ratio['M'])
        self.pspace_r_ratio_controller = ctrl.ControlSystem([self.rule46, self.rule47, self.rule48, self.rule49, self.rule50, self.rule51, self.rule52, self.rule53,
                                                            self.rule54, self.rule55, self.rule56, self.rule57, self.rule58, self.rule59, self.rule60])

        ## fz_navigability | fz_use_external_prediction
        self.rule61 = ctrl.Rule(self.fz_navigability['VL'], self.fz_use_external_prediction['S'])
        self.rule62 = ctrl.Rule(self.fz_navigability['L'], self.fz_use_external_prediction['S'])
        self.rule63 = ctrl.Rule(self.fz_navigability['M'], self.fz_use_external_prediction['S'])
        self.rule64 = ctrl.Rule(self.fz_navigability['H'], self.fz_use_external_prediction['L'])
        self.rule65 = ctrl.Rule(self.fz_navigability['VH'], self.fz_use_external_prediction['L'])
        self.human_path_prediction_controller = ctrl.ControlSystem([self.rule61, self.rule62, self.rule63, self.rule64, self.rule65])

        # ## fz_navigability, fz_right_side_level | fz_pspace_r_ratio
        # self.rule66 = ctrl.Rule(self.fz_navigability['VL'] & self.fz_right_side_level['L'], self.fz_pspace_r_ratio['M'])
        # self.rule67 = ctrl.Rule(self.fz_navigability['VL'] & self.fz_right_side_level['M'], self.fz_pspace_r_ratio['L'])
        # self.rule68 = ctrl.Rule(self.fz_navigability['VL'] & self.fz_right_side_level['H'], self.fz_pspace_r_ratio['VL'])
        # self.rule69 = ctrl.Rule(self.fz_navigability['L'] & self.fz_right_side_level['L'], self.fz_pspace_r_ratio['H'])
        # self.rule60 = ctrl.Rule(self.fz_navigability['L'] & self.fz_right_side_level['M'], self.fz_pspace_r_ratio['M'])
        # self.rule61 = ctrl.Rule(self.fz_navigability['L'] & self.fz_right_side_level['H'], self.fz_pspace_r_ratio['L'])
        # self.rule62 = ctrl.Rule(self.fz_navigability['M'] & self.fz_right_side_level['L'], self.fz_pspace_r_ratio['H'])
        # self.rule63 = ctrl.Rule(self.fz_navigability['M'] & self.fz_right_side_level['M'], self.fz_pspace_r_ratio['M'])
        # self.rule64 = ctrl.Rule(self.fz_navigability['M'] & self.fz_right_side_level['H'], self.fz_pspace_r_ratio['L'])
        # self.rule65 = ctrl.Rule(self.fz_navigability['H'] & self.fz_right_side_level['L'], self.fz_pspace_r_ratio['VH'])
        # self.rule66 = ctrl.Rule(self.fz_navigability['H'] & self.fz_right_side_level['M'], self.fz_pspace_r_ratio['H'])
        # self.rule67 = ctrl.Rule(self.fz_navigability['H'] & self.fz_right_side_level['H'], self.fz_pspace_r_ratio['M'])
        # self.rule68 = ctrl.Rule(self.fz_navigability['VH'] & self.fz_right_side_level['L'], self.fz_pspace_r_ratio['VH'])
        # self.rule69 = ctrl.Rule(self.fz_navigability['VH'] & self.fz_right_side_level['M'], self.fz_pspace_r_ratio['H'])
        # self.rule70 = ctrl.Rule(self.fz_navigability['VH'] & self.fz_right_side_level['H'], self.fz_pspace_r_ratio['M'])
        # self.pspace_r_ratio_controller = ctrl.ControlSystem([self.rule46, self.rule47, self.rule48, self.rule49, self.rule50, self.rule51, self.rule52, self.rule53,
        #                                                     self.rule54, self.rule55, self.rule56, self.rule57, self.rule58, self.rule59, self.rule60])

        # visulization
        # self.fz_speed_up_level.view()
        # self.fz_speed_up_level.view()
        # self.fz_weight_optimaltime.view()
        # self.fz_weight_optimaltime.view()
        plt.show()
