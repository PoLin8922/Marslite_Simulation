#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
import pandas as pd
import matplotlib.pyplot as plt
import os
from collections import defaultdict

# 初始化空的字典來儲存數據
data = defaultdict(list)
temp_data = {
    "speed_up_level": None,
    "robot_invisiable_level": None,
    "right_side_level": None,
    "pspace_level": None,
    "weight_optimaltime": None,
    "weight_cc": None,
    "pspace_cov": None,
    "pspace_r_ratio": None,
    "use_external_prediction": None
}

def update_data():
    # 将当前temp_data中的数据保存到data中
    for key in temp_data:
        data[key].append(temp_data[key])
        temp_data[key] = None  # 重置临时数据

    # 定时调用自身
    rospy.Timer(rospy.Duration(1), update_data, oneshot=True)

def speed_up_level_callback(msg):
    temp_data["speed_up_level"] = msg.data

def robot_invisiable_level_callback(msg):
    temp_data["robot_invisiable_level"] = msg.data

def right_side_level_callback(msg):
    temp_data["right_side_level"] = msg.data

def pspace_level_callback(msg):
    temp_data["pspace_level"] = msg.data

def weight_optimaltime_callback(msg):
    temp_data["weight_optimaltime"] = msg.data

def weight_cc_callback(msg):
    temp_data["weight_cc"] = msg.data

def pspace_cov_callback(msg):
    temp_data["pspace_cov"] = msg.data

def pspace_r_ratio_callback(msg):
    temp_data["pspace_r_ratio"] = msg.data

def use_external_prediction_callback(msg):
    temp_data["use_external_prediction"] = msg.data

def main():
    rospy.init_node('data_collector', anonymous=True)
    
    rospy.Subscriber("/speed_up_level", Float32, speed_up_level_callback)
    rospy.Subscriber("/robot_invisiable_level", Float32, robot_invisiable_level_callback)
    rospy.Subscriber("/right_side_level", Float32, right_side_level_callback)
    rospy.Subscriber("/pspace_level", Float32, pspace_level_callback)
    rospy.Subscriber("/weight_optimaltime", Float32, weight_optimaltime_callback)
    rospy.Subscriber("/weight_cc", Float32, weight_cc_callback)
    rospy.Subscriber("/pspace_cov", Float32, pspace_cov_callback)
    rospy.Subscriber("/pspace_r_ratio", Float32, pspace_r_ratio_callback)
    rospy.Subscriber("/use_external_prediction", Float32, use_external_prediction_callback)
    
    # 启动定时器来更新数据
    update_data()
    
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        save_data()

def save_data():
    df = pd.DataFrame(data)
    
    # 检查文件是否存在
    if os.path.isfile('collected_data.csv'):
        # 文件存在，附加数据
        df.to_csv('collected_data.csv', mode='a', header=False, index=False)
    else:
        # 文件不存在，创建新文件并写入数据
        df.to_csv('collected_data.csv', mode='w', index=False)
        
    plot_data(df)

def plot_data(df):
    df.plot(subplots=True, layout=(3, 3), figsize=(15, 10), title='Collected ROS Data')
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()
