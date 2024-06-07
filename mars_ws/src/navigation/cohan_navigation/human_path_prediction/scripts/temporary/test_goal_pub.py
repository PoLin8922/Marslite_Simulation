#!/usr/bin/env python3

import rospy
from human_path_prediction.msg import PredictedGoal, PredictedGoals

def predicted_goals_callback(data):
    rospy.loginfo("Received predicted goals: %s", data.goals[1])

def predicted_goals_subscriber():
    rospy.init_node('predicted_goals_subscriber', anonymous=True)

    # 创建一个订阅者，订阅名为 "predicted_goals" 的话题，消息类型为 PredictedGoals
    rospy.Subscriber("/human_goals_publisher/predicted_goals", PredictedGoals, predicted_goals_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        predicted_goals_subscriber()
    except rospy.ROSInterruptException:
        pass
