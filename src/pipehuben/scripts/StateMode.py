#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Copyright (c) Deep Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Haoyi Han <hanhaoyi@deeprobotics.cn>, Feb, 2020
"""


import rospy
import threading


class StateMode:
    """
    State_mode is resposible for command the robot to stop, tread or play.
    """

    # _mode_code = {
    #     "STAND": 12,
    #     "DOWN": 14,
    #     "WALK": 11,
    #     "DANCE": 50,
    #     "HEART_BEAT": 50,
    # }

    
    def __init__(self):
        self.current_mode=12
        

        
    def get_mode(self):
        return self.current_mode
    
    
            

    def sendSimple(self, command_code=12):
        self.current_mode=command_code

    def stand_mode(self):
        print("stand1")
        self.sendSimple(12)

    def down_mode(self):
        print("down1")
        self.sendSimple(14)

    def walk_mode(self):
        print("walk1")
        self.sendSimple(11)

    def up_stair_mode(self):
        
        self.sendSimple(11)
        

  
    

def state_node_Pub():
# ROS节点初始化
    rospy.init_node('state_mode', anonymous=True)
    # 创建一个Publisher，发布名为/state_mode的topic，消息类型为int，队列长度10
    State_mode_info_pub = rospy.Publisher('/state_mode',int, queue_size=1)
    #设置循环的频率
    rate = rospy.Rate(10)
    #实体化类
    State_mode_ = StateMode()
    while not rospy.is_shutdown():
    # 发布消息
        State_mode_info_pub.publish(State_mode_.current_mode)
        rate.sleep()# 按照循环频率延时
        rospy.loginfo("Publish laneline message[%d, %d]", State_mode_.current_mode)
    

