#!/usr/bin/python
"""
Copyright (c) Deep Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Haoyi Han <hanhaoyi@deeprobotics.cn>, Feb, 2020
"""

import os   
import re
import json
import rospy
from TaskPoint import TaskPoint, globalTaskPrepare
from TaskTransfer import TaskTransfer
from geometry_msgs.msg import PoseStamped
import numpy as np
import tf
from RobotCommander import RobotCommander
import threading
from pipeline.srv import Detection_srv

class Task:
    def __init__(self):
        self.taskPoints = []
        self.currentIndex = 0
        self.robot_transfer = None
        self.src_index = None
        self.des_index = None
        self.ntask = 0
        self.tf_listener = tf.TransformListener()
        self.send_tf_thread = threading.Thread(target=self.send_tf, name="send_tf")
        self.send_tf_thread.setDaemon(True)
        self.send_tf_thread.start()

    def send_tf(self):
        """
        Daemon Thread
        """
        while not rospy.is_shutdown():
            current_tf = self.listen_tf()
            if current_tf:
                with RobotCommander(local_port=20003) as robot_commander:
                    robot_commander.sendCordinate(
                        command_code=52,
                        x=current_tf[0],
                        y=current_tf[1],
                        yaw=current_tf[2],
                    )

                    robot_commander.sendCordinate(command_code=51, x= 3.769226,y= 1.286894,yaw= 1.555460,)
            rospy.sleep(0.05)

    def listen_tf(self):
        try:
            (pos, ori) = self.tf_listener.lookupTransform(
                "/map", "/base_link", rospy.Duration(0.0)
            )
            
            yaw = tf.transformations.euler_from_quaternion(ori)[2]
            msg_list = [pos[0], pos[1], yaw]
            return msg_list
        except tf.Exception as e:
            print "listen to tf failed"
            print e
            print e.message
            return None

    def init(self):
        # stand up, read to go
        globalTaskPrepare()
        # from task_point1 to task_point2
        self.robot_transfer = TaskTransfer()
        
        self.loadTaskpoints()
        # only using TaskInit() to get nearest task_point
        
        task_init = TaskInit()
        
        nearest_index, initial_point = task_init.getBestTaskInd(self.taskPoints)
        self.robot_transfer.task_transfer(initial_point, self.taskPoints[nearest_index])
        # total number of the task_points
        self.ntask = self.taskPoints.__len__()
        
        self.src_index = nearest_index
        self.des_index = (nearest_index + 1) % self.ntask

    def loadTaskpoints(self):
        folder = str(os.path.dirname(os.path.abspath(__file__))) + "/../data"
        task_json = None
        if os.path.exists(folder):
            task_json = os.listdir(folder) 

        if not task_json:
            raise Exception("No valid task point to tranverse!")

        task_list = []
        for i, file_name in enumerate(task_json):  
            with open(folder + "/" + file_name, "r") as json_fp:
                waypoint_record = json.load(json_fp)
                task_list.append(waypoint_record)
        task_list = sorted(task_list, key=lambda s: s["order"])
        for waypoint_record in task_list:
            self.taskPoints.append(TaskPoint(waypoint_record))

    def run(self):
        while not rospy.is_shutdown():
            
            self.robot_transfer.task_transfer(
                self.taskPoints[self.src_index], self.taskPoints[self.des_index]
            )
            if self.taskPoints[self.des_index].is_autocharge():
                break
            ################
            rospy.sleep(1.0) 
            with RobotCommander() as robot_commander:
                   robot_commander.motion_start_stop()
            if self.taskPoints[self.des_index].name == "waypoint_15" or self.taskPoints[self.des_index].name == "waypoint_7" or self.taskPoints[self.des_index].name == "waypoint_12":
              self.find_test()
            rospy.sleep(3.0)
            with RobotCommander() as robot_commander:
                   robot_commander.motion_start_stop()
            self.src_index = self.des_index
            self.des_index = (self.des_index + 1) % self.ntask

    def find_test(self):
        rospy.sleep(0.5)
        rospy.wait_for_service('detection')
        try:
            val = rospy.ServiceProxy('detection',  Detection_srv)
            resp1 = val(True)
            print resp1.detction_complete, resp1.message
        except rospy.ServiceException, e:
            print e
        rospy.sleep(1.0)
        

        



class TaskInit:
    def __init__(self):
        self.initialPose = None
        self.tf_listener = tf.TransformListener()

    def listen_tf(self):
        try:
            (pos, ori) = self.tf_listener.lookupTransform(
                "/map", "/base_link", rospy.Duration(0.0)
            )
            print "pos: ", pos
            print "ori: ", ori
            msg_list = [pos[0], pos[1], pos[2], ori[0], ori[1], ori[2], ori[3]]
            self.initialPose = msg_list
            return True
        except tf.Exception as e:
            print "listen to tf failed"
            print e
            print e.message
            return False

    def refreshInitialPose(self):
        self.initialPose = None
        RATE = 50
        while not self.initialPose:
            self.listen_tf()
            rospy.sleep(1.0 / RATE)

    def getBestTaskInd(self, task_points):
        self.refreshInitialPose()
        fake_task = TaskPoint()
        fake_task.setRobotPose(self.initialPose)
        dist_list = [fake_task.calDistance(task_point) for task_point in task_points]
        return np.argmin(np.array(dist_list)), fake_task


if __name__ == "__main__":
    rospy.init_node("autonomous_2d_navigation")
    task = Task()
    
    task.init()
    rospy.sleep(0.5)
    
    task.run()
