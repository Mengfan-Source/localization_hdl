#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
Copyright (c) Deep Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Haoyi Han <hanhaoyi@deeprobotics.cn>, Feb, 2020
"""

import sys
sys.path.append("/home/cetc21/nav_ws/src")
import os
import re
import json
import rospy
from TaskPoint import TaskPoint
from TaskTransfer import TaskTransfer
from geometry_msgs.msg import PoseStamped
import numpy as np
import tf
from StateMode import StateMode
import threading
from std_msgs.msg import Int32
# from pipeline.srv import Detection_srv
###################################guo_song_hui_OCR
from collections import Counter
import numpy as np
import rospy
from pipehuben.srv import Order
from pipehuben.srv import OrderRequest
from pipehuben.srv import OrderResponse
###################################

import sys

sys.path.append("/home/cetc21/catkin_tunnel_ws/devel/lib/python3/dist-packages")
import rospy
import os.path
import threading
import time
import cv2
from pipehuben.srv import yt_data, yt_dataRequest, yt_dataResponse

image_thread = None  # 定义全局变量


# 创建一个线程类
class ImageThread(threading.Thread):
    def __init__(self):
        super(ImageThread, self).__init__()
        self.latest_image_hw = None  # 存储最新的图像
        self.latest_image_rgb = None  # 存储最新的图像
        self.lock = threading.Lock()  # 创建一个锁对象
        self.video = True

    def run(self):
        self.cap_rgb = cv2.VideoCapture("rtsp://admin:cetc2121@192.168.254.91:554/h264/ch1/main/av_stream")  # 打开相机
        self.cap_hw = cv2.VideoCapture("rtsp://admin:cetc2121@192.168.254.91:554/h264/ch33/main/av_stream")  # 打开相机
        frame_rgb_width = int(self.cap_rgb.get(3) / 4)  # 设置rgb保存分辨率
        frame_rgb_height = int(self.cap_rgb.get(4) / 4)
        frame_hw_width = int(self.cap_hw.get(3))  # 设置hw保存分辨率
        frame_hw_height = int(self.cap_hw.get(4))

        if self.video == True:
            ttime = time.strftime('%Y-%m-%d_%H:%M:%S', time.localtime(time.time()))
            rgb_video_name = ttime + "_rgb" + ".mp4"
            hw_video_name = ttime + "_hw" + ".mp4"
            video_save_path = "/home/cetc21/video"
            self.out_rgb = cv2.VideoWriter(os.path.join(video_save_path, rgb_video_name),
                                           cv2.VideoWriter_fourcc('m', 'p', '4', 'v'), 30,
                                           (frame_rgb_width, frame_rgb_height))  # 保存rgb视频
            #self.out_hw = cv2.VideoWriter(os.path.join(video_save_path, hw_video_name),
                                          # cv2.VideoWriter_fourcc('m', 'p', '4', 'v'), 30,
                                          # (frame_hw_width, frame_hw_height))  # 保存hw视频
        while True:
            ret_rgb, frame_rgb = self.cap_rgb.read()  # 读取相机图像
            ret_hw, frame_hw = self.cap_hw.read()  # 读取相机图像
            if not (ret_hw or ret_hw):
                # print("can not link camera")
                raise ZeroDivisionError("can not link camera")
                continue
            if self.video == True:
                frame_rgb_v = cv2.resize(frame_rgb, (frame_rgb_width, frame_rgb_height))
                #frame_hw = cv2.resize(frame_hw, (frame_hw_width, frame_hw_height))
                self.out_rgb.write(frame_rgb_v)  # 视频写入
                #self.out_hw.write(frame_hw)  # 视频写入

            with self.lock:  # 获取锁
                self.latest_image_rgb = frame_rgb  # 更新最新的图像
                self.latest_image_hw = frame_hw  # 更新最新的图像
            time.sleep(0.01)  # 线程等待一小段时间

    # --------------------------------------------------------------------------------------------------------------------
    def release(self):
        self.cap_rgb.release()  # 资源释放
        self.cap_hw.release()  # 资源释放
        if self.video == True:
            self.out_rgb.release()  # 资源释放
            #self.out_hw.release()  # 资源释放
            # print("!!!!!!!!!!!!!!!!!!")

    def get_latest_image(self):
        with self.lock:  # 获取锁
            return self.latest_image_rgb, self.latest_image_hw  # 返回最新的图像

def push_sport_node(request):
    call_server = client.call(request)
    print("返回clion==", call_server)

def save_image(request, image_save_dir):
    push_sport_node(request)
    # 存图
    ttime = str(int(round(time.time()*1000)))
    # ttime = time.strftime('%Y-%m-%d_%H:%M:%S', time.localtime(time.time()))
    rgb_img_name = ttime + "_rgb" + ".jpeg"
    hw_img_name = ttime + "_hw" + ".jpeg"
    latest_image_rgb, latest_image_hw = image_thread.get_latest_image()
    # time.sleep(3)
    if latest_image_rgb is not None and latest_image_hw is not None:
        cv2.imwrite(os.path.join(image_save_dir, rgb_img_name), latest_image_rgb)  # 显示最新的图像
        cv2.imwrite(os.path.join(image_save_dir, hw_img_name), latest_image_hw)  # 显示最新的图像
        print("存图成功")
    else:
        print("读取图像失败")
    # time.sleep(3)
    cv2.destroyAllWindows()

# 指定点拍照
def do(node):
    request.num1 = node
    image_save_dir = "/home/cetc21/image"
    if request.num1 == 33:
        push_sport_node(request)
        print("push yuntai init node")
    elif request.num1 <= 16:
        print("push yuntai {0} node".format(str(request.num1)))
        save_image(request, image_save_dir)
    else:
        raise ZeroDivisionError("The yuntai node is not set")


class Task:
    def __init__(self):
        self.taskPoints = []
        self.currentIndex = 0
        # self.robot_transfer = None
        self.robot_transfer = TaskTransfer()
        self.src_index = None
        self.des_index = None
        self.ntask = 0
        self.tf_listener = tf.TransformListener()
        # self.send_tf_thread = threading.Thread(target=self.send_tf, name="send_tf")
        # self.send_tf_thread.setDaemon(True)
        # self.send_tf_thread.start()
        self.comm_lock = threading.Lock()
        self.send_mode_thread = threading.Thread(target=self.send_mode, name="send_mode")
        self.send_mode_thread.setDaemon(True)
        self.send_mode_thread.start()

    def send_mode(self):
        """
        Daemon Thread
        """
        # ROS节点初始化
        # rospy.init_node('state_mode', anonymous=True)
        # 创建一个Publisher，发布名为/state_mode的topic，消息类型为int，队列长度1
        State_mode_info_pub = rospy.Publisher('/state_mode', Int32, queue_size=1)
        # 设置循环的频率
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # 发布消息
            mode_now = self.robot_transfer.modepart.get_mode()
            if mode_now:
                State_mode_info_pub.publish(mode_now)
                # print ("mode:",mode_now)
            rate.sleep()  # 按照循环频率延时

    # def send_tf(self):
    #     """
    #     Daemon Thread
    #     """
    #     while not rospy.is_shutdown():
    #         current_tf = self.listen_tf()
    #         if current_tf:
    #             with RobotCommander(local_port=20003) as robot_commander:
    #                 robot_commander.sendCordinate(
    #                     command_code=52,
    #                     x=current_tf[0],
    #                     y=current_tf[1],
    #                     yaw=current_tf[2],
    #                 )

    #                 robot_commander.sendCordinate(command_code=51, x= 3.769226,y= 1.286894,yaw= 1.555460,)
    #         rospy.sleep(0.05)

    def listen_tf(self):
        try:
            (pos, ori) = self.tf_listener.lookupTransform(
                "/map", "/velodyne", rospy.Duration(0.0)
            )

            yaw = tf.transformations.euler_from_quaternion(ori)[2]
            msg_list = [pos[0], pos[1], yaw]
            return msg_list
        except tf.Exception as e:
            print("listen to tf failed")
            #print(e)
            #print(e.message)
            return None

    def init(self):
        # stand up, read to go
        # globalTaskPrepare()
        # from task_point1 to task_point2
        # self.robot_transfer = TaskTransfer()
        # self.robot_transfer.initstand()

        self.loadTaskpoints()
        # only using TaskInit() to get nearest task_point

        task_init = TaskInit()

        nearest_index, initial_point = task_init.getBestTaskInd(self.taskPoints)
        #add
        self.robot_transfer.modepart.walk_mode()
        self.robot_transfer.task_transfer(initial_point, self.taskPoints[nearest_index])
        # total number of the task_points
        self.ntask = self.taskPoints.__len__()

        self.src_index = nearest_index
        self.des_index = nearest_index

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
        while not rospy.is_shutdown() and not self.des_index ==self.ntask:

            self.robot_transfer.task_transfer(self.taskPoints[self.src_index], self.taskPoints[self.des_index])
            ################
            rospy.sleep(2.0)
            print("stand")
            self.robot_transfer.modepart.stand_mode()
            rospy.sleep(3.0)
            # stop to do scan_task
            if self.taskPoints[self.des_index].name == "waypoint_1":
                self.robot_transfer.modepart.stand_mode()
                rospy.sleep(0.5)
                print("recognize begin!")
                do(3)
                do(33)
                rospy.sleep(3.0)
            if self.taskPoints[self.des_index].name == "waypoint_2":
                self.robot_transfer.modepart.stand_mode()
                rospy.sleep(0.5)
                print("recognize begin!")
                do(6)
                do(33)
                rospy.sleep(3.0)
            if self.taskPoints[self.des_index].name == "waypoint_3":
                self.robot_transfer.modepart.stand_mode()
                rospy.sleep(0.5)
                print("recognize begin!")
                do(5)
                do(33)
                rospy.sleep(3.0)
            if self.taskPoints[self.des_index].name == "waypoint_4":
                self.robot_transfer.modepart.stand_mode()
                rospy.sleep(0.5)
                print("recognize begin!")
                do(2)
                do(33)
                rospy.sleep(3.0)
            self.robot_transfer.modepart.walk_mode()

            # with RobotCommander() as robot_commander:
            # robot_commander.motion_start_stop()
            # if self.taskPoints[self.des_index].name == "waypoint_5":
            #     with RobotCommander() as robot_commander:
            #         robot_commander.motion_start_stop() 
            #     self.find_test()
            #     rospy.sleep(3.0)
            #     with RobotCommander() as robot_commander:
            #         robot_commander.motion_start_stop()

            self.src_index = self.des_index
            self.des_index = self.des_index + 1
            #self.des_index = (self.des_index + 1) % self.ntask
        print("end1")
        self.robot_transfer.modepart.stand_mode()
        rospy.sleep(3.0)
        self.robot_transfer.modepart.down_mode()
        rospy.sleep(3.0)
        

    # def find_test(self):
    #     rospy.sleep(0.5)
    #     rospy.wait_for_service('detection')
    #     try:
    #         val = rospy.ServiceProxy('detection',  Detection_srv)
    #         resp1 = val(True)
    #         print resp1.detction_complete, resp1.message
    #     except rospy.ServiceException, e:
    #         print e
    #     rospy.sleep(4.0)      

    #############################################guo_song_hui_OCR
    def get_longest_element(self, item_list):
        len_list = map(len, item_list)
        li = list(len_list)
        return item_list[np.argmax(li)]

    def rmall(self, a, e):
        """

        Args:
            a: list
            e: element

        Returns: a中去除e的list

        """
        b = []
        for ee in a:
            if ee != e:
                b.append(ee)
            else:
                continue
        return b

    def get_list_max_append_element(self, item_list):
        """

        Args:
            item_list: list

        Returns:列表中出现次数最多的元素->list元素集合

        """
        collection_words = Counter(item_list)
        frequency = []  # 储存出现次数
        for k, v in collection_words.items():
            frequency.append(v)
        max_index = [i for i, j in enumerate(frequency) if j == max(frequency)]
        # print(max_index)
        result_list = []
        for x in max_index:
            result_list.append(item_list[x])
        return result_list

    ################################################################
    def ocr_rocognize(self):
        rospy.sleep(0.5)
        rospy.wait_for_service("start_recognize")
        # rospy.wait_for_service("start_recognize")
        try:
            client = rospy.ServiceProxy("start_recognize", Order)
            print("send order!")
            # rospy.wait_for_service("start_recognize")
            request = OrderRequest()
            request.do_recognize = True
            respond_list = []  # 存储50次检测的结果

            for i in range(20):
                # rospy.loginfo("发送检测请求")
                respond = client.call(request)
                if respond.recognize_result != "null":
                    respond_list.append(respond.recognize_result)

            if len(respond_list) == 0:
                print("\n")
                print("---检测结果是：Null ---")
                # print(f"\033[1;31;40m ---检测结果是：Null --- \033[0m ")
                print("\n")
                # rospy.loginfo("检测结果是：%s", "Null")
            else:
                result_list = self.get_list_max_append_element(respond_list)
                result = self.get_longest_element(result_list)
                print("\n")
                # print(f"\033[1;32;40m ---检测结果是：{result} --- \033[0m ")
                print("---检测结果是： %s ---" % result)
                print("\n")
                # rospy.loginfo("检测结果是：%s", result)
        except rospy.ServiceException as e:
            print(e)
        rospy.sleep(0.5)
        print("recognize finish!")


##################################################################

class TaskInit:
    def __init__(self):
        self.initialPose = None
        self.tf_listener = tf.TransformListener()

    def listen_tf(self):
        try:
            (pos, ori) = self.tf_listener.lookupTransform(
                "/map", "/velodyne", rospy.Duration(0.0)
            )
            print("pos: ", pos)
            print("ori: ", ori)
            msg_list = [pos[0], pos[1], pos[2], ori[0], ori[1], ori[2], ori[3]]
            self.initialPose = msg_list
            return True
        except tf.Exception as e:
            print("listen to tf failed")
            #print(e)
            #print(e.message)
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
    rospy.init_node("autonomous_2d_navigation", anonymous=True)
    rospy.loginfo("Starting Line Follow node")
    image_thread = ImageThread()
    image_thread.daemon = True  # 在 Python 退出之前，它会等待任何非守护线程的线程。而守护线程就是，一个不会阻止 Python 解释器退出的线程。True-->设置为守护线程
    image_thread.video = False  # 是否存录像
    
    image_thread.start()
    # rtsp_getimg.start_thread()
    client = rospy.ServiceProxy("yuntai_sport", yt_data)
    client.wait_for_service()
    request = yt_dataRequest()
    time.sleep(5)
    rospy.loginfo("*******************")

    task = Task()

    task.init()
    rospy.sleep(0.5)

    task.run()
    
