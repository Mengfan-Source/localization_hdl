
import sys
sys.path.append("/home/cetc21/nav_ws/src")


import numpy as np
import rospy


import sys

sys.path.append("/home/cetc21/catkin_tunnel_ws/devel/lib/python3/dist-packages")
sys.path.append("/home/cetc21/nav_ws/src")

import os.path
import threading
import time
import cv2
from pipehuben.srv import yt_data, yt_dataRequest, yt_dataResponse
image_thread = None  # 定义全局变量

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
    do(10)
    
    
    
    
