#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import os
from pipehuben.srv import FindWarn,FindWarnResponse

def fun1(request):
    print ("%s"%request.camera_test)
    os.system('roslaunch apriltag_ros continuous_detection.launch')
    return [True, "find_none"]

def server():
    rospy.init_node('server_find_warn', anonymous = True)
    rospy.Service('test_find', FindWarn, fun1)
    rospy.spin()

if __name__ == '__main__':
    server()


