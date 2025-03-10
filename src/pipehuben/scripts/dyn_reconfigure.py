#!/usr/bin/python

import rospy
import dynamic_reconfigure.client

rospy.init_node("myconfig_py", anonymous=True)
client_teb = dynamic_reconfigure.client.Client("/move_base/TebLocalPlannerROS")
client_global_costmap_1 = dynamic_reconfigure.client.Client(
    "/move_base/global_costmap/obstacle_layer_lidar2scan"
)
client_global_costmap_2 = dynamic_reconfigure.client.Client(
    "/move_base/global_costmap/obstacle_layer_stereo2scan"
)
client_local_costmap_1 = dynamic_reconfigure.client.Client(
    "/move_base/local_costmap/obstacle_layer_lidar2scan"
)
client_local_costmap_2 = dynamic_reconfigure.client.Client(
    "/move_base/local_costmap/obstacle_layer_stereo2scan"
)

slow_vel_params = {
    "max_vel_x": 0.1,
    "max_vel_x_backwards": 0.02,
    "max_vel_y": 0.08,
    "max_vel_theta": 0.2,
    "acc_lim_x": 0.08,
    "acc_lim_y": 0.03,
    "acc_lim_theta": 0.1,
    "yaw_goal_tolerance": 0.1,
    "xy_goal_tolerance": 0.1,
}
client_teb.update_configuration(slow_vel_params)

disable_costmap_params = {"enabled": False}
client_global_costmap_1.update_configuration(disable_costmap_params)
client_global_costmap_2.update_configuration(disable_costmap_params)
client_local_costmap_1.update_configuration(disable_costmap_params)
client_local_costmap_2.update_configuration(disable_costmap_params)
