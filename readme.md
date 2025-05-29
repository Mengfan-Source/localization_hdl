# HDL定位：IMU预测，点云全局配准观测，UKF松耦合
## 编译
```bash
cd localization
catkin_make
```
## 运行
```bash
source ./devel/setup.bash
#对于X30数据
roslaunch hdl_localization my_hdl_X30.launch
#对于airy(自带IMU)激光雷达
roslaunch hdl_localization my_hdl_airy.launch
```

## 更新日志
### 20250301
- 适配X30数据(my_hdl_X30.launch)
    - 激光雷达输入数据：未做处理
    - IMU输入数据：量纲处理（是否揽沃雷达自带IMU，若不是，则）
    - 外参设置：将odom_child_frame_id参数设置为imu_link(程序中会将激光雷达数据转到该坐标系下)参与滤波计算
        设置表示lidar到IMU的静态坐标变换
        <node pkg="tf2_ros" type="static_transform_publisher" name="base_laser_tf_broadcaster" args="-0.08715 0.001 -0.117 0 0 0 imu_link lidar_link" />
    - 发布雷达坐标系下的激光雷达点云数据(体素滤波、点云个数滤波、半径滤波、滤除Z值后，滤波参数可在launch文件中设置)
### 20250315
- 解决定位频率不稳定问题
    - 设置配准迭代阈值20:可稳定输出10HZ的定位数据
- 分析并解决内存占用高问题
    - 1.点云配准迭代次数:设置点云配准迭代阈值
    - 2.点数问题:X30四个激光雷达，点数约8w，实测当狗蹲下后，激光点数量少，这时候CPU占用较低，或者将体素滤波参数由0.1设为0.2也可降低占用
    - 3.体素滤波参数:室外大空间场景下扩大体素滤波参数(0.2,在launch文件中修改两处)也可减少占用
    - 4.配准方式问题:配准方式选择NDT_OMP并且，紧邻搜索方案选择DIRECT1也可有效降低占用
    - 5.CPU工作模式问题:X30感知主机上CPU没设置性能模式，设置性能模式后，大概CPU占用能降低百分之二十到百分之三十
### 20250323
- 添加AIRY雷达的支持，在各个部分添加了适配airy激光雷达的代码
    - 1.添加了airy激光雷达数据类型的支持
    - 2.添加了airy激光雷达自带的IMU处理，在代码中量纲处理和坐标变换，所以外参设置为单位矩阵即可
    - 3.(HDL没有运动畸变去除操作，因此这个操作并未设置)一帧激光雷达中每个点偏移时间与其他雷达不同，接入使用IMU数据进行激光雷达运动畸变去除的SLAM算法时，需要计算偏移量（减去第一点的偏移时间即可） 一帧数据中每个点的偏移时间与其他雷达不同，rs_lidar的偏移时间是运行时刻递增的，而不是一个单纯的偏移量，因此在接入使用IMU进行运动畸变去除的SLAM算法时需要将每个点的偏移时间设置为当前点的时间减去第一个点的相对时间。
    - 4.建立了针对airy激光雷达(airy雷达自带的IMU)，my_hdl_airy.launch
        - 由于在IMU回调函数中处理了外参和量纲，因此无需设置外参，将odom_child_frame_id设置为rslidar即可
### 20250529
- 这是没加z值搜索和体素金字塔，没加leg_odom观测的长期可运行版本，存放在no_init_no_legodom_use分支
