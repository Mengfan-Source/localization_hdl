#include <mutex>
#include <memory>
#include <iostream>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <eigen_conversions/eigen_msg.h>

#include <std_srvs/Empty.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <pcl/filters/voxel_grid.h>

#include <pclomp/ndt_omp.h>

#include <fast_gicp/ndt/ndt_cuda.hpp>
#include <fast_gicp/gicp/fast_gicp.hpp>

#include <hdl_localization/pose_estimator.hpp>
#include <hdl_localization/delta_estimater.hpp>

#include <hdl_localization/ScanMatchingStatus.h>
#include <hdl_global_localization/SetGlobalMap.h>
#include <hdl_global_localization/QueryGlobalLocalization.h>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/pcl_macros.h>
#include <pcl_conversions/pcl_conversions.h>
namespace airy_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      std::uint16_t ring = 0;
      double timestamp = 0;
      std::uint8_t feature;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace airy_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(airy_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint16_t, ring, ring)
    (double, timestamp, timestamp)
    (std::uint8_t, feature, feature)
)
namespace hdl_localization {
class HdlLocalizationNodelet : public nodelet::Nodelet {
        public:
        using PointT = pcl::PointXYZI;
        FILE* pose_file = NULL;

        HdlLocalizationNodelet () : tf_buffer (), tf_listener (tf_buffer) {}
        virtual ~HdlLocalizationNodelet () {
                fclose (this->pose_file);
                delete this->pose_file;
                this->pose_file = NULL;
        }

        void onInit () override {
                nh = getNodeHandle ();
                mt_nh = getMTNodeHandle ();
                private_nh = getPrivateNodeHandle ();
                lidar_type = private_nh.param<std::string> ("lidar_type", "MID360");//自定义参数：激光雷达类型
                point_filter_num = private_nh.param<int> ("point_filter_num", 2);//自定义参数：点数滤波参数
                blind_min = private_nh.param<double> ("blind_min", 0.5);//自定义参数：半径滤波参数
                blind_max = private_nh.param<double> ("blind_max", 20);
                z_filter_min = private_nh.param<double> ("z_filter_min", -0.3);//自定义参数：发布的点云Z轴滤波范围min
                z_filter_max = private_nh.param<double> ("z_filter_max", 20);//自定义参数：发布的点云Z轴滤波范围max
                use_legodom = private_nh.param<int>("use_legodom",0);
                // NOTE1 初始化体素滤波器、点云匹配器、位姿增量估计器、起始位姿及其位姿估计器
                initialize_params ();

                // NOTE2 里程计及机器人本体坐标系设定
                robot_odom_frame_id = private_nh.param<std::string> ("robot_odom_frame_id", "robot_odom");
                odom_child_frame_id = private_nh.param<std::string> ("odom_child_frame_id", "base_link");

                use_imu = private_nh.param<bool> ("use_imu", true);
                invert_acc = private_nh.param<bool> ("invert_acc", false);
                invert_gyro = private_nh.param<bool> ("invert_gyro", false);
                if (use_imu) {
                        NODELET_INFO ("enable imu-based prediction");
                        // NOTE3 将IMU数据压入imu_data数据队列
                        imu_sub = mt_nh.subscribe ("/gpsimu_driver/imu_data", 256, &HdlLocalizationNodelet::imu_callback, this);
                }
                if(use_legodom){
                        NODELET_INFO ("enable leg_odom");
                        legodom_sub = mt_nh.subscribe("/leg_odom",1000,&HdlLocalizationNodelet::legodom_callback, this);
                }

                // NOTE4 订阅实时点云数据不断更新机器人位姿
                points_sub = mt_nh.subscribe ("/velodyne_points", 1, &HdlLocalizationNodelet::points_callback, this);

                // NOTE5 将地图点云压入匹配器的目标点云，并将地图点云发送到全局定位服务器
                globalmap_sub = nh.subscribe ("/globalmap", 1, &HdlLocalizationNodelet::globalmap_callback, this);

                // NOTE6 订阅rviz下发初始位姿的话题，初始化起始机器人位姿
                initialpose_sub = nh.subscribe ("/initialpose", 8, &HdlLocalizationNodelet::initialpose_callback, this);

                // NOTE7 相关数据发布器
                pose_pub = nh.advertise<nav_msgs::Odometry> ("/odom", 5, false);
                aligned_pub = nh.advertise<sensor_msgs::PointCloud2> ("/aligned_points", 5, false);
                status_pub = nh.advertise<ScanMatchingStatus> ("/status", 5, false);
                my_pointpub = nh.advertise<sensor_msgs::PointCloud2> ("/livox_points_transformed", 5, false);
                // NOTE8 全局重定位
                use_global_localization = private_nh.param<bool> ("use_global_localization", true);
                if (use_global_localization) {
                        NODELET_INFO_STREAM ("wait for global localization services");
                        ros::service::waitForService ("/hdl_global_localization/set_global_map");
                        ros::service::waitForService ("/hdl_global_localization/query");

                        // NOTE 向GlobalLocalizationServerNodes发送服务
                        set_global_map_service = nh.serviceClient<hdl_global_localization::SetGlobalMap> ("/hdl_global_localization/set_global_map");

                        // NOTE 如果在终端不向relocalizate发送消息，则不执行下面两个
                        query_global_localization_service = nh.serviceClient<hdl_global_localization::QueryGlobalLocalization> ("/hdl_global_localization/query");
                        relocalize_server = nh.advertiseService ("/relocalize", &HdlLocalizationNodelet::relocalize, this);
                }
        }

        private:
        pcl::Registration<PointT, PointT>::Ptr create_registration () const {
                std::string reg_method = private_nh.param<std::string> ("reg_method", "NDT_OMP");  // 匹配方法
                std::string ndt_neighbor_search_method = private_nh.param<std::string> ("ndt_neighbor_search_method", "DIRECT7");  // 近邻搜索方法
                double ndt_neighbor_search_radius = private_nh.param<double> ("ndt_neighbor_search_radius", 2.0); // 近邻搜索半径
                double ndt_resolution = private_nh.param<double> ("ndt_resolution", 1.0);  // NDT体素分辨率

                // gicp param
                std::string regular_method = private_nh.param<std::string> ("regular_method", "NONE");
                int gicp_numthreads = private_nh.param<int> ("gicp_numThreads", 0);
                int CorrespondRandomness = private_nh.param<int> ("CorrespondRandomness", 20);
                double max_correspondence_distance = private_nh.param<double> ("max_correspondence_distance", 64);
                double transformation_epsilon = private_nh.param<double> ("transformation_epsilon", 0.01);

                if (reg_method == "NDT_OMP") {
                        NODELET_INFO ("NDT_OMP is selected");
                        pclomp::NormalDistributionsTransform<PointT, PointT>::Ptr ndt (new pclomp::NormalDistributionsTransform<PointT, PointT> ());
                        ndt->setTransformationEpsilon (0.01);
                        ndt->setResolution (ndt_resolution);
                        if (ndt_neighbor_search_method == "DIRECT1") {
                                NODELET_INFO ("search_method DIRECT1 is selected");
                                ndt->setNeighborhoodSearchMethod (pclomp::DIRECT1);
                        }
                        else if (ndt_neighbor_search_method == "DIRECT7") {
                                NODELET_INFO ("search_method DIRECT7 is selected");
                                ndt->setNeighborhoodSearchMethod (pclomp::DIRECT7);
                        }
                        else {
                                if (ndt_neighbor_search_method == "KDTREE") {
                                        NODELET_INFO ("search_method KDTREE is selected");
                                }
                                else {
                                        NODELET_WARN ("invalid search method was given");
                                        NODELET_WARN ("default method is selected (KDTREE)");
                                        ndt->setNeighborhoodSearchMethod (pclomp::KDTREE);
                                }
                        }
                        return ndt;
                }
                else if (reg_method.find ("GICP") != std::string::npos) {
                        NODELET_INFO ("GICP is selected");
                        boost::shared_ptr<fast_gicp::FastGICP<PointT, PointT>> gicp (new fast_gicp::FastGICP<PointT, PointT>);
                        gicp->setNumThreads (gicp_numthreads);
                        gicp->setCorrespondenceRandomness (CorrespondRandomness);
                        gicp->setMaxCorrespondenceDistance (max_correspondence_distance);
                        gicp->setTransformationEpsilon (transformation_epsilon);

                        if (regular_method.find ("FROBENIUS") != std::string::npos) {
                                gicp->setRegularizationMethod (fast_gicp::RegularizationMethod::FROBENIUS);
                        }
                        else if (regular_method.find ("NORMALIZED_MIN_EIG") != std::string::npos) {
                                gicp->setRegularizationMethod (fast_gicp::RegularizationMethod::NORMALIZED_MIN_EIG);
                        }
                        else if (regular_method.find ("PLANE") != std::string::npos) {
                                gicp->setRegularizationMethod (fast_gicp::RegularizationMethod::PLANE);
                        }
                        else {
                                gicp->setRegularizationMethod (fast_gicp::RegularizationMethod::NONE);
                        }
                        return gicp;

                }

                NODELET_ERROR_STREAM ("unknown registration method:" << reg_method);
                return nullptr;
        }

        /**
         * @brief 初始化体素滤波器、点云匹配器、位姿增量估计器、起始位姿及其位姿估计器
         */
        void initialize_params () {
                // NOTE 根据降采样体素栅格边长大小初始化体素滤波器
                double downsample_resolution = private_nh.param<double> ("downsample_resolution", 0.1);
                boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid (new pcl::VoxelGrid<PointT> ());
                voxelgrid->setLeafSize (downsample_resolution, downsample_resolution, downsample_resolution);
                downsample_filter = voxelgrid;


                // NOTE 创建点云匹配器，并设置最大匹配迭代次数
                NODELET_INFO ("create registration method for localization");
                registration = create_registration ();
                registration->setMaximumIterations (20);//500
                NODELET_INFO_STREAM ("TransformationEpsilon" << registration->getTransformationEpsilon () << "max iteration" << registration->getMaximumIterations ());

                // NOTE 初始化位姿增量估计器 (目前没有用到，需通过客户端发送服务进行重定位才用到)
                NODELET_INFO ("create registration method for fallback during relocalization");
                relocalizing = false;
                delta_estimater.reset (new DeltaEstimater (create_registration ()));

                // NOTE 从配置参数中初始化起始位姿，并初始化位姿估计器
                if (private_nh.param<bool> ("specify_init_pose", true)) {
                        NODELET_INFO ("initialize pose estimator with specified parameters!!");
                        pose_estimator.reset (new hdl_localization::PoseEstimator (
                                registration,
                                ros::Time::now (),
                                Eigen::Vector3f (private_nh.param<double> ("init_pos_x", 0.0), private_nh.param<double> ("init_pos_y", 0.0), private_nh.param<double> ("init_pos_z", 0.0)),
                                Eigen::Quaternionf (
                                        private_nh.param<double> ("init_ori_w", 1.0),
                                        private_nh.param<double> ("init_ori_x", 0.0),
                                        private_nh.param<double> ("init_ori_y", 0.0),
                                        private_nh.param<double> ("init_ori_z", 0.0)),
                                private_nh.param<double> ("cool_time_duration", 0.5)));
                }
                this->pose_file = fopen (std::string(std::string(ROOT_DIR) +"data/pose.txt").c_str(), "w+");
                if (this->pose_file == NULL) {
                        printf ("位姿保存文件未打开\n");
                }
        }

        private:
        /**
         * @brief callback for imu data
         * @param imu_msg
         */
        void legodom_callback (const nav_msgs::OdometryConstPtr& msg_in){
                if(pose_estimator){
                        PoseEstimator::LegOdomPtr msg = std::make_shared<PoseEstimator::LegOdom>();
                        msg->timestamp = msg_in->header.stamp.toSec();
                        msg->pos.x() = msg_in->pose.pose.position.x;
                        msg->pos.y() = msg_in->pose.pose.position.y;
                        msg->pos.z() = msg_in->pose.pose.position.z;
                        msg->q.x() = msg_in->pose.pose.orientation.x;
                        msg->q.y() = msg_in->pose.pose.orientation.y;
                        msg->q.z() = msg_in->pose.pose.orientation.z;
                        msg->q.w() = msg_in->pose.pose.orientation.w; 
                        std::lock_guard<std::mutex> lock (pose_estimator->leg_odom_mutex);
                        pose_estimator->legodom_buffer.push_back(msg);
                        if(pose_estimator->legodom_buffer.size()>500){
                                pose_estimator->legodom_buffer.pop_front();
                        }
                }
                else{
                        std::cout<<"waiting for pose_estimator object"<<std::endl;
                }
        }
        void imu_callback (const sensor_msgs::ImuConstPtr& imu_msg) {
                /* 原函数体
                std::lock_guard<std::mutex> lock (imu_data_mutex);
                imu_data.push_back (imu_msg);
                */
                sensor_msgs::Imu msg_out = *imu_msg  ;
                if(lidar_type=="AIRY"){
                        Eigen::Quaterniond q_lidar2imu(0.0029166745953261852,0.7073081731796265,-0.7068824768066406,0.004880243446677923);
                        Eigen::Vector3d t_lidar2imu(0.0042, 0.0041, -0.0044);  
                        Eigen::Isometry3d T_imu2lidar = Eigen::Isometry3d::Identity();
                        T_imu2lidar.rotate(q_lidar2imu.toRotationMatrix().inverse());
                        T_imu2lidar.pretranslate(-t_lidar2imu);
                        Eigen::Vector3d acc_imu(imu_msg->linear_acceleration.x*9.81,imu_msg->linear_acceleration.y*9.81,imu_msg->linear_acceleration.z*9.81);
                        Eigen::Vector3d gyro_imu(imu_msg->angular_velocity.x,imu_msg->angular_velocity.y,imu_msg->angular_velocity.z);
                        Eigen::Vector3d acc_lidar = T_imu2lidar * acc_imu;
                        Eigen::Vector3d gyro_lidar = T_imu2lidar * gyro_imu;
                        msg_out.linear_acceleration.x = acc_lidar.x();
                        msg_out.linear_acceleration.y = acc_lidar.y();
                        msg_out.linear_acceleration.z = acc_lidar.z();
                        msg_out.angular_velocity.x = gyro_lidar.x();
                        msg_out.angular_velocity.y = gyro_lidar.y();
                        msg_out.angular_velocity.z = gyro_lidar.z();
                }
                else if(lidar_type=="MID360"){
                        float yaw_angle = 0;  // 90度逆时针偏航
                        float roll_angle = 0; // 45度横滚

                        // 生成绕Z轴的偏航旋转矩阵
                        Eigen::Matrix4d yaw_transform = Eigen::Matrix4d::Identity();
                        // yaw_transform(0, 0) = cos(yaw_angle);
                        // yaw_transform(0, 1) = -sin(yaw_angle);
                        // yaw_transform(1, 0) = sin(yaw_angle);
                        // yaw_transform(1, 1) = cos(yaw_angle);

                        // 生成绕X轴的横滚旋转矩阵
                        Eigen::Matrix4d roll_transform = Eigen::Matrix4d::Identity();
                        // roll_transform(1, 1) = cos(roll_angle);
                        // roll_transform(1, 2) = -sin(roll_angle);
                        // roll_transform(2, 1) = sin(roll_angle);
                        // roll_transform(2, 2) = cos(roll_angle);

                // 组合旋转矩阵（先偏航再横滚）
                        Eigen::Matrix4d combined_transform = yaw_transform * roll_transform;
                        Eigen::Matrix3d rotation_matrix = combined_transform.block<3, 3>(0, 0);


                // 将IMU的加速度、角速度转换为Eigen向量
                        // Eigen::Vector3d linear_acceleration(imu_msg->linear_acceleration.x*9.81, 
                        //                                 imu_msg->linear_acceleration.y*9.81, 
                        //                                 imu_msg->linear_acceleration.z*9.81);
                        Eigen::Vector3d linear_acceleration(imu_msg->linear_acceleration.x*1, 
                                                        imu_msg->linear_acceleration.y*1, 
                                                        imu_msg->linear_acceleration.z*1);

                        Eigen::Vector3d angular_velocity(imu_msg->angular_velocity.x, 
                                                imu_msg->angular_velocity.y, 
                                        imu_msg->angular_velocity.z);
                        Eigen::Vector3d transformed_linear_acceleration = rotation_matrix * linear_acceleration;
                        Eigen::Vector3d transformed_angular_velocity = rotation_matrix * angular_velocity;
                        
                        msg_out.linear_acceleration.x = transformed_linear_acceleration.x();
                        msg_out.linear_acceleration.y = transformed_linear_acceleration.y();
                        msg_out.linear_acceleration.z = transformed_linear_acceleration.z();

                        msg_out.angular_velocity.x = transformed_angular_velocity.x();
                        msg_out.angular_velocity.y = transformed_angular_velocity.y();
                        msg_out.angular_velocity.z = transformed_angular_velocity.z();

                        // 旋转四元数（表示方向），注意四元数需要规范化
                        Eigen::Quaterniond orientation(imu_msg->orientation.w, 
                                                        imu_msg->orientation.x, 
                                                        imu_msg->orientation.y, 
                                                        imu_msg->orientation.z);
                        
                        // 转换四元数到目标坐标系
                        Eigen::Quaterniond transformed_orientation = Eigen::Quaterniond(rotation_matrix) * orientation;
                        transformed_orientation.normalize();

                        msg_out.orientation.w = transformed_orientation.w();
                        msg_out.orientation.x = transformed_orientation.x();
                        msg_out.orientation.y = transformed_orientation.y();
                        msg_out.orientation.z = transformed_orientation.z();

                }
                else{
                        ROS_WARN("lidar type error,MID360 and AIRY supported");
                }
                sensor_msgs::ImuConstPtr imuout_ptr = boost::make_shared<sensor_msgs::Imu>(msg_out);
                std::lock_guard<std::mutex> lock (imu_data_mutex);
                imu_data.push_back (imuout_ptr);
        }

        /**
         * @brief callback for point cloud data
         * @param points_msg
         */
        void points_callback (const sensor_msgs::PointCloud2ConstPtr& points_msg) {
                if(lidar_type=="AIRY"){
                        std::lock_guard<std::mutex> estimator_lock (pose_estimator_mutex);
                        if (!pose_estimator) {
                                NODELET_ERROR ("waiting for initial pose input!!");
                                return;
                        }

                        if (!globalmap) {
                                NODELET_ERROR ("globalmap has not been received!!");
                                return;
                        }
                        // ROS_WARN("run into airy handller 1");
                        const auto& stamp = points_msg->header.stamp;
                        pcl::PointCloud<airy_ros::Point>::Ptr pl_orig (new pcl::PointCloud<airy_ros::Point> ());
                        pcl::fromROSMsg(*points_msg, *pl_orig);
                        pcl::PointCloud<airy_ros::Point>::Ptr pl_orig_filtered(new pcl::PointCloud<airy_ros::Point>());
                        // pcl::VoxelGrid<airy_ros::Point> sor_source;  
                        // sor_source.setInputCloud(pl_orig);  
                        // sor_source.setLeafSize(0.1f, 0.1f, 0.1f); // 设置体素的大小，这里是1cm  
                        // sor_source.filter(*pl_orig_filtered);  
                        // pl_orig_filtered->header = pl_orig->header;
                        // ROS_WARN("run into airy handller 2");
                        // int plsize = pl_orig_filtered->points.size();
                        int plsize = pl_orig->points.size();
                        if (plsize == 0) {
                                ROS_WARN("size of input pointcloud is 0 ");
                               return ; 
                        }
                        //点云配准使用的点
                        pcl::PointCloud<PointT>::Ptr pcl_cloud (new pcl::PointCloud<PointT> ());
                        pcl_cloud->reserve(plsize);
                        //发布给导航用的点
                        pcl::PointCloud<PointT>::Ptr pcl_cloud_pub (new pcl::PointCloud<PointT> ());
                        pcl_cloud_pub->reserve(plsize);
                        for(int i = 0;i<plsize;i++){
                                PointT added_pt;
                                added_pt.x = pl_orig->points[i].x;
                                added_pt.y = pl_orig->points[i].y;
                                added_pt.z = pl_orig->points[i].z;
                                added_pt.intensity = pl_orig->points[i].intensity;
                                if(i%point_filter_num == 0){
                                        double dis_temp = added_pt.x*added_pt.x+added_pt.y*added_pt.y+added_pt.z*added_pt.z;
                                        if(dis_temp > (blind_min * blind_min) && dis_temp < (blind_max*blind_max))
                                                pcl_cloud->points.push_back(added_pt);
                                        if(dis_temp > (blind_min * blind_min) && dis_temp < (blind_max*blind_max) && added_pt.z < z_filter_max && added_pt.z > z_filter_min)
                                                pcl_cloud_pub->points.push_back(added_pt);
                                }
                        }
                        pcl_cloud->header = pl_orig->header;
                        pcl_cloud_pub->header =pl_orig->header;
                        auto pcl_cloud_pub_filtered = downsample (pcl_cloud_pub);
                        // 发布 ROS 点云消息
                        sensor_msgs::PointCloud2 cloud_msg_out;
                        pcl::toROSMsg(*pcl_cloud_pub, cloud_msg_out);
                        cloud_msg_out.header.stamp = points_msg->header.stamp;
                        cloud_msg_out.header.frame_id = points_msg->header.frame_id;  // 设置合适的坐标系
                        my_pointpub.publish(cloud_msg_out);

                        if (pcl_cloud->empty ()) {
                                NODELET_ERROR ("cloud is empty!!");
                                return;
                        }
                        // transform pointcloud into odom_child_frame_id
                        // 将点云数据从雷达坐标系转到odom_child_frame_id坐标系
                        pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT> ());
                        if (!pcl_ros::transformPointCloud (odom_child_frame_id, *pcl_cloud, *cloud, this->tf_buffer)) {
                                NODELET_ERROR ("point cloud cannot be transformed into target frame!!");
                                return;
                        }

                        auto filtered = downsample (cloud);
                        // auto filtered = cloud;
                        last_scan = filtered;

                        if (relocalizing) {
                                delta_estimater->add_frame (filtered);
                        }

                        // NOTE 获取机器人位姿；开始时为初始位姿
                        Eigen::Matrix4f before = pose_estimator->matrix ();
                        // predict
                        if (!use_imu) {
                                // NOTE 刚开始预测处于冻结状态，只依靠NDT及观测模型
                                pose_estimator->predict (stamp);
                        }
                        else {
                                std::lock_guard<std::mutex> lock (imu_data_mutex);
                                auto imu_iter = imu_data.begin ();
                                for (imu_iter; imu_iter != imu_data.end (); imu_iter++) {
                                        if (stamp < (*imu_iter)->header.stamp) {
                                                break;
                                        }
                                        const auto& acc = (*imu_iter)->linear_acceleration;
                                        const auto& gyro = (*imu_iter)->angular_velocity;
                                        // std::cout << acc << std::endl;
                                        // std::cout << gyro << std::endl;
                                        double acc_sign = invert_acc ? -1.0 : 1.0;
                                        double gyro_sign = invert_gyro ? -1.0 : 1.0;
                                        // NODELET_INFO_STREAM("imu is used : " << gyro);
                                        pose_estimator->predict ((*imu_iter)->header.stamp, acc_sign * Eigen::Vector3f (acc.x, acc.y, acc.z), gyro_sign * Eigen::Vector3f (gyro.x, gyro.y, gyro.z));
                                }
                                imu_data.erase (imu_data.begin (), imu_iter);
                        }

                        // odometry-based prediction
                        ros::Time last_correction_time = pose_estimator->last_correction_time ();
                        if (private_nh.param<bool> ("enable_robot_odometry_prediction", false) && !last_correction_time.isZero ()) {
                                geometry_msgs::TransformStamped odom_delta;
                                if (tf_buffer.canTransform (odom_child_frame_id, last_correction_time, odom_child_frame_id, stamp, robot_odom_frame_id, ros::Duration (0.1))) {
                                        odom_delta = tf_buffer.lookupTransform (odom_child_frame_id, last_correction_time, odom_child_frame_id, stamp, robot_odom_frame_id, ros::Duration (0));
                                }
                                else if (tf_buffer.canTransform (odom_child_frame_id, last_correction_time, odom_child_frame_id, ros::Time (0), robot_odom_frame_id, ros::Duration (0))) {
                                        odom_delta = tf_buffer.lookupTransform (odom_child_frame_id, last_correction_time, odom_child_frame_id, ros::Time (0), robot_odom_frame_id, ros::Duration (0));
                                }

                                if (odom_delta.header.stamp.isZero ()) {
                                        NODELET_WARN_STREAM ("failed to look up transform between " << cloud->header.frame_id << " and " << robot_odom_frame_id);
                                }
                                else {
                                        Eigen::Isometry3d delta = tf2::transformToEigen (odom_delta);
                                        pose_estimator->predict_odom (delta.cast<float> ().matrix ());
                                }
                        }

                        // correct
                        auto aligned = pose_estimator->correct (stamp, filtered);
                        if (registration->getFitnessScore () > 2.0) {
                                //   NODELET_INFO_STREAM("gg, localization fail");
                                std::cout << " match fail" << registration->getFitnessScore () << std::endl;
                        }

                        // if (aligned_pub.getNumSubscribers ()) {
                        //         aligned->header.frame_id = "map1";
                        //         aligned->header.stamp = cloud->header.stamp;
                        //         aligned_pub.publish (aligned);
                        // }

                        if (status_pub.getNumSubscribers ()) {
                                publish_scan_matching_status (points_msg->header, aligned);
                        }

                        publish_odometry (points_msg->header.stamp, pose_estimator->matrix ());
                }
                else if(lidar_type=="MID360"){
                        std::lock_guard<std::mutex> estimator_lock (pose_estimator_mutex);
                        if (!pose_estimator) {
                                NODELET_ERROR ("waiting for initial pose input!!");
                                return;
                        }

                        if (!globalmap) {
                                NODELET_ERROR ("globalmap has not been received!!");
                                return;
                        }

                        const auto& stamp = points_msg->header.stamp;

                        pcl::PointCloud<PointT>::Ptr mypcl_cloud (new pcl::PointCloud<PointT> ());
                        // pcl::PointCloud<PointT>::Ptr mytranspitchptr (new pcl::PointCloud<PointT> ());
                        pcl::fromROSMsg (*points_msg, *mypcl_cloud);

                        pcl::PointCloud<PointT>::Ptr mypcl_cloud_filtered(new pcl::PointCloud<PointT>());
                        pcl::VoxelGrid<PointT> sor_source;  
                        sor_source.setInputCloud(mypcl_cloud);  
                        sor_source.setLeafSize(0.2f, 0.2f, 0.2f); // 设置体素的大小，这里是1cm  
                        sor_source.filter(*mypcl_cloud_filtered);  


                        float yaw_angle = 0;  // 90度逆时针偏航
                        float roll_angle = 0; // 45度横滚

                        // 生成绕Z轴的偏航旋转矩阵
                        Eigen::Matrix4f yaw_transform = Eigen::Matrix4f::Identity();
                        // yaw_transform(0, 0) = cos(yaw_angle);
                        // yaw_transform(0, 1) = -sin(yaw_angle);
                        // yaw_transform(1, 0) = sin(yaw_angle);
                        // yaw_transform(1, 1) = cos(yaw_angle);

                        // 生成绕X轴的横滚旋转矩阵
                        Eigen::Matrix4f roll_transform = Eigen::Matrix4f::Identity();
                        // roll_transform(1, 1) = cos(roll_angle);
                        // roll_transform(1, 2) = -sin(roll_angle);
                        // roll_transform(2, 1) = sin(roll_angle);
                        // roll_transform(2, 2) = cos(roll_angle);

                // 组合旋转矩阵（先偏航再横滚）
                        Eigen::Matrix4f combined_transform = yaw_transform * roll_transform;
                        pcl::PointCloud<PointT>::Ptr pcl_cloud1 (new pcl::PointCloud<PointT> ());
                        pcl::transformPointCloud(*mypcl_cloud_filtered,*pcl_cloud1,  combined_transform);
                //做半径滤波
                        pcl::PointCloud<PointT>::Ptr pcl_cloud (new pcl::PointCloud<PointT> ());
                        for(size_t i=0;i<pcl_cloud1->points.size();i++)
                        {
                                double dis_temp = pcl_cloud1->points[i].x*pcl_cloud1->points[i].x + pcl_cloud1->points[i].y*pcl_cloud1->points[i].y + pcl_cloud1->points[i].z*pcl_cloud1->points[i].z;
                                if(dis_temp > (blind_min*blind_min) && dis_temp < (blind_max*blind_max) && pcl_cloud1->points[i].z > z_filter_min && pcl_cloud1->points[i].z < z_filter_max)
                                {
                                        pcl_cloud->points.push_back(pcl_cloud1->points[i]);
                                }
                        }
                        // std::cout<<pcl_cloud->points.size()<<std::endl;
                        pcl_cloud->header = pcl_cloud1->header;

                        // 将变换后的点云发布到ROS话题
                        sensor_msgs::PointCloud2 mycloud_msg;
                        pcl::toROSMsg(*pcl_cloud, mycloud_msg);
                        mycloud_msg.header.stamp = points_msg->header.stamp;
                        mycloud_msg.header.frame_id = points_msg->header.frame_id;  // 设置合适的坐标系

                        // 发布 ROS 点云消息
                        my_pointpub.publish(mycloud_msg);
   
                        // pcl::fromROSMsg (*points_msg, *pcl_cloud);

                        if (pcl_cloud->empty ()) {
                                NODELET_ERROR ("cloud is empty!!");
                                return;
                        }
                        // transform pointcloud into odom_child_frame_id
                        // 将点云数据从雷达坐标系转到odom_child_frame_id坐标系
                        pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT> ());
                        //将mypcl_cloud点云转到odom_child_frame_id坐标系下（这里在launch文件中配置为lidar_link）
                        // NODELET_INFO("before:");
                        // NODELET_INFO(mypcl_cloud->header.frame_id.c_str());------>这里打印出来时lidar_link
                        if (!pcl_ros::transformPointCloud (odom_child_frame_id, *mypcl_cloud, *cloud, this->tf_buffer)) {
                                NODELET_ERROR ("point cloud cannot be transformed into target frame!!");
                                return;
                        }
                        // NODELET_INFO("after:");
                        // NODELET_INFO(cloud->header.frame_id.c_str());------>这里打印出来时lidar_link
                        

                        auto filtered = downsample (cloud);
                        last_scan = filtered;

                        if (relocalizing) {
                                delta_estimater->add_frame (filtered);
                        }

                        // NOTE 获取机器人位姿；开始时为初始位姿
                        Eigen::Matrix4f before = pose_estimator->matrix ();
                        // predict
                        if (!use_imu) {
                                // NOTE 刚开始预测处于冻结状态，只依靠NDT及观测模型
                                pose_estimator->predict (stamp);
                        }
                        else {
                                std::lock_guard<std::mutex> lock (imu_data_mutex);
                                auto imu_iter = imu_data.begin ();
                                for (imu_iter; imu_iter != imu_data.end (); imu_iter++) {
                                        if (stamp < (*imu_iter)->header.stamp) {
                                                break;
                                        }
                                        const auto& acc = (*imu_iter)->linear_acceleration;
                                        const auto& gyro = (*imu_iter)->angular_velocity;
                                        // std::cout << acc << std::endl;
                                        // std::cout << gyro << std::endl;
                                        double acc_sign = invert_acc ? -1.0 : 1.0;
                                        double gyro_sign = invert_gyro ? -1.0 : 1.0;
                                        // NODELET_INFO_STREAM("imu is used : " << gyro);
                                        pose_estimator->predict ((*imu_iter)->header.stamp, acc_sign * Eigen::Vector3f (acc.x, acc.y, acc.z), gyro_sign * Eigen::Vector3f (gyro.x, gyro.y, gyro.z));
                                }
                                imu_data.erase (imu_data.begin (), imu_iter);
                        }

                        // odometry-based prediction
                        ros::Time last_correction_time = pose_estimator->last_correction_time ();
                        if (private_nh.param<bool> ("enable_robot_odometry_prediction", false) && !last_correction_time.isZero ()) {
                                geometry_msgs::TransformStamped odom_delta;
                                //判断是否可以查询从last_correction_time时刻odom_child_frame_id到stamp时刻odom_child_frame_id之间的坐标变换关系，时间容错0.1，参考座标系robot_odom_frame_id
                                if (tf_buffer.canTransform (odom_child_frame_id, last_correction_time, odom_child_frame_id, stamp, robot_odom_frame_id, ros::Duration (0.1))) {
                                        //odom_delta代表在robot_odom_frame_id坐标系下（odom坐标系下），相邻时刻两个odom_child_frame_id的位姿变换关系
                                        odom_delta = tf_buffer.lookupTransform (odom_child_frame_id, last_correction_time, odom_child_frame_id, stamp, robot_odom_frame_id, ros::Duration (0));
                                }
                                //判断是否可以查询从last_correction_time时刻odom_child_frame_id到最新时刻odom_child_frame_id之间的坐标变换关系，时间容错0.1，参考座标系robot_odom_frame_id
                                else if (tf_buffer.canTransform (odom_child_frame_id, last_correction_time, odom_child_frame_id, ros::Time (0), robot_odom_frame_id, ros::Duration (0))) {
                                        odom_delta = tf_buffer.lookupTransform (odom_child_frame_id, last_correction_time, odom_child_frame_id, ros::Time (0), robot_odom_frame_id, ros::Duration (0));
                                }

                                if (odom_delta.header.stamp.isZero ()) {
                                        NODELET_WARN_STREAM ("failed to look up transform between " << cloud->header.frame_id << " and " << robot_odom_frame_id);
                                }
                                else {
                                        Eigen::Isometry3d delta = tf2::transformToEigen (odom_delta);
                                        pose_estimator->predict_odom (delta.cast<float> ().matrix ());
                                }
                        }

                        // correct
                        auto aligned = pose_estimator->correct (stamp, filtered);
                        if (registration->getFitnessScore () > 5.0) {
                                //   NODELET_INFO_STREAM("gg, localization fail");
                                std::cout << " match fail" << registration->getFitnessScore () << std::endl;
                        }

                        // if (aligned_pub.getNumSubscribers ()) {
                        //         aligned->header.frame_id = "map";
                        //         aligned->header.stamp = cloud->header.stamp;
                        //         aligned_pub.publish (aligned);
                        // }

                        if (status_pub.getNumSubscribers ()) {
                                publish_scan_matching_status (points_msg->header, aligned);
                        }

                        publish_odometry (points_msg->header.stamp, pose_estimator->matrix ());

                }
                else{
                        ROS_WARN("lidar type error,MID360 and AIRY supported");
                }
                /*原函数体
                std::lock_guard<std::mutex> estimator_lock (pose_estimator_mutex);
                if (!pose_estimator) {
                        NODELET_ERROR ("waiting for initial pose input!!");
                        return;
                }

                if (!globalmap) {
                        NODELET_ERROR ("globalmap has not been received!!");
                        return;
                }


                const auto& stamp = points_msg->header.stamp;
                pcl::PointCloud<PointT>::Ptr pcl_cloud (new pcl::PointCloud<PointT> ());
                pcl::fromROSMsg (*points_msg, *pcl_cloud);

                if (pcl_cloud->empty ()) {
                        NODELET_ERROR ("cloud is empty!!");
                        return;
                }
                // transform pointcloud into odom_child_frame_id
                // 将点云数据从雷达坐标系转到odom_child_frame_id坐标系
                pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT> ());
                if (!pcl_ros::transformPointCloud (odom_child_frame_id, *pcl_cloud, *cloud, this->tf_buffer)) {
                        NODELET_ERROR ("point cloud cannot be transformed into target frame!!");
                        return;
                }

                auto filtered = downsample (cloud);
                last_scan = filtered;

                if (relocalizing) {
                        delta_estimater->add_frame (filtered);
                }

                // NOTE 获取机器人位姿；开始时为初始位姿
                Eigen::Matrix4f before = pose_estimator->matrix ();
                // predict
                if (!use_imu) {
                        // NOTE 刚开始预测处于冻结状态，只依靠NDT及观测模型
                        pose_estimator->predict (stamp);
                }
                else {
                        std::lock_guard<std::mutex> lock (imu_data_mutex);
                        auto imu_iter = imu_data.begin ();
                        for (imu_iter; imu_iter != imu_data.end (); imu_iter++) {
                                if (stamp < (*imu_iter)->header.stamp) {
                                        break;
                                }
                                const auto& acc = (*imu_iter)->linear_acceleration;
                                const auto& gyro = (*imu_iter)->angular_velocity;
                                // std::cout << acc << std::endl;
                                // std::cout << gyro << std::endl;
                                double acc_sign = invert_acc ? -1.0 : 1.0;
                                double gyro_sign = invert_gyro ? -1.0 : 1.0;
                                // NODELET_INFO_STREAM("imu is used : " << gyro);
                                pose_estimator->predict ((*imu_iter)->header.stamp, acc_sign * Eigen::Vector3f (acc.x, acc.y, acc.z), gyro_sign * Eigen::Vector3f (gyro.x, gyro.y, gyro.z));
                        }
                        imu_data.erase (imu_data.begin (), imu_iter);
                }

                // odometry-based prediction
                ros::Time last_correction_time = pose_estimator->last_correction_time ();
                if (private_nh.param<bool> ("enable_robot_odometry_prediction", false) && !last_correction_time.isZero ()) {
                        geometry_msgs::TransformStamped odom_delta;
                        if (tf_buffer.canTransform (odom_child_frame_id, last_correction_time, odom_child_frame_id, stamp, robot_odom_frame_id, ros::Duration (0.1))) {
                                odom_delta = tf_buffer.lookupTransform (odom_child_frame_id, last_correction_time, odom_child_frame_id, stamp, robot_odom_frame_id, ros::Duration (0));
                        }
                        else if (tf_buffer.canTransform (odom_child_frame_id, last_correction_time, odom_child_frame_id, ros::Time (0), robot_odom_frame_id, ros::Duration (0))) {
                                odom_delta = tf_buffer.lookupTransform (odom_child_frame_id, last_correction_time, odom_child_frame_id, ros::Time (0), robot_odom_frame_id, ros::Duration (0));
                        }

                        if (odom_delta.header.stamp.isZero ()) {
                                NODELET_WARN_STREAM ("failed to look up transform between " << cloud->header.frame_id << " and " << robot_odom_frame_id);
                        }
                        else {
                                Eigen::Isometry3d delta = tf2::transformToEigen (odom_delta);
                                pose_estimator->predict_odom (delta.cast<float> ().matrix ());
                        }
                }

                // correct
                auto aligned = pose_estimator->correct (stamp, filtered);
                if (registration->getFitnessScore () > 2.0) {
                        //   NODELET_INFO_STREAM("gg, localization fail");
                        std::cout << " match fail" << registration->getFitnessScore () << std::endl;
                }

                if (aligned_pub.getNumSubscribers ()) {
                        aligned->header.frame_id = "map1";
                        aligned->header.stamp = cloud->header.stamp;
                        aligned_pub.publish (aligned);
                }

                if (status_pub.getNumSubscribers ()) {
                        publish_scan_matching_status (points_msg->header, aligned);
                }

                publish_odometry (points_msg->header.stamp, pose_estimator->matrix ());
                */
        }


        /**
         * @brief callback for globalmap input  将地图点云压入匹配器的目标点云，并将地图点云发送到全局定位服务器
         * @param points_msg
         */
        void globalmap_callback (const sensor_msgs::PointCloud2ConstPtr& points_msg) {
                NODELET_INFO ("globalmap received!");
                pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT> ());
                pcl::fromROSMsg (*points_msg, *cloud);
                globalmap = cloud;

                registration->setInputTarget (globalmap);

                if (use_global_localization) {
                        NODELET_INFO ("set globalmap for global localization!");
                        hdl_global_localization::SetGlobalMap srv;
                        pcl::toROSMsg (*globalmap, srv.request.global_map);

                        if (!set_global_map_service.call (srv)) {
                                NODELET_INFO ("failed to set global map");
                        }
                        else {
                                NODELET_INFO ("done");
                        }
                }
        }

        /**
         * @brief perform global localization to relocalize the sensor position
         * @param
         */
        bool relocalize (std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res) {
                if (last_scan == nullptr) {
                        NODELET_INFO_STREAM ("no scan has been received");
                        return false;
                }

                relocalizing = true;
                delta_estimater->reset ();
                pcl::PointCloud<PointT>::ConstPtr scan = last_scan;

                hdl_global_localization::QueryGlobalLocalization srv;
                pcl::toROSMsg (*scan, srv.request.cloud);
                srv.request.max_num_candidates = 1;

                if (!query_global_localization_service.call (srv) || srv.response.poses.empty ()) {
                        relocalizing = false;
                        NODELET_INFO_STREAM ("global localization failed");
                        return false;
                }

                const auto& result = srv.response.poses [0];

                NODELET_INFO_STREAM ("--- Global localization result ---");
                NODELET_INFO_STREAM ("Trans :" << result.position.x << " " << result.position.y << " " << result.position.z);
                NODELET_INFO_STREAM ("Quat  :" << result.orientation.x << " " << result.orientation.y << " " << result.orientation.z << " " << result.orientation.w);
                NODELET_INFO_STREAM ("Error :" << srv.response.errors [0]);
                NODELET_INFO_STREAM ("Inlier:" << srv.response.inlier_fractions [0]);

                Eigen::Isometry3f pose = Eigen::Isometry3f::Identity ();
                pose.linear () = Eigen::Quaternionf (result.orientation.w, result.orientation.x, result.orientation.y, result.orientation.z).toRotationMatrix ();
                pose.translation () = Eigen::Vector3f (result.position.x, result.position.y, result.position.z);
                pose = pose * delta_estimater->estimated_delta ();

                std::lock_guard<std::mutex> lock (pose_estimator_mutex);
                pose_estimator.reset (new hdl_localization::PoseEstimator (
                        registration,
                        ros::Time::now (),
                        pose.translation (),
                        Eigen::Quaternionf (pose.linear ()),
                        private_nh.param<double> ("cool_time_duration", 0.5)));

                relocalizing = false;

                return true;
        }

        /**
         * @brief callback for initial pose input ("2D Pose Estimate" on rviz)
         * @param pose_msg
         */
        void initialpose_callback (const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg) {
                NODELET_INFO ("initial pose received!!");
                std::lock_guard<std::mutex> lock (pose_estimator_mutex);
                const auto& p = pose_msg->pose.pose.position;
                const auto& q = pose_msg->pose.pose.orientation;
                pose_estimator.reset (new hdl_localization::PoseEstimator (
                        registration,
                        ros::Time::now (),
                        Eigen::Vector3f (p.x, p.y, p.z),
                        Eigen::Quaternionf (q.w, q.x, q.y, q.z),
                        private_nh.param<double> ("cool_time_duration", 0.5)));
        }

        /**
         * @brief downsampling
         * @param cloud   input cloud
         * @return downsampled cloud
         */
        pcl::PointCloud<PointT>::ConstPtr downsample (const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
                if (!downsample_filter) {
                        return cloud;
                }

                pcl::PointCloud<PointT>::Ptr filtered (new pcl::PointCloud<PointT> ());
                downsample_filter->setInputCloud (cloud);
                downsample_filter->filter (*filtered);
                filtered->header = cloud->header;

                return filtered;
        }


        /**
         * @brief publish odometry
         * @param stamp  timestamp
         * @param pose   odometry pose to be published
         */
        void publish_odometry (const ros::Time& stamp, const Eigen::Matrix4f& pose) {
                // broadcast the transform over tf
                if (tf_buffer.canTransform (robot_odom_frame_id, odom_child_frame_id, ros::Time (0))) {
                        geometry_msgs::TransformStamped map_wrt_frame = tf2::eigenToTransform (Eigen::Isometry3d (pose.inverse ().cast<double> ()));
                        map_wrt_frame.header.stamp = stamp;
                        map_wrt_frame.header.frame_id = odom_child_frame_id;
                        map_wrt_frame.child_frame_id = "map";  // 1126

                        geometry_msgs::TransformStamped frame_wrt_odom = tf_buffer.lookupTransform (robot_odom_frame_id, odom_child_frame_id, ros::Time (0), ros::Duration (0.1));
                        Eigen::Matrix4f frame2odom = tf2::transformToEigen (frame_wrt_odom).cast<float> ().matrix ();

                        geometry_msgs::TransformStamped map_wrt_odom;//在odom下map的位姿
                        tf2::doTransform (map_wrt_frame, map_wrt_odom, frame_wrt_odom);

                        tf2::Transform odom_wrt_map;//在map下odom的位姿
                        tf2::fromMsg (map_wrt_odom.transform, odom_wrt_map);
                        odom_wrt_map = odom_wrt_map.inverse ();

                        geometry_msgs::TransformStamped odom_trans;
                        odom_trans.transform = tf2::toMsg (odom_wrt_map);
                        odom_trans.header.stamp = stamp;
                        odom_trans.header.frame_id = "map";
                        odom_trans.child_frame_id = robot_odom_frame_id;
                        //发布map下odom的位姿  发布tf
                        tf_broadcaster.sendTransform (odom_trans);
                          ROS_INFO("test_brunch");//实际走的是这个分支

                }
                else {
                        geometry_msgs::TransformStamped odom_trans = tf2::eigenToTransform (Eigen::Isometry3d (pose.cast<double> ()));
                        odom_trans.header.stamp = stamp;
                        odom_trans.header.frame_id = "map";
                        odom_trans.child_frame_id = odom_child_frame_id;  // lidar_link
                        tf_broadcaster.sendTransform (odom_trans);
                        // ROS_INFO("test_brunch");//实际走的是这个分支
                }

                // publish the transform
                //发布odom，实际是lidar在map下的位置
                nav_msgs::Odometry odom;
                odom.header.stamp = stamp;
                odom.header.frame_id = "map";

                tf::poseEigenToMsg (Eigen::Isometry3d (pose.cast<double> ()), odom.pose.pose);
                odom.child_frame_id = odom_child_frame_id;  // velodyne
                odom.twist.twist.linear.x = 0.0; //线速度的x分量
                odom.twist.twist.linear.y = 0.0; //线速度的y分量
                odom.twist.twist.angular.z = 0.0;//角速度的z分量

                pose_pub.publish (odom);
                fprintf (this->pose_file, "%lf %lf %lf %lf %lf %lf %lf\n", odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z, odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z);
        }

        /**
         * @brief publish scan matching status information
         */
        void publish_scan_matching_status (const std_msgs::Header& header, pcl::PointCloud<pcl::PointXYZI>::ConstPtr aligned) {
                ScanMatchingStatus status;
                status.header = header;

                status.has_converged = registration->hasConverged ();
                status.matching_error = registration->getFitnessScore ();

                const double max_correspondence_dist = 0.5;

                int num_inliers = 0;
                std::vector<int> k_indices;
                std::vector<float> k_sq_dists;
                for (int i = 0; i < aligned->size (); i++) {
                        const auto& pt = aligned->at (i);
                        registration->getSearchMethodTarget ()->nearestKSearch (pt, 1, k_indices, k_sq_dists);
                        if (k_sq_dists [0] < max_correspondence_dist * max_correspondence_dist) {
                                num_inliers++;
                        }
                }
                status.inlier_fraction = static_cast<float>(num_inliers) / aligned->size ();
                status.relative_pose = tf2::eigenToTransform (Eigen::Isometry3d (registration->getFinalTransformation ().cast<double> ())).transform;

                status.prediction_labels.reserve (2);
                status.prediction_errors.reserve (2);

                std::vector<double> errors (6, 0.0);

                if (pose_estimator->wo_prediction_error ()) {
                        status.prediction_labels.push_back (std_msgs::String ());
                        status.prediction_labels.back ().data = "without_pred";
                        status.prediction_errors.push_back (tf2::eigenToTransform (Eigen::Isometry3d (pose_estimator->wo_prediction_error ().get ().cast<double> ())).transform);
                }

                if (pose_estimator->imu_prediction_error ()) {
                        status.prediction_labels.push_back (std_msgs::String ());
                        status.prediction_labels.back ().data = use_imu ? "imu" : "motion_model";
                        status.prediction_errors.push_back (tf2::eigenToTransform (Eigen::Isometry3d (pose_estimator->imu_prediction_error ().get ().cast<double> ())).transform);
                }

                if (pose_estimator->odom_prediction_error ()) {
                        status.prediction_labels.push_back (std_msgs::String ());
                        status.prediction_labels.back ().data = "odom";
                        status.prediction_errors.push_back (tf2::eigenToTransform (Eigen::Isometry3d (pose_estimator->odom_prediction_error ().get ().cast<double> ())).transform);
                }

                status_pub.publish (status);
        }

        private:
        // ROS
        ros::NodeHandle nh;
        ros::NodeHandle mt_nh;
        ros::NodeHandle private_nh;

        std::string robot_odom_frame_id;
        std::string odom_child_frame_id;
        std::string lidar_type; //自定义参数：雷达类型
        int point_filter_num;//自定义参数：点数滤波参数
        double blind_min;//自定义参数：半径滤波参数
        double blind_max;//自定义参数：半径滤波参数
        double z_filter_min;//自定义参数：发布的点云Z轴滤波范围min
        double z_filter_max;//自定义参数：发布的点云Z轴滤波范围max

        bool use_imu;
        bool invert_acc;
        bool invert_gyro;
        ros::Subscriber imu_sub;
        ros::Subscriber points_sub;
        ros::Subscriber globalmap_sub;
        ros::Subscriber initialpose_sub;

        ros::Publisher pose_pub;
        ros::Publisher aligned_pub;
        ros::Publisher status_pub;
        ros::Subscriber legodom_sub;

          ros::Publisher my_pointpub;

        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener tf_listener;
        tf2_ros::TransformBroadcaster tf_broadcaster;

        // imu input buffer
        std::mutex imu_data_mutex;
        std::vector<sensor_msgs::ImuConstPtr> imu_data;

        // globalmap and registration method
        pcl::PointCloud<PointT>::Ptr globalmap;
        pcl::Filter<PointT>::Ptr downsample_filter;
        pcl::Registration<PointT, PointT>::Ptr registration;

        // pose estimator
        std::mutex pose_estimator_mutex;
        std::unique_ptr<hdl_localization::PoseEstimator> pose_estimator;

        // global localization
        bool use_global_localization;
        std::atomic_bool relocalizing;
        std::unique_ptr<DeltaEstimater> delta_estimater;

        pcl::PointCloud<PointT>::ConstPtr last_scan;
        ros::ServiceServer relocalize_server;
        ros::ServiceClient set_global_map_service;
        ros::ServiceClient query_global_localization_service;
        ros::ServiceServer start_recolize_service;  // extra add
        int use_legodom = 0;
};
}  // namespace hdl_localization

PLUGINLIB_EXPORT_CLASS (hdl_localization::HdlLocalizationNodelet, nodelet::Nodelet)
