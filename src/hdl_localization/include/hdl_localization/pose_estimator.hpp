#ifndef POSE_ESTIMATOR_HPP
#define POSE_ESTIMATOR_HPP

#include <memory>
#include <boost/optional.hpp>

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/registration.h>
#include <deque>
// extract add
// #include <hdl_global_localization/hdl_global_localization.hpp>

namespace kkl {
namespace alg {
template <typename T, class System>
class UnscentedKalmanFilterX;
}
}  // namespace kkl

namespace hdl_localization {

class PoseSystem;
class OdomSystem;

/**
 * @brief scan matching-based pose estimator
 */
class PoseEstimator {
        public:
        struct LegOdom{
                LegOdom() = default;
                LegOdom(double t_in,const Eigen::Vector3f& pos_in,const Eigen::Quaternionf &q_in){
                        timestamp = t_in;
                        pos = pos_in;
                        q = q_in;
                        twist_angular = Eigen::Vector3f::Zero();
                        twist_linear = Eigen::Vector3f::Zero();
                }
                double timestamp = 0.0; //单位秒
                Eigen::Vector3f pos;
                Eigen::Quaternionf q; 
                Eigen::Vector3f twist_linear;
                Eigen::Vector3f twist_angular;
        };
        struct ObtainedOdom{
                ObtainedOdom() = default;
                ObtainedOdom(double t_in,const Eigen::Vector3f& pos_in,const Eigen::Quaternionf &q_in,double & score_in){
                        timestamp = t_in;
                        pos = pos_in;
                        q = q_in;
                        score = score_in;
                }
                double timestamp;//单位秒
                Eigen::Vector3f pos;
                Eigen::Quaternionf q; 
                double score;
        };
        using LegOdomPtr = std::shared_ptr<LegOdom>;
        using ObtainedOdomPtr = std::shared_ptr<ObtainedOdom>;
        using PointT = pcl::PointXYZI;

        /**
         * @brief constructor
         * @param registration        registration method
         * @param stamp               timestamp
         * @param pos                 initial position
         * @param quat                initial orientation
         * @param cool_time_duration  during "cool time", prediction is not performed
         */
        PoseEstimator (
          pcl::Registration<PointT, PointT>::Ptr& registration,
          const ros::Time& stamp,
          const Eigen::Vector3f& pos,
          const Eigen::Quaternionf& quat,
          double cool_time_duration = 1.0);
        ~PoseEstimator ();

        /**
         * @brief predict
         * @param stamp    timestamp
         */
        void predict (const ros::Time& stamp);

        /**
         * @brief predict
         * @param stamp    timestamp
         * @param acc      acceleration
         * @param gyro     angular velocity
         */
        void predict (const ros::Time& stamp, const Eigen::Vector3f& acc, const Eigen::Vector3f& gyro);

        /**
         * @brief update the state of the odomety-based pose estimation
         */
        void predict_odom (const Eigen::Matrix4f& odom_delta);

        /**
         * @brief correct
         * @param cloud   input cloud
         * @return cloud aligned to the globalmap
         */
        //根据输入的点云进行校正步骤，并返回对齐到全局地图的点云
        pcl::PointCloud<PointT>::Ptr correct (const ros::Time& stamp, const pcl::PointCloud<PointT>::ConstPtr& cloud);

        /* getters */
        ros::Time last_correction_time () const;

        Eigen::Vector3f pos () const;
        Eigen::Vector3f vel () const;
        Eigen::Quaternionf quat () const;
        Eigen::Matrix4f matrix () const;

        Eigen::Vector3f odom_pos () const;
        Eigen::Quaternionf odom_quat () const;
        Eigen::Matrix4f odom_matrix () const;

        const boost::optional<Eigen::Matrix4f>& wo_prediction_error () const;
        const boost::optional<Eigen::Matrix4f>& imu_prediction_error () const;
        const boost::optional<Eigen::Matrix4f>& odom_prediction_error () const;

        private:
        ros::Time init_stamp;             // when the estimator was initialized//初始化时间
        ros::Time prev_stamp;             // when the estimator was updated last time//上一次更新时间
        ros::Time last_correction_stamp;  // when the estimator performed the correction step//上一次矫正时间
        double cool_time_duration;        //冷却时间

        Eigen::MatrixXf process_noise;    //过程噪声
        std::unique_ptr<kkl::alg::UnscentedKalmanFilterX<float, PoseSystem>> ukf;//UKF滤波器
        std::unique_ptr<kkl::alg::UnscentedKalmanFilterX<float, OdomSystem>> odom_ukf;//里程计UKF滤波器

        Eigen::Matrix4f last_observation;//上一次观测
        boost::optional<Eigen::Matrix4f> wo_pred_error;//无外力预测误差
        boost::optional<Eigen::Matrix4f> imu_pred_error;//IMU预测误差
        boost::optional<Eigen::Matrix4f> odom_pred_error;//里程计预测误差

        pcl::Registration<PointT, PointT>::Ptr registration;

        public:
        std::deque<LegOdomPtr> legodom_buffer;
        std::deque<ObtainedOdomPtr> obtained_buffer;
        double last_lidar_time;
        double lidar_time;
        // int use_legodom = 0;
        std::mutex leg_odom_mutex;
        Eigen::Matrix4f temp_stable_state;
};

}  // namespace hdl_localization

#endif  // POSE_ESTIMATOR_HPP
