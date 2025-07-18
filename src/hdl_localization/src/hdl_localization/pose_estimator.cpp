#include <hdl_localization/pose_estimator.hpp>

#include <pcl/filters/voxel_grid.h>
#include <hdl_localization/pose_system.hpp>
#include <hdl_localization/odom_system.hpp>
#include <kkl/alg/unscented_kalman_filter.hpp>

namespace hdl_localization {

/**
 * @brief constructor
 * @param registration        registration method
 * @param stamp               timestamp ---> ROS::now()
 * @param pos                 initial position ----> rviz下发的起始位置
 * @param quat                initial orientation ----> rviz下发的起始姿态
 * @param cool_time_duration  during "cool time", prediction is not performed  ---> 位姿预测冻结时间，即开始不执行位姿预测的时间，先通过NDT配准初始位姿
 */
PoseEstimator::PoseEstimator (pcl::Registration<PointT, PointT>::Ptr& registration, const ros::Time& stamp,
                                                        const Eigen::Vector3f& pos,
                                                        const Eigen::Quaternionf& quat,
                                                        double cool_time_duration)
        : init_stamp (stamp),
        registration (registration),
        cool_time_duration (cool_time_duration) {

        // NOTE 上一帧NDT匹配后的位姿，刚开始为初始位姿；后面为上一帧位姿观测值
        last_observation = Eigen::Matrix4f::Identity ();
        last_observation.block<3, 3> (0, 0) = quat.toRotationMatrix ();
        last_observation.block<3, 1> (0, 3) = pos;

        // NOTE 过程噪声，预测过程会有偏差，引入该噪声
        process_noise = Eigen::MatrixXf::Identity (16, 16);
        process_noise.middleRows (0, 3) *= 1.0;   // 位置
        process_noise.middleRows (3, 3) *= 1.0;   //  速度
        process_noise.middleRows (6, 4) *= 0.5;   //  旋转
        process_noise.middleRows (10, 3) *= 1e-6;  // 陀螺仪偏差
        process_noise.middleRows (13, 3) *= 1e-6;   // 加速度计偏差

        // NOTE 测量噪声
        Eigen::MatrixXf measurement_noise = Eigen::MatrixXf::Identity (7, 7);
        measurement_noise.middleRows (0, 3) *= 0.01;   // 位置
        measurement_noise.middleRows (3, 4) *= 0.001;   // 旋转

        // NOTE  初始化滤波状态空间；位置3*3；速度3*3；旋转4*4；偏差Bais  6*6
        Eigen::VectorXf mean (16);
        mean.middleRows (0, 3) = pos;
        mean.middleRows (3, 3).setZero ();
        mean.middleRows (6, 4) = Eigen::Vector4f (quat.w (), quat.x (), quat.y (), quat.z ());
        mean.middleRows (10, 3).setZero ();
        mean.middleRows (13, 3).setZero ();

        // NOTE 初始化滤波状态空间协方差，对角线均为0.01
        Eigen::MatrixXf cov = Eigen::MatrixXf::Identity (16, 16) * 0.01;

        PoseSystem system;
        /**
         * @brief constructor
         * @param system                            状态估计系统
         * @param state_dim                       状态空间维度
         * @param input_dim                       没用到
         * @param measurement_dim       测量空间维度
         * @param process_noise               过程噪声
         * @param measurement_noise    测量噪声
         * @param mean                               初始状态空间向量
         * @param cov                                   初始状态空间协方差
         */
        ukf.reset (new kkl::alg::UnscentedKalmanFilterX<float, PoseSystem> (system, 16, 6, 7, process_noise, measurement_noise, mean, cov));
}

PoseEstimator::~PoseEstimator () {}

/**
 * @brief predict
 * @param stamp    timestamp
 * @param acc      acceleration
 * @param gyro     angular velocity
 */
void PoseEstimator::predict (const ros::Time& stamp) {  //  cool_time_duration 0.5  ---> 在距离开始“cool_time_duration”期间，不执行预测，先通过NDT配准初始位姿
        if ((stamp - init_stamp).toSec () < cool_time_duration || prev_stamp.is_zero () || prev_stamp == stamp) {
                prev_stamp = stamp;
                return;
        }

        double dt = (stamp - prev_stamp).toSec ();
        prev_stamp = stamp;

        ukf->setProcessNoiseCov (process_noise * dt);
        ukf->system.dt = dt;

        ukf->predict ();
}

/**
 * @brief predict
 * @param stamp    timestamp
 * @param acc      acceleration
 * @param gyro     angular velocity
 */
void PoseEstimator::predict (const ros::Time& stamp, const Eigen::Vector3f& acc, const Eigen::Vector3f& gyro) {
        if ((stamp - init_stamp).toSec () < cool_time_duration || prev_stamp.is_zero () || prev_stamp == stamp) {
                prev_stamp = stamp;
                return;
        }
        double dt = (stamp - prev_stamp).toSec ();
        prev_stamp = stamp;
        ukf->setProcessNoiseCov (process_noise * dt);
        ukf->system.dt = dt;

        Eigen::VectorXf control (6);
        control.head<3> () = acc;
        control.tail<3> () = gyro;
        ukf->predict (control);
}

/**
 * @brief update the state of the odomety-based pose estimation
 */
void PoseEstimator::predict_odom (const Eigen::Matrix4f& odom_delta) {
        if (!odom_ukf) {
                Eigen::MatrixXf odom_process_noise = Eigen::MatrixXf::Identity (7, 7);
                Eigen::MatrixXf odom_measurement_noise = Eigen::MatrixXf::Identity (7, 7) * 1e-3;

                Eigen::VectorXf odom_mean (7);
                odom_mean.block<3, 1> (0, 0) = Eigen::Vector3f (ukf->mean [0], ukf->mean [1], ukf->mean [2]);
                odom_mean.block<4, 1> (3, 0) = Eigen::Vector4f (ukf->mean [6], ukf->mean [7], ukf->mean [8], ukf->mean [9]);
                Eigen::MatrixXf odom_cov = Eigen::MatrixXf::Identity (7, 7) * 1e-2;

                OdomSystem odom_system;
                odom_ukf.reset (new kkl::alg::UnscentedKalmanFilterX<float, OdomSystem> (odom_system, 7, 7, 7, odom_process_noise, odom_measurement_noise, odom_mean, odom_cov));
        }

        // invert quaternion if the rotation axis is flipped
        Eigen::Quaternionf quat (odom_delta.block<3, 3> (0, 0));
        if (odom_quat ().coeffs ().dot (quat.coeffs ()) < 0.0) {
                quat.coeffs () *= -1.0f;
        }

        Eigen::VectorXf control (7);
        control.middleRows (0, 3) = odom_delta.block<3, 1> (0, 3);
        control.middleRows (3, 4) = Eigen::Vector4f (quat.w (), quat.x (), quat.y (), quat.z ());

        Eigen::MatrixXf process_noise = Eigen::MatrixXf::Identity (7, 7);
        process_noise.topLeftCorner (3, 3) = Eigen::Matrix3f::Identity () * odom_delta.block<3, 1> (0, 3).norm () + Eigen::Matrix3f::Identity () * 1e-3;
        process_noise.bottomRightCorner (4, 4) = Eigen::Matrix4f::Identity () * (1 - std::abs (quat.w ())) + Eigen::Matrix4f::Identity () * 1e-3;

        odom_ukf->setProcessNoiseCov (process_noise);
        odom_ukf->predict (control);
}

/**
 * @brief correct
 * @param cloud   input cloud
 * @return cloud aligned to the globalmap
 */
pcl::PointCloud<PoseEstimator::PointT>::Ptr PoseEstimator::correct (const ros::Time& stamp, const pcl::PointCloud<PointT>::ConstPtr& cloud) {
        last_correction_stamp = stamp;

        Eigen::Matrix4f no_guess = last_observation;
        Eigen::Matrix4f imu_guess;
        Eigen::Matrix4f odom_guess;
        Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity ();

        if (!odom_ukf) {
                // NOTE predice的mean值
                init_guess = imu_guess = matrix ();
        }
        else {
                imu_guess = matrix ();
                odom_guess = odom_matrix ();

                Eigen::VectorXf imu_mean (7);
                Eigen::MatrixXf imu_cov = Eigen::MatrixXf::Identity (7, 7);
                imu_mean.block<3, 1> (0, 0) = ukf->mean.block<3, 1> (0, 0);
                imu_mean.block<4, 1> (3, 0) = ukf->mean.block<4, 1> (6, 0);

                imu_cov.block<3, 3> (0, 0) = ukf->cov.block<3, 3> (0, 0);
                imu_cov.block<3, 4> (0, 3) = ukf->cov.block<3, 4> (0, 6);
                imu_cov.block<4, 3> (3, 0) = ukf->cov.block<4, 3> (6, 0);
                imu_cov.block<4, 4> (3, 3) = ukf->cov.block<4, 4> (6, 6);

                Eigen::VectorXf odom_mean = odom_ukf->mean;
                Eigen::MatrixXf odom_cov = odom_ukf->cov;

                if (imu_mean.tail<4> ().dot (odom_mean.tail<4> ()) < 0.0) {
                        odom_mean.tail<4> () *= -1.0;
                }

                Eigen::MatrixXf inv_imu_cov = imu_cov.inverse ();
                Eigen::MatrixXf inv_odom_cov = odom_cov.inverse ();

                Eigen::MatrixXf fused_cov = (inv_imu_cov + inv_odom_cov).inverse ();
                Eigen::VectorXf fused_mean = fused_cov * inv_imu_cov * imu_mean + fused_cov * inv_odom_cov * odom_mean;

                init_guess.block<3, 1> (0, 3) = Eigen::Vector3f (fused_mean [0], fused_mean [1], fused_mean [2]);
                init_guess.block<3, 3> (0, 0) = Eigen::Quaternionf (fused_mean [3], fused_mean [4], fused_mean [5], fused_mean [6]).normalized ().toRotationMatrix ();
        }

        pcl::PointCloud<PointT>::Ptr aligned (new pcl::PointCloud<PointT> ());
        registration->setInputSource (cloud);
        registration->align (*aligned, init_guess);

        Eigen::Matrix4f trans = registration->getFinalTransformation ();
        auto error_value = registration->getFitnessScore ();
        if (error_value > 5.0) {
                if((this_pose-last_pose).norm()>3 || error_value > 5.0){
                        exception_flag = true;
                        std::cout<<"Localization error, stop immediately!!!!!!"<<std::endl;
                }
                else{
                        exception_flag = false;
                }
                std::cout << "fitness score" << error_value << std::endl;
                if(legodom_buffer.size()>20){
                        std::cout<<"relocalization using leg_odom"<<std::endl;
                        std::lock_guard<std::mutex> lock (leg_odom_mutex);
                        LegOdomPtr cur_element = legodom_buffer.back();
                        LegOdomPtr last_element = legodom_buffer[legodom_buffer.size() - 20];
                        Eigen::Vector3f relative_pos = cur_element->pos - last_element->pos;
                        Eigen::Matrix3f relative_rot = last_element->q.toRotationMatrix().transpose() * cur_element->q.toRotationMatrix();
                        Eigen::Matrix4f relative_mat = Eigen::Matrix4f::Identity();
                        relative_mat.block<3, 3> (0, 0) = relative_rot;
                        relative_mat.block<3, 1> (0, 3) = relative_pos;
                        Eigen::Matrix4f trans_leg = temp_stable_state * relative_mat;
                        Eigen::Vector3f p = trans_leg.block<3, 1> (0, 3);
                        Eigen::Quaternionf q (trans_leg.block<3, 3> (0, 0));
                        if (quat ().coeffs ().dot (q.coeffs ()) < 0.0f) {
                                q.coeffs () *= -1.0f;
                        }
                        Eigen::VectorXf observation (7);
                        observation.middleRows (0, 3) = p;
                        observation.middleRows (3, 4) = Eigen::Vector4f (q.w (), q.x (), q.y (), q.z ());
                        last_observation = trans_leg;
                        ukf->correct (observation);   
                        temp_stable_state = this->matrix (); 

                }
                else{
                        std::cout<<"relocalization using static align"<<std::endl;
                        Eigen::Vector3f p = temp_stable_state.block<3, 1> (0, 3);
                        Eigen::Quaternionf q (temp_stable_state.block<3, 3> (0, 0));
                        if (quat ().coeffs ().dot (q.coeffs ()) < 0.0f) {
                                q.coeffs () *= -1.0f;
                        }
                        Eigen::VectorXf observation (7);
                        observation.middleRows (0, 3) = p;
                        observation.middleRows (3, 4) = Eigen::Vector4f (q.w (), q.x (), q.y (), q.z ());
                        last_observation = temp_stable_state;
                        ukf->correct (observation);
                }
        }
        else{
                Eigen::Vector3f p = trans.block<3, 1> (0, 3);
                Eigen::Quaternionf q (trans.block<3, 3> (0, 0));
                if (quat ().coeffs ().dot (q.coeffs ()) < 0.0f) {
                        q.coeffs () *= -1.0f;
                }

                Eigen::VectorXf observation (7);
                observation.middleRows (0, 3) = p;
                observation.middleRows (3, 4) = Eigen::Vector4f (q.w (), q.x (), q.y (), q.z ());
                last_observation = trans;
                //
                auto result = registration->getFinalTransformation ();
                // NOTE 上一帧预测与当前帧NDT匹配之后的位姿增量
                wo_pred_error = no_guess.inverse () * result;

                ukf->correct (observation);
                imu_pred_error = imu_guess.inverse () * result;
                if (odom_ukf) {
                        if (observation.tail<4> ().dot (odom_ukf->mean.tail<4> ()) < 0.0) {
                                odom_ukf->mean.tail<4> () *= -1.0;
                        }

                        odom_ukf->correct (observation);
                        odom_pred_error = odom_guess.inverse () * result;
                }
                temp_stable_state = this->matrix ();
                exception_flag = false;
        }
        last_pose = this_pose;
        this_pose = this->pos();
        return aligned;
}

/* getters */
ros::Time PoseEstimator::last_correction_time () const {
        return last_correction_stamp;
}

Eigen::Vector3f PoseEstimator::pos () const {
        return Eigen::Vector3f (ukf->mean [0], ukf->mean [1], ukf->mean [2]);
}

Eigen::Vector3f PoseEstimator::vel () const {
        return Eigen::Vector3f (ukf->mean [3], ukf->mean [4], ukf->mean [5]);
}

Eigen::Quaternionf PoseEstimator::quat () const {
        return Eigen::Quaternionf (ukf->mean [6], ukf->mean [7], ukf->mean [8], ukf->mean [9]).normalized ();
}

Eigen::Matrix4f PoseEstimator::matrix () const {
        Eigen::Matrix4f m = Eigen::Matrix4f::Identity ();
        m.block<3, 3> (0, 0) = quat ().toRotationMatrix ();
        m.block<3, 1> (0, 3) = pos ();
        return m;
}

Eigen::Vector3f PoseEstimator::odom_pos () const {
        return Eigen::Vector3f (odom_ukf->mean [0], odom_ukf->mean [1], odom_ukf->mean [2]);
}

Eigen::Quaternionf PoseEstimator::odom_quat () const {
        return Eigen::Quaternionf (odom_ukf->mean [3], odom_ukf->mean [4], odom_ukf->mean [5], odom_ukf->mean [6]).normalized ();
}

Eigen::Matrix4f PoseEstimator::odom_matrix () const {
        Eigen::Matrix4f m = Eigen::Matrix4f::Identity ();
        m.block<3, 3> (0, 0) = odom_quat ().toRotationMatrix ();
        m.block<3, 1> (0, 3) = odom_pos ();
        return m;
}

const boost::optional<Eigen::Matrix4f>& PoseEstimator::wo_prediction_error () const {
        return wo_pred_error;
}

const boost::optional<Eigen::Matrix4f>& PoseEstimator::imu_prediction_error () const {
        return imu_pred_error;
}

const boost::optional<Eigen::Matrix4f>& PoseEstimator::odom_prediction_error () const {
        return odom_pred_error;
}
}  // namespace hdl_localization
