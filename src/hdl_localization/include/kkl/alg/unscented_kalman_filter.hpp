/**
 * UnscentedKalmanFilterX.hpp
 * @author koide
 * 16/02/01
 **/
#ifndef KKL_UNSCENTED_KALMAN_FILTER_X_HPP
#define KKL_UNSCENTED_KALMAN_FILTER_X_HPP

#include <random>
#include <Eigen/Dense>

namespace kkl {
namespace alg {

/**
 * @brief Unscented Kalman Filter class
 * @param T        scaler type
 * @param System   system class to be estimated
 */
template<typename T, class System>
class UnscentedKalmanFilterX {
        typedef Eigen::Matrix<T, Eigen::Dynamic, 1> VectorXt;
        typedef Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> MatrixXt;
        public:
        /**
         * @brief constructor
         * @param system               system to be estimated
         * @param state_dim            state vector dimension
         * @param input_dim            input vector dimension
         * @param measurement_dim      measurement vector dimension
         * @param process_noise        process noise covariance (state_dim * state_dim)
         * @param measurement_noise    measurement noise covariance (measurement_dim * measuremend_dim)
         * @param mean                 initial mean
         * @param cov                  initial covariance
         */
        UnscentedKalmanFilterX (const System& system, int state_dim, int input_dim, int measurement_dim, const MatrixXt& process_noise, const MatrixXt& measurement_noise, const VectorXt& mean, const MatrixXt& cov)
                : state_dim (state_dim),
                input_dim (input_dim),
                measurement_dim (measurement_dim),  //  测量空间维度measurement_dim
                N (state_dim),  //  状态空间维度N
                M (input_dim),
                K (measurement_dim),   //  测量空间维度K
                S (2 * state_dim + 1),   //  sigma点个数
                mean (mean),
                cov (cov),
                system (system),
                process_noise (process_noise),
                measurement_noise (measurement_noise),
                lambda (1),
                normal_dist (0.0, 1.0)
        {
                weights.resize (S, 1);

                // sigma_point状态空间为S*N维；其中S表示sigma点的个数，N表示每个sigma点的维度
                sigma_points.resize (S, N);
                ext_weights.resize (2 * (N + K) + 1, 1);
                ext_sigma_points.resize (2 * (N + K) + 1, N + K);
                expected_measurements.resize (2 * (N + K) + 1, K);

                // 初始化UKF权重：不变的
                weights [0] = lambda / (N + lambda);
                for (int i = 1; i < 2 * N + 1; i++) {
                        weights [i] = 1 / (2 * (N + lambda));
                }

                // weights for extended state space which includes error variances（包含误差方差的状态空间的权重）
                ext_weights [0] = lambda / (N + K + lambda);
                for (int i = 1; i < 2 * (N + K) + 1; i++) {
                        ext_weights [i] = 1 / (2 * (N + K + lambda));
                }
        }

        /**
         * @brief predict
         * @param control  input vector
         */
        void predict () {
                ensurePositiveFinite (cov);
                computeSigmaPoints (mean, cov, sigma_points);
                for (int i = 0; i < S; i++) {
                        // 把所有单个Sigma点代入非线性函数f（只根据匀速模型预测一下下一时刻的位置）
                        sigma_points.row (i) = system.f (sigma_points.row (i));
                }

                const auto& R = process_noise;

                // unscented transform
                VectorXt mean_pred (mean.size ());
                MatrixXt cov_pred (cov.rows (), cov.cols ());

                mean_pred.setZero ();
                cov_pred.setZero ();
                for (int i = 0; i < S; i++) {
                        mean_pred += weights [i] * sigma_points.row (i);
                }
                for (int i = 0; i < S; i++) {
                        VectorXt diff = sigma_points.row (i).transpose () - mean_pred;
                        cov_pred += weights [i] * diff * diff.transpose ();
                }
                // NOTE 由于此时我们的不确定性增加了一些，因此我们必须考虑添加过程噪声R
                cov_pred += R;
                // NOTE 根据非线性加权和转换的sigma点，重新计算近似高斯分布的均值和协方差
                mean = mean_pred;
                cov = cov_pred;
        }

        /**
         * @brief predict
         * @param control  input vector
         */
        void predict (const VectorXt& control) {
                // calculate sigma points
                ensurePositiveFinite (cov);
                computeSigmaPoints (mean, cov, sigma_points);
                // 预测当前时刻对应的sigma点
                for (int i = 0; i < S; i++) {
                        sigma_points.row (i) = system.f (sigma_points.row (i), control);
                }

                const auto& R = process_noise;

                // unscented transform
                VectorXt mean_pred (mean.size ());
                MatrixXt cov_pred (cov.rows (), cov.cols ());

                mean_pred.setZero ();
                cov_pred.setZero ();
                for (int i = 0; i < S; i++) {
                        mean_pred += weights [i] * sigma_points.row (i);
                }
                for (int i = 0; i < S; i++) {
                        VectorXt diff = sigma_points.row (i).transpose () - mean_pred;
                        cov_pred += weights [i] * diff * diff.transpose ();
                }
                cov_pred += R;

                mean = mean_pred;
                cov = cov_pred;
        }

        /**
         * @brief correct
         * @param measurement  measurement vector：NDT匹配后的位姿测量值三维位置、四维旋转
         */
        void correct (const VectorXt& measurement) {
                // create extended state space which includes error variances
                VectorXt ext_mean_pred = VectorXt::Zero (N + K, 1);
                MatrixXt ext_cov_pred = MatrixXt::Zero (N + K, N + K);
                ext_mean_pred.topLeftCorner (N, 1) = VectorXt (mean);
                ext_cov_pred.topLeftCorner (N, N) = MatrixXt (cov);
                ext_cov_pred.bottomRightCorner (K, K) = measurement_noise;

                ensurePositiveFinite (ext_cov_pred);
                // 根据观测噪声协协方差(measurement_noise)生成对应的sigma观测点，通过正定矩阵的LLT分解完成矩阵形式的开方
                computeSigmaPoints (ext_mean_pred, ext_cov_pred, ext_sigma_points);

                // unscented transform
                expected_measurements.setZero ();
                // 7维状态观测量+7维观测噪声
                // NOTE 通过对系统状态的均值和协方差进行线性变换得到的。
                for (int i = 0; i < ext_sigma_points.rows (); i++) {
                        // NOTE h为观测的非线性变换，sigma点状态空间均值对应的位姿+sigma点状态空间测量噪声引起的位姿
                        // NOTE  z = h(x)+v     v---->测量噪声（基于测量噪声预测出来的sigma噪声点）
                        expected_measurements.row (i) = system.h (ext_sigma_points.row (i).transpose ().topLeftCorner (N, 1));
                        expected_measurements.row (i) += VectorXt (ext_sigma_points.row (i).transpose ().bottomRightCorner (K, 1));
                }

                // 根据权重计算sigma点后均值-预测得到的均值
                VectorXt expected_measurement_mean = VectorXt::Zero (K);
                for (int i = 0; i < ext_sigma_points.rows (); i++) {
                        expected_measurement_mean += ext_weights [i] * expected_measurements.row (i);
                }
                // 根据权重加入观测噪声后sigma点的协方差
                MatrixXt expected_measurement_cov = MatrixXt::Zero (K, K);
                for (int i = 0; i < ext_sigma_points.rows (); i++) {
                        VectorXt diff = expected_measurements.row (i).transpose () - expected_measurement_mean;
                        expected_measurement_cov += ext_weights [i] * diff * diff.transpose ();
                }
                // calculated transformed covariance
                MatrixXt sigma = MatrixXt::Zero (N + K, K);
                for (int i = 0; i < ext_sigma_points.rows (); i++) {
                        // 计算sigma点后均值-预测得到的均值
                        // NOTE 状态空间与测量空间的关联方差
                        auto diffA = (ext_sigma_points.row (i).transpose () - ext_mean_pred);
                        auto diffB = (expected_measurements.row (i).transpose () - expected_measurement_mean);
                        sigma += ext_weights [i] * (diffA * diffB.transpose ());
                }

                kalman_gain = sigma * expected_measurement_cov.inverse ();
                const auto& K = kalman_gain;
                // NOTE measurement测量值
                // NOTE 优化后状态向量  = 预测值 + K(实际测量值 - 估计值)
                VectorXt ext_mean = ext_mean_pred + K * (measurement - expected_measurement_mean);
                MatrixXt ext_cov = ext_cov_pred - K * expected_measurement_cov * K.transpose ();

                mean = ext_mean.topLeftCorner (N, 1);
                cov = ext_cov.topLeftCorner (N, N);
        }

        /*			getter			*/
        const VectorXt& getMean () const { return mean; }
        const MatrixXt& getCov () const { return cov; }
        const MatrixXt& getSigmaPoints () const { return sigma_points; }

        System& getSystem () { return system; }
        const System& getSystem () const { return system; }
        const MatrixXt& getProcessNoiseCov () const { return process_noise; }
        const MatrixXt& getMeasurementNoiseCov () const { return measurement_noise; }

        const MatrixXt& getKalmanGain () const { return kalman_gain; }

        /*			setter			*/
        UnscentedKalmanFilterX& setMean (const VectorXt& m) { mean = m;			return *this; }
        UnscentedKalmanFilterX& setCov (const MatrixXt& s) { cov = s;			return *this; }

        UnscentedKalmanFilterX& setProcessNoiseCov (const MatrixXt& p) { process_noise = p;			return *this; }
        UnscentedKalmanFilterX& setMeasurementNoiseCov (const MatrixXt& m) { measurement_noise = m;	return *this; }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        private:
        const int state_dim;
        const int input_dim;
        const int measurement_dim;

        const int N;
        const int M;
        const int K;
        const int S;

        public:
        VectorXt mean;
        MatrixXt cov;

        System system;
        MatrixXt process_noise;		//
        MatrixXt measurement_noise;	//

        T lambda;
        VectorXt weights;

        MatrixXt sigma_points;

        VectorXt ext_weights;
        MatrixXt ext_sigma_points;
        MatrixXt expected_measurements;

        private:
        /**
         * @brief compute sigma points
         * @param mean          mean
         * @param cov           covariance
         * @param sigma_points  calculated sigma points
         */
        void computeSigmaPoints (const VectorXt& mean, const MatrixXt& cov, MatrixXt& sigma_points) {
                const int n = mean.size ();
                assert (cov.rows () == n && cov.cols () == n);

                // LLT分解是把一个对称正定的矩阵表示成一个下三角矩阵L和其转置的乘积的分解
                Eigen::LLT<MatrixXt> llt;
                llt.compute ((n + lambda) * cov);
                // 将矩阵分解成LLT形式，返回L矩阵，相当于开方操作
                MatrixXt l = llt.matrixL ();

                sigma_points.row (0) = mean;
                for (int i = 0; i < n; i++) {
                        sigma_points.row (1 + i * 2) = mean + l.col (i);
                        sigma_points.row (1 + i * 2 + 1) = mean - l.col (i);
                }
        }

        /**
         * @brief make covariance matrix positive finite
         * @param cov  covariance matrix
         */
        void ensurePositiveFinite (MatrixXt& cov) {
                return;
                const double eps = 1e-9;

                Eigen::EigenSolver<MatrixXt> solver (cov);
                // pseudoEigenvalueMatrix() 获取矩阵A的特征值矩阵，是一个特征值组成对角阵，除了对角线其余都是0
                MatrixXt D = solver.pseudoEigenvalueMatrix ();
                // pseudoEigenvectors() 获取特征向量的矩阵
                MatrixXt V = solver.pseudoEigenvectors ();
                for (int i = 0; i < D.rows (); i++) {
                        if (D (i, i) < eps) {
                                D (i, i) = eps;
                        }
                }
                // 理论上 cov和 V * D * V.inverse ()是一样的
                cov = V * D * V.inverse ();
        }

        public:
        MatrixXt kalman_gain;
        // std::mt19937 随机数生成器
        std::mt19937 mt;
        std::normal_distribution<T> normal_dist;
};

}
}


#endif
