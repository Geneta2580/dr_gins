#pragma once
#include "common/types.h"
#include "core/ins.h"
#include "common/earth.h"
#include "common/math_utils.h"
#include "log/state_logger.h"
#include "fstream"
#include "string"
#include <chrono> 

namespace dr_gins {

class ESKF {

public:
    ESKF();
    ~ESKF() = default;

    // 初始化滤波器
    bool Initialize(const NavState& initial_state, const Matrix<double, 18, 18>& initial_p, 
                    const Matrix3d& gyro_noise, const Matrix3d& accel_noise, const Vector6d& gnss_noise,
                    const Matrix3d& gyro_random_walk, const Matrix3d& accel_random_walk, int max_history_size);

    // IMU预测
    bool ProcessImu(const IMU& imu_data);

    // GNSS校正
    bool ProcessGnss(const GNSS& gnss_data);

    // 获取当前导航状态
    NavState GetNavState() const;

    // 获取最新的历史状态
    const HistoryState& GetLatestHistoryState() const {
        return history_buffer_.back();
    }

    // 检查历史状态是否为空
    bool IsHistoryEmpty() const {
        return history_buffer_.empty();
    }

private:

    // 传播误差状态协方差矩阵
    void PropagateState(const HistoryState& last_history, const IMU& current_imu, HistoryState& new_history);

    // ESKF通用更新过程
    void UpdateState(HistoryState& state_to_update, const Vector6d& residual, const Eigen::Matrix<double, 6, 6>& R);

    // 航向对齐
    bool AlignYawWithImu(const HistoryState& state_at_gnss, const GNSS& gnss_data);
    
    // 尝试进行GNSS更新
    void AttemptGnssUpdate();

    // 查找GNSS数据更新区间
    bool FindUpdateInterval(const GNSS& gnss_data, std::list<HistoryState>::iterator& iter_k);

    // 进行GNSS/IMU同步
    HistoryState SynchronousWithImu(const GNSS& gnss_data, std::list<HistoryState>::iterator iter_k);

    // GNSS的ESKF更新以及重传播
    void UpdateAndRepropagate(const std::list<HistoryState>::iterator& iter_k, const GNSS& gnss_data, HistoryState& state_at_gnss);

    // IMU插值
    void ImuInterpolate(const IMU& prev, const IMU& curr, double timestamp, IMU& output);

    // 状态变量缓存
    std::list<HistoryState> history_buffer_;
    std::list<IMU> imu_buffer_for_align_;
    std::list<GNSS> gnss_buffer_;
    int max_history_size_ = 200;

    // 需要足够的数据点来保证差分和SVD的稳定性
    int required_data_points_ = 6;
    std::vector<GNSS> align_gnss_buffer_;
    std::vector<HistoryState> align_state_buffer_;

    // IMU 偏置
    Vector3d gyro_bias_ = Vector3d::Zero();
    Vector3d accel_bias_ = Vector3d::Zero();

    // 噪声参数
    Matrix<double, 6, 6> imu_noise_cov_;  // 三轴加计、陀螺噪声
    Matrix<double, 6, 6> gnss_noise_cov_; // GNSS位置、速度噪声
    Matrix<double, 18, 18> initial_p_; // 初始状态协方差矩阵
    Matrix3d gyro_bias_random_walk_;  // 陀螺零偏随机游走噪声
    Matrix3d accel_bias_random_walk_; // 加速度计零偏随机游走噪声

    double last_imu_timestamp_ = -1.0;

    IMU last_imu_data_;

    // 标志
    bool initialized_ = false; // 初始化标志
    bool yaw_aligned_ = false; // 航向对齐标志
    bool first_imu_ = true;    // 第一帧IMU标志

    // 状态日志记录器
    std::unique_ptr<StateLogger> test_logger_;

    // 3dof滤波器用
    // void ProcessGravityResidual(NavState nominal_state_, IMU imu_data);
};

} // namespace dr_gins