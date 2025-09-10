#include "core/gins_eskf.h"

namespace dr_gins {

ESKF::ESKF() : test_logger_(std::make_unique<StateLogger>()) {}

bool ESKF::Initialize(const NavState& initial_state, const Matrix<double, 18, 18>& initial_p,
                      const Matrix3d& gyro_noise, const Matrix3d& accel_noise, const Vector6d& gnss_noise, 
                      const Matrix3d& gyro_bias_random_walk, const Matrix3d& accel_bias_random_walk,
                      int max_history_size) {

    // 使用传入的参数创建第一个历史状态，并放入缓存
    HistoryState initial_history;
    initial_history.timestamp = initial_state.timestamp;
    initial_history.state = initial_state;
    initial_p_ = initial_p; 
    initial_history.P = initial_p;
    initial_history.imu_data = IMU(); 
    history_buffer_.push_back(initial_history);

    // 设置IMU噪声协方差矩阵
    imu_noise_cov_.setZero();
    imu_noise_cov_.block<3, 3>(0, 0) = gyro_noise;
    imu_noise_cov_.block<3, 3>(3, 3) = accel_noise;

    // 设置GNSS噪声协方差矩阵
    gnss_noise_cov_ = gnss_noise.asDiagonal();

    // 设置IMU随机游走噪声协方差矩阵
    gyro_bias_random_walk_ = gyro_bias_random_walk;
    accel_bias_random_walk_ = accel_bias_random_walk;

    // 设置最大历史状态数量
    max_history_size_ = max_history_size;

    first_imu_ = true; // 标记需要用第一帧IMU数据更新初始状态
    ROS_INFO("System Parameters Initialized Successfully!");
    return true;
}

bool ESKF::ProcessImu(const IMU& imu_data) {
    // 第一帧IMU数据不进行状态传播，不发布
    if (first_imu_) {
        history_buffer_.front().imu_data = imu_data;
        first_imu_ = false;
        return false;
    }

    if (imu_data.dt <= 0) {
        ROS_WARN("IMU data received wrong timestamp, dt = %f", imu_data.dt);
        return false;
    }

    // IMU前向传播，包含积分和协方差矩阵传播
    HistoryState new_history;
    PropagateState(history_buffer_.back(), imu_data, new_history);

    // 历史状态放入缓存
    history_buffer_.push_back(new_history);

    // 限制历史缓存大小
    if (yaw_aligned_) {
        // 导航状态
        if (history_buffer_.size() >  max_history_size_) {
            history_buffer_.pop_front();
        }
    } else {
        // 对齐状态
        if (history_buffer_.size() > 3000) {
            ROS_WARN_THROTTLE(5.0, "Alignment is taking too long, history buffer reached safety limit. Discarding oldest state.");
            history_buffer_.pop_front();
        }
    }

    // 查找GNSS数据
    AttemptGnssUpdate();

    return yaw_aligned_;
}

bool ESKF::ProcessGnss(const GNSS& gnss_data) {
    // 将GNSS数据放入缓存，等待处理
    gnss_buffer_.push_back(gnss_data);

    return true;
}

void ESKF::AttemptGnssUpdate() {
    if (gnss_buffer_.empty() || history_buffer_.size() < 2) {
        return;
    }

    const auto& gnss_data = gnss_buffer_.front();
    const auto& latest_history = history_buffer_.back();
    const auto& oldest_history = history_buffer_.front();

    // 检查GNSS数据是否太老
    if (gnss_data.timestamp < oldest_history.timestamp) {
        ROS_WARN("GNSS data is too old, discarding. GNSS: %.4f, Oldest History: %.4f", 
                 gnss_data.timestamp, oldest_history.timestamp);
        gnss_buffer_.pop_front(); // 数据太老，直接丢弃并返回
        return;
    }

    // 检查GNSS数据是否在未来（相对于IMU缓存）
    if (gnss_data.timestamp > latest_history.timestamp) {
        // 数据太新，保留在缓存中，等待更多IMU数据
        return;
    }

    // 查找并插值IMU
    std::list<HistoryState>::iterator iter_k;
    if (FindUpdateInterval(gnss_data, iter_k)) {
        // 如果找到，则进行插值同步
        HistoryState state_at_gnss = SynchronousWithImu(gnss_data, iter_k);

        if(!yaw_aligned_) {
            // 对齐航向
            if (AlignYawWithImu(state_at_gnss, gnss_data)) {
                gnss_buffer_.clear();
                return; 
            }
        } else {
            // 已经对齐，进行GNSS更新和重传播
            UpdateAndRepropagate(iter_k, gnss_data, state_at_gnss);
        }
        
        // 只有在对齐或更新成功处理后才移除GNSS数据
        gnss_buffer_.pop_front();
    }
}

bool ESKF::FindUpdateInterval(const GNSS& gnss_data, std::list<HistoryState>::iterator& iter_k) {
    // 遍历历史IMU数据，寻找GNSS时间戳所在的区间 [k, k+1]
    for (auto it = history_buffer_.begin(); it != std::prev(history_buffer_.end()); ++it) {
        auto next_it = std::next(it);
        if (gnss_data.timestamp >= it->timestamp && gnss_data.timestamp < next_it->timestamp) {
            iter_k = it;
            return true;
        }
    }

    ROS_DEBUG("Could not find a suitable IMU interval for GNSS data at %.4f", gnss_data.timestamp);
    return false;
}

HistoryState ESKF::SynchronousWithImu(const GNSS& gnss_data, std::list<HistoryState>::iterator iter_k) {
    // GNSS位于历史状态区间的右端点
    auto iter_k1 = std::next(iter_k);

    // 创建影子状态，并传播到GNSS时刻
    HistoryState state_at_gnss;
    IMU mid_imu;
    ImuInterpolate(iter_k->imu_data, iter_k1->imu_data, gnss_data.timestamp, mid_imu);
    PropagateState(*iter_k, mid_imu, state_at_gnss);

    return state_at_gnss;
}

void ESKF::UpdateAndRepropagate(const std::list<HistoryState>::iterator& iter_k, const GNSS& gnss_data, HistoryState& state_at_gnss) {
    auto iter_k1 = std::next(iter_k);

    // 在GNSS时刻进行更新
    Vector3d lla_position = Earth::LlaToEnu(gnss_data.lla);
    Vector6d residual;
    residual.head<3>() = lla_position - state_at_gnss.state.p;
    residual.tail<3>() = gnss_data.velocity - state_at_gnss.state.v;
    UpdateState(state_at_gnss, residual, gnss_noise_cov_);

    // 从GNSS时刻重播到当前（迭代所有GNSS时刻到当前时刻的历史帧）
    auto iter_current = iter_k1;
    HistoryState last_corrected_state = state_at_gnss;
    while (iter_current != history_buffer_.end()) {
        HistoryState corrected_state;
        // 使用上一个重播状态作为起点进行传播
        PropagateState(last_corrected_state, iter_current->imu_data, corrected_state);
        *iter_current = corrected_state;
        last_corrected_state = corrected_state;
        ++iter_current;
    }

    // ROS_INFO("GNSS data at %.4f processed with replay.", gnss_data.timestamp);
}


void ESKF::PropagateState(const HistoryState& last_history, const IMU& current_imu, HistoryState& new_history) {
    // IMU积分
    NavState predicted_state;
    INS::INSMech(last_history.state, predicted_state, last_history.imu_data, current_imu);

    // 误差状态预测矩阵F
    // 误差状态: [delta_p, delta_v, delta_theta, delta_bg, delta_ba, g] (18x1)
    // 参考高翔自动驾驶与机器人中的SLAM技术 第1版 P79
    Matrix<double, 18, 18> F = Matrix<double, 18, 18>::Identity();
    Matrix3d R = last_history.state.q.toRotationMatrix();
    double dt = current_imu.timestamp - last_history.timestamp;
    Vector3d corrected_accel = current_imu.accel - last_history.state.ba;
    Vector3d corrected_gyro = current_imu.gyro - last_history.state.bg;

    F.block<3, 3>(0, 3) = Matrix3d::Identity() * dt;
    F.block<3, 3>(3, 6) = -R * SkewSymmetric(corrected_accel) * dt;
    F.block<3, 3>(3, 12) = -R * dt;
    F.block<3, 3>(3, 15) = Matrix3d::Identity() * dt;
    F.block<3, 3>(6, 6) = Matrix3d::Identity() - SkewSymmetric(corrected_gyro) * dt; // corrected_gyro = imu.gyro - nominal_state_.bg
    F.block<3, 3>(6, 9) = -Matrix3d::Identity() * dt;

    // 输入噪声矩阵
    Matrix<double, 12, 12> Q_c = Matrix<double, 12, 12>::Zero();
    Q_c.block<6, 6>(0, 0) = imu_noise_cov_; // 陀螺仪和加速度计的测量噪声
    Q_c.block<3, 3>(6, 6) = gyro_bias_random_walk_;  // 陀螺仪零偏的随机游走噪声
    Q_c.block<3, 3>(9, 9) = accel_bias_random_walk_; // 加速度计零偏的随机游走噪声

    // 计算过程噪声传播矩阵 G
    Matrix<double, 18, 12> G = Matrix<double, 18, 12>::Zero();
    G.block<3, 3>(6, 0) = -Matrix3d::Identity();
    G.block<3, 3>(3, 3) = -R;
    G.block<3, 3>(9, 6) = Matrix3d::Identity();
    G.block<3, 3>(12, 9) = Matrix3d::Identity();

    Matrix<double, 18, 18> Q = G * Q_c * G.transpose() * dt; 

    // test_logger_->LogTest(Q);
    // std::cout << "Q: " << Q << std::endl;

    // 噪声矩阵Q
    // 参考高翔自动驾驶与机器人中的SLAM技术 第1版 P86
    // Matrix<double, 18, 18> Q = Matrix<double, 18, 18>::Zero();
    // Q.block<6, 6>(9, 9) = imu_noise_cov_;

    // 状态传播
    new_history.state = predicted_state;
    new_history.state.timestamp = current_imu.timestamp;
    new_history.P = F * last_history.P * F.transpose() + Q;
    new_history.imu_data = current_imu;
    new_history.timestamp = current_imu.timestamp;

    // 误差状态更新，这里由于理论上被置零操作，实际上可以不显示的写出
    // delta_x = K * delta_x;
}

bool ESKF::AlignYawWithImu(const HistoryState& state_at_gnss, const GNSS& gnss_data) {
    align_gnss_buffer_.push_back(gnss_data);
    align_state_buffer_.push_back(state_at_gnss);

    if (align_gnss_buffer_.size() < required_data_points_) {
        return false; // 数据不足，等待下一次调用
    }

    ROS_INFO("Collected enough data (%zu points), attempting alignment.", align_gnss_buffer_.size());

    // 存储用于对齐的向量对
    std::vector<Vector3d> gnss_vecs;
    std::vector<Vector3d> imu_vecs_in_initial_frame;

    // 从第二个点开始，到倒数第二个点结束，进行中心差分
    for (size_t i = 1; i < align_gnss_buffer_.size() - 1; ++i) {
        const auto& prev_gnss = align_gnss_buffer_[i - 1];
        const auto& next_gnss = align_gnss_buffer_[i + 1];
        double dt = next_gnss.timestamp - prev_gnss.timestamp;
        if (dt <= 1e-3) continue;
        
        Vector3d acc_n = (next_gnss.velocity - prev_gnss.velocity) / dt;
        // std::cout << "acc_n norm: " << acc_n.norm() << std::endl;
        if (acc_n.norm() < 0.01) continue;
        
        const auto& state_i = align_state_buffer_[i];
        const Vector3d& f_b = state_i.imu_data.accel;
        const Quaterniond& q_I_B = state_i.state.q;
        Vector3d f_I = q_I_B * f_b;
        
        gnss_vecs.push_back(acc_n);
        imu_vecs_in_initial_frame.push_back(f_I);
    }

    if (gnss_vecs.size() < 8) {
        ROS_WARN("Not enough valid dynamics in window (%zu points) to align.", gnss_vecs.size());
        return false;
    }

    // 使用SVD求解两个坐标系之间的旋转矩阵 R_N_I
    Matrix3d M = Matrix3d::Zero();
    for (size_t i = 0; i < gnss_vecs.size(); ++i) {
        M += gnss_vecs[i] * imu_vecs_in_initial_frame[i].transpose();
    }

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Matrix3d U = svd.matrixU(), V = svd.matrixV();
    Matrix3d S = Matrix3d::Identity();
    if (U.determinant() * V.determinant() < 0) {
        S(2, 2) = -1.0;
    }
    Matrix3d R_N_I = U * S * V.transpose();

    // 计算初始航向角 yaw0
    double yaw0 = atan2(R_N_I(1, 0), R_N_I(0, 0));
    ROS_INFO("Yaw Alignment Successful! Initial Yaw: %.2f degrees. ", yaw0 / D2R);

    // 保存初始对齐状态和GNSS数据
    const auto& latest_history_state = history_buffer_.back();
    const auto& latest_gnss_data = align_gnss_buffer_.back();

    // 提供给滤波器的初始状态帧
    HistoryState new_initial_state;
    new_initial_state.timestamp = latest_history_state.timestamp;
    new_initial_state.imu_data = latest_history_state.imu_data;
    new_initial_state.state.timestamp = latest_history_state.timestamp;
    
    // 修正姿态
    Quaterniond q_N_I(R_N_I);
    new_initial_state.state.q = q_N_I * latest_history_state.state.q;
    new_initial_state.state.q.normalize();

    Vector3d euler_angles1 = latest_history_state.state.q.toRotationMatrix().eulerAngles(2, 1, 0);
    ROS_INFO("Test Yaw: %.2f degrees", euler_angles1[0] / D2R);

    // 用最新的GNSS数据更新P, V
    new_initial_state.state.p = Earth::LlaToEnu(latest_gnss_data.lla);
    new_initial_state.state.v = latest_gnss_data.velocity;

    // 继承历史状态部分数据
    new_initial_state.state.g = latest_history_state.state.g;
    new_initial_state.state.bg = latest_history_state.state.bg;
    new_initial_state.state.ba = latest_history_state.state.ba;

    // 使用指定的初始协方差
    new_initial_state.P = initial_p_;

    // 清空历史，将修正后的最新状态作为新的唯一初始帧
    history_buffer_.clear();
    history_buffer_.push_back(new_initial_state);

    yaw_aligned_ = true;
    align_gnss_buffer_.clear();
    align_state_buffer_.clear();
    
    // 打印日志
    Vector3d euler_angles = new_initial_state.state.q.toRotationMatrix().eulerAngles(2, 1, 0);
    ROS_INFO("Repropagation complete. Initial Yaw: %.2f degrees", euler_angles[0] / D2R);

    return true;
}

void ESKF::UpdateState(HistoryState& state_to_update, const Vector6d& residual, const Matrix<double, 6, 6>& R) {
    // 计算观测矩阵 H
    Matrix<double, 6, 18> H = Matrix<double, 6, 18>::Zero();
    H.block<3, 3>(0, 0) = Matrix3d::Identity();
    H.block<3, 3>(3, 3) = Matrix3d::Identity();

    // 计算卡尔曼增益 K
    Eigen::Matrix<double, 18, 6> K = state_to_update.P * H.transpose() * (H * state_to_update.P * H.transpose() + R).inverse();

    // 更新误差状态
    Eigen::Matrix<double, 18, 1> delta_x = K * residual;

    // 更新协方差矩阵
    state_to_update.P = (Matrix<double, 18, 18>::Identity() - K * H) * state_to_update.P;
    state_to_update.P = 0.5 * (state_to_update.P + state_to_update.P.transpose());

    // 更新状态
    state_to_update.state.p += delta_x.head<3>();
    state_to_update.state.v += delta_x.segment<3>(3);
    Vector3d delta_theta = delta_x.segment<3>(6);
    if (delta_theta.norm() > 1e-12) {
        Quaterniond delta_q(1, delta_theta.x() / 2, delta_theta.y() / 2, delta_theta.z() / 2);
        state_to_update.state.q = (state_to_update.state.q * delta_q).normalized();
    }
    state_to_update.state.bg += delta_x.segment<3>(9);
    state_to_update.state.ba += delta_x.segment<3>(12);
    state_to_update.state.g += delta_x.segment<3>(15);
    
}

void ESKF::ImuInterpolate(const IMU& prev, const IMU& curr, double timestamp, IMU& output) {
    double dt = curr.timestamp - prev.timestamp;
    // 避免除以零或时间戳异常
    if (dt <= 1e-6) {
        output = prev;
        output.timestamp = timestamp;
        return;
    }
    
    double ratio = (timestamp - prev.timestamp) / dt;

    output.timestamp = timestamp;
    // 对加速度和角速度进行线性插值
    output.accel = prev.accel + (curr.accel - prev.accel) * ratio;
    output.gyro = prev.gyro + (curr.gyro - prev.gyro) * ratio;
    
    // 注意：插值后的IMU数据的 'dt' 成员将在之后调用 ProcessIMUTimestamp 时被计算，
    // 例如 ProcessIMUTimestamp(mid_imu, last_imu_data_.timestamp)
}

NavState ESKF::GetNavState() const {
    if (history_buffer_.empty()) {
        ROS_ERROR("History buffer is empty, cannot get current state!");
        return NavState(); // 返回一个默认构造的无效状态
    }
    return history_buffer_.back().state;
}

} // namespace dr_gins