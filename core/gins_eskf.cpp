#include "core/gins_eskf.h"

namespace dr_gins {

bool ESKF::Initialize(const NavState& initial_state, const Matrix<double, 18, 18>& initial_p,
                      const Matrix3d& gyro_noise, const Matrix3d& accel_noise,
                      const Vector6d& gnss_noise) {

    // 使用传入的参数创建第一个历史状态，并放入缓存
    HistoryState initial_history;
    initial_history.timestamp = initial_state.timestamp;
    initial_history.state = initial_state;
    initial_history.P = initial_p;
    initial_history.imu_data = IMU(); 
    history_buffer_.push_back(initial_history);

    // 设置IMU噪声协方差矩阵
    imu_noise_cov_.setZero();
    imu_noise_cov_.block<3, 3>(0, 0) = gyro_noise;
    imu_noise_cov_.block<3, 3>(3, 3) = accel_noise;

    // 设置GNSS噪声协方差矩阵
    gnss_noise_cov_ = gnss_noise.asDiagonal();

    first_imu_ = true; // 标记需要用第一帧IMU数据更新初始状态
    ROS_INFO("System Parameters Initialized Successfully!");
    return true;
}

bool ESKF::ProcessImu(const IMU& imu_data) {
    // 第一帧IMU数据不进行状态传播
    if (first_imu_) {
        history_buffer_.front().imu_data = imu_data;
        first_imu_ = false;
        return true;
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
    if (history_buffer_.size() >  max_history_size_) {
        history_buffer_.pop_front();
    }

    // 查找GNSS数据
    AttemptGnssUpdate();

    return true;
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
            if (AlignYawWithImu(state_at_gnss, gnss_data)) { return; }
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
    HistoryState replay_state = state_at_gnss;
    for (auto it = iter_k1; it != history_buffer_.end(); ++it) {
        HistoryState temp_state;
        // 使用上一个重播状态作为起点进行传播
        PropagateState(replay_state, it->imu_data, temp_state);
        replay_state = temp_state;
    }

    // 更新历史缓存的最后一个状态
    history_buffer_.back() = replay_state;

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

    // Matrix<double, 15, 6> G = Matrix<double, 15, 6>::Zero();
    // G.block<3, 3>(3, 3) = -R;
    // G.block<3, 3>(6, 0) = -Matrix3d::Identity();
    // G.block<3, 3>(9, 0) = Matrix3d::Identity(); // For gyro bias noise
    // G.block<3, 3>(12, 3) = Matrix3d::Identity(); // For accel bias noise
    // Matrix<double, 15, 15> Q = G * imu_noise_cov_ * G.transpose() * dt * dt;

    // 噪声矩阵Q
    // 参考高翔自动驾驶与机器人中的SLAM技术 第1版 P86
    Matrix<double, 18, 18> Q = Matrix<double, 18, 18>::Zero();
    Q.block<6, 6>(9, 9) = imu_noise_cov_;

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

    const size_t required_data_points = 15;
    if (align_gnss_buffer_.size() < required_data_points) {
        return false; // 数据不足，等待下一次调用
    }

    ROS_INFO("Collected enough data (%zu points), attempting alignment.", align_gnss_buffer_.size());

    // 使用GNSS速度计算加速度
    std::vector<Vector3d> acc_n_vec;
    std::vector<Vector3d> f_b_vec;

    // 从第二个点开始，到倒数第二个点结束，进行中心差分
    for (size_t i = 1; i < align_gnss_buffer_.size() - 1; ++i) {
        const auto& prev_gnss = align_gnss_buffer_[i - 1];
        const auto& next_gnss = align_gnss_buffer_[i + 1];

        double dt = next_gnss.timestamp - prev_gnss.timestamp;
        if (dt <= 1e-3) continue;

        // 直接对GNSS速度进行中心差分得到加速度
        // 假设GNSS速度是在ENU坐标系下
        Vector3d acc_n = (next_gnss.velocity - prev_gnss.velocity) / dt;

        if (acc_n.norm() > 5.0 || acc_n.norm() < 0.1) continue;

        // 存储 a_n 和与之对应的 f_b (比力)
        acc_n_vec.push_back(acc_n);
        f_b_vec.push_back(align_state_buffer_[i].imu_data.accel);
    }

    if (acc_n_vec.size() < 5) {
        ROS_WARN("Could not compute enough valid accelerations from GNSS for alignment. Clearing alignment buffer.");
        align_gnss_buffer_.clear();
        align_state_buffer_.clear();
        return false;
    }

    // 构建Wahba问题并求解 R_n_b
    Vector3d g_n = align_state_buffer_[0].state.g;
    Matrix3d M = Matrix3d::Zero();
    for (size_t i = 0; i < acc_n_vec.size(); ++i) {
        M += (acc_n_vec[i] - g_n) * f_b_vec[i].transpose();
    }

    Eigen::JacobiSVD<Matrix3d> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Matrix3d U = svd.matrixU(), V = svd.matrixV();
    Matrix3d S = Matrix3d::Identity();
    if (U.determinant() * V.determinant() < 0) S(2, 2) = -1.0;
    Matrix3d R_n_b = U * S * V.transpose();

    // 对齐成功，修正最新的历史状态
    HistoryState& latest_state = history_buffer_.back();
    latest_state.state.q = Quaterniond(R_n_b);
    latest_state.state.p = Earth::LlaToEnu(align_gnss_buffer_.back().lla);
    latest_state.state.v = align_gnss_buffer_.back().velocity;
    latest_state.state.g = g_n;
    latest_state.P = Matrix<double, 18, 18>::Identity() * 1.0; // 重置协方差矩阵
    yaw_aligned_ = true;

    align_gnss_buffer_.clear();
    align_state_buffer_.clear();
    gnss_buffer_.clear();
    history_buffer_.clear();
    history_buffer_.push_back(latest_state);

    Vector3d euler_angles = R_n_b.eulerAngles(2, 1, 0);
    ROS_INFO("Yaw Alignment Successful! Initial Yaw: %.2f degrees", euler_angles[0] * 180.0 / M_PI);

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