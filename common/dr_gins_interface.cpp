#include "common/dr_gins_interface.h"

namespace dr_gins {

DrGinsInterface::DrGinsInterface(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
    : private_nh_(private_nh) {
    // 创建ESKF求解器的实例
    eskf_solver_ = std::make_unique<ESKF>();

    // 日志
    state_logger_ = std::make_unique<StateLogger>();

    // 初始化ROS相关的订阅者和发布者
    InitRos();

    // 初始化滤波器相关参数
    InitParams();

    ROS_INFO("DrGinsInterface initialized.");

    // 在所有初始化完成后，启动数据处理
    StartDataProcessing();
}

void DrGinsInterface::InitRos() {
    // 从yaml配置文件中加载参数
    private_nh_.getParam("base_frame_id", base_frame_id_);
    private_nh_.getParam("child_frame_id", child_frame_id_);

    // 初始化发布者
    pub_odometry_ = private_nh_.advertise<nav_msgs::Odometry>("/dr_gins/odometry", 10);
    pub_path_ = private_nh_.advertise<nav_msgs::Path>("/dr_gins/path", 10);

    // 新增真值发布器
    truth_odom_pub_ = private_nh_.advertise<nav_msgs::Odometry>("/dr_gins/truth_odometry", 10);
    truth_path_pub_ = private_nh_.advertise<nav_msgs::Path>("/dr_gins/truth_path", 10);

    // 初始化路径消息
    path_msg_.header.frame_id = base_frame_id_;
    truth_path_msg_.header.frame_id = base_frame_id_;  // 新增
    path_msg_.header.stamp = ros::Time::now();
    
}

void DrGinsInterface::InitParams() {
    // 读取数据源配置参数
    use_file_data_ = private_nh_.param("use_file_data", true);
    data_file_path_ = private_nh_.param("data_file_path", std::string(""));

    if (use_file_data_ && data_file_path_.empty()) {
        ROS_ERROR("use_file_data is true but data_file_path is empty!");
        return;
    }

    // 初始化状态变量
    NavState initial_state;
    std::vector<double> vec;
    if (private_nh_.getParam("initial_state/position", vec) && vec.size() >= 3)
        initial_state.p = Vector3d(vec[0], vec[1], vec[2]);
    if (private_nh_.getParam("initial_state/velocity", vec) && vec.size() >= 3)
        initial_state.v = Vector3d(vec[0], vec[1], vec[2]);
    if (private_nh_.getParam("initial_state/orientation_wxyz", vec) && vec.size() >= 4)
        initial_state.q = Quaterniond(vec[0], vec[1], vec[2], vec[3]); // w,x,y,z
    if (private_nh_.getParam("initial_state/accel_bias", vec) && vec.size() >= 3)
        initial_state.ba = Vector3d(vec[0], vec[1], vec[2]);
    if (private_nh_.getParam("initial_state/gyro_bias", vec) && vec.size() >= 3)
        initial_state.bg = Vector3d(vec[0], vec[1], vec[2]);
    if (private_nh_.getParam("initial_state/gravity", vec) && vec.size() >= 3)
        initial_state.g = Vector3d(vec[0], vec[1], vec[2]);

    // 初始协方差缩放因子或完整矩阵
    double p_scale = private_nh_.param("initial_cov_scale", 1e-3);
    Matrix<double, 18, 18> P = Matrix<double, 18, 18>::Identity() * p_scale;

    // 传感器噪声（默认并可由参数覆盖）
    Matrix3d gyro_noise = Matrix3d::Identity() * private_nh_.param("gyro_noise", 1e-3);
    Matrix3d accel_noise = Matrix3d::Identity() * private_nh_.param("accel_noise", 1e-3);

    std::vector<double> gnss_vec;
    Vector6d gnss_noise;
    if (private_nh_.getParam("gnss_noise", gnss_vec) && gnss_vec.size() >= 6)
        gnss_noise << gnss_vec[0], gnss_vec[1], gnss_vec[2], gnss_vec[3], gnss_vec[4], gnss_vec[5];
    else
        gnss_noise << 1.0, 1.0, 2.0, 0.1, 0.1, 0.1;

    // 调用ESKF初始化
    eskf_solver_->Initialize(initial_state, P, gyro_noise, accel_noise, gnss_noise);

    ROS_INFO("ESKF parameters initialized. use_file_data: %s, data_file_path: %s", 
             use_file_data_ ? "true" : "false", data_file_path_.c_str());
}

void DrGinsInterface::StartDataProcessing() {
    ROS_INFO("Start data processing.");
    if (!use_file_data_) {
        // 实时数据模式：订阅ROS话题
        std::string imu_topic, gnss_topic;
        private_nh_.param<std::string>("imu_topic", imu_topic, "/imu/data");
        private_nh_.param<std::string>("gnss_topic", gnss_topic, "/gnss/fix");

        sub_imu_ = private_nh_.subscribe(imu_topic, 100, &DrGinsInterface::ImuCallback, this);
        sub_gnss_ = private_nh_.subscribe(gnss_topic, 100, &DrGinsInterface::GnssCallback, this);
        ROS_INFO("Subscribing to ROS topics for IMU and GNSS data.");
    } else {
        // 文件重放模式：处理文件数据
        ROS_INFO("Starting file data replay from: %s", data_file_path_.c_str());
        ReplayFromFile();
        ROS_INFO("File data replay completed.");
    }
}

void DrGinsInterface::ReplayFromFile() {
    std::string data_path;
    if (!private_nh_.getParam("data_file_path", data_path)) {
        ROS_ERROR("File replay mode is ON, but 'data_file_path' param is not set in YAML!");
        return;
    }

    auto all_data = ReadDataFromTxt(data_path);
    if (all_data.empty()) {
        ROS_WARN("No data could be read from file: %s", data_path.c_str());
        return;
    }

    // 按时间戳排序数据
    std::sort(all_data.begin(), all_data.end(), 
            [this](const auto& a, const auto& b) {
                return this->GetTimestamp(a) < this->GetTimestamp(b);
            });

    ROS_INFO("Starting data replay from file. Total measurements: %zu", all_data.size());

    // 遍历所有数据，模拟接收IMU和GNSS数据
    for (size_t i = 0; i < all_data.size(); ++i) {
        if (!ros::ok()) {
            ROS_WARN("ROS shutting down, stopping file replay.");
            break;
        }
        const auto& data = all_data[i];

        // **************************************这部分可以写成一个开关**************************************
        if (i > 0) {
            double dt = GetTimestamp(data) - GetTimestamp(all_data[i-1]);
            if (dt > 0.0 && dt < 1.0) { // 避免过大的时间跳跃
                ros::Duration(dt).sleep(); // 在没有数据的dt期间进行等待，模拟数据接收的真实过程
            }
        }
        // **************************************这部分可以写成一个开关**************************************

        std::visit([this](auto&& arg) {
            using T = std::decay_t<decltype(arg)>;
            if constexpr (std::is_same_v<T, IMU>) {
                if (eskf_solver_->ProcessImu(ProcessIMUTimestamp(arg))) {
                    PublishState();

                    if (state_logger_ && !eskf_solver_->IsHistoryEmpty()) {
                        state_logger_->LogState(eskf_solver_->GetLatestHistoryState());
                    }

                }
            } else if constexpr (std::is_same_v<T, GNSS>) {
                eskf_solver_->ProcessGnss(arg);
            } else if constexpr (std::is_same_v<T, TruePose>) {
                PublishTruthPose(arg);
            }
        }, data);
    }

    ROS_INFO("File replay finished.");
}

void DrGinsInterface::ImuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
    // 将ROS消息格式转换为内部数据结构
    IMU imu_data;
    imu_data.timestamp = imu_msg->header.stamp.toSec();
    imu_data.gyro.x() = imu_msg->angular_velocity.x;
    imu_data.gyro.y() = imu_msg->angular_velocity.y;
    imu_data.gyro.z() = imu_msg->angular_velocity.z;
    imu_data.accel.x() = imu_msg->linear_acceleration.x;
    imu_data.accel.y() = imu_msg->linear_acceleration.y;
    imu_data.accel.z() = imu_msg->linear_acceleration.z;

    IMU imu_processed = ProcessIMUTimestamp(imu_data);
    // 调用ESKF核心进行处理
    if (eskf_solver_->ProcessImu(imu_processed)) {
        PublishState();
        if (state_logger_ && !eskf_solver_->IsHistoryEmpty()) {
            state_logger_->LogState(eskf_solver_->GetLatestHistoryState());
        }
    }
}

void DrGinsInterface::GnssCallback(const sensor_msgs::NavSatFix::ConstPtr& gnss_msg) {
    // 将ROS消息格式转换为内部数据结构
    GNSS gnss_data;
    gnss_data.timestamp = gnss_msg->header.stamp.toSec();

    gnss_data.lla << gnss_msg->latitude,
                     gnss_msg->longitude,
                     gnss_msg->altitude;
    
    gnss_data.velocity = Vector3d::Zero();

    gnss_data.status = gnss_msg->status.status;
    // gnss_data.horizontal_accuracy = ...; // 如果需要，可以从covariance中计算

    // 调用ESKF核心进行处理
    eskf_solver_->ProcessGnss(gnss_data);
}

void DrGinsInterface::PublishState() {
    // 从ESKF求解器获取最新的导航状态
    NavState current_state = eskf_solver_->GetNavState();
    // current_state.timestamp = ros::Time::now().toSec();
    // ----------------------------------------------------
    // 1. 发布里程计消息 (Odometry)
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = ros::Time(current_state.timestamp);
    odom_msg.header.frame_id = base_frame_id_;
    odom_msg.child_frame_id = child_frame_id_;

    // 平移部分
    odom_msg.pose.pose.position.x = current_state.p.x();
    odom_msg.pose.pose.position.y = current_state.p.y();
    odom_msg.pose.pose.position.z = current_state.p.z();

    // 旋转部分
    Quaterniond eigen_q(current_state.q);
    tf2::Quaternion tf_q(eigen_q.x(), eigen_q.y(), eigen_q.z(), eigen_q.w());
    odom_msg.pose.pose.orientation = tf2::toMsg(tf_q);

    odom_msg.twist.twist.linear.x = current_state.v.x();
    odom_msg.twist.twist.linear.y = current_state.v.y();
    odom_msg.twist.twist.linear.z = current_state.v.z();

    pub_odometry_.publish(odom_msg);

    // 2. 发布路径消息 (Path)
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = odom_msg.header;
    pose_stamped.pose = odom_msg.pose.pose;
    path_msg_.poses.push_back(pose_stamped);
    pub_path_.publish(path_msg_);
}


// 新增：发布真值姿态的函数
void DrGinsInterface::PublishTruthPose(const TruePose& true_pose) {
    ros::Time current_time = ros::Time(true_pose.timestamp);
    
    // 发布真值里程计
    nav_msgs::Odometry truth_odom;
    truth_odom.header.stamp = current_time;
    truth_odom.header.frame_id = base_frame_id_;
    truth_odom.child_frame_id = child_frame_id_;
    
    // 位置
    truth_odom.pose.pose.position.x = true_pose.p.x();
    truth_odom.pose.pose.position.y = true_pose.p.y();
    truth_odom.pose.pose.position.z = true_pose.p.z();
    
    // 姿态
    truth_odom.pose.pose.orientation.w = true_pose.q.w();
    truth_odom.pose.pose.orientation.x = true_pose.q.x();
    truth_odom.pose.pose.orientation.y = true_pose.q.y();
    truth_odom.pose.pose.orientation.z = true_pose.q.z();
    
    truth_odom_pub_.publish(truth_odom);
    
    // 添加到真值路径
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = truth_odom.header;
    pose_stamped.pose = truth_odom.pose.pose;
    
    truth_path_msg_.poses.push_back(pose_stamped);
    truth_path_msg_.header.stamp = current_time;
    
    truth_path_pub_.publish(truth_path_msg_);
}

// 辅助函数
double DrGinsInterface::GetTimestamp(const dr_gins::SensorData& data) {
    return std::visit([](auto&& arg) { return arg.timestamp; }, data);
}

IMU DrGinsInterface::ProcessIMUTimestamp(const IMU& raw_imu) {
    IMU processed_imu = raw_imu;
    
    // 计算dt
    if (last_imu_timestamp_ >= 0) {
        processed_imu.dt = raw_imu.timestamp - last_imu_timestamp_;
    } else {
        processed_imu.dt = 0.0; // 第一个IMU数据，dt设为0
    }
    
    // 更新上一次时间戳
    last_imu_timestamp_ = raw_imu.timestamp;
    
    return processed_imu;
}

} // namespace dr_gins