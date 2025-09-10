#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <algorithm>
#include <ublox_msgs/NavPVT.h>
#include <memory>
#include "common/types.h"
#include "common/file_io.h"
#include "core/gins_eskf.h"
#include "log/state_logger.h"
#include <chrono> 

// 前向声明核心ESKF求解器类。
namespace dr_gins {
    class ESKF; 
}

namespace dr_gins {

class DrGinsInterface {
public:

    DrGinsInterface(ros::NodeHandle& nh, ros::NodeHandle& private_nh);

    ~DrGinsInterface() = default;

    // 禁止拷贝和赋值
    DrGinsInterface(const DrGinsInterface&) = delete;
    DrGinsInterface& operator=(const DrGinsInterface&) = delete;

private:

    void ImuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg);

    void GnssCallback(const ublox_msgs::NavPVT::ConstPtr& gnss_msg);

    void InitRos();

    void InitParams();

    void PublishState();

    void ReplayFromFile();

    void StartDataProcessing();

    void PublishTruthPose(const TruePose& true_pose);  // 新增方法声明

    void ProcessMeasurement(const SensorData& data);

    // 辅助函数
    double GetTimestamp(const dr_gins::SensorData& data);
    IMU ProcessIMUTimestamp(const IMU& raw_imu);

    // ROS 相关成员
    ros::NodeHandle private_nh_;
    ros::Subscriber sub_imu_;
    ros::Subscriber sub_gnss_;
    ros::Publisher pub_odometry_;
    ros::Publisher pub_path_;

    ros::Publisher truth_odom_pub_;    // 新增：真值里程计发布器
    ros::Publisher truth_path_pub_;    // 新增：真值路径发布器

    // 用于可视化的路径消息
    nav_msgs::Path path_msg_;
    nav_msgs::Path truth_path_msg_;    // 新增：真值路径消息

    // 初始化参数
    std::string base_frame_id_;
    std::string child_frame_id_;
    bool use_file_data_ = true;  
    std::string data_file_path_;  

    // 数据缓存
    std::vector<IMU> imu_buffer_;
    std::vector<GNSS> gnss_buffer_;

    // ESKF求解器实例
    std::unique_ptr<ESKF> eskf_solver_;

    // 状态日志记录器
    std::unique_ptr<StateLogger> state_logger_;

    bool align_yaw_ = false; // 初始状态为未对齐
    std::mutex data_mutex_;

    double last_imu_timestamp_ = -1.0;

    // 最大历史状态数量
    int max_history_size_ = 200;
};

} // namespace dr_gins