#include "log/state_logger.h"
#include <iostream> // 用于在文件打开失败时输出错误到控制台
#include <string>   // 用于字符串操作

namespace dr_gins {

StateLogger::StateLogger() {
    // --- 使用 __FILE__ 宏和字符串处理来获取当前文件所在的目录 ---
    // 这使得路径相对于项目结构，而不是可执行文件的运行位置
    std::string current_file_path = __FILE__;
    std::string log_directory = current_file_path.substr(0, current_file_path.find_last_of("/\\"));

    // --- 设置文件路径 ---
    std::string state_log_path = log_directory + "/filter_state.csv";
    std::string cov_log_path = log_directory + "/filter_covariance_diag.csv";

    // 打开状态日志文件
    state_file_.open(state_log_path, std::ios::out);
    if (!state_file_.is_open()) {
        std::cerr << "Error: Could not open state log file: " << state_log_path << std::endl;
    } else {
        // 写入CSV文件头，用逗号分隔
        state_file_ << "timestamp,p_x,p_y,p_z,v_x,v_y,v_z,q_w,q_x,q_y,q_z,bg_x,bg_y,bg_z,ba_x,ba_y,ba_z,g_x,g_y,g_z" << std::endl;
    }

    // 打开协方差日志文件
    cov_file_.open(cov_log_path, std::ios::out);
    if (!cov_file_.is_open()) {
        std::cerr << "Error: Could not open covariance log file: " << cov_log_path << std::endl;
    } else {
        // 写入CSV文件头
        cov_file_ << "timestamp,P_px,P_py,P_pz,P_vx,P_vy,P_vz,P_qw,P_qx,P_qy,P_qz,P_bgx,P_bgy,P_bgz,P_bax,P_bay,P_baz,P_gx,P_gy,P_gz" << std::endl;
    }
}

StateLogger::~StateLogger() {
    // 析构函数确保文件被正确关闭
    if (state_file_.is_open()) {
        state_file_.close();
    }
    if (cov_file_.is_open()) {
        cov_file_.close();
    }
}

void StateLogger::LogState(const HistoryState& history) {
    // 在写入前检查文件是否成功打开
    if (!state_file_.is_open() || !cov_file_.is_open()) {
        return;
    }

    const auto& state = history.state;
    const auto& P = history.P;

    // --- 记录状态向量到CSV文件 ---
    state_file_ << std::fixed << std::setprecision(6)
                << state.timestamp << ","
                << state.p.x() << "," << state.p.y() << "," << state.p.z() << ","
                << state.v.x() << "," << state.v.y() << "," << state.v.z() << ","
                << state.q.w() << "," << state.q.x() << "," << state.q.y() << "," << state.q.z() << ","
                << state.bg.x() << "," << state.bg.y() << "," << state.bg.z() << ","
                << state.ba.x() << "," << state.ba.y() << "," << state.ba.z() << ","
                << state.g.x() << "," << state.g.y() << "," << state.g.z() << std::endl;

    // --- 记录协方差矩阵的对角线元素到CSV文件 ---
    cov_file_ << std::fixed << std::setprecision(9)
              << state.timestamp;
    for (int i = 0; i < 18; ++i) {
        cov_file_ << "," << P(i, i);
    }
    cov_file_ << std::endl;
}

} // namespace dr_gins