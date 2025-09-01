#include "common/state_logger.h"

namespace dr_gins {

void StateLogger::LogState(const HistoryState& history) {
    const auto& state = history.state;
    const auto& P = history.P;

    // --- 记录状态向量 ---
    std::stringstream ss_state;
    ss_state << std::fixed << std::setprecision(6)
             << "[STATE_VECTOR] " << state.timestamp << " "
             << state.p.x() << " " << state.p.y() << " " << state.p.z() << " "
             << state.v.x() << " " << state.v.y() << " " << state.v.z() << " "
             << state.q.w() << " " << state.q.x() << " " << state.q.y() << " " << state.q.z() << " "
             << state.bg.x() << " " << state.bg.y() << " " << state.bg.z() << " "
             << state.ba.x() << " " << state.ba.y() << " " << state.ba.z() << " "
             << state.g.x() << " " << state.g.y() << " " << state.g.z();
    ROS_DEBUG_STREAM(ss_state.str());

    // --- 记录协方差矩阵的对角线元素 ---
    std::stringstream ss_cov;
    ss_cov << std::fixed << std::setprecision(9)
           << "[COV_DIAG] " << state.timestamp;
    for (int i = 0; i < 18; ++i) {
        ss_cov << " " << P(i, i);
    }
    ROS_DEBUG_STREAM(ss_cov.str());
}

} // namespace dr_gins