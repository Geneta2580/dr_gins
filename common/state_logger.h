#pragma once

#include "common/types.h"
#include <ros/ros.h>
#include <sstream>
#include <iomanip>

namespace dr_gins {

class StateLogger {
public:
    StateLogger() = default;
    ~StateLogger() = default;

    /**
     * @brief 记录滤波器状态和协方差。这是被回调函数调用的核心方法。
     * @param history 包含状态和协方差的 HistoryState 对象
     */
    void LogState(const HistoryState& history);
};

} // namespace dr_gins