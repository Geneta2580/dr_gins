#pragma once

#include "common/types.h"
#include "common/math_utils.h"
#include <fstream> // 1. 包含 fstream 用于文件操作
#include <string>
#include <sstream>
#include <iomanip>

namespace dr_gins {

class StateLogger {
public:
    StateLogger();
    ~StateLogger();

    void LogState(const HistoryState& history);
    void LogTest(const Matrix<double, 18, 18>& test_value);

private:
    std::ofstream state_file_;
    std::ofstream cov_file_;
    std::ofstream test_file_;
};

} // namespace dr_gins