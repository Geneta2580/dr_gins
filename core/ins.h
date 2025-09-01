#pragma once

#include <Eigen/Dense>
#include "common/types.h"
#include "common/math_utils.h"

namespace dr_gins {
namespace INS {

    void INSMech(const NavState &pvapre, NavState &pvacur, const IMU &imupre, const IMU &imucur);

    void velUpdate(const NavState &pvapre, NavState &pvacur, const IMU &imupre, const IMU &imucur);

    void posUpdate(const NavState &pvapre, NavState &pvacur, const IMU &imupre, const IMU &imucur);
    
    void attUpdate(const NavState &pvapre, NavState &pvacur, const IMU &imupre, const IMU &imucur);

};
} // namespace dr_gins
