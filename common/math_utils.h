#pragma once
#include "common/types.h"
#include <cmath>

namespace dr_gins {

static constexpr double D2R = M_PI / 180.0;

inline Matrix3d SkewSymmetric(const Vector3d& v) {
    Matrix3d skew;
    skew << 0,    -v.z(),  v.y(),
            v.z(),  0,    -v.x(),
           -v.y(),  v.x(),  0;
    return skew;
}


} // namespace dr_gins