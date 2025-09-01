#pragma once
#include "common/types.h"
#include "common/math_utils.h"

namespace dr_gins {

class Earth {
public:
    Earth();

    // WGS84 Earth parameters
    static constexpr double WGS84_A = 6378137.0;        // Semi-major axis in meters
    static constexpr double WGS84_F = 1.0 / 298.257223563; // Flattening
    static constexpr double WGS84_E2 = WGS84_F * (2.0 - WGS84_F); // First eccentricity squared

    // 设置GNSS坐标系原点
    static void SetOrigin(const Vector3d& lla_origin) {
        lla_origin_ = lla_origin;
        origin_initialized_ = true;
    }

    // 将纬经高转换为以起点ENU坐标系作为原点的坐标系
    static Vector3d LlaToEnu(const Vector3d& lla) {
        if (!origin_initialized_) {
            SetOrigin(lla);
            // The origin is at (0,0,0) in its own ENU frame.
            return Vector3d::Zero();
        }

        // Convert current and origin coordinates from degrees to radians
        double lat_rad = lla.x() * D2R;
        double lon_rad = lla.y() * D2R;
        double origin_lat_rad = lla_origin_.x() * D2R;
        double origin_lon_rad = lla_origin_.y() * D2R;

        // Use a simple equirectangular projection, which is accurate for local areas.
        // It approximates the Earth as a sphere with radius WGS84_A for this conversion.
        double dx = (lon_rad - origin_lon_rad) * WGS84_A * cos(origin_lat_rad);
        double dy = (lat_rad - origin_lat_rad) * WGS84_A;
        double dz = lla.z() - lla_origin_.z();

        return Vector3d(dx, dy, dz); // Returns [East, North, Up]
    }


private:
    inline static Vector3d lla_origin_;
    inline static bool origin_initialized_ = false;
};
} // namespace dr_gins