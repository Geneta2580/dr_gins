#include "core/ins.h"

namespace dr_gins {
namespace INS {

void INSMech(const NavState &pre_state, NavState &cur_state, const IMU &imupre, const IMU &imucur) {
    cur_state.g = pre_state.g; // 假设重力在短时间内不变
    cur_state.ba = pre_state.ba; // 假设偏置在短时间内不变
    cur_state.bg = pre_state.bg; // 假设偏置在短时间内不变

    // perform velocity update, position updata and attitude update in sequence, irreversible order
    // 依次进行速度更新、位置更新、姿态更新, 不可调换顺序?
    velUpdate(pre_state, cur_state, imupre, imucur);
    posUpdate(pre_state, cur_state, imupre, imucur);
    attUpdate(pre_state, cur_state, imupre, imucur);
}

void velUpdate(const NavState &pre_state, NavState &cur_state, const IMU &imupre, const IMU &imucur) {

    // 计算b系的平均比力
    Vector3d specific_force_b = 0.5 * (imupre.accel + imucur.accel) - pre_state.ba;
    // 平均比力转到n系
    Vector3d specific_force_n = pre_state.q.toRotationMatrix() * specific_force_b;
    
    cur_state.v = pre_state.v + (specific_force_n + pre_state.g) * imucur.dt;
}

void posUpdate(const NavState &pre_state, NavState &cur_state, const IMU &imupre, const IMU &imucur) {
    cur_state.p = pre_state.p + 0.5 * (pre_state.v + cur_state.v) * imucur.dt;
}

void attUpdate(const NavState &pre_state, NavState &cur_state, const IMU &imupre, const IMU &imucur) {
    
    // 扣除零偏后的角速度
    Vector3d gyro_corrected = 0.5 * (imupre.gyro + imucur.gyro) - pre_state.bg;

    // 角增量
    Vector3d delta_theta = gyro_corrected * imucur.dt;

    // 见秦永元《惯性导航》第三版 P264，四元数毕卡算法的一阶近似
    Matrix<double, 4, 4> Omega;
    Omega.block<3, 3>(1, 1) = -SkewSymmetric(delta_theta);
    Omega.block<3, 1>(1, 0) = delta_theta;
    Omega.block<1, 3>(0, 1) = -delta_theta.transpose();
    Omega(0, 0) = 0;

    // 四元数转换为向量表示
    Vector4d q_vec_pre(pre_state.q.w(), pre_state.q.x(), pre_state.q.y(), pre_state.q.z());

    // 使用一阶近似更新: q_new ≈ (I + 0.5 * Ω) * q_old
    Vector4d q_vec_cur = q_vec_pre + 0.5 * Omega * q_vec_pre;

    // 转换为四元数并归一化
    cur_state.q = Quaterniond(q_vec_cur(0), q_vec_cur(1), q_vec_cur(2), q_vec_cur(3));
    cur_state.q.normalize();
}

} // namespace ins
} // namespace dr_gins