#include "chassis_fksolver.hpp"

namespace robot {
namespace chassis_fk_solver {

HW_OPTIMIZE_O2_START

ChassisFkSolver &ChassisFkSolver::operator=(const ChassisFkSolver &other) {
  if (this == &other) {
    return *this;
  }
  length_ = other.length_;
  width_ = other.width_;
  wheel_radius_ = other.wheel_radius_;

  return *this;
}

ChassisFkSolver::ChassisFkSolver(ChassisFkSolver &&other) {
  length_ = other.length_;
  width_ = other.width_;
  wheel_radius_ = other.wheel_radius_;
}

ChassisFkSolver &ChassisFkSolver::operator=(ChassisFkSolver &&other) {
  if (this == &other) {
    return *this;
  }
  length_ = other.length_;
  width_ = other.width_;
  wheel_radius_ = other.wheel_radius_;

  return *this;
}

void ChassisFkSolver::calc(const float wheel_speeds[4], const float theta_i2r,
                           ChassisState &state) const {
  // 轮子顺序：左前(0)、左后(1)、右后(2)、右前(3)
  float lf_spd = wheel_speeds[0]; // 左前
  float lb_spd = wheel_speeds[1]; // 左后
  float rb_spd = wheel_speeds[2]; // 右后
  float rf_spd = wheel_speeds[3]; // 右前

  // 半轮距半轴距
  float half_width = width_ / 2.0f;
  float half_length = length_ / 2.0f;

  // 逆矩阵计算 - 底盘坐标系
  float k = wheel_radius_ / 4.0f;

  float chassis_v_x = k * (lf_spd + rf_spd + lb_spd + rb_spd);
  float chassis_v_y = k * (lf_spd - rf_spd - lb_spd + rb_spd);
  float chassis_w =
      -k * (lf_spd - rf_spd + lb_spd - rb_spd) / (half_length + half_width);

  // 坐标系转换 - 底盘坐标系 -> 云台坐标系
  // 右手系旋转矩阵: R(θ) = [cos(θ) -sin(θ); sin(θ) cos(θ)]
  float sin_theta = sinf(theta_i2r);
  float cos_theta = cosf(theta_i2r);

  // 计算云台坐标系下的速度
  state.v_x = chassis_v_x * cos_theta + chassis_v_y * sin_theta;
  state.v_y =
      chassis_v_x * sin_theta -
      chassis_v_y *
          cos_theta; // 遥控器遥杆向左，指令为负数，指令体系遵循左手系，所以这里取反
  state.w = chassis_w; // 角速度不受坐标系旋转影响
}
} // namespace chassis_fk_solver
} // namespace robot
