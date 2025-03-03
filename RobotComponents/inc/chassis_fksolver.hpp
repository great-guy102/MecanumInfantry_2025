#ifndef CHASSIS_FK_SOLVER_HPP
#define CHASSIS_FK_SOLVER_HPP

#include <cmath>
#include <cstddef>

#include "allocator.hpp"
#include "base.hpp"
#include "system.hpp"

namespace robot {
namespace chassis_fk_solver {

HW_OPTIMIZE_O2_START
/**
 * @brief 底盘状态
 */
struct ChassisState {
  float v_x; // X方向速度
  float v_y; // Y方向速度
  float w;   // 绕Z轴角速度
};

class ChassisFkSolver : public hello_world::MemMgr {
public:
  /**
   * @brief 构造函数
   * @param length 前后轮距离(m)
   * @param width 左右轮距离(m)
   * @param wheel_radius 轮子半径(m)
   */
  explicit ChassisFkSolver(float length, float width, float wheel_radius)
      : length_(length), width_(width), wheel_radius_(wheel_radius) {}
  ChassisFkSolver(const ChassisFkSolver &) = default;
  ChassisFkSolver &operator=(const ChassisFkSolver &other);
  ChassisFkSolver(ChassisFkSolver &&other);
  ChassisFkSolver &operator=(ChassisFkSolver &&other);

  ~ChassisFkSolver(void) {}

  /**
   * @brief 计算底盘运动速度
   * @param wheel_speeds 轮子转速数组(rad/s)，顺序为：左前、左后、右后、右前
   * @param state 计算结果存储引用
   */
  void calc(const float wheel_speeds[4], const float theta_i2r,
                             ChassisState &state) const;

private:
  float length_;       // 前后轮距离(m)
  float width_;        // 左右轮距离(m)
  float wheel_radius_; // 轮子半径(m)
};

HW_OPTIMIZE_O2_END
} // namespace chassis_fk_solver
} // namespace robot

#endif // CHASSIS_FK_SOLVER_HPP