/**
 * @file robot_model.hpp
 * @brief EasyArm 刚体动力学模型封装（基于 Pinocchio）
 *
 * 从 URDF 加载机器人模型，并提供重力项、非线性项、质量矩阵和逆动力学计算接口。
 */

#ifndef EASYARM_DYNAMICS_ROBOT_MODEL_HPP_
#define EASYARM_DYNAMICS_ROBOT_MODEL_HPP_

#include <string>

#include <Eigen/Core>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

namespace easyarm_dynamics
{

/**
 * @brief Pinocchio 机器人动力学模型封装类
 */
class RobotModel
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief 从 URDF 文件加载 Pinocchio 刚体动力学模型
   * @param urdf_path URDF 文件路径
   */
  explicit RobotModel(const std::string & urdf_path);

  /**
   * @brief 计算广义重力力矩 g(q)
   * @param q 关节位置向量，维度必须为 nq()
   * @return 广义重力力矩向量，维度为 nv()
   */
  Eigen::VectorXd gravity(const Eigen::VectorXd & q);

  /**
   * @brief 计算非线性项 NLE(q, qd)，即科氏/离心项 C(q, qd) * qd 与重力项 g(q) 的和
   * @param q 关节位置向量，维度必须为 nq()
   * @param qd 关节速度向量，维度必须为 nv()
   * @return 非线性力矩向量，维度为 nv()
   */
  Eigen::VectorXd nle(const Eigen::VectorXd & q, const Eigen::VectorXd & qd);

  /**
   * @brief 计算关节空间质量矩阵 M(q)
   * @param q 关节位置向量，维度必须为 nq()
   * @return 质量矩阵，维度为 nv() x nv()
   */
  Eigen::MatrixXd massMatrix(const Eigen::VectorXd & q);

  /**
   * @brief 计算逆动力学力矩 tau = M(q)qdd + C(q, qd)qd + g(q)
   * @param q 关节位置向量，维度必须为 nq()
   * @param qd 关节速度向量，维度必须为 nv()
   * @param qdd 关节加速度向量，维度必须为 nv()
   * @return 逆动力学力矩向量，维度为 nv()
   */
  Eigen::VectorXd inverseDynamics(
    const Eigen::VectorXd & q,
    const Eigen::VectorXd & qd,
    const Eigen::VectorXd & qdd);

  /**
   * @brief 获取 Pinocchio 配置向量维度
   * @return 配置向量维度 nq
   */
  Eigen::Index nq() const noexcept;

  /**
   * @brief 获取 Pinocchio 速度向量维度
   * @return 速度向量维度 nv
   */
  Eigen::Index nv() const noexcept;

private:
  void validateConfiguration(const Eigen::VectorXd & q) const;
  void validateTangentVector(const Eigen::VectorXd & vector, const char * name) const;

  pinocchio::Model model_;
  pinocchio::Data data_;
};

}  // namespace easyarm_dynamics

#endif  // EASYARM_DYNAMICS_ROBOT_MODEL_HPP_
