#ifndef DROK_ARM_KINEMATICS__INVERSE_KINEMATICS_HPP_
#define DROK_ARM_KINEMATICS__INVERSE_KINEMATICS_HPP_

#include "drok_arm_kinematics/forward_kinematics.hpp"
#include "drok_arm_kinematics/robot_model.hpp"

#include <Eigen/Dense>

#include <cstddef>
#include <string>
#include <vector>

namespace drok_arm_kinematics
{

struct IkOptions
{
  std::size_t max_iterations{500};

  double position_tolerance{1.0e-5};
  double orientation_tolerance{1.0e-5};

  double damping{1.0e-2};
  double numerical_delta{1.0e-6};

  double position_weight{1.0};
  double orientation_weight{0.5};

  double maximum_joint_step{0.10};

  // Initial seed continuity.
  //
  // Zero preserves the original solver behavior.
  // A positive value biases the numerical IK toward the supplied
  // initial joint configuration through the damped null space.
  double seed_continuity_gain{0.0};

  // The seed-continuity term is gradually reduced as task error
  // approaches zero so exact pose convergence remains possible.
  double seed_continuity_activation_error{0.25};
};

struct IkResult
{
  bool success{false};

  std::vector<double> joint_positions;

  std::size_t iterations{0};

  double position_error{0.0};
  double orientation_error{0.0};

  std::string message;
};

class InverseKinematics
{
public:
  explicit InverseKinematics(
    const RobotModel & model,
    const IkOptions & options = IkOptions());

  IkResult solve(
    const Eigen::Matrix4d & target_transform,
    const std::vector<double> & initial_joint_positions) const;

private:
  Eigen::Matrix<double, 6, 1> computePoseError(
    const Eigen::Matrix4d & current_transform,
    const Eigen::Matrix4d & target_transform) const;

  Eigen::MatrixXd computeNumericalJacobian(
    const std::vector<double> & joint_positions,
    const Eigen::Matrix4d & current_transform) const;

  Eigen::Vector3d rotationLogarithm(
    const Eigen::Matrix3d & rotation) const;

  void applyJointLimits(
    std::vector<double> & joint_positions) const;

  std::vector<const JointModel *> movableJoints() const;

  RobotModel model_;
  ForwardKinematics forward_kinematics_;
  IkOptions options_;
};

}  // namespace drok_arm_kinematics

#endif  // DROK_ARM_KINEMATICS__INVERSE_KINEMATICS_HPP_
