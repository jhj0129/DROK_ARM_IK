#include "drok_arm_kinematics/inverse_kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace drok_arm_kinematics
{

InverseKinematics::InverseKinematics(
  const RobotModel & model,
  const IkOptions & options)
: model_(model),
  forward_kinematics_(model),
  options_(options)
{
  if (model_.movable_joint_count <= 0) {
    throw std::runtime_error(
            "InverseKinematics requires at least one movable joint.");
  }

  if (options_.max_iterations == 0) {
    throw std::runtime_error(
            "IK max_iterations must be greater than zero.");
  }

  if (options_.damping <= 0.0) {
    throw std::runtime_error(
            "IK damping must be greater than zero.");
  }

  if (options_.numerical_delta <= 0.0) {
    throw std::runtime_error(
            "IK numerical_delta must be greater than zero.");
  }

  if (options_.maximum_joint_step <= 0.0) {
    throw std::runtime_error(
            "IK maximum_joint_step must be greater than zero.");
  }

  if (options_.seed_continuity_gain < 0.0) {
    throw std::runtime_error(
            "IK seed_continuity_gain must be non-negative.");
  }

  if (
    options_.seed_continuity_activation_error <=
    0.0)
  {
    throw std::runtime_error(
            "IK seed_continuity_activation_error "
            "must be greater than zero.");
  }
}

std::vector<const JointModel *>
InverseKinematics::movableJoints() const
{
  std::vector<const JointModel *> joints;

  joints.reserve(
    static_cast<std::size_t>(
      model_.movable_joint_count));

  for (const auto & joint : model_.chain) {
    if (
      joint.type == "revolute" ||
      joint.type == "continuous" ||
      joint.type == "prismatic")
    {
      joints.push_back(&joint);
    }
  }

  return joints;
}

Eigen::Vector3d InverseKinematics::rotationLogarithm(
  const Eigen::Matrix3d & rotation) const
{
  const double cosine_angle =
    std::clamp(
    (rotation.trace() - 1.0) * 0.5,
    -1.0,
    1.0);

  const double angle =
    std::acos(cosine_angle);

  if (angle < 1.0e-9) {
    return Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d axis;

  axis <<
    rotation(2, 1) - rotation(1, 2),
    rotation(0, 2) - rotation(2, 0),
    rotation(1, 0) - rotation(0, 1);

  const double denominator =
    2.0 * std::sin(angle);

  if (std::abs(denominator) < 1.0e-9) {
    Eigen::AngleAxisd angle_axis(rotation);

    return
      angle_axis.axis() *
      angle_axis.angle();
  }

  axis /= denominator;

  return axis * angle;
}

Eigen::Matrix<double, 6, 1>
InverseKinematics::computePoseError(
  const Eigen::Matrix4d & current_transform,
  const Eigen::Matrix4d & target_transform) const
{
  Eigen::Matrix<double, 6, 1> error;

  const Eigen::Vector3d current_position =
    current_transform.block<3, 1>(0, 3);

  const Eigen::Vector3d target_position =
    target_transform.block<3, 1>(0, 3);

  const Eigen::Matrix3d current_rotation =
    current_transform.block<3, 3>(0, 0);

  const Eigen::Matrix3d target_rotation =
    target_transform.block<3, 3>(0, 0);

  error.head<3>() =
    options_.position_weight *
    (target_position - current_position);

  const Eigen::Matrix3d rotation_error =
    target_rotation *
    current_rotation.transpose();

  error.tail<3>() =
    options_.orientation_weight *
    rotationLogarithm(rotation_error);

  return error;
}

Eigen::MatrixXd InverseKinematics::computeNumericalJacobian(
  const std::vector<double> & joint_positions,
  const Eigen::Matrix4d & current_transform) const
{
  const std::size_t joint_count =
    joint_positions.size();

  Eigen::MatrixXd jacobian(
    6,
    static_cast<Eigen::Index>(joint_count));

  const Eigen::Vector3d current_position =
    current_transform.block<3, 1>(0, 3);

  const Eigen::Matrix3d current_rotation =
    current_transform.block<3, 3>(0, 0);

  for (std::size_t index = 0;
    index < joint_count;
    ++index)
  {
    std::vector<double> perturbed_positions =
      joint_positions;

    perturbed_positions[index] +=
      options_.numerical_delta;

    const Eigen::Matrix4d perturbed_transform =
      forward_kinematics_.compute(
      perturbed_positions);

    const Eigen::Vector3d perturbed_position =
      perturbed_transform.block<3, 1>(0, 3);

    const Eigen::Matrix3d perturbed_rotation =
      perturbed_transform.block<3, 3>(0, 0);

    jacobian.block<3, 1>(
      0,
      static_cast<Eigen::Index>(index)) =
      options_.position_weight *
      (perturbed_position - current_position) /
      options_.numerical_delta;

    const Eigen::Matrix3d delta_rotation =
      perturbed_rotation *
      current_rotation.transpose();

    jacobian.block<3, 1>(
      3,
      static_cast<Eigen::Index>(index)) =
      options_.orientation_weight *
      rotationLogarithm(delta_rotation) /
      options_.numerical_delta;
  }

  return jacobian;
}

void InverseKinematics::applyJointLimits(
  std::vector<double> & joint_positions) const
{
  const auto movable_joints =
    movableJoints();

  if (joint_positions.size() !=
    movable_joints.size())
  {
    throw std::runtime_error(
            "Joint position count does not match "
            "the movable joint count.");
  }

  for (std::size_t index = 0;
    index < joint_positions.size();
    ++index)
  {
    const JointModel & joint =
      *movable_joints[index];

    if (
      joint.type == "continuous" ||
      !joint.has_limit)
    {
      continue;
    }

    joint_positions[index] =
      std::clamp(
      joint_positions[index],
      joint.lower,
      joint.upper);
  }
}

IkResult InverseKinematics::solve(
  const Eigen::Matrix4d & target_transform,
  const std::vector<double> &
  initial_joint_positions) const
{
  IkResult result;

  const std::size_t expected_joint_count =
    static_cast<std::size_t>(
    model_.movable_joint_count);

  if (
    initial_joint_positions.size() !=
    expected_joint_count)
  {
    result.message =
      "Initial joint position count does not "
      "match the robot model.";

    return result;
  }

  std::vector<double> joint_positions =
    initial_joint_positions;

  applyJointLimits(joint_positions);

  // Keep the supplied initial posture as the continuity reference.
  // This reference remains fixed for one solve() call.
  const std::vector<double>
    reference_joint_positions =
    joint_positions;

  for (std::size_t iteration = 0;
    iteration < options_.max_iterations;
    ++iteration)
  {
    const Eigen::Matrix4d current_transform =
      forward_kinematics_.compute(
      joint_positions);

    const Eigen::Matrix<double, 6, 1> error =
      computePoseError(
      current_transform,
      target_transform);

    const double position_error =
      (
      target_transform.block<3, 1>(0, 3) -
      current_transform.block<3, 1>(0, 3)
      ).norm();

    const Eigen::Matrix3d rotation_difference =
      target_transform.block<3, 3>(0, 0) *
      current_transform.block<3, 3>(
      0, 0).transpose();

    const double orientation_error =
      rotationLogarithm(
      rotation_difference).norm();

    result.iterations = iteration;
    result.position_error = position_error;
    result.orientation_error =
      orientation_error;

    if (
      position_error <=
      options_.position_tolerance &&
      orientation_error <=
      options_.orientation_tolerance)
    {
      result.success = true;
      result.joint_positions =
        joint_positions;
      result.message = "IK converged.";

      return result;
    }

    const Eigen::MatrixXd jacobian =
      computeNumericalJacobian(
      joint_positions,
      current_transform);

    const Eigen::Matrix<double, 6, 6>
      task_identity =
      Eigen::Matrix<double, 6, 6>::Identity();

    const Eigen::Matrix<double, 6, 6>
      regularized_matrix =
      jacobian * jacobian.transpose() +
      std::pow(options_.damping, 2.0) *
      task_identity;

    const Eigen::LDLT<
      Eigen::Matrix<double, 6, 6>>
      decomposition(regularized_matrix);

    if (
      decomposition.info() !=
      Eigen::Success)
    {
      result.message =
        "IK regularized matrix decomposition failed.";

      result.joint_positions =
        joint_positions;

      return result;
    }

    // Damped least-squares pseudoinverse:
    //
    // J# = J^T (J J^T + lambda^2 I)^-1
    const Eigen::MatrixXd
      damped_pseudoinverse =
      jacobian.transpose() *
      decomposition.solve(task_identity);

    Eigen::VectorXd delta_joint_positions =
      damped_pseudoinverse * error;

    if (
      options_.seed_continuity_gain >
      0.0)
    {
      Eigen::VectorXd reference_error(
        static_cast<Eigen::Index>(
          joint_positions.size()));

      for (std::size_t index = 0;
        index < joint_positions.size();
        ++index)
      {
        reference_error[
          static_cast<Eigen::Index>(index)] =
          reference_joint_positions[index] -
          joint_positions[index];
      }

      // Damped null-space projector:
      //
      // N = I - J# J
      //
      // Near a wrist singularity this allows the solver to prefer
      // the branch closest to the supplied seed without replacing
      // the primary Cartesian task.
      const Eigen::MatrixXd
        joint_identity =
        Eigen::MatrixXd::Identity(
        static_cast<Eigen::Index>(
          joint_positions.size()),
        static_cast<Eigen::Index>(
          joint_positions.size()));

      const Eigen::MatrixXd
        null_space_projector =
        joint_identity -
        damped_pseudoinverse * jacobian;

      // Fade the posture term to zero near convergence.
      // This prevents the reference posture from creating a fixed
      // residual pose error.
      const double continuity_scale =
        std::min(
        1.0,
        error.norm() /
        options_.
        seed_continuity_activation_error);

      delta_joint_positions +=
        continuity_scale *
        options_.seed_continuity_gain *
        null_space_projector *
        reference_error;
    }

    if (!delta_joint_positions.allFinite()) {
      result.message =
        "IK produced a non-finite joint step.";

      result.joint_positions =
        joint_positions;

      return result;
    }

    const double maximum_absolute_step =
      delta_joint_positions
      .cwiseAbs()
      .maxCoeff();

    if (
      maximum_absolute_step >
      options_.maximum_joint_step)
    {
      delta_joint_positions *=
        options_.maximum_joint_step /
        maximum_absolute_step;
    }

    for (std::size_t index = 0;
      index < joint_positions.size();
      ++index)
    {
      joint_positions[index] +=
        delta_joint_positions[
        static_cast<Eigen::Index>(index)];
    }

    applyJointLimits(joint_positions);
  }

  const Eigen::Matrix4d final_transform =
    forward_kinematics_.compute(
    joint_positions);

  result.success = false;
  result.joint_positions =
    std::move(joint_positions);

  result.position_error =
    (
    target_transform.block<3, 1>(0, 3) -
    final_transform.block<3, 1>(0, 3)
    ).norm();

  const Eigen::Matrix3d final_rotation_error =
    target_transform.block<3, 3>(0, 0) *
    final_transform.block<3, 3>(
    0, 0).transpose();

  result.orientation_error =
    rotationLogarithm(
    final_rotation_error).norm();

  result.iterations =
    options_.max_iterations;

  result.message =
    "IK reached the maximum iteration count.";

  return result;
}

}  // namespace drok_arm_kinematics
