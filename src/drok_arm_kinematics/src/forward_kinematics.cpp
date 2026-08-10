#include "drok_arm_kinematics/forward_kinematics.hpp"

#include "drok_arm_kinematics/transform.hpp"

#include <stdexcept>
#include <utility>

namespace drok_arm_kinematics
{

ForwardKinematics::ForwardKinematics(
  RobotModel model)
: model_(std::move(model))
{
}

Eigen::Matrix4d ForwardKinematics::compute(
  const std::vector<double> & joint_positions) const
{
  if (
    static_cast<int>(joint_positions.size()) !=
    model_.movable_joint_count)
  {
    throw std::runtime_error(
            "Joint position count does not match "
            "the robot movable joint count.");
  }

  Eigen::Matrix4d transform =
    Eigen::Matrix4d::Identity();

  std::size_t movable_index = 0;

  for (const JointModel & joint : model_.chain) {
    transform *= originTransform(
      joint.origin_xyz,
      joint.origin_rpy);

    if (
      joint.type == "revolute" ||
      joint.type == "continuous")
    {
      const Eigen::Matrix3d rotation =
        rotationFromAxisAngle(
        joint.axis,
        joint_positions.at(movable_index));

      transform *= makeTransform(
        Eigen::Vector3d::Zero(),
        rotation);

      ++movable_index;

    } else if (joint.type == "prismatic") {
      const double axis_norm =
        joint.axis.norm();

      if (axis_norm < 1e-12) {
        throw std::runtime_error(
                "Prismatic joint has zero axis: " +
                joint.name);
      }

      const Eigen::Vector3d displacement =
        joint.axis.normalized() *
        joint_positions.at(movable_index);

      transform *= makeTransform(
        displacement,
        Eigen::Matrix3d::Identity());

      ++movable_index;

    } else if (joint.type == "fixed") {
      continue;

    } else {
      throw std::runtime_error(
              "Unsupported joint type: " +
              joint.type);
    }
  }

  if (movable_index != joint_positions.size()) {
    throw std::runtime_error(
            "Movable joint indexing mismatch.");
  }

  return transform;
}

}  // namespace drok_arm_kinematics
