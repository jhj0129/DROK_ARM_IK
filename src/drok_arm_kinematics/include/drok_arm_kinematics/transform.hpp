#pragma once

#include <Eigen/Dense>

namespace drok_arm_kinematics
{

Eigen::Matrix3d rotationX(double angle);
Eigen::Matrix3d rotationY(double angle);
Eigen::Matrix3d rotationZ(double angle);

Eigen::Matrix3d rotationFromRpy(
  const Eigen::Vector3d & rpy);

Eigen::Matrix3d rotationFromAxisAngle(
  const Eigen::Vector3d & axis,
  double angle);

Eigen::Matrix4d makeTransform(
  const Eigen::Vector3d & translation,
  const Eigen::Matrix3d & rotation);

Eigen::Matrix4d originTransform(
  const Eigen::Vector3d & xyz,
  const Eigen::Vector3d & rpy);

}  // namespace drok_arm_kinematics
