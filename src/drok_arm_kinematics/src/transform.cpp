#include "drok_arm_kinematics/transform.hpp"

#include <Eigen/Geometry>

#include <cmath>
#include <stdexcept>

namespace drok_arm_kinematics
{

Eigen::Matrix3d rotationX(double angle)
{
  const double c = std::cos(angle);
  const double s = std::sin(angle);

  Eigen::Matrix3d rotation;

  rotation <<
    1.0, 0.0, 0.0,
    0.0, c, -s,
    0.0, s, c;

  return rotation;
}

Eigen::Matrix3d rotationY(double angle)
{
  const double c = std::cos(angle);
  const double s = std::sin(angle);

  Eigen::Matrix3d rotation;

  rotation <<
    c, 0.0, s,
    0.0, 1.0, 0.0,
    -s, 0.0, c;

  return rotation;
}

Eigen::Matrix3d rotationZ(double angle)
{
  const double c = std::cos(angle);
  const double s = std::sin(angle);

  Eigen::Matrix3d rotation;

  rotation <<
    c, -s, 0.0,
    s, c, 0.0,
    0.0, 0.0, 1.0;

  return rotation;
}

Eigen::Matrix3d rotationFromRpy(
  const Eigen::Vector3d & rpy)
{
  return rotationZ(rpy.z()) *
         rotationY(rpy.y()) *
         rotationX(rpy.x());
}

Eigen::Matrix3d rotationFromAxisAngle(
  const Eigen::Vector3d & axis,
  double angle)
{
  const double norm = axis.norm();

  if (norm < 1e-12) {
    throw std::runtime_error(
            "Axis norm is zero for movable joint.");
  }

  const Eigen::Vector3d normalized_axis =
    axis / norm;

  return Eigen::AngleAxisd(
    angle,
    normalized_axis).toRotationMatrix();
}

Eigen::Matrix4d makeTransform(
  const Eigen::Vector3d & translation,
  const Eigen::Matrix3d & rotation)
{
  Eigen::Matrix4d transform =
    Eigen::Matrix4d::Identity();

  transform.block<3, 3>(0, 0) = rotation;
  transform.block<3, 1>(0, 3) = translation;

  return transform;
}

Eigen::Matrix4d originTransform(
  const Eigen::Vector3d & xyz,
  const Eigen::Vector3d & rpy)
{
  return makeTransform(
    xyz,
    rotationFromRpy(rpy));
}

}  // namespace drok_arm_kinematics
