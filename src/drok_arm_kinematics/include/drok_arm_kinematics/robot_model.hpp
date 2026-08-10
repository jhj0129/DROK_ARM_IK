#pragma once

#include <Eigen/Dense>

#include <string>
#include <vector>

namespace drok_arm_kinematics
{

struct JointModel
{
  std::string name;
  std::string type;
  std::string parent;
  std::string child;

  Eigen::Vector3d origin_xyz{Eigen::Vector3d::Zero()};
  Eigen::Vector3d origin_rpy{Eigen::Vector3d::Zero()};
  Eigen::Vector3d axis{Eigen::Vector3d::Zero()};

  bool has_limit{false};
  double lower{0.0};
  double upper{0.0};
  double effort{0.0};
  double velocity{0.0};
};

struct RobotModel
{
  std::string name;
  std::string base_frame;
  std::string tool_frame;

  int movable_joint_count{0};

  std::vector<JointModel> chain;
};

}  // namespace drok_arm_kinematics
