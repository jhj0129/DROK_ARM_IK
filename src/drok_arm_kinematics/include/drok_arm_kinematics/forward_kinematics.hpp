#pragma once

#include "drok_arm_kinematics/robot_model.hpp"

#include <Eigen/Dense>

#include <vector>

namespace drok_arm_kinematics
{

class ForwardKinematics
{
public:
  explicit ForwardKinematics(RobotModel model);

  Eigen::Matrix4d compute(
    const std::vector<double> & joint_positions) const;

private:
  RobotModel model_;
};

}  // namespace drok_arm_kinematics
