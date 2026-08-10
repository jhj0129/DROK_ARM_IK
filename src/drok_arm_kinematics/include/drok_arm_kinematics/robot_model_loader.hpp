#pragma once

#include "drok_arm_kinematics/robot_model.hpp"

#include <filesystem>

namespace drok_arm_kinematics
{

class RobotModelLoader
{
public:
  static RobotModel loadFromYaml(
    const std::filesystem::path & yaml_path);
};

}  // namespace drok_arm_kinematics
