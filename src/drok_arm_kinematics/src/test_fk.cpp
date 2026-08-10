#include "drok_arm_kinematics/forward_kinematics.hpp"
#include "drok_arm_kinematics/robot_model_loader.hpp"

#include <Eigen/Dense>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

double parseDouble(
  const char * text,
  const std::string & argument_name)
{
  try {
    std::size_t parsed_length = 0;

    const double value =
      std::stod(text, &parsed_length);

    if (parsed_length != std::string(text).size()) {
      throw std::runtime_error(
              "Unexpected characters");
    }

    if (!std::isfinite(value)) {
      throw std::runtime_error(
              "Value is not finite");
    }

    return value;

  } catch (const std::exception &) {
    throw std::runtime_error(
            "Invalid numerical value for " +
            argument_name + ": " + text);
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 2 && argc != 8) {
    std::cerr
      << "Usage:\n"
      << "  Zero pose:\n"
      << "    test_fk <robot_geometry.yaml>\n\n"
      << "  Custom pose:\n"
      << "    test_fk <robot_geometry.yaml> "
      << "<q1> <q2> <q3> <q4> <q5> <q6>\n";

    return 1;
  }

  try {
    const std::filesystem::path yaml_path =
      argv[1];

    const auto model =
      drok_arm_kinematics::RobotModelLoader::
      loadFromYaml(yaml_path);

    if (model.movable_joint_count != 6) {
      throw std::runtime_error(
              "This FK test currently expects exactly "
              "six movable joints.");
    }

    std::vector<double> joint_positions(
      static_cast<std::size_t>(
        model.movable_joint_count),
      0.0);

    if (argc == 8) {
      for (std::size_t index = 0;
        index < joint_positions.size();
        ++index)
      {
        joint_positions[index] =
          parseDouble(
          argv[index + 2],
          "q" + std::to_string(index + 1));
      }
    }

    std::size_t movable_index = 0;

    for (const auto & joint : model.chain) {
      if (
        joint.type != "revolute" &&
        joint.type != "continuous" &&
        joint.type != "prismatic")
      {
        continue;
      }

      const double position =
        joint_positions.at(movable_index);

      if (
        joint.has_limit &&
        joint.type != "continuous" &&
        (position < joint.lower ||
        position > joint.upper))
      {
        throw std::runtime_error(
                joint.name +
                " is outside its joint limit. q=" +
                std::to_string(position) +
                ", lower=" +
                std::to_string(joint.lower) +
                ", upper=" +
                std::to_string(joint.upper));
      }

      ++movable_index;
    }

    const drok_arm_kinematics::ForwardKinematics
      fk(model);

    const auto start =
      std::chrono::steady_clock::now();

    const Eigen::Matrix4d transform =
      fk.compute(joint_positions);

    const auto end =
      std::chrono::steady_clock::now();

    const double elapsed_us =
      std::chrono::duration<double, std::micro>(
      end - start).count();

    const Eigen::Vector3d position =
      transform.block<3, 1>(0, 3);

    const Eigen::Matrix3d rotation =
      transform.block<3, 3>(0, 0);

    const Eigen::Vector3d zyx =
      rotation.eulerAngles(2, 1, 0);

    const Eigen::Vector3d rpy(
      zyx.z(),
      zyx.y(),
      zyx.x());

    std::cout
      << std::fixed
      << std::setprecision(15);

    std::cout
      << "========================================\n"
      << " DROK Forward Kinematics Test\n"
      << "========================================\n"
      << "Robot       : " << model.name << '\n'
      << "Base frame  : " << model.base_frame << '\n'
      << "Tool frame  : " << model.tool_frame << '\n'
      << "Joint count : "
      << model.movable_joint_count << '\n'
      << "----------------------------------------\n"
      << "Joint positions [rad]\n";

    for (std::size_t index = 0;
      index < joint_positions.size();
      ++index)
    {
      std::cout
        << "q" << index + 1
        << " = "
        << joint_positions[index]
        << '\n';
    }

    std::cout
      << "----------------------------------------\n"
      << "Transformation matrix\n"
      << transform << '\n'
      << "----------------------------------------\n"
      << "TCP position [m]\n"
      << "x = " << position.x() << '\n'
      << "y = " << position.y() << '\n'
      << "z = " << position.z() << '\n'
      << "----------------------------------------\n"
      << "TCP RPY [rad]\n"
      << "roll  = " << rpy.x() << '\n'
      << "pitch = " << rpy.y() << '\n'
      << "yaw   = " << rpy.z() << '\n'
      << "----------------------------------------\n"
      << "FK calculation time = "
      << elapsed_us << " us\n"
      << "========================================\n";

  } catch (const std::exception & exception) {
    std::cerr
      << "[ERROR] "
      << exception.what()
      << '\n';

    return 1;
  }

  return 0;
}
