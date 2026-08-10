#include "drok_arm_kinematics/forward_kinematics.hpp"
#include "drok_arm_kinematics/inverse_kinematics.hpp"
#include "drok_arm_kinematics/robot_model_loader.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cmath>
#include <cstdlib>
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
  const std::string & name)
{
  try {
    std::size_t parsed_length = 0;

    const std::string value_string(text);

    const double value =
      std::stod(value_string, &parsed_length);

    if (parsed_length != value_string.size()) {
      throw std::runtime_error(
              "Unexpected trailing characters.");
    }

    if (!std::isfinite(value)) {
      throw std::runtime_error(
              "Value is not finite.");
    }

    return value;

  } catch (const std::exception &) {
    throw std::runtime_error(
            "Invalid numerical value for " +
            name + ": " + text);
  }
}

Eigen::Matrix3d rotationFromRpy(
  const double roll,
  const double pitch,
  const double yaw)
{
  const Eigen::AngleAxisd roll_rotation(
    roll,
    Eigen::Vector3d::UnitX());

  const Eigen::AngleAxisd pitch_rotation(
    pitch,
    Eigen::Vector3d::UnitY());

  const Eigen::AngleAxisd yaw_rotation(
    yaw,
    Eigen::Vector3d::UnitZ());

  return
    (
    yaw_rotation *
    pitch_rotation *
    roll_rotation
    ).toRotationMatrix();
}

void printUsage()
{
  std::cerr
    << "Usage:\n\n"
    << "  Zero initial pose:\n"
    << "    solve_ik_pose <robot_geometry.yaml> "
    << "<x> <y> <z> <roll> <pitch> <yaw>\n\n"
    << "  Custom initial pose:\n"
    << "    solve_ik_pose <robot_geometry.yaml> "
    << "<x> <y> <z> <roll> <pitch> <yaw> "
    << "<q1> <q2> <q3> <q4> <q5> <q6>\n\n"
    << "Units:\n"
    << "  position    : meter\n"
    << "  orientation : radian\n"
    << "  joint angle : radian\n";
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 8 && argc != 14) {
    printUsage();
    return 1;
  }

  try {
    const std::filesystem::path yaml_path =
      argv[1];

    const double target_x =
      parseDouble(argv[2], "x");

    const double target_y =
      parseDouble(argv[3], "y");

    const double target_z =
      parseDouble(argv[4], "z");

    const double target_roll =
      parseDouble(argv[5], "roll");

    const double target_pitch =
      parseDouble(argv[6], "pitch");

    const double target_yaw =
      parseDouble(argv[7], "yaw");

    const auto model =
      drok_arm_kinematics::RobotModelLoader::
      loadFromYaml(yaml_path);

    if (model.movable_joint_count != 6) {
      throw std::runtime_error(
              "This executable currently expects "
              "exactly six movable joints.");
    }

    std::vector<double> initial_joint_positions(
      6,
      0.0);

    if (argc == 14) {
      for (std::size_t index = 0;
        index < initial_joint_positions.size();
        ++index)
      {
        initial_joint_positions[index] =
          parseDouble(
          argv[index + 8],
          "q" + std::to_string(index + 1));
      }
    }

    Eigen::Matrix4d target_transform =
      Eigen::Matrix4d::Identity();

    target_transform.block<3, 3>(0, 0) =
      rotationFromRpy(
      target_roll,
      target_pitch,
      target_yaw);

    target_transform(0, 3) = target_x;
    target_transform(1, 3) = target_y;
    target_transform(2, 3) = target_z;

    drok_arm_kinematics::IkOptions options;

    const char * ik_mode_environment =
      std::getenv("DROK_IK_MODE");

    const std::string ik_mode =
      ik_mode_environment == nullptr ?
      "position" :
      std::string(ik_mode_environment);

    if (
      ik_mode != "position" &&
      ik_mode != "full")
    {
      throw std::runtime_error(
              "DROK_IK_MODE must be "
              "'position' or 'full'.");
    }

    options.max_iterations = 1500;

    options.position_tolerance = 1.0e-5;
    options.damping = 1.0e-2;
    options.numerical_delta = 1.0e-6;
    options.position_weight = 1.0;
    options.maximum_joint_step = 0.05;

    if (ik_mode == "full") {
      // Exact full-pose IK:
      // used for cube-face alignment and X-axis approach.
      options.orientation_tolerance = 1.0e-5;
      options.orientation_weight = 0.5;

      // A six-axis full-pose task generally leaves almost
      // no null space, so the nearest solution is selected
      // outside the solver by comparing multiple IK results.
      options.seed_continuity_gain = 0.0;
      options.seed_continuity_activation_error = 0.05;

    } else {
      // Position-only IK:
      // used only for unconstrained free-space movement.
      options.orientation_tolerance = 4.0;
      options.orientation_weight = 0.0;

      options.seed_continuity_gain = 0.20;
      options.seed_continuity_activation_error = 0.05;
    }

    const drok_arm_kinematics::InverseKinematics
      inverse_kinematics(
      model,
      options);

    const auto result =
      inverse_kinematics.solve(
      target_transform,
      initial_joint_positions);

    const drok_arm_kinematics::ForwardKinematics
      forward_kinematics(model);

    const Eigen::Matrix4d solved_transform =
      forward_kinematics.compute(
      result.joint_positions);

    std::cout
      << std::fixed
      << std::setprecision(9);

    std::cout
      << "========================================\n"
      << " DROK TCP Pose IK Solver\n"
      << "========================================\n"
      << "IK mode: " << ik_mode << '\n'
      << "----------------------------------------\n"
      << "Target position [m]\n"
      << "x = " << target_x << '\n'
      << "y = " << target_y << '\n'
      << "z = " << target_z << '\n'
      << "----------------------------------------\n"
      << "Target RPY [rad]\n"
      << "roll  = " << target_roll << '\n'
      << "pitch = " << target_pitch << '\n'
      << "yaw   = " << target_yaw << '\n'
      << "----------------------------------------\n"
      << "Initial joint positions [rad]\n";

    for (std::size_t index = 0;
      index < initial_joint_positions.size();
      ++index)
    {
      std::cout
        << "q" << index + 1
        << " = "
        << initial_joint_positions[index]
        << '\n';
    }

    std::cout
      << "----------------------------------------\n"
      << "IK result joint positions [rad]\n";

    for (std::size_t index = 0;
      index < result.joint_positions.size();
      ++index)
    {
      std::cout
        << "q" << index + 1
        << " = "
        << result.joint_positions[index]
        << '\n';
    }

    std::cout
      << "----------------------------------------\n"
      << "Target transform\n"
      << target_transform << '\n'
      << "----------------------------------------\n"
      << "Solved transform\n"
      << solved_transform << '\n'
      << "----------------------------------------\n"
      << "Success           : "
      << std::boolalpha
      << result.success << '\n'
      << "Iterations        : "
      << result.iterations << '\n'
      << "Position error    : "
      << result.position_error
      << " m\n"
      << "Orientation error : "
      << result.orientation_error
      << " rad\n"
      << "Message           : "
      << result.message << '\n'
      << "----------------------------------------\n"
      << "Machine-readable joint result\n"
      << "JOINT_RESULT=";

    for (std::size_t index = 0;
      index < result.joint_positions.size();
      ++index)
    {
      if (index > 0) {
        std::cout << ',';
      }

      std::cout << result.joint_positions[index];
    }

    std::cout
      << '\n'
      << "========================================\n";

    return result.success ? 0 : 2;

  } catch (const std::exception & exception) {
    std::cerr
      << "[ERROR] "
      << exception.what()
      << '\n';

    return 1;
  }
}
