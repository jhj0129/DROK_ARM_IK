#include "drok_arm_kinematics/robot_model_loader.hpp"

#include <yaml-cpp/yaml.h>

#include <stdexcept>
#include <string>

namespace drok_arm_kinematics
{

namespace
{

Eigen::Vector3d readVector3(
  const YAML::Node & node,
  const std::string & context)
{
  if (!node || !node.IsSequence() || node.size() != 3) {
    throw std::runtime_error(
            context + " must contain exactly 3 values.");
  }

  return Eigen::Vector3d(
    node[0].as<double>(),
    node[1].as<double>(),
    node[2].as<double>());
}

}  // namespace

RobotModel RobotModelLoader::loadFromYaml(
  const std::filesystem::path & yaml_path)
{
  const YAML::Node root =
    YAML::LoadFile(yaml_path.string());

  if (!root["robot"]) {
    throw std::runtime_error(
            "Missing 'robot' section in YAML.");
  }

  if (!root["kinematic_chain"]) {
    throw std::runtime_error(
            "Missing 'kinematic_chain' section in YAML.");
  }

  RobotModel model;

  const YAML::Node robot = root["robot"];

  model.name =
    robot["name"].as<std::string>();

  model.base_frame =
    robot["base_frame"].as<std::string>();

  model.tool_frame =
    robot["tool_frame"].as<std::string>();

  model.movable_joint_count =
    robot["movable_joint_count"].as<int>();

  const YAML::Node chain =
    root["kinematic_chain"];

  for (const auto & joint_node : chain) {
    JointModel joint;

    joint.name =
      joint_node["name"].as<std::string>();

    joint.type =
      joint_node["type"].as<std::string>();

    joint.parent =
      joint_node["parent"].as<std::string>();

    joint.child =
      joint_node["child"].as<std::string>();

    joint.origin_xyz = readVector3(
      joint_node["origin"]["xyz"],
      joint.name + ".origin.xyz");

    joint.origin_rpy = readVector3(
      joint_node["origin"]["rpy"],
      joint.name + ".origin.rpy");

    joint.axis = readVector3(
      joint_node["axis"],
      joint.name + ".axis");

    if (joint_node["limit"]) {
      joint.has_limit = true;

      joint.lower =
        joint_node["limit"]["lower"].as<double>();

      joint.upper =
        joint_node["limit"]["upper"].as<double>();

      joint.effort =
        joint_node["limit"]["effort"].as<double>();

      joint.velocity =
        joint_node["limit"]["velocity"].as<double>();
    }

    model.chain.push_back(joint);
  }

  return model;
}

}  // namespace drok_arm_kinematics
