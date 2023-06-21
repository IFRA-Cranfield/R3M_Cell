/*

    ===== COPYRIGHT HERE =====

*/

// ========================================================================================= //
// INCLUDE:

// Include standard libraries:
#include <string>
#include <vector>

// Include RCLCPP and RCLCPP_ACTION:
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// Include MoveIt!2:
#include <moveit/move_group_interface/move_group_interface_improved.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// Include -> ROS2 Actions:
#include "r3mcell_data/action/execute_skill.hpp"

// Include -> ROS2 Services:
#include "r3mcell_data/srv/object_list.hpp"

// Include -> ROS2 Messages:
#include "r3mcell_data/msg/grip.hpp"
#include "r3mcell_data/msg/move.hpp"
#include "r3mcell_data/msg/product.hpp"
#include "r3mcell_data/msg/skillresult.hpp"

// Declaration of GLOBAL VARIABLES --> INPUT PARAMETERS:
std::string[100] param_ObjectList;

// Declaration of GLOBAL VARIABLES --> MoveIt!2 Interface:
moveit::planning_interface::MoveGroupInterface move_group_interface_ROB;
moveit::planning_interface::MoveGroupInterface move_group_interface_EE;

// Declaration of GLOBAL VARIABLES --> JointModelGroup:
const moveit::core::JointModelGroup* joint_model_group_ROB;
const moveit::core::JointModelGroup* joint_model_group_EE;

// Declaration of GLOBAL VARIABLE --> RES:
std::string RES = "none";