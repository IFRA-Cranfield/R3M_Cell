/*

    ===== COPYRIGHT HERE =====

*/

// ========================================================================================= //
// INCLUDE:

// Include standard libraries:
#include <string>
#include <vector>

// Include to read/write .yaml files:
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

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
#include <linkattacher_msgs/srv/attach_link.hpp>   
#include <linkattacher_msgs/srv/detach_link.hpp>

// Include -> ROS2 Messages:
#include "r3mcell_data/msg/grip.hpp"
#include "r3mcell_data/msg/move.hpp"
#include "r3mcell_data/msg/product.hpp"
#include "r3mcell_data/msg/skillresult.hpp"
#include "geometry_msgs/msg/pose.hpp"

// Declaration of GLOBAL VARIABLES --> INPUT PARAMETERS:
std::string param_ObjectList[100];
std::string param_ROB = "none";
std::string param_EE = "none";
std::string param_OL = "none";

// Declaration of GLOBAL VARIABLES --> MoveIt!2 Interface:
moveit::planning_interface::MoveGroupInterface move_group_interface_ROB;
moveit::planning_interface::MoveGroupInterface move_group_interface_EE;

// Declaration of GLOBAL VARIABLES --> JointModelGroup:
const moveit::core::JointModelGroup* joint_model_group_ROB;
const moveit::core::JointModelGroup* joint_model_group_EE;

// Declaration of GLOBAL VARIABLE --> RES:
std::string RES = "none";

// ========================================================================================= //
// ROBOT + END-EFFECTOR -> Input parameters:

class ros2_RobotParam : public rclcpp::Node
{
public:
    ros2_RobotParam() : Node("ros2_RobotParam") 
    {
        this->declare_parameter("ROB_PARAM", "none");
        param_ROB = this->get_parameter("ROB_PARAM").get_parameter_value().get<std::string>();
        RCLCPP_INFO(this->get_logger(), "ROB_PARAM received -> %s", param_ROB.c_str());
    }
private:
};

class ros2_EEParam : public rclcpp::Node
{
public:
    ros2_EEParam() : Node("ros2_EEParam") 
    {
        this->declare_parameter("EE_PARAM", "none");
        param_EE = this->get_parameter("EE_PARAM").get_parameter_value().get<std::string>();
        RCLCPP_INFO(this->get_logger(), "EE_PARAM received -> %s", param_EE.c_str());
    }
private:
};

// ========================================================================================= //
// ObjectList to monitor -> Input parameter:

class ros2_ObjectListParam : public rclcpp::Node
{
public:
    ros2_ObjectListParam() : Node("ros2_ObjectListParam") 
    {
        this->declare_parameter("OL_PARAM", "none");
        param_OL = this->get_parameter("OL_PARAM").get_parameter_value().get<std::string>();
        RCLCPP_INFO(this->get_logger(), "OL_PARAM received -> %s", param_OL.c_str());
    }
private:
};

// ========================================================================================= //
// PLAN function:

// ROBOT:
moveit::planning_interface::MoveGroupInterface::Plan plan_ROB() {
    
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_interface_ROB.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // Execute the plan
    if (success)
    {
        RES = "PLANNING: OK";
        return(my_plan);
    }
    else
    {
        RES = "PLANNING: ERROR";
        return(my_plan);
    }

};

// END-EFFECTOR:
moveit::planning_interface::MoveGroupInterface::Plan plan_EE() {
    
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_interface_EE.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // Execute the plan
    if (success)
    {
        RES = "PLANNING: OK";
        return(my_plan);
    }
    else
    {
        RES = "PLANNING: ERROR (EE)";
        return(my_plan);
    }
    
};

// ========================================================================================= //
// ExecuteSkill ACTION SERVER:

class ActionServer : public rclcpp::Node
{
public:

    using ExecuteSkill = r3mcell_data::action::ExecuteSkill;
    using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteSkill>;

    explicit ActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("ExecuteSkill_ACTIONSERVER", options)
    {

        action_server_ = rclcpp_action::create_server<ExecuteSkill>(
            this,
            "/ExecuteSkill",
            std::bind(&ActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&ActionServer::handle_accepted, this, std::placeholders::_1));

    }

private:
    rclcpp_action::Server<ExecuteSkill>::SharedPtr action_server_;
    
    // HANDLE GOAL: Check if received recipe EXISTS and ACCEPT/REJECT:
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const ExecuteSkill::Goal> goal)
    {
        // 1. Obtain RECIPE ID + PATH:
        std::string ID = goal->id;
        std::string FilePath = "/recipes/" + ID + ".yaml";
        std::string PackagePath = ament_index_cpp::get_package_share_directory("r3mcell_execution");
        std::string yamlPath = PackagePath + FilePath;

        RCLCPP_INFO(this->get_logger(), "yamlPath: %s", yamlPath.c_str());

        // 2. CHECK if RECIPE exists:
        std::ifstream file(yamlPath);
        bool FileExists = file.good();

        // 3. Open YAML file:
        if (FileExists){
            
            YAML::Node config = YAML::LoadFile(yamlPath);

            // Check SKILL TYPE:
            auto TYPE = config["type"].as<std::string>();
            std::string ACTION;
            double SPEED;

            if (TYPE == "GRIP"){

                ACTION = config["action"].as<std::string>();
                SPEED = config["speed"].as<double>();

            } else {
                
                geometry_msgs::msg::Pose POSE;
                POSE.position.x = config["pose"]["x"].as<double>();
                POSE.position.y = config["pose"]["x"].as<double>();
                POSE.position.z = config["pose"]["x"].as<double>();
                POSE.orientation.x = config["pose"]["qx"].as<double>();
                POSE.orientation.y = config["pose"]["qy"].as<double>();
                POSE.orientation.z = config["pose"]["qz"].as<double>();
                POSE.orientation.w = config["pose"]["qw"].as<double>();

                SPEED = config["speed"].as<double>();
                TYPE = "MOVE: " + TYPE;

                ACTION = "Position: (x: " + std::to_string(POSE.position.x) + ", y: " + std::to_string(POSE.position.y) + ", z: " + std::to_string(POSE.position.z) + ") + Orientation: (x: " + std::to_string(POSE.orientation.x) + ", y: " + std::to_string(POSE.orientation.y) + ", z: " + std::to_string(POSE.orientation.z) + ", w: " + std::to_string(POSE.orientation.w) + ")";

            }

            // LOG and ACCEPT:
            RCLCPP_INFO(this->get_logger(), "ExecuteSkill ACTION: GOAL REQUEST received!");
            RCLCPP_INFO(this->get_logger(), "   - Recipe ID: %s", ID.c_str());
            RCLCPP_INFO(this->get_logger(), "   - Skill type: %s", TYPE.c_str());
            RCLCPP_INFO(this->get_logger(), "   - Input: %s, Speed: %.2f", ACTION.c_str(), SPEED);

            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; 

        } else {

            // REJECT, since file (RECIPE) does not exist:
            RCLCPP_INFO(this->get_logger(), "ExecuteSkill ACTION: ERROR -> Recipe (ID: %s) does not exist!", ID.c_str());
            return rclcpp_action::GoalResponse::REJECT;

        }
        
    }

    // No idea about what this function does:
    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
    {
        // This needs to return quickly to avoid blocking the executor, so spin up a new thread:
        std::thread(
            [this, goal_handle]() {
                execute(goal_handle);
            }).detach();
        
    }

    // Function that cancels the goal request:
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received a cancel request.");

        // We call the -> void moveit::planning_interface::MoveGroupInterface::stop(void) method,
        // which stops any trajectory execution, if one is active.
        if (param_ROB != "none"){
            move_group_interface_ROB.stop();
        }
        if (param_EE != "none"){
            move_group_interface_EE.stop();
        }

        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // MAIN LOOP OF THE ACTION SERVER -> EXECUTION:
    void execute(const std::shared_ptr<GoalHandle> goal_handle)
    {

        const auto goal = goal_handle->get_goal();

        // 1. Obtain RECIPE ID + PATH:
        std::string ID = goal->id;
        std::string FilePath = "/recipes/" + ID + ".yaml";
        std::string PackagePath = ament_index_cpp::get_package_share_directory("r3mcell_execution");
        std::string yamlPath = PackagePath + FilePath;

        // 2. Obtain INPUT VALUES from YAML file:
        YAML::Node config = YAML::LoadFile(yamlPath);

        // Check SKILL TYPE + EXECUTE according to it:
        auto TYPE = config["type"].as<std::string>();
        auto SPEED = config["speed"].as<double>();

        if (TYPE == "GRIP"){

            auto ACTION = config["action"].as<std::string>();

        } else {
            
            geometry_msgs::msg::Pose POSE;
            POSE.position.x = config["pose"]["x"].as<double>();
            POSE.position.y = config["pose"]["x"].as<double>();
            POSE.position.z = config["pose"]["x"].as<double>();
            POSE.orientation.x = config["pose"]["qx"].as<double>();
            POSE.orientation.y = config["pose"]["qy"].as<double>();
            POSE.orientation.z = config["pose"]["qz"].as<double>();
            POSE.orientation.w = config["pose"]["qw"].as<double>();

        }

    }

};

// ==================== MAIN ==================== //

int main(int argc, char ** argv)
{
    // Initialise MAIN NODE:
    rclcpp::init(argc, argv);
    auto const logger = rclcpp::get_logger("r3m_wrapper");

    // Obtain ROBOT + END-EFFECTOR + ENVIRONMENT parameters:
    auto node_PARAM_ROB = std::make_shared<ros2_RobotParam>();
    rclcpp::spin_some(node_PARAM_ROB);
    auto node_PARAM_EE = std::make_shared<ros2_EEParam>();
    rclcpp::spin_some(node_PARAM_EE);
    auto node_PARAM_OL = std::make_shared<ros2_ObjectListParam>();
    rclcpp::spin_some(node_PARAM_OL);

    // Launch and spin (EXECUTOR) MoveIt!2 Interface node:
    auto name = "R3MCell_WRAPPER";
    auto const node2 = std::make_shared<rclcpp::Node>(
        name, rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    rclcpp::executors::SingleThreadedExecutor executor; 
    executor.add_node(node2);
    std::thread([&executor]() { executor.spin(); }).detach();

    // CREATE -> MoveGroupInterface(s):
    using moveit::planning_interface::MoveGroupInterface;
    // 1. ROBOT:
    if (param_ROB != "none"){
        auto name = param_ROB + "_arm";
        
        move_group_interface_ROB = MoveGroupInterface(node2, name);
        move_group_interface_ROB.setPlanningPipelineId("move_group");

        move_group_interface_ROB.setMaxVelocityScalingFactor(1.0);
    
        // ACCELERATION SCALING FACTOR:
        // This value needs to be tuned for the robots, since joint speed/acceleration limits are exceeded otherwise.
        // IRB120:
        if (param_ROB == "irb120"){
            move_group_interface_ROB.setMaxAccelerationScalingFactor(0.5); // AFTER TUNING VALUES in ABB RobotStudio, 0.5 is a reasonable value for the IRB120.
        }
        // UR3 + UR10e:
        else if (param_ROB == "ur3" || param_ROB == "ur10e") {
            move_group_interface_ROB.setMaxAccelerationScalingFactor(0.5);
        }

        joint_model_group_ROB = move_group_interface_ROB.getCurrentState()->getJointModelGroup(name);
        RCLCPP_INFO(logger, "MoveGroupInterface object created for ROBOT: %s", param_ROB.c_str());
    }
    // 2. END-EFFECTOR:
    if (param_EE != "none"){
        move_group_interface_EE = MoveGroupInterface(node2, param_EE);
        move_group_interface_EE.setPlanningPipelineId("move_group");
        move_group_interface_EE.setMaxVelocityScalingFactor(1.0);
        move_group_interface_EE.setMaxAccelerationScalingFactor(1.0);
        joint_model_group_EE = move_group_interface_EE.getCurrentState()->getJointModelGroup(param_EE);
        RCLCPP_INFO(logger, "MoveGroupInterface object created for END-EFFECTOR: %s", param_EE.c_str());
    }

    // CREATE -> PlanningSceneInterface:
    using moveit::planning_interface::PlanningSceneInterface;
    auto planning_scene_interface = PlanningSceneInterface();

    // Declare and spin ACTION SERVER:
    auto action_server = std::make_shared<ActionServer>();
    rclcpp::spin(action_server);

    rclcpp::shutdown();
    return 0;
}