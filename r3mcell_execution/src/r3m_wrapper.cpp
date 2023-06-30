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

// Include to calculate execution time:
#include <chrono>

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
#include "r3mcell_data/msg/product.hpp"
#include "r3mcell_data/msg/skillresult.hpp"
#include "r3mcell_data/msg/pose.hpp"
#include "objectpose_msgs/msg/object_pose.hpp"
#include "linkpose_msgs/msg/link_pose.hpp"

// Declaration of GLOBAL VARIABLES --> INPUT PARAMETERS:
std::string param_ROB = "none";
std::string param_EE = "none";
std::vector<std::string> param_OL(100);

// Declaration of GLOBAL VARIABLE --> ObjectPoseVECTOR:
using DataType = objectpose_msgs::msg::ObjectPose;
std::vector<DataType> ObjectPoseVECTOR;
std::vector<DataType> PreviousPoseVECTOR;
// Declaration of GLOBAL VARIABLE --> ObjectPoseSUB NODE:
std::shared_ptr<rclcpp::Node> node_ObjectPoseSUB;

// Declaration of GLOBAL VARIABLE --> EEPose:
linkpose_msgs::msg::LinkPose EEPose;
// Declaration of GLOBAL VARIABLE --> EEPoseSUB NODE:
std::shared_ptr<rclcpp::Node> node_EEPoseSUB;

// Declaration of GLOBAL VARIABLES --> MoveIt!2 Interface:
moveit::planning_interface::MoveGroupInterface move_group_interface_ROB;
moveit::planning_interface::MoveGroupInterface move_group_interface_EE;

// Declaration of GLOBAL VARIABLES --> JointModelGroup:
const moveit::core::JointModelGroup* joint_model_group_ROB;
const moveit::core::JointModelGroup* joint_model_group_EE;

// Declaration of GLOBAL VARIABLES --> Attacher & Detacher:
std::shared_ptr<rclcpp::Node> AttacherNode;
std::shared_ptr<rclcpp::Node> DetacherNode;

// Declaration of GLOBAL VARIABLE --> RES:
std::string RES = "none";

// DECLARE GLOBAL --> LOGGER;
auto const GLOBAL_LOG = rclcpp::get_logger("r3m_wrapper");

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
        this->declare_parameter("OL_PARAM", param_OL);
        param_OL = this->get_parameter("OL_PARAM").get_parameter_value().get<std::vector<std::string>>();
        RCLCPP_INFO(this->get_logger(), "OL_PARAM received:");

        int N = param_OL.size()/2;
        for (int i=1; i<=N; i++){
            int k = (i-1)*2;
            RCLCPP_INFO(this->get_logger(), "OBJECT N%i -> MODEL: %s, LINK: %s", i, param_OL[k].c_str(), param_OL[k+1].c_str());
        }

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
        RES = "PLANNING: OK (EE)";
        return(my_plan);
    }
    else
    {
        RES = "PLANNING: ERROR (EE)";
        return(my_plan);
    }
    
};

// ========================================================================================= //
// Calculate ACCURACY:

// ROBOT:
std::vector<double> ERROR_ROB(geometry_msgs::msg::Pose TARGET_POSE){

    std::vector<double> ERROR_ROB = {-1.0, -1.0};

    auto CURRENT_POSE = move_group_interface_ROB.getCurrentPose();

    // POSITION ERROR -> Norm of the (p1-p0) difference vector:
    double DIF_x = abs(CURRENT_POSE.pose.position.x - TARGET_POSE.position.x);
    double DIF_y = abs(CURRENT_POSE.pose.position.y - TARGET_POSE.position.y);
    double DIF_z = abs(CURRENT_POSE.pose.position.z - TARGET_POSE.position.z);

    ERROR_ROB[0] = sqrt(DIF_x*DIF_x + DIF_y*DIF_y + DIF_z*DIF_z);
    
    // ROTATION ERROR -> Norm of the (C = A*inv(B)) difference quaternion:
    double Ax = CURRENT_POSE.pose.orientation.x;
    double Ay = CURRENT_POSE.pose.orientation.y;
    double Az = CURRENT_POSE.pose.orientation.z;
    double Aw = CURRENT_POSE.pose.orientation.w;

    double Bx = -TARGET_POSE.orientation.x;
    double By = -TARGET_POSE.orientation.y;
    double Bz = -TARGET_POSE.orientation.z;
    double Bw = TARGET_POSE.orientation.w;

    double qw = Aw*Bw - Ax*Bx - Ay*By - Az*Bz;
    double qx = Aw*Bx + Ax*Bw + Ay*Bz - Az*By;
    double qy = Aw*By - Ax*Bz + Ay*Bw + Az*Bx;
    double qz = Aw*Bz + Ax*By - Ay*Bx + Az*Bw; 

    ERROR_ROB[1] = sqrt((qx*qx)+(qy*qy)+(qz*qz));

    return(ERROR_ROB);

}

// END-EFFECTOR:
std::vector<double> ERROR_EE(std::vector<double> TARGET_JP){

    std::vector<double> ERROR_EE = {-1.0};
    
    std::vector<double> JP;
    moveit::core::RobotStatePtr current_state = move_group_interface_EE.getCurrentState(10);
    current_state->copyJointGroupPositions(joint_model_group_EE, JP);

    double DIF_00 = abs(JP[0] - TARGET_JP[0]);
    double DIF_01 = abs(JP[1] - TARGET_JP[1]);

    ERROR_EE[0] = (DIF_00 + DIF_01) / 2;
    return(ERROR_EE);

}

// ========================================================================================= //
// Subscribe to ObjectPose:

class ObjectPose_Subscriber : public rclcpp::Node
{
    public:

        ObjectPose_Subscriber() : Node("ObjectPose_Subscriber"){

            objectpose_msgs::msg::ObjectPose EmptyPose;

            int N = param_OL.size()/2;
            for (int i=1; i<=N; i++){
                int k = (i-1)*2;

                std::string TopicName = param_OL[k] + "/ObjectPose";
                SubscriberVECTOR.push_back(this->create_subscription<objectpose_msgs::msg::ObjectPose>(TopicName, 10, std::bind(&ObjectPose_Subscriber::CALLBACK_FN, this, std::placeholders::_1)));

                EmptyPose.objectname = param_OL[k];
                EmptyPose.x = 0.0;
                EmptyPose.y = 0.0;
                EmptyPose.z = 0.0;
                EmptyPose.qx = 0.0;
                EmptyPose.qy = 0.0;
                EmptyPose.qz = 0.0;
                EmptyPose.qw = 0.0;

                PreviousPoseVECTOR.push_back(EmptyPose);
                ObjectPoseVECTOR.push_back(EmptyPose);

            }

        }

    private:

        void CALLBACK_FN(const objectpose_msgs::msg::ObjectPose msg) const
        {

            int N = param_OL.size()/2;
            for (int i=0; i<N; i++){

                if (msg.objectname == ObjectPoseVECTOR[i].objectname){
                    PreviousPoseVECTOR[i] = ObjectPoseVECTOR[i];
                    ObjectPoseVECTOR[i] = msg;
                }

            }
        }

        std::vector<rclcpp::Subscription<objectpose_msgs::msg::ObjectPose>::SharedPtr> SubscriberVECTOR;
};

// ========================================================================================= //
// End-Effector POSE:

class EEPose_Subscriber : public rclcpp::Node
{
    public:

        EEPose_Subscriber() : Node("EEPose_Subscriber"){
            Subscriber = this->create_subscription<linkpose_msgs::msg::LinkPose>("/LinkPose_irb120_EE_egp64", 10, std::bind(&EEPose_Subscriber::CALLBACK_FN, this, std::placeholders::_1));
        }

    private:

        void CALLBACK_FN(const linkpose_msgs::msg::LinkPose msg) const
        {
            EEPose = msg;
        }

        rclcpp::Subscription<linkpose_msgs::msg::LinkPose>::SharedPtr Subscriber;
};

// ========================================================================================= //
// ATTACH/DETACH:

struct ATTACH_ST{             
  bool success;        
  std::string model;
  std::string link;  
};     

ATTACH_ST AttachedOBJ;

void AttachDetach_NODE(){

    AttacherNode = rclcpp::Node::make_shared("ATTACHER_SC_node");
    DetacherNode = rclcpp::Node::make_shared("DETACHER_SC_node");

}

ATTACH_ST CHECK_ATTACH(){

    // EE POSE:
    rclcpp::spin_some(node_EEPoseSUB);

    ATTACH_ST RESULT;
    bool CHECK = true;
    
    // Iterate and compare:
    int N = param_OL.size()/2;
    for (int i=1; i<=N; i++){

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "EEPose.x -> %.4f / ObjectPose.x -> %.4f", EEPose.x, ObjectPoseVECTOR[i-1].x);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "EEPose.y -> %.4f / ObjectPose.y -> %.4f", EEPose.y, ObjectPoseVECTOR[i-1].y);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "EEPose.z -> %.4f / ObjectPose.z -> %.4f", EEPose.z, ObjectPoseVECTOR[i-1].z);

        if ((EEPose.x - 0.01 > ObjectPoseVECTOR[i-1].x) || ((EEPose.x + 0.01 < ObjectPoseVECTOR[i-1].x))){
            CHECK = false;
        }  

        if ((EEPose.y - 0.01 > ObjectPoseVECTOR[i-1].y) || ((EEPose.y + 0.01 < ObjectPoseVECTOR[i-1].y))){
            CHECK = false;
        } 

        if ((EEPose.z - 0.01 > ObjectPoseVECTOR[i-1].z) || ((EEPose.z + 0.01 < ObjectPoseVECTOR[i-1].z))){
            CHECK = false;
        } 

        if (CHECK == true){

            RESULT.success = true;
            RESULT.model = ObjectPoseVECTOR[i-1].objectname;
            RESULT.link = param_OL[(2*i)-1];

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ATTACH CHECK: Successful.");
            break;

        }

    }

    if (CHECK == false){
        RESULT.success = false;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ATTACH CHECK: Unsuccessful.");
    }

    return(RESULT);
}

bool ATTACH(){

    ATTACH_ST CHECK = CHECK_ATTACH();

    if (CHECK.success == true){

        auto ATTACHER_SC = AttacherNode->create_client<linkattacher_msgs::srv::AttachLink>("ATTACHLINK");
        auto request = std::make_shared<linkattacher_msgs::srv::AttachLink::Request>();

        request->model1_name = "irb120";
        request->link1_name = "EE_egp64";
        request->model2_name = CHECK.model;
        request->link2_name = CHECK.link;

        auto result = ATTACHER_SC->async_send_request(request);

        if (rclcpp::spin_until_future_complete(AttacherNode, result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto RES = result.get();
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "MSG: %s", RES->message.c_str());
            if (bool attachOK = RES->success) {
                AttachedOBJ.success = true;
                AttachedOBJ.model = CHECK.model;
                AttachedOBJ.link = CHECK.link;
                return true;
            } else {
                return false;
            }
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service /ATTACHLINK");
            return false;
        }
    
    } else {
        return false;
    }

}

bool DETACH(){
    
    if (AttachedOBJ.success == true){

        auto DETACHER_SC = DetacherNode->create_client<linkattacher_msgs::srv::DetachLink>("DETACHLINK");
        auto request = std::make_shared<linkattacher_msgs::srv::DetachLink::Request>();

        request->model1_name = "irb120";
        request->link1_name = "EE_egp64";
        request->model2_name = AttachedOBJ.model;
        request->link2_name = AttachedOBJ.link;

        auto result = DETACHER_SC->async_send_request(request);

        if (rclcpp::spin_until_future_complete(DetacherNode, result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto RES = result.get();
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "MSG: %s", RES->message.c_str());
            if (bool detachOK = RES->success) {
                AttachedOBJ.success = true;
                AttachedOBJ.model = "";
                AttachedOBJ.link = "";
                return true;
            } else {
                return false;
            }
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service /DETACHLINK");
            return false;
        }
    
    } else {
        return false;
    }
    
}


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
                POSE.position.y = config["pose"]["y"].as<double>();
                POSE.position.z = config["pose"]["z"].as<double>();
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

        // EXECUTION TIME -> START:
        auto t_start = std::chrono::high_resolution_clock::now();

        // 0. DECLARE -> GOAL + RESULT + TARGET POSE+JP:
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<ExecuteSkill::Result>();

        geometry_msgs::msg::Pose TR_POSE;
        std::vector<double> JP;
        std::vector<double> ERROR = {-1.0};

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

        // Declare PLAN:
        moveit::planning_interface::MoveGroupInterface::Plan MyPlan;

        if (TYPE == "GRIP"){

            auto ACTION = config["action"].as<std::string>();
            move_group_interface_EE.setPlannerId("PTP");

            moveit::core::RobotStatePtr current_state = move_group_interface_EE.getCurrentState(10);
            current_state->copyJointGroupPositions(joint_model_group_EE, JP);

            if (ACTION == "CLOSE"){
                JP[0] = 0.006;
                JP[1] = 0.006;
                ATTACH();
            } else if (ACTION == "OPEN"){
                JP[0] = 0.0;
                JP[1] = 0.0;
                DETACH();
            }

            move_group_interface_EE.setJointValueTarget(JP);
            move_group_interface_EE.setMaxVelocityScalingFactor(SPEED);

            MyPlan = plan_EE();

        } else if (TYPE == "PTP" || TYPE == "LIN"){

            auto CURRENT_POSE = move_group_interface_ROB.getCurrentPose();
            RCLCPP_INFO(this->get_logger(), "CURRENT POSE:");
            RCLCPP_INFO(this->get_logger(), "x: %.2f", CURRENT_POSE.pose.position.x);
            RCLCPP_INFO(this->get_logger(), "y: %.2f", CURRENT_POSE.pose.position.y);
            RCLCPP_INFO(this->get_logger(), "z: %.2f", CURRENT_POSE.pose.position.z);
            RCLCPP_INFO(this->get_logger(), "qx: %.2f", CURRENT_POSE.pose.orientation.x);
            RCLCPP_INFO(this->get_logger(), "qy: %.2f", CURRENT_POSE.pose.orientation.y);
            RCLCPP_INFO(this->get_logger(), "qz: %.2f", CURRENT_POSE.pose.orientation.z);
            RCLCPP_INFO(this->get_logger(), "qw: %.2f", CURRENT_POSE.pose.orientation.w);
            
            move_group_interface_ROB.setPlannerId(TYPE);

            geometry_msgs::msg::Pose TARGET_POSE;
            TARGET_POSE.position.x = config["pose"]["x"].as<double>();
            TARGET_POSE.position.y = config["pose"]["y"].as<double>();
            TARGET_POSE.position.z = config["pose"]["z"].as<double>();
            TARGET_POSE.orientation.x = config["pose"]["qx"].as<double>();
            TARGET_POSE.orientation.y = config["pose"]["qy"].as<double>();
            TARGET_POSE.orientation.z = config["pose"]["qz"].as<double>();
            TARGET_POSE.orientation.w = config["pose"]["qw"].as<double>();

            // Convert from EE_FRAME to tool0:
            TR_POSE.orientation = TARGET_POSE.orientation;

            // 1. Obtain ROTATION MATRIX of TARGET_POSE (quaternion) -> R():
            // 1.1 QUATERNION:
            double Ax = TARGET_POSE.orientation.x;
            double Ay = TARGET_POSE.orientation.y;
            double Az = TARGET_POSE.orientation.z;
            double Aw = TARGET_POSE.orientation.w;
            // 1.2 NORMALISE:
            double norm = sqrt((Ax*Ax)+(Ay*Ay)+(Az*Az)+(Aw*Aw));
            double Qx = Ax/norm;
            double Qy = Ay/norm;
            double Qz = Az/norm;
            double Qw = Aw/norm;
            // 1.3 ROTATION MATRIX:
            double R_00 = 1 - 2*(Qy*Qy) - 2*(Qz*Qz);
            double R_01 = 2*(Qx*Qy) - 2*(Qw*Qz);
            double R_02 = 2*(Qx*Qz) + 2*(Qw*Qy);
            double R_10 = 2*(Qx*Qy) + 2*(Qw*Qz);
            double R_11 = 1 - 2*(Qx*Qx) - 2*(Qz*Qz);
            double R_12 = 2*(Qy*Qz) - 2*(Qw*Qx);
            double R_20 = 2*(Qx*Qz) - 2*(Qw*Qy);
            double R_21 = 2*(Qy*Qz) + 2*(Qw*Qx);
            double R_22 = 1 - 2*(Qx*Qx) - 2*(Qy*Qy);
            // 2. TRANSLATION: From EE_FRAME to tool0:
            double Tx = 0.0;
            double Ty = 0.0;
            double Tz = -0.19; // Difference between tool0 and EE_FRAME in LOCAL COORDINATES.
            TR_POSE.position.x = TARGET_POSE.position.x + R_00*Tx + R_01*Ty + R_02*Tz;
            TR_POSE.position.y = TARGET_POSE.position.y + R_10*Tx + R_11*Ty + R_12*Tz;
            TR_POSE.position.z = TARGET_POSE.position.z + R_20*Tx + R_21*Ty + R_22*Tz;

            move_group_interface_ROB.setPoseTarget(TR_POSE);
            move_group_interface_ROB.setMaxVelocityScalingFactor(SPEED);

            MyPlan = plan_ROB();

        }

        if (RES == "PLANNING: OK"){

            bool ExecSUCCESS = (move_group_interface_ROB.execute(MyPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

            // EXECUTION TIME -> END:
            auto t_end = std::chrono::high_resolution_clock::now();
            auto ms_duration = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start);
            auto s_duration = ms_duration.count() / 1000000.0;
            result->result.exectime = s_duration;

            // OBJECT POSE:
            rclcpp::spin_some(node_ObjectPoseSUB);
            
            int N = param_OL.size()/2;
            for (int i=0; i<N; i++){

                RCLCPP_INFO(this->get_logger(), "OBJECT: %s", ObjectPoseVECTOR[i].objectname.c_str());
                RCLCPP_INFO(this->get_logger(), "POS: (x: %.3f , Y: %.3f , z: %.3f)", ObjectPoseVECTOR[i].x, ObjectPoseVECTOR[i].y, ObjectPoseVECTOR[i].z);
                RCLCPP_INFO(this->get_logger(), "ROT: (qx: %.3f , qy: %.3f , qz: %.3f , w: %.3f)", ObjectPoseVECTOR[i].qx, ObjectPoseVECTOR[i].qy, ObjectPoseVECTOR[i].qz, ObjectPoseVECTOR[i].qw);

                // PRODUCT INFORMATION:
                r3mcell_data::msg::Product PROD;

                PROD.name = ObjectPoseVECTOR[i].objectname;
                PROD.error = 0.0;
                // CurrentPose:
                PROD.currentpose.x = ObjectPoseVECTOR[i].x;
                PROD.currentpose.y = ObjectPoseVECTOR[i].y;
                PROD.currentpose.z = ObjectPoseVECTOR[i].z;
                PROD.currentpose.qx = ObjectPoseVECTOR[i].qx;
                PROD.currentpose.qy = ObjectPoseVECTOR[i].qy;
                PROD.currentpose.qz = ObjectPoseVECTOR[i].qz;
                PROD.currentpose.qw = ObjectPoseVECTOR[i].qw;
                // PreviousPose:
                PROD.previouspose.x = PreviousPoseVECTOR[i].x;
                PROD.previouspose.y = PreviousPoseVECTOR[i].y;
                PROD.previouspose.z = PreviousPoseVECTOR[i].z;
                PROD.previouspose.qx = PreviousPoseVECTOR[i].qx;
                PROD.previouspose.qy = PreviousPoseVECTOR[i].qy;
                PROD.previouspose.qz = PreviousPoseVECTOR[i].qz;
                PROD.previouspose.qw = PreviousPoseVECTOR[i].qw;

                result->result.product.push_back(PROD);
            }

            if (goal_handle->is_canceling()) {
                RCLCPP_INFO(this->get_logger(), "Goal canceled.");
                result->result.message = "RECIPE N-" + ID + " (" + TYPE + ")" + ":CANCELED";
                result->result.success = false;
                result->result.id = ID;
                result->result.error = ERROR;
                goal_handle->canceled(result);
                return;
            } 
            
            if (ExecSUCCESS){
                RCLCPP_INFO(this->get_logger(), "RECIPE ID: %s -> %s - %s: Movement executed!", ID.c_str(), param_ROB.c_str(), TYPE.c_str());
                result->result.message = "RECIPE N-" + ID + " (" + TYPE + ")" + ":SUCCESS";
                result->result.success = true;
                result->result.id = ID;
                result->result.error = ERROR_ROB(TR_POSE);
                goal_handle->succeed(result);
            } else {
                RCLCPP_INFO(this->get_logger(), "RECIPE ID: %s -> %s - %s: Movement execution failed!", ID.c_str(), param_ROB.c_str(), TYPE.c_str());
                result->result.message = "RECIPE N-" + ID + " (" + TYPE + ")" + ":FAILED. Reason -> Execution error.";
                result->result.success = false;
                result->result.id = ID;
                result->result.error = ERROR;
                goal_handle->succeed(result);
            }

        } else if (RES == "PLANNING: OK (EE)"){
            
            move_group_interface_EE.execute(MyPlan);

            // EXECUTION TIME -> END:
            auto t_end = std::chrono::high_resolution_clock::now();
            auto ms_duration = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start);
            auto s_duration = ms_duration.count() / 1000000.0;
            result->result.exectime = s_duration;

            // OBJECT POSE:
            rclcpp::spin_some(node_ObjectPoseSUB);
            
            int N = param_OL.size()/2;
            for (int i=0; i<N; i++){

                RCLCPP_INFO(this->get_logger(), "OBJECT: %s", ObjectPoseVECTOR[i].objectname.c_str());
                RCLCPP_INFO(this->get_logger(), "POS: (x: %.3f , Y: %.3f , z: %.3f)", ObjectPoseVECTOR[i].x, ObjectPoseVECTOR[i].y, ObjectPoseVECTOR[i].z);
                RCLCPP_INFO(this->get_logger(), "ROT: (qx: %.3f , qy: %.3f , qz: %.3f , w: %.3f)", ObjectPoseVECTOR[i].qx, ObjectPoseVECTOR[i].qy, ObjectPoseVECTOR[i].qz, ObjectPoseVECTOR[i].qw);

                // PRODUCT INFORMATION:
                r3mcell_data::msg::Product PROD;

                PROD.name = ObjectPoseVECTOR[i].objectname;
                PROD.error = 0.0;
                // CurrentPose:
                PROD.currentpose.x = ObjectPoseVECTOR[i].x;
                PROD.currentpose.y = ObjectPoseVECTOR[i].y;
                PROD.currentpose.z = ObjectPoseVECTOR[i].z;
                PROD.currentpose.qx = ObjectPoseVECTOR[i].qx;
                PROD.currentpose.qy = ObjectPoseVECTOR[i].qy;
                PROD.currentpose.qz = ObjectPoseVECTOR[i].qz;
                PROD.currentpose.qw = ObjectPoseVECTOR[i].qw;
                // PreviousPose:
                PROD.previouspose.x = PreviousPoseVECTOR[i].x;
                PROD.previouspose.y = PreviousPoseVECTOR[i].y;
                PROD.previouspose.z = PreviousPoseVECTOR[i].z;
                PROD.previouspose.qx = PreviousPoseVECTOR[i].qx;
                PROD.previouspose.qy = PreviousPoseVECTOR[i].qy;
                PROD.previouspose.qz = PreviousPoseVECTOR[i].qz;
                PROD.previouspose.qw = PreviousPoseVECTOR[i].qw;

                result->result.product.push_back(PROD);
            }

            if (goal_handle->is_canceling()) {
                RCLCPP_INFO(this->get_logger(), "Goal canceled.");
                result->result.message = "RECIPE N-" + ID + " (" + TYPE + ")" + ":CANCELED";
                result->result.success = false;
                result->result.id = ID;
                result->result.error = ERROR;
                goal_handle->canceled(result);
                return;
            } else {
                RCLCPP_INFO(this->get_logger(), "RECIPE ID: %s -> %s - %s: Movement executed!", ID.c_str(), param_EE.c_str(), TYPE.c_str());
                result->result.message = "RECIPE N-" + ID + " (" + TYPE + ")" + ":SUCCESS";
                result->result.success = true;
                result->result.id = ID;
                result->result.error = ERROR_EE(JP);
                goal_handle->succeed(result);
            }
            
        } else if (RES == "PLANNING: ERROR"){
            RCLCPP_INFO(this->get_logger(), "RECIPE ID: %s -> %s - %s: Planning failed!", ID.c_str(), param_ROB.c_str(), TYPE.c_str());
            result->result.message = "RECIPE N-" + ID + " (" + TYPE + ")" + ":FAILED. Reason -> Planning failed.";
            result->result.success = false;
            result->result.id = ID;
            result->result.error = ERROR;
            goal_handle->succeed(result);
        } else if (RES == "PLANNING: ERROR (EE)"){
            RCLCPP_INFO(this->get_logger(), "RECIPE ID: %s -> %s - %s: Planning failed!", ID.c_str(), param_EE.c_str(), TYPE.c_str());
            result->result.message = "RECIPE N-" + ID + " (" + TYPE + ")" + ":FAILED. Reason -> Planning failed.";
            result->result.success = false;
            result->result.id = ID;
            result->result.error = ERROR;
            goal_handle->succeed(result);
        } 

        // RE-INITIALISE RES variable:
        RES = "none";

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

    // Launch ObjectPose subscriber ROS2 Node:
    node_ObjectPoseSUB = std::make_shared<ObjectPose_Subscriber>();
    node_EEPoseSUB = std::make_shared<EEPose_Subscriber>();

    // AttachDetach NODE -> declare:
    AttachDetach_NODE();

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