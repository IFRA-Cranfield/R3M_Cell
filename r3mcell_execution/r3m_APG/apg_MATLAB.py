#!/usr/bin/python3

# # # # # # # # # # # # # # # # # #                                  
#                                 #
#   ===== COPYRIGHT HERE =====    #
#                                 #
# # # # # # # # # # # # # # # # # #

# ========================================================================================= #
# ======================================== INCLUDE ======================================== #
# ========================================================================================= #

# System:
import os
import time

# ROS2:
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

# CUSTOM ROS2 MSG/SRV/ACTION:
from ros2srrc_data.msg import Robpose
from r3mcell_data.srv import SkillExecution

# Required for MATLAB AGENT EXECUTION:
import numpy as np
import matlab.engine

# ================ #
# GLOBAL VARIABLES #
RobPose = Robpose()
GripperState = 0

# ========================================================================================= #
# Execute Skill -> ROS 2 Service Client:

class R3MSkillClient(Node):

    def __init__(self):

        # Initialise ROS2 Node:
        super().__init__('r3mcell_APGMatlab_SkillClient')

        # Create ROS2 Service Client:
        self.cli_SKILL = self.create_client(SkillExecution, "/r3m_SkillExecution")  

        # Declare REQUEST variable (of CUSTOM DATA type):
        self.req_SKILL = SkillExecution.Request()  

    def SKILL_REQUEST(self, ID):
        
        self.req_SKILL.id = ID
        self.future_SKILL = self.cli_SKILL.call_async(self.req_SKILL)

# ========================================================================================= #
# Get RobotPose:
class RobotPoseSubscriber(Node):

    def __init__(self):

        super().__init__("r3mcell_APGMatlab_RobotPoseSubscriber")
        self.subscription_ = self.create_subscription(Robpose, "/Robpose", self.CALLBACK, 1)

    def CALLBACK(self, POSE):

        global RobPose
        RobPose = POSE

        RobPose.x = round(RobPose.x, 2)
        RobPose.y = round(RobPose.y, 2)
        RobPose.z = round(RobPose.z, 2)
        RobPose.qx = round(RobPose.qx, 2)
        RobPose.qy = round(RobPose.qy, 2)
        RobPose.qz = round(RobPose.qz, 2)
        RobPose.qw = round(RobPose.qw, 2)
        
# ========================================================================================= #
# MatlabAgent:
class MatlabAgent():
    
    def __init__(self):

        super().__init__()
        self.MATLAB = matlab.engine.start_matlab()
        
        self.PATH = os.path.join(get_package_share_directory('r3mcell_execution'), 'r3m_APG')
        self.AGENT = "agentData.mat"
        
    def Execute(self, id, RobPose, EEState, ObjectPose):
        
        # ASSIGN -> Input values to AGENT:
        observation = [id, RobPose.x, RobPose.y, RobPose.z, RobPose.qx, RobPose.qy, RobPose.qz, RobPose.qw, EEState, ObjectPose.x, ObjectPose.y, ObjectPose.z, ObjectPose.qx, ObjectPose.qy, ObjectPose.qz, ObjectPose.qw]
        observation = np.array(observation)
        
        self.MATLAB.addpath(self.PATH)
        var = self.MATLAB.evaluatePolicy(observation, self.AGENT)
        
        return(var)
        
# ========================================================================================= #
# ========================================= MAIN ========================================== #
# ========================================================================================= #

def main(args=None):
    
    global RobPose
    
    rclpy.init(args=args)
    
    # PRINT:
    print("==========================================================")
    print("R3M - AUTOMATIC PROGRAM GENERATION: Execution of APG Agent")
    print("")

    # INITIALISE ROS 2 Classes:
    Node_SkillExecution = R3MSkillClient()
    Node_RobotPose = RobotPoseSubscriber()
    APG_Agent = MatlabAgent()
    
    print("Initialising RLA: Executing Recipe N1...")
    
    # INITIAL STATE -> Recipe N1 will be executed as a initial step, to get the state of the system:
    Node_SkillExecution.SKILL_REQUEST(1)
    while rclpy.ok():
        rclpy.spin_once(Node_SkillExecution)
        if Node_SkillExecution.future_SKILL.done():
            try:
                skillRES = Node_SkillExecution.future_SKILL.result().result
            except Exception as exc:
                print("[R3M Cell] - /ExecuteSkill ROS2 Service call failed. ERROR: " + str(exc))
                return(False)
            else:
                print("[R3M Cell] - /ExecuteSkill RESULT:")
                print(skillRES)
                print("")
            break
        
    ID = 0
    
    T = time.time() + 0.5
    while (time.time() < T):
        rclpy.spin_once(Node_RobotPose) # We assign current pose to RobPose.
    
    ObjPose = skillRES.product[0].currentpose
    ObjPose.x = round(ObjPose.x, 2)
    ObjPose.y = round(ObjPose.y, 2)
    ObjPose.z = round(ObjPose.z, 2)
    ObjPose.qx = round(ObjPose.qx, 2)
    ObjPose.qy = round(ObjPose.qy, 2)
    ObjPose.qz = round(ObjPose.qz, 2)
    ObjPose.qw = round(ObjPose.qw, 2)
    
    EEState = skillRES.endeffector
    
    CONTINUE = True
    while CONTINUE:
        
        print("RECIPE EXECUTION: Getting Recipe ID from RLA...")
        print("Robot Pose:")
        print(RobPose)
        print("Object Pose:")
        print(ObjPose)
        print("Gripper State:")
        print(EEState)
        print("")
        
        # GET -> RECIPE ID from AGENT:
        RECIPE = APG_Agent.Execute(ID, RobPose, EEState, ObjPose)
        RECIPE_ID = int(RECIPE)
        
        print("RECIPE ID obtained! ")
        print("ID: " + str(RECIPE_ID))
        print("")
        
        print("Executing Recipe N" + str(RECIPE_ID) + "...")
        
        # EXECUTE -> RECIPE:
        Node_SkillExecution.SKILL_REQUEST(RECIPE_ID)
        while rclpy.ok():
            rclpy.spin_once(Node_SkillExecution)
            if Node_SkillExecution.future_SKILL.done():
                try:
                    skillRES = Node_SkillExecution.future_SKILL.result().result
                except Exception as exc:
                    print("[R3M Cell] - /ExecuteSkill ROS2 Service call failed. ERROR: " + str(exc))
                    return(False)
                else:
                    print("[R3M Cell] - /ExecuteSkill RESULT:")
                    print(skillRES)
                    print("")
                break
            
        ID = float(skillRES.id)
        T = time.time() + 0.5
        while (time.time() < T):
            rclpy.spin_once(Node_RobotPose) # We assign current pose to RobPose.
        
        ObjPose = skillRES.product[0].currentpose
        ObjPose.x = round(ObjPose.x, 2)
        ObjPose.y = round(ObjPose.y, 2)
        ObjPose.z = round(ObjPose.z, 2)
        ObjPose.qx = round(ObjPose.qx, 2)
        ObjPose.qy = round(ObjPose.qy, 2)
        ObjPose.qz = round(ObjPose.qz, 2)
        ObjPose.qw = round(ObjPose.qw, 2)
        
        EEState = skillRES.endeffector

    # FINISH:
    rclpy.shutdown()

if __name__ == '__main__':
    main()
