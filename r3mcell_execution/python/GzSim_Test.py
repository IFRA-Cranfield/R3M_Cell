#!/usr/bin/python3

# # # # # # # # # # # # # # # # # #                                  
#                                 #
#   ===== COPYRIGHT HERE =====    #
#                                 #
# # # # # # # # # # # # # # # # # #

# ========================================================================================= #
# INCLUDE:

# ROS2:
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import xacro
import os
import ast
import time 

# Std_msgs:
from std_msgs.msg import String
from std_msgs.msg import Int32 

# MSG:
from objectpose_msgs.msg import ObjectPose

# SRV:
from r3mcell_data.srv import SkillExecution  

# Random:
import random

# ========================================================================================= #
# ServiceClient (SkillExecution):

class SkillExecution_CLIENT(Node):

    def __init__(self):

        # Initialise ROS2 Node:
        super().__init__('r3m_M6DTest_EntityClient')

        # Create ROS2 Service Client:
        self.cli_SKILL = self.create_client(SkillExecution, "/r3m_SkillExecution")  

        # Declare REQUEST variable:
        self.req_SKILL = SkillExecution.Request()  

    def Recipe_REQUEST(self, ACTION, ID):
        
        if ACTION == "RESET":
            self.req_SKILL.id = 0
            print("REQUEST: Gazebo Environment RESET.")
            print("")
        elif ACTION == "MOVE":
            self.req_SKILL.id = random.randint(1,7) 
            print("REQUEST: Recipe execution -> N" + str(self.req_SKILL.id) + ".")
            print("")
        elif ACTION == "SEQUENCE":
            self.req_SKILL.id = ID
            print("REQUEST: Recipe execution -> N" + str(self.req_SKILL.id) + ".")
            print("")

        self.future_SKILL = self.cli_SKILL.call_async(self.req_SKILL)  


def SEQUENCE(CLIENT):

    print("Executing a whole proper sequence...")
    print("")

    # Step 1 -> RECIPE N2:
    CLIENT.Recipe_REQUEST("SEQUENCE", 2)
    print("STEP1 -> Recipe N2:")
                
    while rclpy.ok():
        rclpy.spin_once(CLIENT)
        
        if CLIENT.future_SKILL.done():
            try:
                RESULT = CLIENT.future_SKILL.result()
                RES = RESULT.result
            
            except Exception as exc:
                print("/ExecuteSkill ROS2 Service call failed. ERROR: " + str(exc))
                print("")
                print("CLOSING PROGRAM... BYE!")
                rclpy.shutdown()
                exit()
            
            else:
                print("PickApproach step successful. RESULTS:")
                print(" - Recipe ID: " + str(RES.id))
                print(" - Execution Time: " + str(RES.exectime))
                print(" - Message: " + RES.message)
                print(" - Success? -> " + str(RES.success))
                print("")
            
            break

    # Step 2 -> RECIPE N3:
    CLIENT.Recipe_REQUEST("SEQUENCE", 3)
    print("STEP2 -> Recipe N3:")
                
    while rclpy.ok():
        rclpy.spin_once(CLIENT)
        
        if CLIENT.future_SKILL.done():
            try:
                RESULT = CLIENT.future_SKILL.result()
                RES = RESULT.result
            
            except Exception as exc:
                print("/ExecuteSkill ROS2 Service call failed. ERROR: " + str(exc))
                print("")
                print("CLOSING PROGRAM... BYE!")
                rclpy.shutdown()
                exit()
            
            else:
                print("MovePick step successful. RESULTS:")
                print(" - Recipe ID: " + str(RES.id))
                print(" - Execution Time: " + str(RES.exectime))
                print(" - Message: " + RES.message)
                print(" - Success? -> " + str(RES.success))
                print("")
            
            break

    # Step 3 -> RECIPE N6:
    CLIENT.Recipe_REQUEST("SEQUENCE", 6)
    print("STEP3 -> Recipe N6:")
                
    while rclpy.ok():
        rclpy.spin_once(CLIENT)
        
        if CLIENT.future_SKILL.done():
            try:
                RESULT = CLIENT.future_SKILL.result()
                RES = RESULT.result
            
            except Exception as exc:
                print("/ExecuteSkill ROS2 Service call failed. ERROR: " + str(exc))
                print("")
                print("CLOSING PROGRAM... BYE!")
                rclpy.shutdown()
                exit()
            
            else:
                print("Pick step successful. RESULTS:")
                print(" - Recipe ID: " + str(RES.id))
                print(" - Execution Time: " + str(RES.exectime))
                print(" - Message: " + RES.message)
                print(" - Success? -> " + str(RES.success))
                print("")
            
            break

    # Step 4 -> RECIPE N4:
    CLIENT.Recipe_REQUEST("SEQUENCE", 4)
    print("STEP4 -> Recipe N4:")
                
    while rclpy.ok():
        rclpy.spin_once(CLIENT)
        
        if CLIENT.future_SKILL.done():
            try:
                RESULT = CLIENT.future_SKILL.result()
                RES = RESULT.result
            
            except Exception as exc:
                print("/ExecuteSkill ROS2 Service call failed. ERROR: " + str(exc))
                print("")
                print("CLOSING PROGRAM... BYE!")
                rclpy.shutdown()
                exit()
            
            else:
                print("ApproachPick step successful. RESULTS:")
                print(" - Recipe ID: " + str(RES.id))
                print(" - Execution Time: " + str(RES.exectime))
                print(" - Message: " + RES.message)
                print(" - Success? -> " + str(RES.success))
                print("")
            
            break

    # Step 5 -> RECIPE N5:
    CLIENT.Recipe_REQUEST("SEQUENCE", 5)
    print("STEP5 -> Recipe N5:")
                
    while rclpy.ok():
        rclpy.spin_once(CLIENT)
        
        if CLIENT.future_SKILL.done():
            try:
                RESULT = CLIENT.future_SKILL.result()
                RES = RESULT.result
            
            except Exception as exc:
                print("/ExecuteSkill ROS2 Service call failed. ERROR: " + str(exc))
                print("")
                print("CLOSING PROGRAM... BYE!")
                rclpy.shutdown()
                exit()
            
            else:
                print("PlaceApproach step successful. RESULTS:")
                print(" - Recipe ID: " + str(RES.id))
                print(" - Execution Time: " + str(RES.exectime))
                print(" - Message: " + RES.message)
                print(" - Success? -> " + str(RES.success))
                print("")
            
            break
    
    # CHECK IF BREAKS BY ASKING TO GRASP AGAIN (w/ object attached):
    CLIENT.Recipe_REQUEST("SEQUENCE", 6)
    print("STEP3 -> Recipe N6:")
                
    while rclpy.ok():
        rclpy.spin_once(CLIENT)
        
        if CLIENT.future_SKILL.done():
            try:
                RESULT = CLIENT.future_SKILL.result()
                RES = RESULT.result
            
            except Exception as exc:
                print("/ExecuteSkill ROS2 Service call failed. ERROR: " + str(exc))
                print("")
                print("CLOSING PROGRAM... BYE!")
                rclpy.shutdown()
                exit()
            
            else:
                print("Pick step successful. RESULTS:")
                print(" - Recipe ID: " + str(RES.id))
                print(" - Execution Time: " + str(RES.exectime))
                print(" - Message: " + RES.message)
                print(" - Success? -> " + str(RES.success))
                print("")
            
            break

    # Step 6 -> RECIPE N7:
    CLIENT.Recipe_REQUEST("SEQUENCE", 7)
    print("STEP6 -> Recipe N7:")
                
    while rclpy.ok():
        rclpy.spin_once(CLIENT)
        
        if CLIENT.future_SKILL.done():
            try:
                RESULT = CLIENT.future_SKILL.result()
                RES = RESULT.result
            
            except Exception as exc:
                print("/ExecuteSkill ROS2 Service call failed. ERROR: " + str(exc))
                print("")
                print("CLOSING PROGRAM... BYE!")
                rclpy.shutdown()
                exit()
            
            else:
                print("Place step successful. RESULTS:")
                print(" - Recipe ID: " + str(RES.id))
                print(" - Execution Time: " + str(RES.exectime))
                print(" - Message: " + RES.message)
                print(" - Success? -> " + str(RES.success))
                print("")
            
            break

    print("WHOLE SEQUENCE EXECUTION SUCCESSFUL!")
    print("")

# ===================================================================================== #
# ======================================= MAIN ======================================== #
# ===================================================================================== #

def main(args=None):

    rclpy.init()
    
    print("R3M Project: GazeboSim Testing")
    print("This script calls the /ExecuteSkill ROS2 Service in Gazebo 100 times, resetting the Gazebo environment every 10 service calls.")
    print("")

    # DECLARE SERVICE CLIENT:
    EXECUTE = SkillExecution_CLIENT()

    # MAIN LOOP:
    N = 100
    j = 1
    for i in range(1,N+1):
        
        print("===== EXECUTION NUMBER -> " + str(i) + " =====")

        if (j != 10 and j != 1):

            EXECUTE.Recipe_REQUEST("MOVE", 1)
                
            while rclpy.ok():
                rclpy.spin_once(EXECUTE)
                
                if EXECUTE.future_SKILL.done():
                    try:
                        RESULT = EXECUTE.future_SKILL.result()
                        RES = RESULT.result
                    
                    except Exception as exc:
                        print("/ExecuteSkill ROS2 Service call failed. ERROR: " + str(exc))
                        print("")
                        print("CLOSING PROGRAM... BYE!")
                        rclpy.shutdown()
                        exit()
                    
                    else:
                        print("Execution N" + str(i) + " successful. RESULTS:")
                        print(" - Recipe ID: " + str(RES.id))
                        print(" - Execution Time: " + str(RES.exectime))
                        print(" - Message: " + RES.message)
                        print(" - Success? -> " + str(RES.success))
                        print("")
                    
                    break

            j = j + 1

        elif j == 1:

            SEQUENCE(EXECUTE)
            j = j + 1
        
        elif j == 10:

            EXECUTE.Recipe_REQUEST("RESET", 0)
                
            while rclpy.ok():
                rclpy.spin_once(EXECUTE)
                
                if EXECUTE.future_SKILL.done():
                    try:
                        RESULT = EXECUTE.future_SKILL.result()
                        RES = RESULT.result
                    
                    except Exception as exc:
                        print("/ExecuteSkill ROS2 Service call failed. ERROR: " + str(exc))
                        print("")
                        print("CLOSING PROGRAM... BYE!")
                        rclpy.shutdown()
                        exit()
                    
                    else:
                        print("Execution N" + str(i) + " successful. RESULTS:")
                        print(" - Recipe ID: " + str(RES.id))
                        print(" - Message: " + RES.message)
                        print(" - Success? -> " + str(RES.success))
                        print("")
                    
                    break

            j = 1

    print("Program execution successfully finished!")
    print("Closing .py script... Bye!")

    rclpy.shutdown()

if __name__ == '__main__':
    main()