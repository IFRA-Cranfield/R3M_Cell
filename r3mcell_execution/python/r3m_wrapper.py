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
import yaml

# ROS2:
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

# ROS2 MSG/SRV/ACTION:
from geometry_msgs.msg import Pose

# CUSTOM ROS2 MSG/SRV/ACTION:
from r3mcell_data.msg import Skillresult

# Import CLASSES/Functions:
from Robot import RobotClient
from Gripper_Gz import ParallelGripper
from ObjectState import OBJECT
from ResetGazebo import GzRESET

# ========================================================================================= #
# ================================ ROS2 - INPUT PARAMETERS ================================ #
# ========================================================================================= #


# ========================================================================================= #
# =================================== CLASSES/FUNCTIONS =================================== #
# ========================================================================================= #

# ========================================================================================= #
# ExecuteSkill CLASS:
class ExecuteSkill():
    
    def __init__(self):
        
        # Initialise CLASSES that are needed for every skill execution:
        None
    
    def EXECUTE(self, RECIPE):
        
        None
        
# ========================================================================================= #
# GetRecipe FUNCTION:
def GetRecipe(RECIPE_ID):
    
    RECIPE = {"Exists": True}
    
    PATH = os.path.join(get_package_share_directory('r3mcell_execution'), 'recipes')
    RECIPE_PATH = PATH + "/" + RECIPE_ID + ".yaml"
    
    if not os.path.exists(RECIPE_PATH):
        RECIPE["Exists"] = False
        return (RECIPE)

    # Get RECIPE VALUES:
    with open(RECIPE_PATH, 'r') as YAML:
        RecipeYAML = yaml.safe_load(YAML)
      
    RECIPE["id"] = RecipeYAML["id"] 
    RECIPE["type"] = RecipeYAML["type"]
    RECIPE["speed"] = RecipeYAML["speed"]
      
    if (RECIPE["type"] == "PTP" or RECIPE["type"] == "LIN"):
        
        POSE = Pose()
        POSE.position.x = RecipeYAML["pose"]["x"]
        POSE.position.y = RecipeYAML["pose"]["y"]
        POSE.position.z = RecipeYAML["pose"]["z"]
        POSE.orientation.x = RecipeYAML["pose"]["qx"]
        POSE.orientation.y = RecipeYAML["pose"]["qy"]
        POSE.orientation.z = RecipeYAML["pose"]["qz"]
        POSE.orientation.w = RecipeYAML["pose"]["qw"]
        
        RECIPE["pose"] = POSE
    
    elif RECIPE["type"] == "GRIP":
        
        RECIPE["action"] = RecipeYAML["action"]
        
    return(RECIPE)

# ========================================================================================= #
# ========================================= MAIN ========================================== #
# ========================================================================================= #

def main(args=None):

    print ("======= R3M PROJECT =======")
    print ("== r3m_wrapper.py script ==")
    print ("")
    print ("This script makes the .. ROS2 service available, which executes any RECIPE stored in the /r3mcell_execution/recipes folder.")
    print ("")
    
    # Initialise NODE:
    rclpy.init(args=args)
    r3mNode = serviceServer()
    print ("r3m_serviceSERVER ROS2 node generated.")
    print ("")

    # Spin SERVICE -> The Service Server will execute the service_Callback every single time the service is called.
    rclpy.spin(r3mNode)                                                                             # Spin SERVICE SERVER.

    r3mNode.destroy_node
    rclpy.shutdown()

if __name__ == '__main__':
    main()

            
