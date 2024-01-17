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
import sys
import ast
import time
import yaml

# ROS2:
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient

# ROS2 MSG/SRV/ACTION:
from geometry_msgs.msg import Pose

# CUSTOM ROS2 MSG/SRV/ACTION:
from r3mcell_data.msg import Product
from r3mcell_data.msg import Skillresult
from ros2srrc_data.action import Robmove

# ========================================================================================= #
# =================================== CLASSES/FUNCTIONS =================================== #
# ========================================================================================= #

# ========================================================================================= #
# ExecuteSkill CLASS:
class ExecuteSkill():
    
    def __init__(self):
        
        # Initialise CLASSES that are needed for every skill execution:
        self.ROBOT = RobMoveCLIENT()
    
    def EXECUTE(self, RECIPE):
        
        # Initialise RESULT variable:
        RESULT = Skillresult()
        
        # For execution time calculations:
        t_start = time.time()
        
        # 1. EXECUTE ROBOT MOVEMENT:
        if RECIPE["type"] 
        
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


            
