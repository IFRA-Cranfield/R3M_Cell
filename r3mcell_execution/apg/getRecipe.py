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

# CUSTOM ROS2 MSG/SRV/ACTION:
from objectpose_msgs.msg import ObjectPose
from ros2srrc_data.msg import Robpose

# ================ #
# GLOBAL VARIABLES #
OP = ObjectPose()

# ========================================================================================= #
# Get ObjectPose:
class ObjectPoseSubscriber(Node):

    def __init__(self, OBJECT):

        super().__init__("r3mcell_getRecipe_ObjectPoseSubscriber")
        
        ObjectTopic = "/" + OBJECT + "/ObjectPose"
        self.subscription_ = self.create_subscription(ObjectPose, ObjectTopic, self.CALLBACK, 1)

    def CALLBACK(self, POSE):
        
        global OP
        OP = POSE
        
# ========================================================================================= #
# COMPUTE RECIPE from ObjectPose:
class computeRecipe():
    
    def __init__(self, OBJECT):
    
        self.OBJECT = ObjectPoseSubscriber(OBJECT)
        
    # R3M Cell - Cranfield University: CUBE PICK&PLACE USE-CASE -> RECIPES (N2) and (N3).
    def smallCUBE(self):
        
        global OP
        
        T = time.time() + 0.5
        while (time.time() < T):
            rclpy.spin_once(self.OBJECT) 
            
        print(OP)
            
        POSE = Robpose()
        
        # By now, we assume that the box is not rotated:    
        POSE.qx = 0.0
        POSE.qy = 1.0
        POSE.qz = 0.0
        POSE.qw = 0.0
            
        # EDIT RECIPE N2 w/ SMALL CUBE POSE:
        POSE.x = OP.x
        POSE.y = OP.y
        POSE.z = OP.z + 0.25
        EditPoseRecipe(2,POSE)
         
        # EDIT RECIPE N3 w/ SMALL CUBE POSE:
        POSE = Robpose()
        POSE.z = OP.z + 0.17
        EditPoseRecipe(3,POSE)
        
# ========================================================================================= #
# FUNCTION -> EDIT POSE RECIPE:
def EditPoseRecipe(RECIPE, POSE):
    
    yaml_FILE = os.path.join(get_package_share_directory('r3mcell_execution'), 'recipes') + "/" + str(RECIPE) + ".yaml"
    
    with open(yaml_FILE) as F:
        RECIPE = yaml.safe_load(F)
        
    RECIPE["pose"]["x"] = POSE.x
    RECIPE["pose"]["y"] = POSE.y
    RECIPE["pose"]["z"] = POSE.z
    RECIPE["pose"]["qx"] = POSE.qx
    RECIPE["pose"]["qy"] = POSE.qy
    RECIPE["pose"]["qz"] = POSE.qz
    RECIPE["pose"]["qw"] = POSE.qw
    
    with open(yaml_FILE, "w") as F:
        yaml.dump(RECIPE, F)