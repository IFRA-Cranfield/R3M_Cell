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
import time

# ROS2:
import rclpy
from rclpy.node import Node

# CUSTOM ROS2 MSG/SRV/ACTION:
from objectpose_msgs.msg import ObjectPose

# ========================================================================================= #
# =================================== CLASSES/FUNCTIONS =================================== #
# ========================================================================================= #

# ========================================================================================= #
# OBJECT CLASS:
class OBJECT(Node):

    def __init__(self, ObjectList):
        
        self.CurrentPose = []
        self.PreviousPose = []
        self.ObjectList = ObjectList

        self.DET =[]
                
        # ObjectList = [] of dict{Object}, where Object = {"Model": "", "Link": ""}

        super().__init__("r3mcell_ObjectPose_Subscriber")
        self.SUBList = []
        
        for x in ObjectList:
            
            TopicName = "/" + x["Model"] + "/ObjectPose"
            self.SUBList.append(self.create_subscription(ObjectPose, TopicName, self.CALLBACK_FN, 10))
            
            EmptyPose = ObjectPose()
            EmptyPose.objectname = x["Model"]
            EmptyPose.x = 0.0
            EmptyPose.y = 0.0
            EmptyPose.z = 0.0
            EmptyPose.qx = 0.0
            EmptyPose.qy = 0.0
            EmptyPose.qz = 0.0
            EmptyPose.qw = 0.0
            
            x["CurrentPose"] = EmptyPose
            x["PreviousPose"] = EmptyPose

            # Initialise self.DET:
            self.DET.append({"Model": x["Model"], "Detected": False})

    def CALLBACK_FN(self, OBJ):

        # 1. Assign CURRENTPOSE to PREVIOUSPOSE:
        # 2. Assign NEWPOSE to CURRENTPOSE:

        for x in self.DET:
            if (x["Model"] == OBJ.objectname and x["Detected"] == False):

                x["Detected"] = True

                for y in self.ObjectList:
                    if (OBJ.objectname == y["Model"]):
                        
                        y["PreviousPose"] = y["CurrentPose"]
                        y["CurrentPose"] = OBJ

    def ResetObjectList(self):

        for x in self.ObjectList:
            
            EmptyPose = ObjectPose()
            EmptyPose.objectname = x["Model"]
            EmptyPose.x = 0.0
            EmptyPose.y = 0.0
            EmptyPose.z = 0.0
            EmptyPose.qx = 0.0
            EmptyPose.qy = 0.0
            EmptyPose.qz = 0.0
            EmptyPose.qw = 0.0
            
            x["CurrentPose"] = EmptyPose
            x["PreviousPose"] = EmptyPose

        
    def GetObjectPose(self):
        
        # 1. Spin node:
        T = time.time() + 0.25
        while time.time() < T:
            rclpy.spin_once(self)

        # 2. Reset self.DET:
        for x in self.DET:
            x["Detected"] = False
        
        # 3. RETURN:
        return(self.ObjectList)