#!/usr/bin/python3

# # # # # # # # # # # # # # # # # #                                  
#                                 #
#   ===== COPYRIGHT HERE =====    #
#                                 #
# # # # # # # # # # # # # # # # # #

# ========================================================================================= #
# ======================================== INCLUDE ======================================== #
# ========================================================================================= #

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
                
        # ObjectList = [] of dict{Object}, where Object = {"Model": "", "Link": ""}

        super().__init__("r3mcell_ObjectPose_Subscriber")
        self.SUBList = []
        
        for x in ObjectList:
            
            TopicName = "/" + x["Model"] + "/ObjectPose"
            print(TopicName)
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
            
            self.CurrentPose.append(EmptyPose)
            self.PreviousPose.append(EmptyPose)

    def CALLBACK_FN(self, OBJ):

        # 1. Assign CURRENTPOSE to PREVIOUSPOSE:
        # 2. Assign NEWPOSE to CURRENTPOSE:

        for x in self.CurrentPose:
            if (OBJ.objectname == x.objectname):
                
                i = self.CurrentPose.index(x)
                self.PreviousPose[i] = x
                   
                self.CurrentPose[i] = OBJ
        
    def GetObjectPose(self):
        
        # 1. Spin node:
        rclpy.spin_once(self)
        
        # 2. RETURN:
        RESULT = {}
        RESULT["CurrentPose"] = self.CurrentPose
        RESULT["PreviousPose"] = self.PreviousPose
        
        return(RESULT)