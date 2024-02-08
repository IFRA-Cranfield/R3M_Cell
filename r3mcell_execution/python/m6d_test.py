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
import sys
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import xacro
import os
import ast
import time 
# Std_msgs:
from std_msgs.msg import String
from std_msgs.msg import Int32 
# CAMERA ROS2msg:
from sensor_msgs.msg import Image
# Pose msg:
from objectpose_msgs.msg import ObjectPose
# IMPORT /SpawnEntity and /DeleteEntity ROS2 Services:
from gazebo_msgs.srv import SpawnEntity
from gazebo_msgs.srv import DeleteEntity

# Random:
import random

# OpenCV:
import cv2
from cv_bridge import CvBridge, CvBridgeError

# ===== GLOBAL VARIABLES ===== #
Gz_CAM = None
ObjPose = ObjectPose()
i = None

# ========================================================================================= #
# ServiceClient (SPAWN/DELETE OBJECT):

class EntityClient(Node):

    def __init__(self, object):

        # Initialise ROS2 Node:
        super().__init__('r3m_M6DTest_EntityClient')
        
        self.object = object

        # Create ROS2 Service Clients:
        self.cli_SPAWN = self.create_client(SpawnEntity, "/spawn_entity")  
        self.cli_DELETE = self.create_client(DeleteEntity, "/delete_entity") 

        # Declare REQUEST variable (of CUSTOM DATA type):
        self.req_SPAWN = SpawnEntity.Request()  
        self.req_DELETE = DeleteEntity.Request()

    def spawn_REQUEST(self):
        
        # LOAD URDF of R3M_OBJECT:
        urdf_file_path = os.path.join(get_package_share_directory('r3mcell_cu_gazebo'), 'urdf', 'objects', 'r3m_object.urdf')
        xacro_file = xacro.process_file(urdf_file_path, mappings={"name": self.object})
        
        # ARGUMENTS:
        self.req_SPAWN.name = self.object
        self.req_SPAWN.xml = xacro_file.toxml()
        self.req_SPAWN.initial_pose.position.x = random.uniform(0.5, 0.7)
        self.req_SPAWN.initial_pose.position.y = random.uniform(0.1, 0.9)
        self.req_SPAWN.initial_pose.position.z = 1.0
        # Add here -> Random orientation.

        # Assign RESULT value (future):
        self.future_SPAWN = self.cli_SPAWN.call_async(self.req_SPAWN)

    def delete_REQUEST(self):

        self.req_DELETE.name = self.object
        self.future_DELETE = self.cli_DELETE.call_async(self.req_DELETE)

# ========================================================================================= #
# SUBSCRIBER (ObjectPose):

class PoseSubscriber(Node):

    def __init__(self, object):

        super().__init__("r3m_M6DTest_PoseSubscriber")
        TopicName = "/" + object + "/ObjectPose"
        self.subscription_ = self.create_subscription(ObjectPose, TopicName, self.listener_callback, 10)
        
        self.object = object

    def listener_callback(self, POSE):

        print("ObjectPose (obj: " + self.object + ") information obtained.")

        global ObjPose
        ObjPose = POSE

# =============================================================================== #
# CLASS -> ImgSUB:

class ImgSUB(Node):

    def __init__(self):

        super().__init__("r3m_M6DTest_GzCAM_Subscriber")
        self.SubIMAGE = self.create_subscription(Image, "/camera/image_raw", self.CALLBACK_FN, 10)
        self.BRIDGE = CvBridge()

    def CALLBACK_FN(self, ROS2img):

        global Gz_CAM

        try:
            Gz_CAM = self.BRIDGE.imgmsg_to_cv2(ROS2img, "bgr8")
        except CvBridgeError as ERR:
            print("(cv_bridge): ERROR -> " + ERR) 
            print("")

# =============================================================================== #
# CLASS -> CAMERA:

class CAMERA():

    def __init__(self, object):
        
        self.GzCAM = ImgSUB()
        self.IMGPath = os.path.expanduser('~') + "/M6D_TEST" # This folder MUST BE CREATED in the PC!
        
        self.object = object

    def SaveIMG(self):

        global i

        T = time.time() + 1.0
        while time.time() < T:
            rclpy.spin_once(self.GzCAM)
            self.IMG = Gz_CAM
        
        if self.IMG is not None:
            imgNAME = self.IMGPath + "/R3M_M6DTest_" + self.object + "_IMG_" + str(i) + ".png"
            cv2.imwrite(imgNAME, self.IMG)
            print("Image saved -> " + imgNAME)

# =============================================================================== #
# CLASS -> ObjectPose_LOG:

class ObjectPose_LOG():

    def __init__(self, object):
         
        PATH = os.path.expanduser('~') + "/M6D_TEST"
        self.FilePath = PATH + "/ObjectPose_LOG_" + object + ".txt"
        f = open(self.FilePath, "x")
        f.close()

    def LOGPose(self, POSE):

        f = open(self.FilePath, "a")
        f.write(str(POSE))
        f.write("\n")
        f.close()

        print("ObjectPose logged -> " + str(POSE))

# =============================================================================== #
# Megapose6D:
# TBD.

# ===================================================================================== #
# ======================================= MAIN ======================================== #
# ===================================================================================== #

def AssignArgument(ARGUMENT):
    
    ARGUMENTS = sys.argv
    for y in ARGUMENTS:
        if (ARGUMENT + ":=") in y:
            ARG = y.replace((ARGUMENT + ":="),"")
            return(ARG)

def main(args=None):

    global i
    i = 1

    print("")
    print(" --- R3M Project --- ")
    print("")

    print("Megapose 6D Testing - R3MCell")
    print("Python script -> m6d_test.py")
    print("")
    
    # INPUT ARGUMENT -> OBJECT:
    object = AssignArgument("object")
    if object != None:
        None
    else:
        print("")
        print("ERROR: object INPUT ARGUMENT has not been defined. Please try again.")
        print("Closing... BYE!")
        exit()

    # Initialise ROS2:
    rclpy.init(args=None)

    # Initialise CLASSES:
    ENTITY_CLIENT = EntityClient(object)
    OBJECTPOSE_CLIENT = PoseSubscriber(object)
    IMG_CLIENT = CAMERA(object)
    CPLOG_CLIENT = ObjectPose_LOG(object)

    # Initialise POSE:
    global ObjPose
    POSE = dict()

    # MAIN LOOP:
    while i <= 100:

        # 0. SPAWN OBJECTS:
        ENTITY_CLIENT.spawn_REQUEST()

        print("=============")
        print("Iteration N:" + str(i))

        # 1. Save IMG:
        IMG_CLIENT.SaveIMG() 

        # 2. Get OBJECTPose:    
        rclpy.spin_once(OBJECTPOSE_CLIENT)

        POSE["N"] = i
        POSE["Object Name"] = ObjPose.objectname
        POSE["x"] = round(ObjPose.x, 2)
        POSE["y"] = round(ObjPose.y, 2)
        POSE["z"] = round(ObjPose.z, 2)
        POSE["qx"] = round(ObjPose.qx, 2)
        POSE["qy"] = round(ObjPose.qy, 2)
        POSE["qz"] = round(ObjPose.qz, 2)
        POSE["qw"] = round(ObjPose.qw, 2)

        # 3. Write ObjPose into file:
        CPLOG_CLIENT.LOGPose(POSE)

        # 4. MEGAPOSE:
        # TBD.
        
        # 5. Calculate TRANSFORM and COMPARE values:
        # TBD.
        
        # 6. LOG MEGAPOSE VALUES + ACCURACY:
        # TBD.

        # 7. DELETE OBJECT:
        ENTITY_CLIENT.delete_REQUEST()

        print("")
        i = i+1

    print("Program execution successfully finished!")
    print("Closing .py script... Bye!")

    rclpy.shutdown()

if __name__ == '__main__':
    main()