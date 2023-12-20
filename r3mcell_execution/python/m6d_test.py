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
CubePose = ObjectPose()
i = None

# ========================================================================================= #
# ServiceClient (SPAWN/DELETE CUBE):

class EntityClient(Node):

    def __init__(self):

        # Initialise ROS2 Node:
        super().__init__('r3m_M6DTest_EntityClient')

        # Create ROS2 Service Clients:
        self.cli_SPAWN = self.create_client(SpawnEntity, "/spawn_entity")  
        self.cli_DELETE = self.create_client(DeleteEntity, "/delete_entity") 

        # Declare REQUEST variable (of CUSTOM DATA type):
        self.req_SPAWN = SpawnEntity.Request()  
        self.req_DELETE = DeleteEntity.Request()

    def spawn_REQUEST(self):
        
        # LOAD URDF of CUBE:
        urdf_file_path = os.path.join(get_package_share_directory('r3mcell_gazebo'), 'urdf', 'box.urdf')
        xacro_file = xacro.process_file(urdf_file_path, mappings={"name": "box"})
        
        # ARGUMENTS:
        self.req_SPAWN.name = "box"
        self.req_SPAWN.xml = xacro_file.toxml()
        self.req_SPAWN.initial_pose.position.x = random.uniform(0.40, 0.75)
        self.req_SPAWN.initial_pose.position.y = random.uniform(0.0, 1.05)
        self.req_SPAWN.initial_pose.position.z = 0.88

        # Assign RESULT value (future):
        self.future_SPAWN = self.cli_SPAWN.call_async(self.req_SPAWN)

    def delete_REQUEST(self):

        self.req_DELETE.name = "box"
        self.future_DELETE = self.cli_DELETE.call_async(self.req_DELETE)

# ========================================================================================= #
# SUBSCRIBER (CubePose):

class PoseSubscriber(Node):

    def __init__(self):

        super().__init__("r3m_M6DTest_PoseSubscriber")
        self.subscription_ = self.create_subscription(ObjectPose, "/box/ObjectPose", self.listener_callback, 10)

    def listener_callback(self, POSE):

        print("CubePose information obtained.")

        global CubePose
        CubePose = POSE

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

    def __init__(self):
        self.GzCAM = ImgSUB()
        self.IMGPath = os.path.expanduser('~') + "/M6D_TEST" # This folder MUST BE CREATED in the PC!

    def SaveIMG(self):

        global i

        T = time.time() + 1.0
        while time.time() < T:
            rclpy.spin_once(self.GzCAM)
            self.IMG = Gz_CAM
        
        if self.IMG is not None:
            imgNAME = self.IMGPath + "/R3M_M6DTest_IMG_" + str(i) + ".png"
            cv2.imwrite(imgNAME, self.IMG)
            print("Image saved -> " + imgNAME)

# =============================================================================== #
# CLASS -> CubePose_LOG:

class CubePose_LOG():

    def __init__(self):
        
        PATH = os.path.expanduser('~') + "/M6D_TEST"
        self.FilePath = PATH + "/CubePose_LOG.txt"
        f = open(self.FilePath, "x")
        f.close()

    def LOGPose(self, POSE):

        f = open(self.FilePath, "a")
        f.write(str(POSE))
        f.close()

        print("CubePose logged -> " + str(POSE))

# =============================================================================== #
# Megapose6D:
# TBD.

# ===================================================================================== #
# ======================================= MAIN ======================================== #
# ===================================================================================== #

def main(args=None):

    global i
    i = 1

    print("")
    print(" --- R3M Project --- ")
    print("")

    print("Megapose 6D Testing - R3MCell")
    print("Python script -> m6d_test.py")
    print("")

    # Initialise ROS2:
    rclpy.init(args=None)

    # Initialise CLASSES:
    ENTITY_CLIENT = EntityClient()
    CUBEPOSE_CLIENT = PoseSubscriber()
    IMG_CLIENT = CAMERA()
    CPLOG_CLIENT = CubePose_LOG()

    # Initialise POSE:
    global CubePose
    POSE = dict()

    # MAIN LOOP:
    while i <= 100:

        # 0. SPAWN CUBE:
        ENTITY_CLIENT.spawn_REQUEST()

        print("=============")
        print("Iteration N:" + str(i))

        # 1. Save IMG:
        IMG_CLIENT.SaveIMG() 

        # 2. Get CubePose:    
        rclpy.spin_once(CUBEPOSE_CLIENT)

        POSE["N"] = i
        POSE["Object Name"] = CubePose.objectname
        POSE["x"] = round(CubePose.x, 2)
        POSE["y"] = round(CubePose.y, 2)
        POSE["z"] = round(CubePose.z, 2)
        POSE["qx"] = round(CubePose.qx, 2)
        POSE["qy"] = round(CubePose.qy, 2)
        POSE["qz"] = round(CubePose.qz, 2)
        POSE["qw"] = round(CubePose.qw, 2)

        # 3. Write CubePose into file:
        CPLOG_CLIENT.LOGPose(POSE)

        # 4. MEGAPOSE:
        # TBD.

        # 5. DELETE CUBE:
        ENTITY_CLIENT.delete_REQUEST()

        print("")
        i = i+1

    print("Program execution successfully finished!")
    print("Closing .py script... Bye!")

    rclpy.shutdown()

if __name__ == '__main__':
    main()