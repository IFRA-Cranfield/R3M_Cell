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
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient

# ROS2 MSG/SRV/ACTION:
from std_msgs.msg import String

# CUSTOM ROS2 MSG/SRV/ACTION:
from ros2srrc_data.action import Move
from ros2srrc_data.msg import Action
from linkattacher_msgs.srv import AttachLink
from linkattacher_msgs.srv import DetachLink
from linkpose_msgs.msg import LinkPose

# ========================================================================================= #
# ==================================== GLOBAL VARIABLES =================================== #
# ========================================================================================= #

# RES:
from dataclasses import dataclass
@dataclass
class RobotRES:
    Message: String
    Success: bool
    ExecTime: float
    Error: float
    
RES = RobotRES("", False, -1.0, -1.0)

# AttachCheck:
@dataclass
class AttDetCHECK:
    Attached: bool
    Object: dict()

AttachCheck = AttDetCHECK(False, None)

# EEPose:
EEPose = LinkPose()

# EEState:
EEState = 1

# ========================================================================================= #
# =================================== CLASSES/FUNCTIONS =================================== #
# ========================================================================================= #

# ========================================================================================= #
# Vacuum Gripper:
class VacuumGripper():
    
    # For information, the inputs to this class are:
    # Robot = {"Model": "", "Link": "", "EEPose": Robpose()}
    # ObjectList = [{"Model": "box", "Link": "box", "CurrentPose": ObjectPose()}, ...]
    
    def __init__(self, Robot):

        # Initialise CLASSES to be used:
        self.LinkAttacher_CLIENT = LinkAttacher()
        self.EEPose_CLIENT = EEPoseCLIENT(Robot)

        self.Robot = Robot

    def Execute(self, Robot, ObjectList, ACTION):
        
        # ===== VacuumON // VacuumOFF ===== #
        # RESULT -> Convert to DICTIONARY:
        RESULT = {}
        
        global EEState
        RESULT["EEState"] = EEState 

        # Quick fix:
        if Robot == None:
            Robot = self.Robot

        # === DETACH === #
        if ACTION == "VacuumOFF":

            RESULT["Message"] = "Vacuum Gripper: VACUUM DEACTIVATED."
            
            EEState = 1
            RESULT["EEState"] = EEState

            # DET(1) -> CHECK for DETACHMENTS:
            if AttachCheck.Attached == True:
                
                # DET(2) -> DETACH:
                DETACH_RES = self.LinkAttacher_CLIENT.DETACH(Robot, AttachCheck.Object)

                if DETACH_RES:
                    self.EEPose_CLIENT.get_logger().info("[R3M Cell] - (VacuumGripper): Gripper OFF, OBJECT -> " + AttachCheck.Object["Model"] + " detached.")
                else:
                    self.EEPose_CLIENT.get_logger().info("[R3M Cell] - (VacuumGripper): Gripper OFF, OBJECT -> " + AttachCheck.Object["Model"] + " not detached, LinkAttacher plugin failed.")

            else:
                self.EEPose_CLIENT.get_logger().info("[R3M Cell] - (VacuumGripper): Gripper OFF without dropping any object.")

        # === ATTACH === #
        if ACTION == "VacuumON":

            RESULT["Message"] = "Vacuum Gripper: VACUUM ACTIVATED."
            
            EEState = 0
            RESULT["EEState"] = EEState
            
            # ATT(1) -> CHECK for ATTACHMENTS:
            CHECK_RES = self.CHECK(ObjectList)

            if CHECK_RES["Success"]:

                # ATT(2) -> ATTACH:
                OBJ = {"Model": CHECK_RES["Model"], "Link": CHECK_RES["Link"]}
                ATTACH_RES = self.LinkAttacher_CLIENT.ATTACH(Robot, OBJ)

                if ATTACH_RES:
                    self.EEPose_CLIENT.get_logger().info("[R3M Cell] - (ParallelGripper): Gripper ON, OBJECT -> " + OBJ["Model"] + " attached.")
                else:
                    self.EEPose_CLIENT.get_logger().info("[R3M Cell] - (ParallelGripper): Gripper ON, OBJECT -> " + OBJ["Model"] + " not attached, LinkAttacher plugin failed.")

            else:
                self.EEPose_CLIENT.get_logger().info("[R3M Cell] - (ParallelGripper): Gripper ON without grasping any object.")

        RESULT["Success"] = True
        RESULT["ExecTime"] = 0.0
        RESULT["Error"] = 0.0

        # Return RESULT:
        return(RESULT)
    
    def CHECK(self, ObjectList):

        # RESULT:
        RESULT = {"Success": False, "Model": "", "Link": ""}

        # Get EEPose:
        global EEPose
        T = time.time() + 0.1
        while (time.time() < T):
            rclpy.spin_once(self.EEPose_CLIENT)

        Check = True
        for x in ObjectList:
            
            ObjectPose = x["CurrentPose"]

            # Print:
            self.EEPose_CLIENT.get_logger().info("[R3M Cell] - Checking if object is attached for OBJECT: " + x["Model"])
            self.EEPose_CLIENT.get_logger().info("[R3M Cell] - EEPose.x -> " + str(EEPose.x) + " / ObjectPose.x -> " + str(ObjectPose.x))
            self.EEPose_CLIENT.get_logger().info("[R3M Cell] - EEPose.y -> " + str(EEPose.y) + " / ObjectPose.y -> " + str(ObjectPose.y))
            self.EEPose_CLIENT.get_logger().info("[R3M Cell] - EEPose.z -> " + str(EEPose.z) + " / ObjectPose.z -> " + str(ObjectPose.z))

            if (EEPose.x - 0.01 > ObjectPose.x) or (EEPose.x + 0.01 < ObjectPose.x): 
                Check = False
            if (EEPose.y - 0.01 > ObjectPose.y) or (EEPose.y + 0.01 < ObjectPose.y): 
                Check = False
            if (EEPose.z - 0.02 > ObjectPose.z) or (EEPose.z < ObjectPose.z): 
                Check = False

            if Check == True:

                RESULT["Success"] = True
                RESULT["Model"] = x["Model"]
                RESULT["Link"] = x["Link"]

        return(RESULT)

# ========================================================================================= #
# Parallel Gripper:
class ParallelGripper():
    
    # For information, the inputs to this class are:
    # Robot = {"Model": "", "Link": "", "EEPose": Robpose()}
    # ObjectList = [{"Model": "box", "Link": "box", "CurrentPose": ObjectPose()}, ...]
    
    def __init__(self, Robot):

        # Initialise CLASSES to be used:
        self.Gripper_CLIENT = MoveCLIENT()
        self.LinkAttacher_CLIENT = LinkAttacher()
        self.EEPose_CLIENT = EEPoseCLIENT(Robot)

        self.Robot = Robot

    def Execute(self, Robot, ObjectList, ACTION, SPEED):

        RESULT = {}
        
        global EEState
        RESULT["EEState"] = EEState 
        
        # Quick fix:
        if Robot == None:
            Robot = self.Robot
        
        global RES
        global AttachCheck
        
        # EXECUTE GRIPPER MOVEMENT:
        
        MoveG = Action()
        MoveG.speed = SPEED
        if ACTION == "CLOSE":
            MoveG.moveg = 0.006
        elif ACTION == "OPEN":
            MoveG.moveg = 0.00
        
        self.Gripper_CLIENT.send_goal(MoveG)

        while rclpy.ok():
            
            rclpy.spin_once(self.Gripper_CLIENT)

            if (RES.Message != ""):
                break

        # === DETACH === #
        if ACTION == "OPEN":
            
            # Define EEState:
            if RES.Success:
                EEState = 1
                RESULT["EEState"] = EEState

            # DET(1) -> CHECK for DETACHMENTS:
            if AttachCheck.Attached == True:
                
                # DET(2) -> DETACH:
                DETACH_RES = self.LinkAttacher_CLIENT.DETACH(Robot, AttachCheck.Object)

                if DETACH_RES:
                    self.Gripper_CLIENT.get_logger().info("[R3M Cell] - (ParallelGripper): Gripper opened, OBJECT -> " + AttachCheck.Object["Model"] + " detached.")
                else:
                    self.Gripper_CLIENT.get_logger().info("[R3M Cell] - (ParallelGripper): Gripper opened, OBJECT -> " + AttachCheck.Object["Model"] + " not detached, LinkAttacher plugin failed.")

            else:
                self.Gripper_CLIENT.get_logger().info("[R3M Cell] - (ParallelGripper): Gripper opened without dropping any object.")

        # === ATTACH === #
        if ACTION == "CLOSE":
            
            # Define EEState:
            if RES.Success:
                EEState = 0
                RESULT["EEState"] = EEState
            
            # ATT(1) -> CHECK for ATTACHMENTS:
            CHECK_RES = self.CHECK(ObjectList)

            if CHECK_RES["Success"]:

                # ATT(2) -> ATTACH:
                OBJ = {"Model": CHECK_RES["Model"], "Link": CHECK_RES["Link"]}
                ATTACH_RES = self.LinkAttacher_CLIENT.ATTACH(Robot, OBJ)

                if ATTACH_RES:
                    self.Gripper_CLIENT.get_logger().info("[R3M Cell] - (ParallelGripper): Gripper closed, OBJECT -> " + OBJ["Model"] + " attached.")
                else:
                    self.Gripper_CLIENT.get_logger().info("[R3M Cell] - (ParallelGripper): Gripper closed, OBJECT -> " + OBJ["Model"] + " not attached, LinkAttacher plugin failed.")

            else:
                self.Gripper_CLIENT.get_logger().info("[R3M Cell] - (ParallelGripper): Gripper closed without grasping any object.")

        # RESULT -> Convert to DICTIONARY:
        RESULT["Message"] = RES.Message
        RESULT["Success"] = RES.Success
        RESULT["ExecTime"] = RES.ExecTime
        RESULT["Error"] = RES.Error

        # Reset RES variable:
        RES = RobotRES("", False, -1.0, -1.0)

        # Return RESULT:
        return(RESULT)
    
    def CHECK(self, ObjectList):

        # RESULT:
        RESULT = {"Success": False, "Model": "", "Link": ""}

        # Get EEPose:
        global EEPose
        T = time.time() + 0.1
        while (time.time() < T):
            rclpy.spin_once(self.EEPose_CLIENT)

        Check = True
        for x in ObjectList:
            
            ObjectPose = x["CurrentPose"]

            # Print:
            self.Gripper_CLIENT.get_logger().info("[R3M Cell] - Checking if object is attached for OBJECT: " + x["Model"])
            self.Gripper_CLIENT.get_logger().info("[R3M Cell] - EEPose.x -> " + str(EEPose.x) + " / ObjectPose.x -> " + str(ObjectPose.x))
            self.Gripper_CLIENT.get_logger().info("[R3M Cell] - EEPose.y -> " + str(EEPose.y) + " / ObjectPose.y -> " + str(ObjectPose.y))
            self.Gripper_CLIENT.get_logger().info("[R3M Cell] - EEPose.z -> " + str(EEPose.z) + " / ObjectPose.z -> " + str(ObjectPose.z))

            if (EEPose.x - 0.01 > ObjectPose.x) or (EEPose.x + 0.01 < ObjectPose.x): 
                Check = False
            if (EEPose.y - 0.01 > ObjectPose.y) or (EEPose.y + 0.01 < ObjectPose.y): 
                Check = False
            if (EEPose.z - 0.01 > ObjectPose.z) or (EEPose.z + 0.01 < ObjectPose.z): 
                Check = False

            if Check == True:

                RESULT["Success"] = True
                RESULT["Model"] = x["Model"]
                RESULT["Link"] = x["Link"]

        return(RESULT)
        
# ========================================================================================= #
# CLASS to check the EEPose:
class EEPoseCLIENT(Node):

    def __init__(self, Robot):

        super().__init__("r3mcell_EEPose_Subscriber")

        TopicName = "/LinkPose_" + Robot["Model"] + "_" + Robot["Link"]
        self.SUB = self.create_subscription(LinkPose, TopicName, self.CALLBACK_FN, 1)

    def CALLBACK_FN(self, POSE):

        global EEPose
        EEPose = POSE

# ========================================================================================= #
# /Move ACTION CLIENT:
class MoveCLIENT(Node):

    def __init__(self):

        super().__init__('r3mcell_gripper_Gz_Move_Client')
        self._action_client = ActionClient(self, Move, 'Move')

        self.get_logger().info("[R3M Cell] - (/Move)-Gripper: Initialising ROS2 Action Client!")
        self.get_logger().info("[R3M Cell] - (/Move)-Gripper: Waiting for /Move ROS2 ActionServer to be available...")
        self._action_client.wait_for_server()
        self.get_logger().info("[R3M Cell] - (/Move)-Gripper: /Move ACTION SERVER detected.")

    def send_goal(self, ACTION):

        self.T_start = time.time()

        goal_msg = Move.Goal()
        goal_msg.action = "MoveG"
        goal_msg.speed = ACTION.speed
        goal_msg.moveg = ACTION.moveg
        
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info("[R3M Cell] - (/Move)-Gripper: Move ACTION CALL -> GOAL has been REJECTED.")
            return
        
        # print('(/Move): Move ACTION CALL -> GOAL has been ACCEPTED.')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        
        global RES

        # 0. Compute time difference:
        self.T_end = time.time()
        T = round((self.T_end - self.T_start), 4)
        RES.ExecTime = T

        # 1. ERROR in ParallelGriper = 0.0:
        RES.Error = 0.0

        # 2. GET RESULT:
        RESULT = future.result().result
        RES.Message = RESULT.result

        if "FAILED" in RES.Message:
            RES.Success = False
        else:
            RES.Success = True

# ========================================================================================= #
# ===================================== LINK ATTACHER ===================================== #
# ========================================================================================= #
    
# ========================================================================================= #
# LinkAttacher ROS2 Service (/ATTACHLINK and /DETACHLINK) CLIENTS:
class LinkAttacher_Client(Node):

    def __init__(self):

        super().__init__("r3mcell_LinkAttacher_Client")

        self.AttachClient = self.create_client(AttachLink, "/ATTACHLINK")
        self.DetachClient = self.create_client(DetachLink, "/DETACHLINK")

        while not self.AttachClient.wait_for_service(timeout_sec=1.0): 
            self.get_logger().info("[R3M Cell] - (LinkAttacher): /ATTACHLINK ROS2 Service not still available, waiting...")
        self.get_logger().info("[R3M Cell] - (LinkAttacher): /ATTACHLINK ROS2 Service ready.")
        while not self.DetachClient.wait_for_service(timeout_sec=1.0): 
            self.get_logger().info("[R3M Cell] - (LinkAttacher): /DETACHLINK ROS2 Service not still available, waiting...")
        self.get_logger().info("[R3M Cell] - (LinkAttacher): /DETACHLINK ROS2 Service ready.")

        self.AttachRequest = AttachLink.Request()
        self.DetachRequest = DetachLink.Request()

    def ATTACHService(self, Robot, Object):

        self.AttachRequest.model1_name = Robot["Model"]
        self.AttachRequest.link1_name = Robot["Link"]
        self.AttachRequest.model2_name = Object["Model"]
        self.AttachRequest.link2_name = Object["Link"]

        self.AttachFuture = self.AttachClient.call_async(self.AttachRequest)

    def DETACHService(self, Robot, Object):

        self.DetachRequest.model1_name = Robot["Model"]
        self.DetachRequest.link1_name = Robot["Link"]
        self.DetachRequest.model2_name = Object["Model"]
        self.DetachRequest.link2_name = Object["Link"]

        self.DetachFuture = self.DetachClient.call_async(self.DetachRequest)

# ========================================================================================= #
# LinkAttacher CLASS:
class LinkAttacher():

    def __init__(self):

        # Initialise ROS2 Service CLIENTS:
        self.CLIENT = LinkAttacher_Client()

    def ATTACH(self, Robot, Object):

        global AttachCheck

        self.CLIENT.ATTACHService(Robot,Object)

        while rclpy.ok():
            rclpy.spin_once(self.CLIENT)
            if self.CLIENT.AttachFuture.done():
                try:
                    AttachRES = self.CLIENT.AttachFuture.result()
                except Exception as exc:
                    self.CLIENT.get_logger().info("[R3M Cell] - (LinkAttacher): /ATTACHLINK Service call failed -> " + str(exc))
                    return(False)
                else:
                    if (AttachRES.success):
                        self.CLIENT.get_logger().info("[R3M Cell] - (LinkAttacher): /ATTACHLINK successful -> " + str(AttachRES.message))

                        AttachCheck.Attached = True
                        AttachCheck.Object = Object

                        return(True)
                    else:
                        self.CLIENT.get_logger().info("[R3M Cell] - (LinkAttacher): /ATTACHLINK unuccessful -> " + str(AttachRES.message))
                        return(False)
                    
    def DETACH(self, Robot, Object):

        global AttachCheck

        self.CLIENT.DETACHService(Robot,Object)

        while rclpy.ok():
            rclpy.spin_once(self.CLIENT)
            if self.CLIENT.DetachFuture.done():
                try:
                    DetachRES = self.CLIENT.DetachFuture.result()
                except Exception as exc:
                    self.CLIENT.get_logger().info("[R3M Cell] - (LinkAttacher): /DETACHLINK Service call failed -> " + str(exc))
                    return(False)
                else:
                    if (DetachRES.success):
                        self.CLIENT.get_logger().info("[R3M Cell] - (LinkAttacher): /DETACHLINK successful -> " + str(DetachRES.message))

                        AttachCheck.Attached = False
                        AttachCheck.Object = {"Model": "", "Link": ""}

                        # We wait Xs in order to give time to the ObjectPose() subscriber to wait until the object is dropped.
                        time.sleep(1)

                        return(True)
                    else:
                        self.CLIENT.get_logger().info("[R3M Cell] - (LinkAttacher): /DETACHLINK unuccessful -> " + str(DetachRES.message))
                        return(False)