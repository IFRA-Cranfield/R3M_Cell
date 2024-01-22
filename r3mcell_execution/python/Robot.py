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
import math
import time

# ROS2:
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

# ROS2 MSG/SRV/ACTION:
from std_msgs.msg import String

# CUSTOM ROS2 MSG/SRV/ACTION:
from ros2srrc_data.action import Robmove
from ros2srrc_data.msg import Robpose

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

# RobotPose:
RobotPose = Robpose()

# ========================================================================================= #
# =================================== CLASSES/FUNCTIONS =================================== #
# ========================================================================================= #

# ========================================================================================= #
# /Robpose -> SUBSCRIBER:
class RobPoseCLIENT(Node):

    def __init__(self):

        super().__init__("r3mcell_RobPose_Subscriber")
        self.SUB = self.create_subscription(Robpose, "/Robpose", self.CALLBACK_FN, 10)

    def CALLBACK_FN(self, POSE):

        global RobotPose
        RobotPose = POSE

# ========================================================================================= #
# CLASS to -> Execute ROBOT MOVEMENT:
class RobotClient():

    def __init__(self):

        # Initialise RobMoveCLIENT:
        self.RobMove_Client = RobMoveCLIENT()

    def Execute(self, TYPE, SPEED, POSE):

        global RES

        # Execute ROS2 ACTION:
        self.RobMove_Client.send_goal(TYPE,SPEED,POSE)

        while rclpy.ok():

            rclpy.spin_once(self.RobMove_Client)

            if (RES.Message != ""):
                break

        # Convert to DICTIONARY:
        RESULT = {}
        RESULT["Message"] = RES.Message
        RESULT["Success"] = RES.Success
        RESULT["ExecTime"] = RES.ExecTime
        RESULT["Error"] = RES.Error

        # Reset RES variable:
        RES = RobotRES("", False, -1.0, -1.0)

        # Return RESULT:
        return(RESULT)
        
# ========================================================================================= #
# /RobMove ACTION CLIENT:
class RobMoveCLIENT(Node):

    def __init__(self):

        super().__init__('r3mcell_RobMove_Client')
        self._action_client = ActionClient(self, Robmove, 'Robmove')

        self.get_logger().info("[R3M Cell] - (/RobMove): Initialising ROS2 Action Client!")
        self.get_logger().info("[R3M Cell] - (/RobMove): Waiting for /Robmove ROS2 ActionServer to be available...")
        self._action_client.wait_for_server()
        self.get_logger().info("[R3M Cell] - (/RobMove): /Robmove ACTION SERVER detected.")
        
        # Initialise RobPose CLIENT:
        self.RP = RobPoseCLIENT()

    def send_goal(self, TYPE, SPEED, TARGET_POSE):

        self.T_start = time.time()
        
        # STORE POSE1 for ERROR CALCULATION:
        self.POSE1 = Robpose()
        self.POSE1.x = TARGET_POSE.x
        self.POSE1.y = TARGET_POSE.y
        self.POSE1.z = TARGET_POSE.z
        self.POSE1.qx = TARGET_POSE.qx
        self.POSE1.qy = TARGET_POSE.qy
        self.POSE1.qz = TARGET_POSE.qz
        self.POSE1.qw = TARGET_POSE.qw
        
        # Define GOAL:
        goal_msg = Robmove.Goal()
        goal_msg.type = TYPE
        goal_msg.speed = SPEED
        goal_msg.x = TARGET_POSE.x
        goal_msg.y = TARGET_POSE.y
        goal_msg.z = TARGET_POSE.z
        goal_msg.qx = TARGET_POSE.qx
        goal_msg.qy = TARGET_POSE.qy
        goal_msg.qz = TARGET_POSE.qz
        goal_msg.qw = TARGET_POSE.qw
        
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('[R3M Cell] - (/RobMove): RobMove ACTION CALL -> GOAL has been REJECTED.')
            return
        
        # print('(/RobMove): RobMove ACTION CALL -> GOAL has been ACCEPTED.')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        
        global RES
        RESULT = future.result().result

        # 0. Compute time difference:
        self.T_end = time.time()
        T = round((self.T_end - self.T_start), 4)
        RES.ExecTime = T
        
        # 1. Get /RobPose ACTION RESULT:
        RES.Message = RESULT.message
        RES.Success = RESULT.success 
        
        # 2. Get NEW ROBOTPOSE:
        global RobotPose
        T = time.time() + 0.1
        while (time.time() < T):
            rclpy.spin_once(self.RP)
        self.POSE2 = RobotPose
        
        # 3. Compare values and GET ERROR:
        ERROR = round(CalculateError(self.POSE1, self.POSE2), 10)
        RES.Error = ERROR

# ========================================================================================= #
# Function -> Get Error (ROBOT):
def CalculateError(TARGET_POSE, OBTAINED_POSE):

    # POSITION ERROR: Norm of the (p1-p0) difference vector:

    DIFx = abs(OBTAINED_POSE.x - TARGET_POSE.x)
    DIFy = abs(OBTAINED_POSE.y - TARGET_POSE.y)
    DIFz = abs(OBTAINED_POSE.z - TARGET_POSE.z)

    ERROR_POS = math.sqrt(DIFx*DIFx + DIFy*DIFy + DIFz*DIFz)

    # ROTATION ERROR: Norm of the (C = A*inv(B)) difference quaternion:

    Ax = OBTAINED_POSE.qx
    Ay = OBTAINED_POSE.qy
    Az = OBTAINED_POSE.qz
    Aw = OBTAINED_POSE.qw

    Bx = -TARGET_POSE.qx
    By = -TARGET_POSE.qy
    Bz = -TARGET_POSE.qz
    Bw = TARGET_POSE.qw

    qw = Aw*Bw - Ax*Bx - Ay*By - Az*Bz # Not needed.
    qx = Aw*Bx + Ax*Bw + Ay*Bz - Az*By
    qy = Aw*By - Ax*Bz + Ay*Bw + Az*Bx
    qz = Aw*Bz + Ax*By - Ay*Bx + Az*Bw

    ERROR_ROT = math.sqrt((qx*qx)+(qy*qy)+(qz*qz))

    # ABSOLUTE ERROR: Average between POSITION and ROTATION errors:
    ERROR = (ERROR_POS + ERROR_ROT)/2
    return(ERROR)