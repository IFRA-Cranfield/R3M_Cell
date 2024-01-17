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
# /RobMove ACTION CLIENT:
class RobMoveCLIENT(Node):

    def __init__(self):

        super().__init__('r3mcell_RobMove_Client')
        self._action_client = ActionClient(self, Robmove, 'Robmove')

        print("(/RobMove): Initialising ROS2 Action Client!")
        print("(/RobMove): Waiting for /Robmove ROS2 ActionServer to be available...")
        self._action_client.wait_for_server()
        print("(/RobMove): /Robmove ACTION SERVER detected.")
        
        # Initialise RobPose CLIENT:
        self.RP = RobPoseCLIENT()

    def send_goal(self, TYPE, SPEED, TARGET_POSE):
        
        # STORE POSE1 for ERROR CALCULATION:
        self.POSE1 = Robpose()
        self.POSE1.x = TARGET_POSE.position.x
        self.POSE1.y = TARGET_POSE.position.y
        self.POSE1.z = TARGET_POSE.position.z
        self.POSE1.qx = TARGET_POSE.orientation.x
        self.POSE1.qy = TARGET_POSE.orientation.y
        self.POSE1.qz = TARGET_POSE.orientation.z
        self.POSE1.qw = TARGET_POSE.orientation.w
        
        # Define GOAL:
        goal_msg = Robmove.Goal()
        goal_msg.type = TYPE
        goal_msg.speed = SPEED
        goal_msg.x = TARGET_POSE.position.x
        goal_msg.y = TARGET_POSE.position.y
        goal_msg.z = TARGET_POSE.position.z
        goal_msg.qx = TARGET_POSE.orientation.x
        goal_msg.qy = TARGET_POSE.orientation.y
        goal_msg.qz = TARGET_POSE.orientation.z
        goal_msg.qw = TARGET_POSE.orientation.w
        
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        
        goal_handle = future.result()

        if not goal_handle.accepted:
            print('(/RobMove): RobMove ACTION CALL -> GOAL has been REJECTED.')
            return
        
        # print('(/RobMove): RobMove ACTION CALL -> GOAL has been ACCEPTED.')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        
        global RES
        RESULT = future.result().result
        
        # 1. Get /RobPose ACTION RESULT:
        RES.MESSAGE = RESULT.message
        RES.SUCCESS = RESULT.success 
        
        # 2. Get NEW ROBOTPOSE:
        global RobotPose
        rclpy.spin_once(self.RP)
        self.POSE2 = RobotPose
        
        # 3. Compare values and GET ERROR:
        
        
        