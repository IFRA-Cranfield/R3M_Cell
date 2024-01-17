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
from std_msgs.msg import String

# CUSTOM ROS2 MSG/SRV/ACTION:
from linkpose_msgs.msg import LinkPose
from objectpose_msgs.msg import ObjectPose
from ros2srrc_data.action import Move

# ========================================================================================= #
# ==================================== GLOBAL VARIABLES =================================== #
# ========================================================================================= #

# CUBES:
CUBES = []

# AttachCheck:
from dataclasses import dataclass
@dataclass
class AttDetCHECK:
    ATTACHED: bool
    MODEL: String
    LINK: String
AttachCheck = AttDetCHECK(False, "", "")

# RES:
@dataclass
class RobotRES:
    MESSAGE: String
    SUCCESS: bool
RES = RobotRES("", False)

# EEPose:
EEPose = LinkPose()

# ========================================================================================= #
# =================================== CLASSES/FUNCTIONS =================================== #
# ========================================================================================= #

# ========================================================================================= #
# /Move ACTION CLIENT:
class MoveCLIENT(Node):

    def __init__(self):

        super().__init__('r3mcell(gripper_Gz)_Move_Client')
        self._action_client = ActionClient(self, Move, 'Move')

        print("(/Move)-Gripper: Initialising ROS2 Action Client!")
        print("(/Move)-Gripper: Waiting for /Move ROS2 ActionServer to be available...")
        self._action_client.wait_for_server()
        print("(/Move)-Gripper: /Move ACTION SERVER detected.")

    def send_goal(self, ACTION):

        goal_msg = Move.Goal()
        goal_msg.action = ACTION.action
        goal_msg.speed = ACTION.speed
        goal_msg.moveg = ACTION.moveg
        
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        
        goal_handle = future.result()

        if not goal_handle.accepted:
            print('(/Move)-Gripper: Move ACTION CALL -> GOAL has been REJECTED.')
            return
        
        # print('(/Move): Move ACTION CALL -> GOAL has been ACCEPTED.')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        
        global RES
        
        RESULT = future.result().result
        RES.MESSAGE = RESULT.result

        if "FAILED" in RES.MESSAGE:
            RES.SUCCESS = False
        else:
            RES.SUCCESS = True