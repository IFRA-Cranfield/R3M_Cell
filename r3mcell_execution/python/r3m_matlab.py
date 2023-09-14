#!/usr/bin/python3

# # # # # # # # # # # # # # # # # #                                  
#                                 #
#   ===== COPYRIGHT HERE =====    #
#                                 #
# # # # # # # # # # # # # # # # # #

# ========================================================================================= #
# INCLUDE:

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import os
import ast
import time

# Std_msgs:
from std_msgs.msg import String
from std_msgs.msg import Int32 as Int

# IMPORT /ExecuteSkill ROS2 Action:
from r3mcell_data.action import ExecuteSkill

# IMPORT /Pose, /Product and /Skillresult ROS2 Messages:
from r3mcell_data.msg import Pose
from r3mcell_data.msg import Product
from r3mcell_data.msg import Skillresult

# GLOBAL VARIABLES -> SkillResult, RES:
SkillResult = Skillresult()
SkillResult.message = "none"

# ========================================================================================= #
# ACTION CLIENT (/ExecuteSkill):

class SkillClient(Node):

    def __init__(self):

        super().__init__("r3m_MATLAB_SkillClient")
        self.actionclient_ = ActionClient(self, ExecuteSkill, "ExecuteSkill")

        print("Waiting for /ExecuteSkill ROS2 Action Server to be available...")
        self.actionclient_.wait_for_server()
        print("/ExecuteSkill ROS2 Action Server detected!")

    def send_goal(self, RECIPE):

        goal_msg = ExecuteSkill.Goal()
        goal_msg.id = RECIPE

        self._send_goal_future = self.actionclient_.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):

        goal_handle = future.result()
        if not goal_handle.accepted:
            #self.get_logger().info('Goal rejected :(')
            return
        #self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        
        global SkillResult

        result = future.result().result
        SkillResult = result.result


# ========================================================================================= #
# PUBLISHER (RESULT):

class ResultPublisher(Node):
    def __init__(self):
        super().__init__("r3m_MATLAB_ResultPublisher")
        self.publisher_ = self.create_publisher(String, "r3m_MATLAB_RES", 10)

# ========================================================================================= #
# PUBLISHER (FEEDBACK):

class FeedbackPublisher(Node):
    def __init__(self):
        super().__init__("r3m_MATLAB_FeedbackPublisher")
        self.publisher_ = self.create_publisher(Skillresult, "r3m_FEEDBACK", 10)

# ========================================================================================= #
# SUBSCRIBER (RESULT):

class RecipeSubscriber(Node):

    def __init__(self):

        super().__init__("r3m_MATLAB_RecipeSubscriber")
        self.subscription_ = self.create_subscription(Int, "r3m_MATLAB", self.listener_callback, 10)

        # INITIALISE:
        self.SKILL_CLIENT = SkillClient()
        self.RESULT_PUBLISHER = ResultPublisher()
        self.FEEDBACK_PUBLISHER = FeedbackPublisher()

    def listener_callback(self, RECIPE):

        global SkillResult

        if (RECIPE.data != 0):
        
            self.SKILL_CLIENT.send_goal(RECIPE.data)
            while rclpy.ok():
                rclpy.spin_once(self.SKILL_CLIENT)
                if (SkillResult.message != "none"):
                    break

            MSG = String()

            if (SkillResult.success == True):
                MSG.data = "SUCCESS"
                self.RESULT_PUBLISHER.publisher_.publish(MSG)
            elif (SkillResult.success == False):
                MSG.data = "ERROR"
                self.RESULT_PUBLISHER.publisher_.publish(MSG)

            self.FEEDBACK_PUBLISHER.publisher_.publish(SkillResult)
            SkillResult.message = "none"

        if (RECIPE.data == 0):
            
            # RESET Gz Environment:
            None

# ========================================================================================= #
# ========================================================================================= #
# ========================================================================================= #

# =================== MAIN =================== #
def main(args=None):

    print(" *** r3m_matlab *** ")
    print("")

    # Initialise NODES:
    rclpy.init(args=args)
    r3mNodeMatlab = RecipeSubscriber()

    # Use a MultiThreadedExecutor to enable processing goals concurrently:
    executor = MultiThreadedExecutor()

    # Spin ACTION:
    rclpy.spin(r3mNodeMatlab, executor=executor)
    r3mNodeMatlab.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()