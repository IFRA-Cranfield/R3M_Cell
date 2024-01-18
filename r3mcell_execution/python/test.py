import rclpy
import time

rclpy.init()

OPTION = "Gripper"

# TEST: ObjectState.py:
from ObjectState import OBJECT

OL = [{"Model": "box", "Link": "box", "CurrentPose": None}, {"Model": "box1", "Link": "box1", "CurrentPose": None}]
OBJ = OBJECT(OL)

RES = OBJ.GetObjectPose()
print(RES)

for x in OL:

    for y in RES["CurrentPose"]:

        if x["Model"] == y.objectname:

            x["CurrentPose"] = y
            break

print(OL)

time.sleep(1.0)

# TEST: Robot.py:
from Robot import RobotClient
from ros2srrc_data.msg import Robpose

ROBOT = RobotClient()

POSE = Robpose()
POSE.x = 0.6
POSE.y = 0.8
POSE.z = 1.12
POSE.qx = 0.0
POSE.qy = 1.0
POSE.qz = 0.0
POSE.qw = 0.0

TYPE = "PTP"
SPEED = 1.0

RESULT = ROBOT.Execute(TYPE,SPEED,POSE)
print(RESULT)

POSE.x = 0.6
POSE.y = 0.8
POSE.z = 1.07
POSE.qx = 0.0
POSE.qy = 1.0
POSE.qz = 0.0
POSE.qw = 0.0

TYPE = "LIN"
SPEED = 0.01

RESULT = ROBOT.Execute(TYPE,SPEED,POSE)
print(RESULT)

# TEST: Gripper_Gz.py:
from Gripper_Gz import ParallelGripper

Robot = {"Model": "irb120", "Link": "EE_egp64"}
GRIPPER = ParallelGripper(Robot)

ACTION = "CLOSE"
SPEED = 1.0

R = GRIPPER.Execute(Robot, OL, ACTION, SPEED)
print(R)

POSE.x = 0.6
POSE.y = 0.8
POSE.z = 1.4
POSE.qx = 0.0
POSE.qy = 1.0
POSE.qz = 0.0
POSE.qw = 0.0

TYPE = "LIN"
SPEED = 0.1

RESULT = ROBOT.Execute(TYPE,SPEED,POSE)
print(RESULT)

ACTION = "OPEN"
SPEED = 1.0

R = GRIPPER.Execute(Robot, OL, ACTION, SPEED)
print(R)

time.sleep(1)

# RESET -> Gazebo:
from ResetGazebo import GzRESET

ResetCond = {}

RP = Robpose()
RP.x = 0.0
RP.y = 0.0
RP.z = 0.0
Robot = {"Package": "r3mcell_cu_gazebo", "Model": "irb120", "Pose": RP}

OP = Robpose()
OP.x = 0.6
OP.y = 0.8
OP.z = 0.88
Cube = {"Package": "r3mcell_cu_gazebo", "Model": "box", "Name": "box", "Pose": OP}
ObjectList = [Cube]

ControllerList = ["joint_state_broadcaster", "irb120_controller", "egp64_finger_right_controller", "egp64_finger_left_controller"]

ResetCond["Robot"] = Robot
ResetCond["ObjectList"] = ObjectList
ResetCond["ControllerList"] = ControllerList

RST = GzRESET(ResetCond)
RST_RES = RST.RESET()
print(RST_RES)