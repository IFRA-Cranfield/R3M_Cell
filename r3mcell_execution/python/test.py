import rclpy
from ObjectState import OBJECT

rclpy.init()

OL = [{"Model": "box", "Link": "box"}, {"Model": "box1", "Link": "box1"}]
OBJ = OBJECT(OL)

RES = OBJ.GetObjectPose()
print(RES)

RES2 = OBJ.GetObjectPose()
print(RES2)