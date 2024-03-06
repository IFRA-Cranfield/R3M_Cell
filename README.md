# R3M_Cell

## INSTALLATION
0. Ubuntu 22.04 + ROS2 Humble is required.
1. Install ros2_SimRealRobotControl repository. Follow steps in: https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl/tree/humble
2. Install IFRA_LinkAttacher (Grasping PLUGIN): https://github.com/IFRA-Cranfield/IFRA_LinkAttacher
3. Install IFRA_ObjectPose (ObjectPose PLUGIN): https://github.com/IFRA-Cranfield/IFRA_ObjectPose
4. Install IFRA_LinkPose (EEPose PLUGIN): https://github.com/IFRA-Cranfield/IFRA_LinkPose
5. Install R3M_Cell (https://github.com/R3M-UK/R3M_Cell):
    ```sh
    cd ~/dev_ws/src
    git clone https://github.com/R3M-UK/R3M_Cell
    cd ~/dev_ws
    colcon build
    ```

## STEPS TO EXECUTE R3M_Cell for R3M-Perception testing:

Launch Simulation Environment:
```sh
ros2 launch r3mcell_cu_moveit2 moveit.launch.py layout:=r3mcell_cu_4
```

Spawn any object manually to the workspace:
```sh
ros2 run r3mcell_execution SpawnObject.py --package "r3mcell_cu_gazebo" --urdf "r3m_object.urdf" --name "{}" --x {} --y {} --z {}

REMOVE OBJECT:
To remove any object from the Gazebo environment, right-click the object on the left-panel and click "delete/remove".

RECOMMENDED POSES to Spawn Objects on top of the panel:
- x -> (0.5 , 0.7)
- y -> (0.1 , 0.9)
- z -> 1.0
```

Execute m6d_test script to save images and ObjectPoses to the /M6D_TEST folder (this script automatically generates 100 images and the .txt log file with the pose information for the selected object):
```sh
ros2 run r3mcell_execution m6d_test.py object:={}
```
</br>

__OBJECTS__

Replace the {} tag in the commandline for any of the following names to spawn the objects in Gazebo:
- adapter_plate_square
- adapter_plate_triangular
- bracket_big
- bracket_planar
- bracket_screw
- can
- cap
- car_rim
- clamp_big
- clamp_small
- connector_planar
- engine_part_bearing
- engine_part_cooler_round
- engine_part_cooler_square
- engine_part_cover
- filter
- fuse
- injection_pump
- LamSheet
- multi_bracket
- punched_rail
- screw
- star
- tee_connector
- thread
- washer

## STEPS TO EXECUTE the APG Agent:

Launch Simulation Environment:
```sh
ros2 launch r3mcell_cu_moveit2 moveit2.launch.py layout:=r3mcell_cu_1 autoOP:=True
```

Launch APG Agent:
```sh
ros2 run r3mcell_execution apg_MATLAB.py
```