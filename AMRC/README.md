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

## STEPS TO EXECUTE R3M_Cell -> Pick&Place use-case:

Launch Simulation Environment:
```sh
ros2 launch r3mcell_cu_moveit2 r3mcell_cu_moveit2.launch.py
```

Spawn box into R3M Cell:
```sh
ros2 run r3mcell_execution SpawnObject.py --package "r3mcell_cu_gazebo" --urdf "box.urdf" --name "box" --x -0.45 --y 0.85 --z 0.88
```

Execute RECIPES:
```sh
ros2 action send_goal -f /ExecuteSkill r3mcell_data/action/ExecuteSkill "{id: '---'}"
```

RECIPE LIST:
- RECIPE N1: Move "PTP" to HomePos.
- RECIPE N2: Move "PTP" to PickApproach.
- RECIPE N3: Move "LIN" to Pick.
- RECIPE N4: Move "LIN" to PickApproach.
- RECIPE N5: Move "PTP" to Place.
- RECIPE N6: Grip "CLOSE".
- RECIPE N7: Grip "OPEN".