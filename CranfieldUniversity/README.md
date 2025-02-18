# R3M Research Project - R3M-Cell (Cranfield University)

## R3M-Cell: Cranfield University

This folder contains the ROS 2 packages for the robotic cells at Cranfield University, designed for simulation, motion planning, and real-world deployment.

## ros2_SimRealRobotControl - ROS 2 Package Structure

The ROS 2 package structure follows a modular approach, integrating Gazebo for simulation, MoveIt! 2 for motion planning, and hardware drivers for real robot control. Each package is configured to support different R3M use cases, allowing seamless transition between simulation and real-world execution.

These packages are based on the [ros2_SimRealRobotControl framework](https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl/tree/humble/packages), ensuring compatibility with standardized ROS 2 workflows and launch configurations.

## Cell Configurations & R3M Use-Cases (+ ROS 2 Launch Commands)

More information about the R3M-Cell/CranfieldUni robot cell configurations can be found in the r3mcell_cu_gazebo/config/configurations.yaml file. The following commands launch the Simulation/Sim+MoveIt!2/Bringup+MoveIt!2 ROS 2 Environments for the R3M Cells:

ABB IRB-120 Cell:

- r3mcell_cu_1: Empty Cell.

    ```sh

    # Gazebo Simulation:
    ros2 launch ros2srrc_launch simulation.launch.py package:=r3mcell_cu config:=r3mcell_cu_1

    # Gazebo Simulation + MoveIt!2 Framework:
    ros2 launch ros2srrc_launch moveit2.launch.py package:=r3mcell_cu config:=r3mcell_cu_1

    # Robot Bringup (ROS 2 Driver) + MoveIt!2 Framework:
    ros2 launch ros2srrc_launch bringup_abb.launch.py package:=r3mcell_cu config:=r3mcell_cu_1 robot_ip:=0.0.0.0

    ```

- r3mcell_cu_11: Cube Stacking Use-Case.

    ```sh

    # Gazebo Simulation:
    ros2 launch ros2srrc_launch simulation.launch.py package:=r3mcell_cu config:=r3mcell_cu_11

    # Gazebo Simulation + MoveIt!2 Framework:
    ros2 launch ros2srrc_launch moveit2.launch.py package:=r3mcell_cu config:=r3mcell_cu_11

    # Robot Bringup (ROS 2 Driver) + MoveIt!2 Framework:
    ros2 launch ros2srrc_launch bringup_abb.launch.py package:=r3mcell_cu config:=r3mcell_cu_11 robot_ip:=0.0.0.0

    ```

- r3mcell_cu_12: Coloured Cube Kitting Use-Case.

    ```sh

    # Gazebo Simulation:
    ros2 launch ros2srrc_launch simulation.launch.py package:=r3mcell_cu config:=r3mcell_cu_12

    # Gazebo Simulation + MoveIt!2 Framework:
    ros2 launch ros2srrc_launch moveit2.launch.py package:=r3mcell_cu config:=r3mcell_cu_12

    # Robot Bringup (ROS 2 Driver) + MoveIt!2 Framework:
    ros2 launch ros2srrc_launch bringup_abb.launch.py package:=r3mcell_cu config:=r3mcell_cu_12 robot_ip:=0.0.0.0

    ```

- r3mcell_cu_13: Cylinder Stacking Use-Case.

    ```sh

    # Gazebo Simulation:
    ros2 launch ros2srrc_launch simulation.launch.py package:=r3mcell_cu config:=r3mcell_cu_13

    # Gazebo Simulation + MoveIt!2 Framework:
    ros2 launch ros2srrc_launch moveit2.launch.py package:=r3mcell_cu config:=r3mcell_cu_13

    # Robot Bringup (ROS 2 Driver) + MoveIt!2 Framework:
    ros2 launch ros2srrc_launch bringup_abb.launch.py package:=r3mcell_cu config:=r3mcell_cu_13 robot_ip:=0.0.0.0

    ```

- r3mcell_cu_14: Can Pick-and-Place Use-Case.

    ```sh

    # Gazebo Simulation:
    ros2 launch ros2srrc_launch simulation.launch.py package:=r3mcell_cu config:=r3mcell_cu_14

    # Gazebo Simulation + MoveIt!2 Framework:
    ros2 launch ros2srrc_launch moveit2.launch.py package:=r3mcell_cu config:=r3mcell_cu_14

    # Robot Bringup (ROS 2 Driver) + MoveIt!2 Framework:
    N/A

    ```

- r3mcell_cu_15: Battery Disassembly Use-Case.

    ```sh

    # Gazebo Simulation:
    ros2 launch ros2srrc_launch simulation.launch.py package:=r3mcell_cu config:=r3mcell_cu_15

    # Gazebo Simulation + MoveIt!2 Framework:
    ros2 launch ros2srrc_launch moveit2.launch.py package:=r3mcell_cu config:=r3mcell_cu_15

    # Robot Bringup (ROS 2 Driver) + MoveIt!2 Framework:
    ros2 launch ros2srrc_launch bringup_abb.launch.py package:=r3mcell_cu config:=r3mcell_cu_15 robot_ip:=0.0.0.0

    ```

- r3mcell_cu_16: Lamination Sheet Pick-and-Place Use-Case.

    ```sh

    # Gazebo Simulation:
    ros2 launch ros2srrc_launch simulation.launch.py package:=r3mcell_cu config:=r3mcell_cu_16

    # Gazebo Simulation + MoveIt!2 Framework:
    ros2 launch ros2srrc_launch moveit2.launch.py package:=r3mcell_cu config:=r3mcell_cu_16

    # Robot Bringup (ROS 2 Driver) + MoveIt!2 Framework:
    N/A

    ```

- r3mcell_cu_99: R3M-Perception Testing set-up.

    ```sh

    # Gazebo Simulation:
    ros2 launch ros2srrc_launch simulation.launch.py package:=r3mcell_cu config:=r3mcell_cu_99

    # Gazebo Simulation + MoveIt!2 Framework:
    ros2 launch ros2srrc_launch moveit2.launch.py package:=r3mcell_cu config:=r3mcell_cu_99

    # Robot Bringup (ROS 2 Driver) + MoveIt!2 Framework:
    N/A

    ```

Universal Robots UR3 Cell:

- r3mcell_cu_2: Empty Cell.

    ```sh

    # Gazebo Simulation:
    ros2 launch ros2srrc_launch simulation.launch.py package:=r3mcell_cu config:=r3mcell_cu_2

    # Gazebo Simulation + MoveIt!2 Framework:
    ros2 launch ros2srrc_launch moveit2.launch.py package:=r3mcell_cu config:=r3mcell_cu_2

    # Robot Bringup (ROS 2 Driver) + MoveIt!2 Framework:
    ros2 launch ros2srrc_launch bringup_ur.launch.py package:=r3mcell_cu config:=r3mcell_cu_2 robot_ip:=0.0.0.0

    ```

- r3mcell_cu_21: Cube Stacking Use-Case.

    ```sh

    # Gazebo Simulation:
    ros2 launch ros2srrc_launch simulation.launch.py package:=r3mcell_cu config:=r3mcell_cu_21

    # Gazebo Simulation + MoveIt!2 Framework:
    ros2 launch ros2srrc_launch moveit2.launch.py package:=r3mcell_cu config:=r3mcell_cu_21

    # Robot Bringup (ROS 2 Driver) + MoveIt!2 Framework:
    ros2 launch ros2srrc_launch bringup_ur.launch.py package:=r3mcell_cu config:=r3mcell_cu_21 robot_ip:=0.0.0.0

    ```

- r3mcell_cu_22: Coloured Cube Kitting Use-Case.

    ```sh

    # Gazebo Simulation:
    ros2 launch ros2srrc_launch simulation.launch.py package:=r3mcell_cu config:=r3mcell_cu_22

    # Gazebo Simulation + MoveIt!2 Framework:
    ros2 launch ros2srrc_launch moveit2.launch.py package:=r3mcell_cu config:=r3mcell_cu_22

    # Robot Bringup (ROS 2 Driver) + MoveIt!2 Framework:
    ros2 launch ros2srrc_launch bringup_ur.launch.py package:=r3mcell_cu config:=r3mcell_cu_22 robot_ip:=0.0.0.0

    ```

- r3mcell_cu_23: Cylinder Stacking Use-Case.

    ```sh

    # Gazebo Simulation:
    ros2 launch ros2srrc_launch simulation.launch.py package:=r3mcell_cu config:=r3mcell_cu_23

    # Gazebo Simulation + MoveIt!2 Framework:
    ros2 launch ros2srrc_launch moveit2.launch.py package:=r3mcell_cu config:=r3mcell_cu_23

    # Robot Bringup (ROS 2 Driver) + MoveIt!2 Framework:
    ros2 launch ros2srrc_launch bringup_ur.launch.py package:=r3mcell_cu config:=r3mcell_cu_23 robot_ip:=0.0.0.0

    ```