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
import xacro

# ROS2:
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

# ROS2 MSG/SRV/ACTION:
from std_srvs.srv import Empty
from gazebo_msgs.srv import SpawnEntity
from gazebo_msgs.srv import DeleteEntity
from controller_manager_msgs.srv import LoadController
from controller_manager_msgs.srv import ConfigureController
from controller_manager_msgs.srv import SwitchController

# ========================================================================================= #
# =================================== CLASSES/FUNCTIONS =================================== #
# ========================================================================================= #

# ========================================================================================= #
# CLASS -> RESET GAZEBO:
class GzRESET():

    def __init__(self,  ResetCondition):

        # Initialise ROS2 Clients:
        self.ENTITY_CLIENT = EntityClient()
        self.CONTROLLER_CLIENT = ControllerClient()
        self.GAZEBO_CLIENT = GazeboClient()

        self.ResetCond = ResetCondition

        # InitialCondition: dict() with:
        # "Robot": {"Package": "", "Model": "", "Pose": ""}
        # "ObjectList": [{"Package": "", "Model": "", "Name": "", "Pose": ""}, ...]
        # "ControllerList": ["", ...]

        # Spawn OBJECTS IN INITIAL CONDITIONS:
        for x in self.ResetCond["ObjectList"]:

            self.ENTITY_CLIENT.spawn_REQUEST("OBJECT", x)
            while rclpy.ok():
                rclpy.spin_once(self.ENTITY_CLIENT)
                if self.ENTITY_CLIENT.future_SPAWN.done():
                    try:
                        spawnRES = self.ENTITY_CLIENT.future_SPAWN.result()
                    except Exception as exc:
                        print("/SpawnEntity ROS2 Service call failed. ERROR: " + str(exc))
                        return(False)
                    else:
                        print("RESULT: " + str(spawnRES.status_message))
                    break

    def RESET(self):

        # 1. Delete any object that could be in the workspace:
        for x in self.ResetCond["ObjectList"]:

            self.ENTITY_CLIENT.delete_REQUEST("OBJECT", x)
            while rclpy.ok():
                rclpy.spin_once(self.ENTITY_CLIENT)
                if self.ENTITY_CLIENT.future_DELETE.done():
                    try:
                        deleteRES = self.ENTITY_CLIENT.future_DELETE.result()
                    except Exception as exc:
                        print("/DeleteEntity ROS2 Service call failed. ERROR: " + str(exc))
                        return(False)
                    else:
                        print("RESULT: " + str(deleteRES.status_message))
                    break

        # 2. Delete ROBOT:
        self.ENTITY_CLIENT.delete_REQUEST("ROBOT", self.ResetCond["Robot"])
        while rclpy.ok():
            rclpy.spin_once(self.ENTITY_CLIENT)
            if self.ENTITY_CLIENT.future_DELETE.done():
                try:
                    deleteRES = self.ENTITY_CLIENT.future_DELETE.result()
                except Exception as exc:
                    print("/DeleteEntity ROS2 Service call failed. ERROR: " + str(exc))
                    return(False)
                else:
                    print("RESULT: " + str(deleteRES.status_message))
                break

        # 3. Reset WORLD and SIMULATION:
        self.GAZEBO_CLIENT.resetWORLD_REQUEST()
        while rclpy.ok():
            rclpy.spin_once(self.GAZEBO_CLIENT)
            if self.GAZEBO_CLIENT.future_RESETWorld.done():
                try:
                    resetRES = self.GAZEBO_CLIENT.future_RESETWorld.result()
                except Exception as exc:
                    print("/reset_world ROS2 Service call failed. ERROR: " + str(exc))
                    return(False)
                else:
                    print("RESULT: GzWorld reset complete.")
                break
        self.GAZEBO_CLIENT.resetSIM_REQUEST()
        while rclpy.ok():
            rclpy.spin_once(self.GAZEBO_CLIENT)
            if self.GAZEBO_CLIENT.future_RESETSim.done():
                try:
                    resetRES = self.GAZEBO_CLIENT.future_RESETSim.result()
                except Exception as exc:
                    print("/reset_simulation ROS2 Service call failed. ERROR: " + str(exc))
                    return(False)
                else:
                    print("RESULT: GzClient reset complete.")
                break

        # 4. Spawn ROBOT:
        self.ENTITY_CLIENT.spawn_REQUEST("ROBOT", self.ResetCond["Robot"])
        while rclpy.ok():
            rclpy.spin_once(self.ENTITY_CLIENT)
            if self.ENTITY_CLIENT.future_SPAWN.done():
                try:
                    spawnRES = self.ENTITY_CLIENT.future_SPAWN.result()
                except Exception as exc:
                    print("/SpawnEntity ROS2 Service call failed. ERROR: " + str(exc))
                    return(False)
                else:
                    print("RESULT: " + str(spawnRES.status_message))
                break
        
        # 5. Spawn OBJECTS:
        for x in self.ResetCond["ObjectList"]:

            self.ENTITY_CLIENT.spawn_REQUEST("OBJECT", x)
            while rclpy.ok():
                rclpy.spin_once(self.ENTITY_CLIENT)
                if self.ENTITY_CLIENT.future_SPAWN.done():
                    try:
                        spawnRES = self.ENTITY_CLIENT.future_SPAWN.result()
                    except Exception as exc:
                        print("/SpawnEntity ROS2 Service call failed. ERROR: " + str(exc))
                        return(False)
                    else:
                        print("RESULT: " + str(spawnRES.status_message))
                    break
        
        # 6. Load/Configure/Switch CONTROLLERS:
        for x in self.ResetCond["ControllerList"]:

            self.CONTROLLER_CLIENT.load_REQUEST(x)
            while rclpy.ok():
                rclpy.spin_once(self.CONTROLLER_CLIENT)
                if self.CONTROLLER_CLIENT.future_LOAD.done():
                    try:
                        loadRES = self.CONTROLLER_CLIENT.future_LOAD.result()
                    except Exception as exc:
                        print("Load Controller ROS2 Service call failed. ERROR: " + str(exc))
                        return(False)
                    else:
                        print("RESULT (Load Controller - " + x + "): " + str(loadRES.ok))
                    break
        
        for x in self.ResetCond["ControllerList"]:
            
            self.CONTROLLER_CLIENT.configure_REQUEST(x)
            while rclpy.ok():
                rclpy.spin_once(self.CONTROLLER_CLIENT)
                if self.CONTROLLER_CLIENT.future_CONFIGURE.done():
                    try:
                        configureRES = self.CONTROLLER_CLIENT.future_CONFIGURE.result()
                    except Exception as exc:
                        print("Configure Controller ROS2 Service call failed. ERROR: " + str(exc))
                        return(False)
                    else:
                        print("RESULT (Configure Controller - " + x + ") " + str(configureRES.ok))
                    break

        self.CONTROLLER_CLIENT.switch_REQUEST(self.ResetCond["ControllerList"])   
        while rclpy.ok():
            rclpy.spin_once(self.CONTROLLER_CLIENT)
            if self.CONTROLLER_CLIENT.future_SWITCH.done():
                try:
                    switchRES = self.CONTROLLER_CLIENT.future_SWITCH.result()
                except Exception as exc:
                    print("Switch Controller ROS2 Service call failed. ERROR: " + str(exc))
                    return(False)
                else:
                    print("RESULT (Switch Controllers): " + str(switchRES.ok))
                break   
    
        return(True)


# ========================================================================================= #
# ServiceClient (SPAWN/DELETE ROBOT + OBJECT):

class EntityClient(Node):

    def __init__(self):

        # Initialise ROS2 Node:
        super().__init__('r3mcell_EntityClient')

        # Create ROS2 Service Clients:
        self.cli_SPAWN = self.create_client(SpawnEntity, "/spawn_entity")  
        self.cli_DELETE = self.create_client(DeleteEntity, "/delete_entity") 

        # Declare REQUEST variable (of CUSTOM DATA type):
        self.req_SPAWN = SpawnEntity.Request()  
        self.req_DELETE = DeleteEntity.Request()

    def spawn_REQUEST(self, ELEMENT, INFORMATION):
        
        # ELEMENT can be: "ROBOT"/ "OBJECT"
        # INFORMATION: {"Package": "", "Model": "", "Pose": ""}
        
        # 1. SPAWN ROBOT:

        if ELEMENT == "ROBOT":
        
            # LOAD URDF of ROBOT:
            urdf_file_path = os.path.join(get_package_share_directory(INFORMATION["Package"]), 'urdf', INFORMATION["Model"] + '.urdf.xacro')
            xacro_file = xacro.process_file(urdf_file_path, mappings={"name": INFORMATION["Model"]})
            
            # ARGUMENTS:
            self.req_SPAWN.name = INFORMATION["Model"]
            self.req_SPAWN.xml = xacro_file.toxml()
            self.req_SPAWN.initial_pose.position.x = INFORMATION["InitialPose"]["x"]
            self.req_SPAWN.initial_pose.position.y = INFORMATION["InitialPose"]["y"]
            self.req_SPAWN.initial_pose.position.z = INFORMATION["InitialPose"]["x"]

            # Assign RESULT value (future):
            self.future_SPAWN = self.cli_SPAWN.call_async(self.req_SPAWN)

        ## 2. SPAWN CUBE:

        elif ELEMENT == "OBJECT":

            # LOAD URDF of CUBE:
            urdf_file_path = os.path.join(get_package_share_directory(INFORMATION["Package"]), 'urdf', INFORMATION["Model"] + '.urdf')
            xacro_file = xacro.process_file(urdf_file_path, mappings={"name": INFORMATION["Name"]})
            
            # ARGUMENTS:
            self.req_SPAWN.name = INFORMATION["Name"]
            self.req_SPAWN.xml = xacro_file.toxml()
            self.req_SPAWN.initial_pose.position.x = INFORMATION["InitialPose"]["x"]
            self.req_SPAWN.initial_pose.position.y = INFORMATION["InitialPose"]["y"]
            self.req_SPAWN.initial_pose.position.z = INFORMATION["InitialPose"]["z"]

            # Assign RESULT value (future):
            self.future_SPAWN = self.cli_SPAWN.call_async(self.req_SPAWN)

    def delete_REQUEST(self, ELEMENT, INFORMATION):

        ## 1. DELETE ROBOT:
        if ELEMENT == "ROBOT":
            self.req_DELETE.name = INFORMATION["Model"]
            self.future_DELETE = self.cli_DELETE.call_async(self.req_DELETE)
        
        ## 2. DELETE OBJECT:
        elif ELEMENT == "OBJECT":
            self.req_DELETE.name = INFORMATION["Name"]
            self.future_DELETE = self.cli_DELETE.call_async(self.req_DELETE)

# ========================================================================================= #
# ServiceClient (CONTROLLER MANAGER):

class ControllerClient(Node):

    def __init__(self):

        # Initialise ROS2 Node:
        super().__init__('r3mcell_ControllerClient')

        # Create ROS2 Service Clients:
        self.cli_LOAD = self.create_client(LoadController, "/controller_manager/load_controller")  
        self.cli_CONFIGURE = self.create_client(ConfigureController, "/controller_manager/configure_controller")
        self.cli_SWITCH = self.create_client(SwitchController, "/controller_manager/switch_controller") 

        # Declare REQUEST variable (of CUSTOM DATA type):
        self.req_LOAD = LoadController.Request()  
        self.req_CONFIGURE = ConfigureController.Request()
        self.req_SWITCH = SwitchController.Request()

    def load_REQUEST(self, CONTROLLER):
        
        self.req_LOAD.name = CONTROLLER
        self.future_LOAD = self.cli_LOAD.call_async(self.req_LOAD)

    def configure_REQUEST(self, CONTROLLER):
        
        self.req_CONFIGURE.name = CONTROLLER
        self.future_CONFIGURE = self.cli_CONFIGURE.call_async(self.req_CONFIGURE)

    # All controllers can be activated at once:
    def switch_REQUEST(self, CONTROLLERS):
        
        self.req_SWITCH.activate_controllers = CONTROLLERS
        self.future_SWITCH = self.cli_SWITCH.call_async(self.req_SWITCH)

# ========================================================================================= #
# ServiceClient (Reset Gazebo):

class GazeboClient(Node):

    def __init__(self):

        # Initialise ROS2 Node:
        super().__init__('r3mcell_GazeboClient')

        # Create ROS2 Service Clients:
        self.cli_RESETWorld = self.create_client(Empty, "/reset_world")  
        self.cli_RESETSim = self.create_client(Empty, "/reset_simulation")

        # Declare REQUEST variable (of CUSTOM DATA type):
        self.req_RESETWorld = Empty.Request()  
        self.req_RESETSim = Empty.Request()

    def resetWORLD_REQUEST(self):
        
        self.future_RESETWorld = self.cli_RESETWorld.call_async(self.req_RESETWorld)

    def resetSIM_REQUEST(self):

        self.future_RESETSim = self.cli_RESETSim.call_async(self.req_RESETSim)