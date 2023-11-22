#!/usr/bin/python3

# # # # # # # # # # # # # # # # # #                                  
#                                 #
#   ===== COPYRIGHT HERE =====    #
#                                 #
# # # # # # # # # # # # # # # # # #

# ========================================================================================= #
# INCLUDE:

# ROS2:
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import xacro
import os
import ast
import time

# ROS2 MSG:
from r3mcell_data.msg import Skillresult

# ROS2 Service:
from r3mcell_data.srv import SkillExecution   

# ROS2 Action:
from rclpy.action import ActionClient
from r3mcell_data.action import ExecuteSkill

# Std_msgs:
from std_msgs.msg import String
from std_msgs.msg import Int32 
# Std_srvs:
from std_srvs.srv import Empty
# IMPORT /SpawnEntity and /DeleteEntity ROS2 Services:
from gazebo_msgs.srv import SpawnEntity
from gazebo_msgs.srv import DeleteEntity
# IMPORT /controller_manager ROS2 Services:
from controller_manager_msgs.srv import LoadController
from controller_manager_msgs.srv import ConfigureController
from controller_manager_msgs.srv import SwitchController

# GLOBAL VARIABLES -> SkillResult, RES:
SkillResult = Skillresult()
SkillResult.message = "none"

# ========================================================================================= #
# ServiceClient (SPAWN/DELETE ROBOT + CUBE):

class EntityClient(Node):

    def __init__(self):

        # Initialise ROS2 Node:
        super().__init__('r3m_MATLAB_EntityClient')

        # Create ROS2 Service Clients:
        self.cli_SPAWN = self.create_client(SpawnEntity, "/spawn_entity")  
        self.cli_DELETE = self.create_client(DeleteEntity, "/delete_entity") 

        # Declare REQUEST variable (of CUSTOM DATA type):
        self.req_SPAWN = SpawnEntity.Request()  
        self.req_DELETE = DeleteEntity.Request()

    def spawn_REQUEST(self, ELEMENT):
        
        ## 1. SPAWN ROBOT:

        if ELEMENT == "ROBOT":
        
            # LOAD URDF of ROBOT:
            urdf_file_path = os.path.join(get_package_share_directory('r3mcell_gazebo'), 'urdf', 'irb120.urdf.xacro')
            xacro_file = xacro.process_file(urdf_file_path, mappings={"name": "irb120"})
            
            # ARGUMENTS:
            self.req_SPAWN.name = "irb120"
            self.req_SPAWN.xml = xacro_file.toxml()
            self.req_SPAWN.initial_pose.position.x = 0.0
            self.req_SPAWN.initial_pose.position.y = 0.0
            self.req_SPAWN.initial_pose.position.z = 0.0

            # Assign RESULT value (future):
            self.future_SPAWN = self.cli_SPAWN.call_async(self.req_SPAWN)

        ## 2. SPAWN CUBE:

        elif ELEMENT == "CUBE":

            # LOAD URDF of CUBE:
            urdf_file_path = os.path.join(get_package_share_directory('r3mcell_gazebo'), 'urdf', 'box.urdf')
            xacro_file = xacro.process_file(urdf_file_path, mappings={"name": "box"})
            
            # ARGUMENTS:
            self.req_SPAWN.name = "box"
            self.req_SPAWN.xml = xacro_file.toxml()
            self.req_SPAWN.initial_pose.position.x = -0.45
            self.req_SPAWN.initial_pose.position.y = 0.85
            self.req_SPAWN.initial_pose.position.z = 0.88

            # Assign RESULT value (future):
            self.future_SPAWN = self.cli_SPAWN.call_async(self.req_SPAWN)

    def delete_REQUEST(self, ELEMENT):

        ## 1. DELETE ROBOT:
        if ELEMENT == "ROBOT":
            self.req_DELETE.name = "irb120"
            self.future_DELETE = self.cli_DELETE.call_async(self.req_DELETE)
        
        ## 2. DELETE CUBE:
        elif ELEMENT == "CUBE":
            self.req_DELETE.name = "box"
            self.future_DELETE = self.cli_DELETE.call_async(self.req_DELETE)

# ========================================================================================= #
# ServiceClient (CONTROLLER MANAGER):

class ControllerClient(Node):

    def __init__(self):

        # Initialise ROS2 Node:
        super().__init__('r3m_MATLAB_ControllerClient')

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
        super().__init__('r3m_MATLAB_GazeboClient')

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
# SERVICE SERVER (/SkillExecution):

class serviceServer(Node):

    def __init__(self):

        # INITIALISE Service:
        super().__init__('r3m_MATLAB_SkillExecution_ServiceServer')                                              
        self.srv = self.create_service(SkillExecution, "/r3m_SkillExecution", self.service_Callback)       

        # INITIALISE:
        self.SKILL_CLIENT = SkillClient()
        self.ENTITY_CLIENT = EntityClient()
        self.CONTROLLER_CLIENT = ControllerClient()
        self.GAZEBO_CLIENT = GazeboClient()

        ## INIT: Spawn cube!
        self.ENTITY_CLIENT.spawn_REQUEST("CUBE")
        while rclpy.ok():
            rclpy.spin_once(self.ENTITY_CLIENT)
            if self.ENTITY_CLIENT.future_SPAWN.done():
                try:
                    spawnRES = self.ENTITY_CLIENT.future_SPAWN.result()
                except Exception as exc:
                    print("/SpawnEntity ROS2 Service call failed. ERROR: " + str(exc))
                else:
                    print("RESULT: " + str(spawnRES.status_message))
                break

    def service_Callback(self, request, response):
        
        global SkillResult
        
        # Get RECIPE ID:
        ID = request.id

        # CALL ROS2 ACTION:
        if (ID != 0):
        
            self.SKILL_CLIENT.send_goal(ID)
            while rclpy.ok():
                rclpy.spin_once(self.SKILL_CLIENT)
                if (SkillResult.message != "none"):
                    break

            response.result = SkillResult
            SkillResult.message = "none"
            return(response)

        if (ID == 0):
            
            # === RESET Gz Environment === #

            # 1. CUBE MUST BE DETACHED -> Otherwise Gazebo breaks!
            self.SKILL_CLIENT.send_goal(7)
            while rclpy.ok():
                rclpy.spin_once(self.SKILL_CLIENT)
                if (SkillResult.message != "none"):
                    break

            SkillResult.message = "none"
            
            # DELETE ROBOT and CUBE:
            self.ENTITY_CLIENT.delete_REQUEST("CUBE")
            while rclpy.ok():
                rclpy.spin_once(self.ENTITY_CLIENT)
                if self.ENTITY_CLIENT.future_DELETE.done():
                    try:
                        deleteRES = self.ENTITY_CLIENT.future_DELETE.result()
                    except Exception as exc:
                        print("/DeleteEntity ROS2 Service call failed. ERROR: " + str(exc))
                    else:
                        print("RESULT: " + str(deleteRES.status_message))
                    break
            self.ENTITY_CLIENT.delete_REQUEST("ROBOT")
            while rclpy.ok():
                rclpy.spin_once(self.ENTITY_CLIENT)
                if self.ENTITY_CLIENT.future_DELETE.done():
                    try:
                        deleteRES = self.ENTITY_CLIENT.future_DELETE.result()
                    except Exception as exc:
                        print("/DeleteEntity ROS2 Service call failed. ERROR: " + str(exc))
                    else:
                        print("RESULT: " + str(deleteRES.status_message))
                    break

            # RESET WORLD AND SIMULATION:
            self.GAZEBO_CLIENT.resetWORLD_REQUEST()
            while rclpy.ok():
                rclpy.spin_once(self.GAZEBO_CLIENT)
                if self.GAZEBO_CLIENT.future_RESETWorld.done():
                    try:
                        resetRES = self.GAZEBO_CLIENT.future_RESETWorld.result()
                    except Exception as exc:
                        print("/reset_world ROS2 Service call failed. ERROR: " + str(exc))
                    else:
                        print("RESULT: RESET COMPLETE.")
                    break
            self.GAZEBO_CLIENT.resetSIM_REQUEST()
            while rclpy.ok():
                rclpy.spin_once(self.GAZEBO_CLIENT)
                if self.GAZEBO_CLIENT.future_RESETSim.done():
                    try:
                        resetRES = self.GAZEBO_CLIENT.future_RESETSim.result()
                    except Exception as exc:
                        print("/reset_simulation ROS2 Service call failed. ERROR: " + str(exc))
                    else:
                        print("RESULT: RESET COMPLETE.")
                    break

            # SPAWN ROBOT and CUBE:
            self.ENTITY_CLIENT.spawn_REQUEST("ROBOT")
            while rclpy.ok():
                rclpy.spin_once(self.ENTITY_CLIENT)
                if self.ENTITY_CLIENT.future_SPAWN.done():
                    try:
                        spawnRES = self.ENTITY_CLIENT.future_SPAWN.result()
                    except Exception as exc:
                        print("/SpawnEntity ROS2 Service call failed. ERROR: " + str(exc))
                    else:
                        print("RESULT: " + str(spawnRES.status_message))
                    break
            self.ENTITY_CLIENT.spawn_REQUEST("CUBE")
            while rclpy.ok():
                rclpy.spin_once(self.ENTITY_CLIENT)
                if self.ENTITY_CLIENT.future_SPAWN.done():
                    try:
                        spawnRES = self.ENTITY_CLIENT.future_SPAWN.result()
                    except Exception as exc:
                        print("/SpawnEntity ROS2 Service call failed. ERROR: " + str(exc))
                    else:
                        print("RESULT: " + str(spawnRES.status_message))
                    break

            # LOAD CONTROLLERS:
            self.CONTROLLER_CLIENT.load_REQUEST("joint_state_broadcaster")
            while rclpy.ok():
                rclpy.spin_once(self.CONTROLLER_CLIENT)
                if self.CONTROLLER_CLIENT.future_LOAD.done():
                    try:
                        loadRES = self.CONTROLLER_CLIENT.future_LOAD.result()
                    except Exception as exc:
                        print("Load Controller ROS2 Service call failed. ERROR: " + str(exc))
                    else:
                        print("RESULT: " + str(loadRES.ok))
                    break
            self.CONTROLLER_CLIENT.load_REQUEST("irb120_controller")
            while rclpy.ok():
                rclpy.spin_once(self.CONTROLLER_CLIENT)
                if self.CONTROLLER_CLIENT.future_LOAD.done():
                    try:
                        loadRES = self.CONTROLLER_CLIENT.future_LOAD.result()
                    except Exception as exc:
                        print("Load Controller ROS2 Service call failed. ERROR: " + str(exc))
                    else:
                        print("RESULT: " + str(loadRES.ok))
                    break
            self.CONTROLLER_CLIENT.load_REQUEST("egp64_finger_right_controller")
            while rclpy.ok():
                rclpy.spin_once(self.CONTROLLER_CLIENT)
                if self.CONTROLLER_CLIENT.future_LOAD.done():
                    try:
                        loadRES = self.CONTROLLER_CLIENT.future_LOAD.result()
                    except Exception as exc:
                        print("Load Controller ROS2 Service call failed. ERROR: " + str(exc))
                    else:
                        print("RESULT: " + str(loadRES.ok))
                    break
            self.CONTROLLER_CLIENT.load_REQUEST("egp64_finger_left_controller")
            while rclpy.ok():
                rclpy.spin_once(self.CONTROLLER_CLIENT)
                if self.CONTROLLER_CLIENT.future_LOAD.done():
                    try:
                        loadRES = self.CONTROLLER_CLIENT.future_LOAD.result()
                    except Exception as exc:
                        print("Load Controller ROS2 Service call failed. ERROR: " + str(exc))
                    else:
                        print("RESULT: " + str(loadRES.ok))
                    break   
            # CONFIGURE CONTROLLERS:
            self.CONTROLLER_CLIENT.configure_REQUEST("joint_state_broadcaster")
            while rclpy.ok():
                rclpy.spin_once(self.CONTROLLER_CLIENT)
                if self.CONTROLLER_CLIENT.future_CONFIGURE.done():
                    try:
                        configureRES = self.CONTROLLER_CLIENT.future_CONFIGURE.result()
                    except Exception as exc:
                        print("Configure Controller ROS2 Service call failed. ERROR: " + str(exc))
                    else:
                        print("RESULT: " + str(configureRES.ok))
                    break
            self.CONTROLLER_CLIENT.configure_REQUEST("irb120_controller")
            while rclpy.ok():
                rclpy.spin_once(self.CONTROLLER_CLIENT)
                if self.CONTROLLER_CLIENT.future_CONFIGURE.done():
                    try:
                        configureRES = self.CONTROLLER_CLIENT.future_CONFIGURE.result()
                    except Exception as exc:
                        print("Configure Controller ROS2 Service call failed. ERROR: " + str(exc))
                    else:
                        print("RESULT: " + str(configureRES.ok))
                    break   
            self.CONTROLLER_CLIENT.configure_REQUEST("egp64_finger_right_controller") 
            while rclpy.ok():
                rclpy.spin_once(self.CONTROLLER_CLIENT)
                if self.CONTROLLER_CLIENT.future_CONFIGURE.done():
                    try:
                        configureRES = self.CONTROLLER_CLIENT.future_CONFIGURE.result()
                    except Exception as exc:
                        print("Configure Controller ROS2 Service call failed. ERROR: " + str(exc))
                    else:
                        print("RESULT: " + str(configureRES.ok))
                    break 
            self.CONTROLLER_CLIENT.configure_REQUEST("egp64_finger_left_controller")   
            while rclpy.ok():
                rclpy.spin_once(self.CONTROLLER_CLIENT)
                if self.CONTROLLER_CLIENT.future_CONFIGURE.done():
                    try:
                        configureRES = self.CONTROLLER_CLIENT.future_CONFIGURE.result()
                    except Exception as exc:
                        print("Configure Controller ROS2 Service call failed. ERROR: " + str(exc))
                    else:
                        print("RESULT: " + str(configureRES.ok))
                    break
            # ACTIVATE CONTROLLERS: 
            ControllerList = ["joint_state_broadcaster", "irb120_controller", "egp64_finger_right_controller", "egp64_finger_left_controller"] 
            self.CONTROLLER_CLIENT.switch_REQUEST(ControllerList)   
            while rclpy.ok():
                rclpy.spin_once(self.CONTROLLER_CLIENT)
                if self.CONTROLLER_CLIENT.future_SWITCH.done():
                    try:
                        switchRES = self.CONTROLLER_CLIENT.future_SWITCH.result()
                    except Exception as exc:
                        print("Switch Controller ROS2 Service call failed. ERROR: " + str(exc))
                    else:
                        print("RESULT: " + str(switchRES.ok))
                    break   

            RESULT = Skillresult()
            RESULT.id = 0
            RESULT.message = "R3M CELL Gazebo environment reset successful."
            RESULT.success = True

            response.result = RESULT
            return(response)

# =================== MAIN =================== #
def main(args=None):
    print ("R3M - CRANFIELD UNIVERSITY")
    print ("ROS2.0 TEMPLATES - Service Server")
    print ("")
    
    # Initialise NODE:
    rclpy.init(args=args)
    r3mNode = serviceServer()
    print ("r3m_serviceSERVER ROS2 node generated.")
    print ("")

    # Spin SERVICE -> The Service Server will execute the service_Callback every single time the service is called.
    rclpy.spin(r3mNode)                                                                             # Spin SERVICE SERVER.

    r3mNode.destroy_node
    rclpy.shutdown()

if __name__ == '__main__':
    main()