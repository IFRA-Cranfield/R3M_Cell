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
import time
import yaml

# ROS2:
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

# ROS2 MSG/SRV/ACTION:
from geometry_msgs.msg import Pose

# CUSTOM ROS2 MSG/SRV/ACTION:
from r3mcell_data.srv import SkillExecution
from r3mcell_data.msg import Product
from r3mcell_data.msg import Pose

# Import CLASSES/Functions:
from Robot import RobotClient
from Gripper_Gz import ParallelGripper
from Gripper_Gz import VacuumGripper
from ObjectState import OBJECT
from ResetGazebo import GzRESET

# ========================================================================================= #
# ================================ ROS2 - INPUT PARAMETERS ================================ #
# ========================================================================================= #

# ========================================================================================= #
# Get AUTO CLASS:
PARAM_AUTO = "default"
P_CHECK_AUTO = False

class getAUTO(Node):

    def __init__(self):
        
        global PARAM_AUTO
        global P_CHECK_AUTO
        
        super().__init__('r3mcell_AUTO_PARAM')
        self.declare_parameter('AUTO', "default")
        PARAM_AUTO = self.get_parameter('AUTO').get_parameter_value().string_value
        if (PARAM_AUTO == "default"):
            self.get_logger().info('[R3M Cell] - AUTO ROS2 Parameter was not defined.')
            exit()
        else:    
            self.get_logger().info('[R3M Cell] - AUTO ROS2 Parameter received: ' + PARAM_AUTO)
        
        P_CHECK_AUTO = True

# Get InitialConditions CLASS:
PARAM_IC = "default"
P_CHECK_IC = False

class getIC(Node):

    def __init__(self):
        
        global PARAM_IC
        global P_CHECK_IC
        
        super().__init__('r3mcell_IC_PARAM')
        self.declare_parameter('InitialConditions', "default")
        PARAM_IC = self.get_parameter('InitialConditions').get_parameter_value().string_value
        if (PARAM_IC == "default"):
            self.get_logger().info('[R3M Cell] - InitialConditions ROS2 Parameter was not defined.')
            exit()
        else:    
            self.get_logger().info('[R3M Cell] - InitialConditions ROS2 Parameter received: ' + PARAM_IC)
        
        P_CHECK_IC = True

def GetIC_YAML(NAME):

    RESULT = {"ObjectList": None, "Robot": None, "ControllerList": None, "Success": True}
    
    PATH = os.path.join(get_package_share_directory('r3mcell_execution'), 'initialconditions')
    YAML_PATH = PATH + "/" + NAME + ".yaml"
    
    if not os.path.exists(YAML_PATH):
        RESULT["Success"] = False
        return (RESULT)

    # Get VALUES:
    with open(YAML_PATH, 'r') as YAML:
        RecipeYAML = yaml.safe_load(YAML)

    RESULT["ObjectList"] = RecipeYAML["ObjectList"]
    RESULT["Robot"] = RecipeYAML["Robot"]
    RESULT["ControllerList"] = RecipeYAML["ControllerList"]

    return(RESULT)

# ========================================================================================= #
# =================================== CLASSES/FUNCTIONS =================================== #
# ========================================================================================= #

# ========================================================================================= #
# ExecuteSkill_SERVER CLASS:
class ExecuteSkill_SERVER(Node):
    
    def __init__(self, OL, ROB, CL):
        
        # Initialise VARIABLES using the information from the INPUT PARAMETERS:
        self.ObjectList = OL                       # [{"Model": "", "Link": "", "Package":, "", "InitialPose": "", "CurrentPose": "", "PreviousPose": ""}, ...]
        self.RBT = ROB                             # {"Model": "", "Link": "", "InitialPose": ""}
    
        self.ResetCond = {}
        self.ResetCond["Robot"] = ROB
        self.ResetCond["ObjectList"] = OL
        self.ResetCond["ControllerList"] = CL      # ["", "", ...]

        # Initialise CLASSES needed for the Skill Execution:
        self.OBJECTS = OBJECT(self.ObjectList)
        self.ROBOT = RobotClient()
        
        if ROB["EEType"] == "ParallelGripper":
            self.GRIPPER = ParallelGripper(self.RBT)
        elif ROB["EEType"] == "VacuumGripper":
            self.GRIPPER = VacuumGripper(self.RBT)
        
        self.RESET = GzRESET(self.ResetCond)

        # Initialise SERVICE SERVER:
        super().__init__('r3mcell_SkillExecution_ServiceServer')                                              
        self.srv = self.create_service(SkillExecution, "/r3m_SkillExecution", self.EXECUTE)
    
    def EXECUTE(self, request, response):
        
        # Get RECIPE ID:
        ID = request.id

        # EXECUTE according to RECIPE NUMBER:
        if (ID == 999):

            response.result.id = 999
            response.result.message = "999 received from Simulink -> No execution."
            response.result.success = True
            return(response)

        elif (ID == 0):

            if self.RBT["EEType"] == "ParallelGripper":
                self.GRIPPER.Execute(None, None, "OPEN", 1.0)
            elif self.RBT["EEType"] == "VacuumGripper":
                self.GRIPPER.Execute(None, None, "VacuumOFF")

            RES = self.RESET.RESET()
            self.OBJECTS.ResetObjectList()
            response.result.id = 0

            if RES == True:
                response.result.message = "ROS2 Environment RESET successful."
                response.result.success = True
                return(response)
            else:
                response.result.message = "ROS2 Environment RESET failed."
                response.result.success = False
                return(response)
            
        else:

            # Get RECIPE VALUES:
            RECIPE = GetRecipe(ID)

            if RECIPE["Exists"] == True:

                # Check MOVEMENT TYPE and EXECUTE ACCORDINGLY:
                
                # ROBOT:
                if (RECIPE["type"] == "PTP" or RECIPE["type"] == "LIN"):

                    RES = self.ROBOT.Execute(RECIPE["type"], RECIPE["speed"], RECIPE["pose"])

                    response.result.id = ID
                    response.result.message = RES["Message"]
                    response.result.success = RES["Success"]
                    response.result.exectime = RES["ExecTime"]
                    response.result.error = RES["Error"]
                
                # ParallelGripper:
                elif (RECIPE["type"] == "GRIP"):

                    RES = self.GRIPPER.Execute(self.RBT, self.ObjectList, RECIPE["action"], RECIPE["speed"])

                    response.result.id = ID
                    response.result.message = RES["Message"]
                    response.result.success = RES["Success"]
                    response.result.exectime = RES["ExecTime"]
                    response.result.error = RES["Error"]
                
                # VacuumGripper:
                elif (RECIPE["type"] == "VACUUM"):
                    
                    RES = self.GRIPPER.Execute(self.RBT, self.ObjectList, RECIPE["action"])

                    response.result.id = ID
                    response.result.message = RES["Message"]
                    response.result.success = RES["Success"]
                    response.result.exectime = RES["ExecTime"]
                    response.result.error = RES["Error"]

                # GET ObjectList -> OBJECT POSES:
                OL = self.OBJECTS.GetObjectPose()
                PRODUCTS = []
                P = Product()

                for x in OL:

                    P.name = x["Name"]

                    P.currentpose = Pose()
                    P.currentpose.x = x["CurrentPose"].x
                    P.currentpose.y = x["CurrentPose"].y
                    P.currentpose.z = x["CurrentPose"].z
                    P.currentpose.qx = x["CurrentPose"].qx
                    P.currentpose.qy = x["CurrentPose"].qy
                    P.currentpose.qz = x["CurrentPose"].qz
                    P.currentpose.qw = x["CurrentPose"].qw

                    P.previouspose = Pose()
                    P.previouspose.x = x["PreviousPose"].x
                    P.previouspose.y = x["PreviousPose"].y
                    P.previouspose.z = x["PreviousPose"].z
                    P.previouspose.qx = x["PreviousPose"].qx
                    P.previouspose.qy = x["PreviousPose"].qy
                    P.previouspose.qz = x["PreviousPose"].qz
                    P.previouspose.qw = x["PreviousPose"].qw

                    P.error = 0.0 # Error when retrieving from Gazebo is null.

                    PRODUCTS.append(P)

                response.result.product = PRODUCTS

                return(response)

            else:

                response.result.id = ID
                response.result.message = "ERROR. Recipe N -> " + str(ID) + " does not exist."
                response.result.success = False
                return(response)
        
# ========================================================================================= #
# GetRecipe FUNCTION:
def GetRecipe(RECIPE_ID):
    
    RECIPE = {"Exists": True}
    
    PATH = os.path.join(get_package_share_directory('r3mcell_execution'), 'recipes')
    RECIPE_PATH = PATH + "/" + str(RECIPE_ID) + ".yaml"
    
    if not os.path.exists(RECIPE_PATH):
        RECIPE["Exists"] = False
        return (RECIPE)

    # Get RECIPE VALUES:
    with open(RECIPE_PATH, 'r') as YAML:
        RecipeYAML = yaml.safe_load(YAML)
      
    RECIPE["id"] = RecipeYAML["id"] 
    RECIPE["type"] = RecipeYAML["type"]
    RECIPE["speed"] = RecipeYAML["speed"]
      
    if (RECIPE["type"] == "PTP" or RECIPE["type"] == "LIN"):
        
        POSE = Pose()
        POSE.x = RecipeYAML["pose"]["x"]
        POSE.y = RecipeYAML["pose"]["y"]
        POSE.z = RecipeYAML["pose"]["z"]
        POSE.qx = RecipeYAML["pose"]["qx"]
        POSE.qy = RecipeYAML["pose"]["qy"]
        POSE.qz = RecipeYAML["pose"]["qz"]
        POSE.qw = RecipeYAML["pose"]["qw"]
        
        RECIPE["pose"] = POSE
    
    elif RECIPE["type"] == "GRIP":
        
        RECIPE["action"] = RecipeYAML["action"]

    elif RECIPE["type"] == "VACUUM":
        
        RECIPE["action"] = RecipeYAML["action"]
        
    return(RECIPE)

# ========================================================================================= #
# ========================================= MAIN ========================================== #
# ========================================================================================= #

def main(args=None):
    
    rclpy.init(args=args)

    # Check if NODE is going to be EXECUTED:
    global PARAM_AUTO
    global P_CHECK_AUTO
    paramAUTO = getAUTO()
    while (P_CHECK_AUTO == False):
        rclpy.spin_once(paramAUTO)

    if PARAM_AUTO == "False":
        paramAUTO.get_logger().info("[R3M Cell] - R3M AutomaticOperation is not required for this simulation. Shutting down node!")
        exit()
    
    paramAUTO.destroy_node()

    # === INITIAL CONDITIONS === #
    # Get ROS2 Parameter value:
    global PARAM_IC
    global P_CHECK_IC
    paramIC = getIC()
    while (P_CHECK_IC == False):
        rclpy.spin_once(paramIC)
    paramIC.destroy_node()
    # Get InitialConditions from yaml file:
    IC = GetIC_YAML(PARAM_IC)

    # Initialise NODE:
    r3mNode = ExecuteSkill_SERVER(IC["ObjectList"], IC["Robot"], IC["ControllerList"])
    r3mNode.get_logger().info("[R3M Cell] - /ExecuteSkill ROS2 Service Server running, ROS2 node generated.")

    rclpy.spin(r3mNode)                                                                     

    r3mNode.destroy_node
    rclpy.shutdown()

if __name__ == '__main__':
    main()

            
