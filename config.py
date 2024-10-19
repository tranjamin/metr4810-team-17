'''
Processes and parses the config files
'''

import json

from localisation import *
from controllers import *
from planning import *
from robot import *
from fmi import *

class Config():
    '''
    A class to hold all methods related to loading in configuration files.

    Config files must be structured in the standard JSON format with the following fields:
        - controllers: a superobject with the following fields
            - forward-controller: an object with the following fields
                - k_angle (float): the angle gain of the forward path
                - k_v (float): the velocity gain of the forward path
                - w (float): the line parameter of the forward path
                - goal_tolerance (float): the linear margin of error to classify a waypoint as reached [mm]
                - reversing_allowed (bool): whether the robot is permitted to reverse
            - spin-controller: an object with the following fields
                - k_angle (float): the angle gain of the spin path
                - k_v (float): the velocity gain of the spin path
                - angle_tolerance (float): the angular margin of error to classify a waypoint as reached [rad]
        - localisation: an object with the following fields
            - localisation-class (str): the name of the localisation class to instantiate
            - args (dict): a dictionary of arguments the localisation class' init accepts
        - robot: an object with the following fields
            - robot-class (str): the name of the robot class to instantiate
            - args (dict): a dictionary of arguments the robot class' init accepts
            - padding (float): the robot padding to consider for the path planner
            - emergency-padding (float): the robot padding to consider when moving to high ground
        - pathplan: an object with the following fields
            - reference-class (str): the name of the pathplan's waypoint class to instantiate
            - args (dict): a dictionary of arguments the reference class' init accepts
            - aim-enabled (bool): allows aim assist for the plan
        - debogger: an object with the following fields
            - enabled (bool): whether to enable the debogger or not
            - args (dict): an optional dictionary of arguments the debogger accepts
        - extraction: an object with the following fields
            - reference-class (str): the name of the extraction strategy class to instantiate
            - args (dict): a dictionary of arguments the reference class' init accepts

    Example Usage:
        cfg = Config(config_filename)
        
        localiser = cfg.load_localiser()
        localiser.setup()

        plan = cfg.load_pathplanning()
        robot_comms = cfg.load_robot()
        plan.set_robot(robot_comms)
    '''

    def __init__(self, filename: str):
        '''
        Parameters:
            filename (str): the filename and path of the config file
        '''
        self.config_file = json.load(open(filename))

    def load_localiser(self) -> Localisation:
        '''
        Load the localisation from the config file.

        Returns:
            (Localisation): the localisation object
        '''
        config_localiser: str = self.config_file["localisation"]
        config_localiser_args: dict = config_localiser.get("args", {})
        localiser: Localisation =  eval(config_localiser["localisation-class"])(**config_localiser_args)
        assert isinstance(localiser, Localisation)

        return localiser
    
    def load_pathplanning(self) -> Pathplanner:
        '''
        Load the pathplanner from the config file.

        Returns:
            (Pathplanner): the pathplanner object
        '''
        # load in pathplanner from config file
        plan = Pathplanner()

        # load in controllers from config file
        controller_config = self.config_file["controllers"]
        forward_controller = FowardController(**controller_config["forward-controller"])
        spin_controller = SpinController(**controller_config["spin-controller"])
        controller = LineFollowerController(forward_controller, spin_controller)
        plan.set_controller(controller)

        # load in extraction mode from config file
        extraction_mode: str = self.config_file["extraction"]["reference-class"]
        extraction_args: dict = self.config_file["extraction"].get("args", {})
        plan.set_extraction_strategy(eval(extraction_mode)(**extraction_args))
        
        # load in debogger mode from config file
        debogger_config = self.config_file.get("debogger")
        if debogger_config:
            if debogger_config.get("enabled"):
                debogger_args = debogger_config.get("args", {})
                plan.set_debogging_strategy(ActiveDebogger(**debogger_args))
            else:
                plan.set_debogging_strategy(NoDebogger())

        # load in waypoints from config file
        pathplanner_class: WaypointSequence = eval(self.config_file["pathplan"]["reference-class"])
        pathplanner_kwargs = self.config_file["pathplan"].get("args", {})
        plan.set_waypoints(pathplanner_class(**pathplanner_kwargs))

        if self.config_file["pathplan"]["aim-enabled"]:
            plan.waypoints.aim_enable()

        return plan

    def load_robot(self) -> Robot:
        '''
        Load the robot from the config file.

        Returns:
            (Robot): the robot.
        '''

        # get the robot class
        robot_config = self.config_file["robot"]
        robot_class = eval(robot_config["robot-class"])
        robot_comms: Robot = robot_class(**robot_config.get("args", {}))

        # set the robot geometry
        RobotGeometry.PADDING = robot_config.get("padding", RobotGeometry.PADDING)
        RobotGeometry.EMERGENCY_PADDING = robot_config.get("emergency-padding", RobotGeometry.EMERGENCY_PADDING)

        return robot_comms
