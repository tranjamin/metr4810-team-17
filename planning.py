from __future__ import annotations
import numpy as np
import math
from typing import *
from robot import Controller
from abc import ABC

class Pathplanner():
    '''
    A class to hold the pathplanning algorithm. 
    '''
    def __init__(self):
        self.waypoints: WaypointSequence
        self.controller: Controller

        # desired signals derived from the controller
        self.desired_velocity: float = 0
        self.desired_angular: float = 0

        # the current positions of the robot
        self.current_x: float = 0
        self.current_y: float = 0
        self.current_theta: float = 0

        # the current waypoint we are tracking
        self.current_waypoint: Waypoint = None

    def set_waypoints(self, waypoints: WaypointSequence):
        '''
        Set the waypoint sequence the planner will be following
        '''
        self.waypoints = waypoints
        self.current_waypoint = self.waypoints.get_current_waypoint()
    
    def set_controller(self, controller: Controller):
        '''
        Sets the controller used for the planning
        '''
        self.controller = controller
    
    def update_robot_position(self, current_x, current_y, current_theta):
        '''
        Update the planner's knowledge of the robot. Moves to the next waypoint if necessary
        '''
        self.current_x = current_x
        self.current_y = current_y
        self.current_theta = current_theta

        # if we have reached the current waypoint, move to the next one
        if self.current_waypoint is None:
            return
        if self.current_waypoint.is_reached(self.current_x, self.current_y, self.current_theta):
            self.current_waypoint = self.waypoints.move_to_next_waypoint()
    
    def controller_step(self):
        '''
        Use the controller to calcualte the desired movements
        '''
        # stop the robot if there are no more waypoints
        if self.current_waypoint is None:
            self.desired_velocity = 0
            self.desired_angular = 0
        # else, get the control actions
        else:
            self.controller.set_path((self.current_x, self.current_y), self.current_waypoint.coords, self.current_waypoint.heading)
            self.desired_velocity, self.desired_angular = self.controller.get_control_action(self.current_x, self.current_y, self.current_theta)

class RobotGeometry():
    '''
    A class to represent the physical bounds of the robot
    '''
    WIDTH: float = 400
    LENGTH: float = 200
    RADIUS: float = math.sqrt(WIDTH**2 + LENGTH**2)/2

    DIGGER_WIDTH: float = 200
    DIGGER_RADIUS: float = math.sqrt(DIGGER_WIDTH**2 + LENGTH**2)/2

class Waypoint():
    '''
    A coordinate and heading which the robot will move through. Coordinates are measured through the geometric centre of the robot.
    '''

    # tolerances for how close we need to get to the waypoints
    LINEAR_EPS = 0.01 # units are in #TODO??
    ANGULAR_EPS = 0.1 # units are in #TODO??

    def __init__(self, 
                 x: float, 
                 y: float, 
                 heading: Optional[float] = None, 
                 vel: Optional[float] = None,
                 suspendExtraction: bool = False,
                 resumeExtraction: bool = False, 
                 ):
        """
        Parameters
            x: the x-coordinate of the waypoint in millimetres
            y: the y-coordinate of the waypoint in millimetres
            heading: the angle the robot should reach the waypoint at, in degrees CW of positive y [-180 to 180] or None if the heading is unimportant
            vel: the target velocity the robot should reach the waypoint at, in metres per second, or None if unimportant
            suspendExtraction: stops extraction when reaching this waypoint
            resumeExtraction: starts extraction when reaching this waypoint
        """
        self.x = x
        self.y = y
        self.coords = (x, y)
        self.heading = heading
        self.vel = vel
        self.suspendFlag = suspendExtraction
        self.resumeFlag = resumeExtraction

        # TODO: make sure the units line up
        assert self.x < 2000 and self.x > 0
        assert self.y < 2000 and self.y > 0
        assert self.heading is None or (self.heading > -180 and self.heading < 180)
    
    def is_reached(self, current_x: float, current_y: float, current_heading: float) -> bool:
        '''
        determines whether the waypoint has been reached wrt the current position
        '''
        delta_x = abs(current_x - self.x)
        delta_y = abs(current_y - self.y)

        if math.sqrt(delta_x**2 + delta_y**2) > Waypoint.LINEAR_EPS:
            return False
        if self.heading is None:
            return True
        else:  
            delta_theta = abs(current_heading - self.heading) # ??
            return delta_theta < Waypoint.ANGULAR_EPS

class DepositWaypoint(Waypoint):
    '''
    The specific waypoint of being next to the deposit chamber.
    '''

    # parameters of waypoint
    DEPOSIT_X: float = 1600
    DEPOSIT_Y: float = 1900
    DEPOSIT_HEADING: float = 0

    def __init__(self):
        super().__init__(
            DepositWaypoint.DEPOSIT_X,
            DepositWaypoint.DEPOSIT_Y,
            DepositWaypoint.DEPOSIT_HEADING,
            vel = 0
        )

class DepositHelperWaypoint(Waypoint):
    '''
    The specific waypoint of being just underneath the deposit chamber.
    '''

    # parameters of waypoint
    DEPOSIT_HELPER_X: float = 1600
    DEPOSIT_HELPER_Y: float = 1600

    def __init__(self):
        super().__init__(
            DepositHelperWaypoint.DEPOSIT_HELPER_X,
            DepositHelperWaypoint.DEPOSIT_HELPER_Y,
            heading = None,
            vel = None
        )

class WaypointSequence(ABC):
    '''
    A sequence of waypoints
    '''
    def __init__(self):
        self.waypoints: list[float] = list()
    
    def plan_to_deposit(self, current_x: float, current_y: float):
        '''
        Plans to the deposit chamber.
            1. Adds a waypoint to just below the deposit chamber
            2. Adds a waypoint to next to the deposit chamber
            3. Adds a waypoint to the current position
        
        Parameters:
            current_x: the current x position of the robot
            current_y: the current y position of the robot
        '''

        self.waypoints.insert(0, Waypoint(current_x, current_y))
        self.waypoints.insert(0, DepositWaypoint())
        self.waypoints.insert(0, DepositHelperWaypoint())
    
    def plan_to_emergency(self, current_x: float, current_y: float):
        pass
    
    def get_current_waypoint(self) -> Waypoint | None:
        return self.waypoints[0] if len(self.waypoints) else None
    
    def move_to_next_waypoint(self) -> Waypoint | None:
        if len(self.waypoints) == 0:
            return None
        self.waypoints.pop(0)
        return self.get_current_waypoint()

class SnakeWaypointSequence(WaypointSequence):
    '''
    A sequence of waypoints based on a snake search
    '''

    BORDER_PADDING: float = RobotGeometry.RADIUS
    ENV_LENGTH: float = 2000
    ENV_WIDTH: float = 2000
    POINTS_PER_LINE: int = 4

    def __init__(self):
        super().__init__()
        
        # calculate the y coordinates
        y_spacing: float = (SnakeWaypointSequence.ENV_LENGTH - 2*SnakeWaypointSequence.BORDER_PADDING)/(SnakeWaypointSequence.POINTS_PER_LINE - 1)
        y_coords: list[float] = [SnakeWaypointSequence.BORDER_PADDING + i*y_spacing for i in range(SnakeWaypointSequence.POINTS_PER_LINE)]
        reversed_y_coords: list[float] = y_coords[::-1]

        # calculate the x coordinates
        num_lines: int = math.ceil((SnakeWaypointSequence.ENV_WIDTH - 2*SnakeWaypointSequence.BORDER_PADDING)/(RobotGeometry.DIGGER_WIDTH))
        x_coords: list[float] = [min(SnakeWaypointSequence.BORDER_PADDING + i*RobotGeometry.DIGGER_WIDTH, SnakeWaypointSequence.ENV_WIDTH - SnakeWaypointSequence.BORDER_PADDING) for i in range(num_lines)]

        for i, x in enumerate(x_coords):
            for j, y in enumerate(reversed_y_coords if i % 2 else y_coords): # toggle the direction of each line
                waypoint = Waypoint(
                    x, y,
                    heading = None,
                    vel = 0 if j == 0 or j == len(y_coords) else None, # set velocity to 0 if it is the first or last in a line
                )
                self.waypoints.append(waypoint)

class RectangleWaypointSequence(WaypointSequence):
    '''
    A sequence of waypoints which is just a rectangle
    '''

    LENGTH = 1000
    WIDTH = 1000
    STOPPING = False
    NUM_LOOPS = 5

    def __init__(self):
        super().__init__()

        for i in range(RectangleWaypointSequence.NUM_LOOPS):
            self.waypoints.append(Waypoint(500, 500, heading=None, vel=0 if RectangleWaypointSequence.STOPPING else None))
            self.waypoints.append(Waypoint(500, 500 + RectangleWaypointSequence.LENGTH, heading=None, vel=0 if RectangleWaypointSequence.STOPPING else None))
            self.waypoints.append(Waypoint(500 + RectangleWaypointSequence.WIDTH, 500 + RectangleWaypointSequence.LENGTH, heading=None, vel=0 if RectangleWaypointSequence.STOPPING else None))
            self.waypoints.append(Waypoint(500 + RectangleWaypointSequence.WIDTH, 500, heading=None, vel=0 if RectangleWaypointSequence.STOPPING else None))

class MockLocalisationWaypointSequence(WaypointSequence):
    '''
    A waypoint to force the robot into a circle if using mock localisation
    '''
    def __init__(self):
        super().__init__()

        self.waypoints.append(Waypoint(500, 500, heading=None))