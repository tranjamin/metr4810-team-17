import numpy as np
import math
from typing import *

class RobotGeometry():
    WIDTH: float = 400
    LENGTH: float = 200
    RADIUS: float = math.sqrt(WIDTH**2 + LENGTH**2)/2

    DIGGER_WIDTH: float = 200
    DIGGER_RADIUS: float = math.sqrt(DIGGER_WIDTH**2 + LENGTH**2)/2

class Waypoint():
    '''
    A coordinate and heading which the robot will move through. Coordinates are measured through the geometric centre of the robot.
    '''
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
        """
        self.x = x
        self.y = y
        self.heading = heading
        self.vel = vel
        self.suspendFlag = suspendExtraction
        self.resumeFlag = resumeExtraction

        assert self.x < 2000 and self.x > 0
        assert self.y < 2000 and self.y > 0
        assert self.heading is None or (self.heading > -180 and self.heading < 180)

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

class WaypointSequence():
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

class WaypointFollower():
    def __init__(self):
        pass