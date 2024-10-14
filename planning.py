from __future__ import annotations
import numpy as np
import math
from typing import *
from robot import Controller, Robot
from abc import ABC
from math import pi, atan2

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
        self.previous_waypoint: tuple = (0, 0)
        self.update_controller_path = True

        self.stopFlag = False
        self.extractionFlag = False
        self.extraction_allowed = True

    def set_waypoints(self, waypoints: WaypointSequence):
        '''
        Set the waypoint sequence the planner will be following
        '''
        self.waypoints = waypoints
        self.current_waypoint = self.waypoints.get_current_waypoint()
        print("---- FIRST WAYPOINT ----")
        print(f"(Next waypoint is at: {self.current_waypoint.coords})")
    
    def set_controller(self, controller: Controller):
        '''
        Sets the controller used for the planning
        '''
        self.controller = controller
    
    def set_robot(self, robot: Robot):
        self.robot = robot

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
        if self.controller.has_reached_goal() and not self.stopFlag:
            # do stuff
            if self.current_waypoint.stopFlag:
                print("Should be stopping now")
                self.stopFlag = True
            if self.current_waypoint.suspendFlag:
                self.signal_extraction_stop()
                self.extractionFlag = False
            if self.current_waypoint.resumeFlag:
                self.signal_extraction_start()
                self.extractionFlag = True

            self.previous_waypoint = self.current_waypoint.coords
            self.current_waypoint = self.waypoints.move_to_next_waypoint()
            self.update_controller_path = True
            if self.current_waypoint is None:
                print("---- FINISHED PATH ----")
                return
            print("---- MOVE TO NEXT WAYPOINT ----")
            print(f"(Next waypoint is at: {self.current_waypoint.coords})")
    
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
            # new path
            if self.update_controller_path:
                self.controller.set_path(self.previous_waypoint, self.current_waypoint.coords, self.current_waypoint.heading)
                self.update_controller_path = False
                if self.extractionFlag:
                    self.signal_extraction_start()
            
            self.desired_velocity, self.desired_angular = self.controller.get_control_action(self.current_x, self.current_y, self.current_theta, stop_extraction=self.signal_extraction_stop)
            if self.stopFlag:
                self.desired_angular = 0
                self.desired_velocity = 0

    def add_emergency(self):
        self.previous_waypoint = (self.current_x, self.current_y)
        self.waypoints.plan_to_emergency(self.current_x, self.current_y, self.current_theta)
        self.current_waypoint = self.waypoints.get_current_waypoint()
        self.update_controller_path = True
        self.signal_extraction_stop()
        self.extractionFlag = False
    
    def add_delivery(self):
        self.previous_waypoint = (self.current_x, self.current_y)
        self.waypoints.plan_to_deposit(self.current_x, self.current_y, self.current_theta)
        self.current_waypoint = self.waypoints.get_current_waypoint()
        self.update_controller_path = True
        self.signal_extraction_stop()
        self.extractionFlag = False

    def signal_delivery_start(self):
        self.robot.send_control_command("command=0")
        self.stopFlag = False
    
    def signal_extraction_start(self):
        self.robot.send_control_command("command=7")
    
    def signal_extraction_stop(self):
        self.robot.send_control_command("command=8")

    def signal_pathplanning_stop(self):
        self.stopFlag = True

    def signal_pathplanning_start(self):
        self.stopFlag = False

class RobotGeometry():
    '''
    A class to represent the physical bounds of the robot
    '''
    WIDTH: float = 400
    LENGTH: float = 176
    PADDING: float = 50
    RADIUS: float = math.sqrt(WIDTH**2 + LENGTH**2)/2

    DIGGER_WIDTH: float = 110
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
                 stopMovement: bool = False):
        """
        Parameters
            x (float): the x-coordinate of the waypoint in millimetres
            y (float): the y-coordinate of the waypoint in millimetres
            heading (float): the angle the robot should reach the waypoint at, in radians CCW of positive x [-pi/2 to pi/2] or None if the heading is unimportant
            vel (float): the target velocity the robot should reach the waypoint at, in metres per second, or None if unimportant
            suspendExtraction (bool): stops extraction when reaching this waypoint
            resumeExtraction (bool): starts extraction when reaching this waypoint
        """
        self.x = x
        self.y = y
        self.coords = (x, y)
        self.heading = heading
        self.vel = vel
        self.suspendFlag = suspendExtraction
        self.resumeFlag = resumeExtraction
        self.stopFlag = stopMovement

        assert self.x <= 2000 and self.x >= 0
        assert self.y <= 2000 and self.y >= 0
        assert self.heading is None or (self.heading >= -180 and self.heading <= 180)    

class DepositWaypoint(Waypoint):
    '''
    The specific waypoint of being next to the deposit chamber.
    '''

    # parameters of waypoint
    DEPOSIT_SIZE: float = 200
    DEPOSIT_X: float = 2000 - DEPOSIT_SIZE - RobotGeometry.WIDTH / 2
    DEPOSIT_Y: float = 2000 - RobotGeometry.LENGTH / 2 - RobotGeometry.PADDING
    DEPOSIT_HEADING: float = -pi/2

    def __init__(self):
        super().__init__(
            DepositWaypoint.DEPOSIT_X,
            DepositWaypoint.DEPOSIT_Y,
            DepositWaypoint.DEPOSIT_HEADING,
            stopMovement=True
        )

class DepositHelperWaypoint(Waypoint):
    '''
    The specific waypoint of being just underneath the deposit chamber.
    '''

    # parameters of waypoint
    DEPOSIT_HELPER_X: float = 2000 - DepositWaypoint.DEPOSIT_SIZE - RobotGeometry.WIDTH / 2
    DEPOSIT_HELPER_Y: float = 2000 - DepositWaypoint.DEPOSIT_SIZE - RobotGeometry.RADIUS - 2*RobotGeometry.PADDING

    def __init__(self, heading=-pi/2):
        super().__init__(
            DepositHelperWaypoint.DEPOSIT_HELPER_X,
            DepositHelperWaypoint.DEPOSIT_HELPER_Y,
            heading = heading,
            vel = None
        )

class WaypointSequence(ABC):
    '''
    A sequence of waypoints
    '''
    def __init__(self):
        self.waypoints: list[float] = list() # the list of waypoints
    
    def plan_to_deposit(self, current_x: float, current_y: float, current_theta: float):
        '''
        Plans to the deposit chamber.
            1. Adds a waypoint to just below the deposit chamber
            2. Adds a waypoint to next to the deposit chamber
            3. Adds a waypoint to the current position
        
        Parameters:
            current_x (float): the current x position of the robot
            current_y (float): the current y position of the robot
            current_that (float): the current angle of the robot
        '''

        # calculate the return angle after going to thewaypoint
        return_angle = atan2(
            current_y - DepositHelperWaypoint.DEPOSIT_HELPER_Y, 
            current_x - DepositHelperWaypoint.DEPOSIT_HELPER_X)

        # insert the return waypoint
        self.waypoints.insert(0, Waypoint(current_x, current_y, current_theta, resumeExtraction=True))

        # insert the deposit waypoint, sandwiched by the helpers
        self.waypoints.insert(0, DepositHelperWaypoint(heading=return_angle))
        self.waypoints.insert(0, DepositWaypoint())
        self.waypoints.insert(0, DepositHelperWaypoint())
    
    def plan_to_emergency(self, current_x: float, current_y: float, current_theta: float):
        '''
        Plans to the emergency high ground (the edges of the arena)

        Parameters:
            current_x (float): the robot's current x position
            current_y (float): the robot's current y position
            current_theta (float): the robot's current angle
        '''

        # all four possible emergency points
        left_emergency = current_x
        right_emergency = 2000 - current_x
        down_emergency = current_y
        up_emergency = 2000 - current_y
        
        # calculate closest point
        emergency_side = min([left_emergency, right_emergency, down_emergency, up_emergency])

        # generate relevant waypoint
        if emergency_side == left_emergency:
            emergency_waypoint = Waypoint(RobotGeometry.LENGTH/2 + RobotGeometry.PADDING, current_y, -pi)
        elif emergency_side == right_emergency:
            emergency_waypoint = Waypoint(2000 - RobotGeometry.LENGTH/2 - RobotGeometry.PADDING, current_y, 0)
        elif emergency_side == up_emergency:
            emergency_waypoint = Waypoint(current_x, 2000 - RobotGeometry.LENGTH/2 - RobotGeometry.PADDING, pi/2)
        else:
            emergency_waypoint = Waypoint(current_x, RobotGeometry.LENGTH/2 + RobotGeometry.PADDINGs, -pi/2)
        
        # add waypoint
        self.waypoints.insert(0, Waypoint(current_x, current_y, current_theta, resumeExtraction=True))
        self.waypoints.insert(0, emergency_waypoint)
    
    def get_current_waypoint(self) -> Waypoint | None:
        '''
        Gets the current waypoint being tracked.

        Returns:
            (Waypoint | None): the current waypoint being tracked, or None if there are no waypoints left.
        '''
        return self.waypoints[0] if len(self.waypoints) else None
    
    def move_to_next_waypoint(self) -> Waypoint | None:
        '''
        Transitions tracking to the next waypoint.

        Returns:
            (Waypoint | None): the next waypoint to track to, or None if there are no waypoints left.
        '''
        if len(self.waypoints) == 0:
            return None
        self.waypoints.pop(0)
        return self.get_current_waypoint()

class SnakeWaypointSequence(WaypointSequence):
    '''
    A sequence of waypoints based on a snake search.
    '''

    BORDER_PADDING: float = RobotGeometry.RADIUS + RobotGeometry.PADDING 
    ENV_LENGTH: float = 2000
    ENV_WIDTH: float = 2000
    POINTS_PER_LINE: int = 2

    def __init__(self, theta_agnostic=False):
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
                if (j != len(y_coords) - 1): # if not at the end
                    target_heading = pi/2 if not i % 2 else -pi/2
                else:
                    target_heading = 0
                
                waypoint = Waypoint(
                    x, y,
                    heading = None if theta_agnostic else target_heading,
                    vel = 0 if j == 0 or j == len(y_coords) else None, # set velocity to 0 if it is the first or last in a line
                )
                self.waypoints.append(waypoint)

        self.waypoints.pop(0)

class RectangleWaypointSequence(WaypointSequence):
    '''
    A sequence of waypoints which is just a rectangle
    '''

    STOPPING = True

    def __init__(self, length_x, length_y, origin_x, origin_y, num_loops, angle_agnostic=False):
        super().__init__()

        for i in range(num_loops):
            self.waypoints.append(Waypoint(
                origin_x, 
                origin_y + length_y, 
                heading=None if angle_agnostic else 0, 
                vel=0 if RectangleWaypointSequence.STOPPING else None))
            self.waypoints.append(Waypoint(
                origin_x + length_x, 
                origin_y + length_y, 
                heading=None if angle_agnostic else -pi/2,
                vel=0 if RectangleWaypointSequence.STOPPING else None))
            self.waypoints.append(Waypoint(
                origin_x + length_x, 
                origin_y, 
                heading=None if angle_agnostic else pi, 
                vel=0 if RectangleWaypointSequence.STOPPING else None))
            self.waypoints.append(Waypoint(
                origin_x, 
                origin_y, 
                heading=None if angle_agnostic else pi/2, 
                vel=0 if RectangleWaypointSequence.STOPPING else None))

class MockLocalisationWaypointSequence(WaypointSequence):
    '''
    A waypoint to force the robot into a circle if using mock localisation
    '''
    def __init__(self):
        super().__init__()

        self.waypoints.append(Waypoint(500, 500, heading=None))