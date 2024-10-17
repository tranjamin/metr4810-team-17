from __future__ import annotations
import math
from typing import *
from abc import ABC, abstractmethod
from math import pi, atan2
from enum import Enum
from robot import wrapToPi
import time

from robot import Controller, Robot, LineFollowerController

class ExtractionStrategies(Enum):
    '''
    A choice of extraction strategies the robot can be configured to.
    '''
    NONE = 1 # does not run extraction
    CONTINUOUS = 2 # always runs extraction (on forward controllers)
    PERIODIC = 3 # only runs extraction at each waypoint

class Debogger(ABC):
    @abstractmethod
    def is_bogged(self, current_x: float, current_y: float, current_theta: float) -> bool:
        pass

    @abstractmethod
    def attempt_debog(self, planner: Pathplanner) -> bool:
        pass

    @abstractmethod
    def pause_debogger(self):
        pass

    @abstractmethod
    def unpause_debogger(self):
        pass


class NoDebogger(Debogger):
    def is_bogged(self, current_x, current_y, current_theta) -> bool:
        return False
    
    def attempt_debog(self, planner) -> bool:
        return super().attempt_debog(planner)
    
    def pause_debogger(self):
        return super().pause_debogger()
    
    def unpause_debogger(self):
        return super().unpause_debogger()

class ActiveDebogger(Debogger):
    EPSILON_X: float
    EPSILON_THETA: float
    PATIENCE: float
    REVERSE_DISTANCE: float
    ANGLE_DEVIATION: float

    def __init__(self, 
                 epsilon_x=30, 
                 epsilon_theta=pi/4,
                 patience=2,
                 reverse_distance=400,
                 angle_deviation=pi/6):
        
        ActiveDebogger.EPSILON_X = epsilon_x
        ActiveDebogger.EPSILON_THETA = epsilon_theta
        ActiveDebogger.PATIENCE = patience
        ActiveDebogger.REVERSE_DISTANCE = reverse_distance
        ActiveDebogger.ANGLE_DEVIATION = angle_deviation

        self.last_debog_position: tuple[float, float, float] = None
        self.last_debog_time: float = None
        self.debogger_enabled: bool = True
        self.debogger_pause_time: float = None

    def is_bogged(self, current_x: float, current_y: float, current_theta: float) -> bool:
        if self.last_debog_position is None:
            self.last_debog_position = (current_x, current_y, current_theta)
        if self.last_debog_time is None:
            self.last_debog_time = time.time()

        debog_x, debog_y, debog_theta = self.last_debog_position
        x_deviation = math.sqrt((current_x - debog_x)**2 + (current_y - debog_y)**2)
        theta_deviation = abs(current_theta - debog_theta)

        if (x_deviation > ActiveDebogger.EPSILON_X or theta_deviation > ActiveDebogger.EPSILON_THETA):
                # we have not bogged
                self.last_debog_time = time.time()
                self.last_debog_position = (current_x, current_y, current_theta)
                return False
        elif time.time() - self.last_debog_time > ActiveDebogger.PATIENCE:
                return True
        return False


    def attempt_debog(self, planner) -> bool:
        reverse = -1 if planner.desired_velocity > 0 else 1
        print("Reversing: ", reverse)

        new_x = min(max(planner.current_x + reverse*math.cos(planner.current_theta)*ActiveDebogger.REVERSE_DISTANCE, RobotGeometry.RADIUS), 2000 - RobotGeometry.RADIUS)
        new_y = min(max(planner.current_y + reverse*math.sin(planner.current_theta)*ActiveDebogger.REVERSE_DISTANCE, RobotGeometry.RADIUS), 2000 - RobotGeometry.RADIUS)

        theta_option_one = wrapToPi(planner.current_theta + ActiveDebogger.ANGLE_DEVIATION)
        theta_option_two = wrapToPi(planner.current_theta - ActiveDebogger.ANGLE_DEVIATION)

        distance_option_one = (abs(1000 - (new_x + math.cos(theta_option_one))))**2 + (abs(1000 - (new_y + math.ain(theta_option_one))))**2
        distance_option_two = (abs(1000 - (new_x + math.cos(theta_option_two))))**2 + (abs(1000 - (new_y + math.ain(theta_option_two))))**2

        new_heading = theta_option_one if distance_option_one < distance_option_two else theta_option_two

        new_waypoint = Waypoint(new_x, new_y, heading=new_heading)

        planner.waypoints.waypoints.insert(0, new_waypoint)
        planner.current_waypoint = new_waypoint
        planner.previous_waypoint = (planner.current_x, planner.current_y)

        print("Current position: ", (planner.current_x, planner.current_y))
        print("New waypoint: ", planner.current_waypoint.coords)

        # update debog parameters
        self.last_debog_time = time.time()
        self.last_debog_position = (planner.current_x, planner.current_y, planner.current_theta)

        planner.update_controller_path = True
    
    def pause_debogger(self):
        self.debogger_enabled = False
        self.debogger_pause_time = time.time()
    
    def unpause_debogger(self):
        self.debogger_enabled = True

        if self.debogger_pause_time is not None and self.last_debog_time is not None:
            self.last_debog_time = time.time() - (self.debogger_pause_time - self.last_debog_time)


class Pathplanner():
    '''
    A class to hold the pathplanning algorithm. 
    '''
    def __init__(self):
        self.waypoints: WaypointSequence
        self.controller: LineFollowerController

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

        self.extraction_strategy: ExtractionStrategies = None

    def set_extraction_strategy(self, strategy: ExtractionStrategies):
        '''
        Sets the extraction strategy to be used
        '''
        self.extraction_strategy = strategy
    
    def set_debogging_strategy(self, strategy: Debogger):
        '''
        Sets the debog strategy to be used
        '''
        self.debog_strategy: Debogger = strategy

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

        if self.debog_strategy.is_bogged(current_x, current_y, current_theta):
            print("Bogging Detected")
            self.debog_strategy.attempt_debog(self)

        # if we have reached the current waypoint, move to the next one
        if self.current_waypoint is None:
            return
        if self.controller.has_reached_goal() and not self.stopFlag:
            # do stuff
            if self.current_waypoint.stopFlag:
                print("Should be stopping now")
                self.signal_pathplanning_stop()
            if self.current_waypoint.suspendFlag:
                self.signal_extraction_stop()
                self.extractionFlag = False
            if self.current_waypoint.resumeFlag:
                self.signal_extraction_start()
                self.signal_pathplanning_start()

            self.previous_waypoint = self.current_waypoint.coords
            self.current_waypoint = self.waypoints.move_to_next_waypoint()
            self.update_controller_path = True
            if self.current_waypoint is None:
                print("---- FINISHED PATH. RESTARTING ----")
                self.waypoints.reset_waypoints()
                self.current_waypoint = self.waypoints.get_current_waypoint()
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
                if self.extractionFlag and self.extraction_strategy == ExtractionStrategies.CONTINUOUS:
                    self.signal_extraction_start()
            
            if self.extraction_strategy == ExtractionStrategies.CONTINUOUS:
                self.desired_velocity, self.desired_angular = self.controller.get_control_action(self.current_x, self.current_y, self.current_theta, 
                on_spin_start=(lambda: self.signal_extraction_stop(), self.pause_debogger()),
                on_spin_end=self.unpause_debogger)
            elif self.extraction_strategy == ExtractionStrategies.PERIODIC and self.extraction_strategy:
                self.desired_velocity, self.desired_angular = self.controller.get_control_action(self.current_x, self.current_y, self.current_theta, 
                on_spin_start=self.pause_debogger,
                on_spin_end=(lambda: self.signal_extraction_execute(), self.unpause_debogger()))
            else:
                self.desired_velocity, self.desired_angular = self.controller.get_control_action(self.current_x, self.current_y, self.current_theta,
                on_spin_start=self.pause_debogger,
                on_spin_end=self.unpause_debogger)

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
        if isinstance(self.debog_strategy, ActiveDebogger):
            self.debog_strategy.last_debog_time = time.time() + 23
    
    def signal_extraction_start(self):
        if self.extraction_strategy == ExtractionStrategies.CONTINUOUS:
            self.robot.send_control_command("command=7")
    
    def signal_extraction_stop(self):
        if self.extraction_strategy == ExtractionStrategies.CONTINUOUS:
            self.robot.send_control_command("command=8")

    def signal_pathplanning_stop(self):
        self.stopFlag = True
        self.pause_debogger()

    def signal_pathplanning_start(self):
        self.stopFlag = False
        self.unpause_debogger()

    def signal_extraction_execute(self):
        print("sending commands")
        for _ in range(20):
            self.robot.send_control_command("command=9")

    def signal_robot_forward(self):
        self.robot.send_control_command("command=1")

    def signal_robot_backward(self):
        self.robot.send_control_command("command=2")

    def signal_robot_stopped(self):
        self.robot.send_control_command("command=3")

    def pause_debogger(self):
        self.debog_strategy.pause_debogger()

    def unpause_debogger(self):
        self.debog_strategy.unpause_debogger()
    
class RobotGeometry():
    '''
    A class to represent the physical bounds of the robot
    '''
    WIDTH: float = 400
    LENGTH: float = 176
    PADDING: float = 80
    EMERGENCY_PADDING: float = 30
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
        assert self.heading is None or (self.heading >= -pi and self.heading <= pi)    

    def __str__(self):
        repr(self)
    
    def __repr__(self):
        return f"{self.x}, {self.y}, {self.heading}"

class DepositWaypoint(Waypoint):
    '''
    The specific waypoint of being next to the deposit chamber.
    '''

    # parameters of waypoint
    DEPOSIT_SIZE: float = 165
    DEPOSIT_X: float = 350
    DEPOSIT_Y: float = 350
    DEPOSIT_HEADING: float = 3*pi/4

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
    DEPOSIT_HELPER_X: float = DepositWaypoint.DEPOSIT_X - (DepositWaypoint.DEPOSIT_SIZE/2 + RobotGeometry.RADIUS)*math.cos(DepositWaypoint.DEPOSIT_HEADING)
    DEPOSIT_HELPER_Y: float = DepositWaypoint.DEPOSIT_Y - (DepositWaypoint.DEPOSIT_SIZE/2 + RobotGeometry.RADIUS)*math.sin(DepositWaypoint.DEPOSIT_HEADING)

    def __init__(self, heading=DepositWaypoint.DEPOSIT_HEADING):
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
        self.repeat_waypoints: list[float] = list()
    
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
        
        return_angle_2 = atan2(
            - (current_y - DepositHelperWaypoint.DEPOSIT_HELPER_Y), 
            - (current_x - DepositHelperWaypoint.DEPOSIT_HELPER_X))

        # insert the return waypoint
        self.waypoints.insert(0, Waypoint(current_x, current_y, current_theta, resumeExtraction=True))

        # insert the deposit waypoint, sandwiched by the helpers
        self.waypoints.insert(0, DepositHelperWaypoint(heading=return_angle))
        self.waypoints.insert(0, DepositWaypoint())
        self.waypoints.insert(0, DepositHelperWaypoint())
        self.waypoints.insert(0, Waypoint(current_x, current_y, return_angle_2))
    
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
        
        # disregard orthogonal directions
        if ((current_theta < pi/4) and (current_theta > -pi/4)) or ((current_theta > 3*pi/4) and (current_theta < -3*pi/4)):
            # disregard vertical
            up_emergency = math.inf
            down_emergency = math.inf
        else:
            # disregard horiontal
            left_emergency = math.inf
            right_emergency = math.inf

        # calculate closest point
        emergency_side = min([left_emergency, right_emergency, down_emergency, up_emergency])

        # generate relevant waypoint
        if emergency_side == left_emergency:
            emergency_waypoint = Waypoint(RobotGeometry.LENGTH/2 + RobotGeometry.EMERGENCY_PADDING, current_y, -pi)
        elif emergency_side == right_emergency:
            emergency_waypoint = Waypoint(2000 - RobotGeometry.LENGTH/2 - RobotGeometry.EMERGENCY_PADDING, current_y, 0)
        elif emergency_side == up_emergency:
            emergency_waypoint = Waypoint(current_x, 2000 - RobotGeometry.LENGTH/2 - RobotGeometry.EMERGENCY_PADDING, pi/2)
        else:
            emergency_waypoint = Waypoint(current_x, RobotGeometry.LENGTH/2 + RobotGeometry.EMERGENCY_PADDING, -pi/2)
        
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

    def reset_waypoints(self):
        self.waypoints = self.repeat_waypoints.copy()

class SnakeWaypointSequence(WaypointSequence):
    '''
    A sequence of waypoints based on a snake search.
    '''

    BORDER_PADDING: float = RobotGeometry.RADIUS + RobotGeometry.PADDING 
    ENV_LENGTH: float = 2000
    ENV_WIDTH: float = 2000

    def __init__(self, 
                 points_per_line: int,
                 theta_agnostic=False):
        super().__init__()
        
        # calculate the y coordinates
        y_spacing: float = (SnakeWaypointSequence.ENV_LENGTH - 2*SnakeWaypointSequence.BORDER_PADDING)/(points_per_line - 1)
        y_coords: list[float] = [SnakeWaypointSequence.BORDER_PADDING + i*y_spacing for i in range(points_per_line)]
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

        self.repeat_waypoints = self.waypoints.copy()
        self.waypoints.pop(0)

class SpiralWaypointSequence(WaypointSequence):
    '''
    A sequence of waypoints based on a snake search.
    '''

    BORDER_PADDING: float = RobotGeometry.RADIUS + RobotGeometry.PADDING 
    ENV_LENGTH: float = 2000
    ENV_WIDTH: float = 2000

    def __init__(self, 
                 points_per_line: int,
                 chamfered: bool,
                 theta_agnostic=False):
        super().__init__()
        
        # calculate the y coordinates
        y_spacing: float = (SpiralWaypointSequence.ENV_LENGTH - 2*SpiralWaypointSequence.BORDER_PADDING)/(points_per_line - 1)
        y_coords: list[float] = [SpiralWaypointSequence.BORDER_PADDING + i*y_spacing for i in range(points_per_line)]

        # calculate the x coordinates
        x_coords: list[float] = y_coords.copy()

        while len(x_coords) and len(y_coords):
            for j, y in enumerate(y_coords):
                target_heading = pi/2 if j != (len(y_coords) - 1) else 0
                waypoint: Waypoint = Waypoint(
                    x_coords[0], y,
                    heading = None if theta_agnostic else target_heading,
                    vel = 0 if j == 0 or j == len(y_coords) else None
                )
                if j != len(y_coords) - 1 or not chamfered:
                    self.waypoints.append(waypoint)
            x_coords.pop(0)

            if not len(x_coords):
                break
            
            for i, x in enumerate(x_coords):
                target_heading = 0 if i != (len(x_coords) - 1) else -pi/2
                waypoint: Waypoint = Waypoint(
                    x, y_coords[-1],
                    heading = None if theta_agnostic else target_heading,
                    vel = 0 if i == 0 or i == len(y_coords) else None
                )
                if i != len(x_coords) - 1 or not chamfered:
                    self.waypoints.append(waypoint)
            y_coords.pop(-1)

            if not len(y_coords):
                break

            for j, y in enumerate(y_coords[::-1]):
                target_heading = -pi/2 if j != (len(y_coords) - 1) else pi
                waypoint: Waypoint = Waypoint(
                    x_coords[-1], y,
                    heading = None if theta_agnostic else target_heading,
                    vel = 0 if j == 0 or j == len(y_coords) else None
                )
                if j != len(y_coords) - 1 or not chamfered:
                    self.waypoints.append(waypoint)
            x_coords.pop(-1)

            if not len(x_coords):
                break
            
            for i, x in enumerate(x_coords[::-1]):
                target_heading = pi if i != (len(x_coords) - 1) else pi/2
                waypoint: Waypoint = Waypoint(
                    x, y_coords[0],
                    heading = None if theta_agnostic else target_heading,
                    vel = 0 if i == 0 or i == len(y_coords) else None
                )
                if i != len(x_coords) - 1 or not chamfered:
                    self.waypoints.append(waypoint)
            y_coords.pop(0)

            if not len(y_coords):
                break

        while self.waypoints[0].y < 700:
            self.waypoints.pop(0)

        self.repeat_waypoints = self.waypoints.copy()

class SpiralWaypointSequenceChamfered(WaypointSequence):
    '''
    A sequence of waypoints based on a snake search.
    '''

    BORDER_PADDING: float = RobotGeometry.RADIUS + RobotGeometry.PADDING 
    ENV_LENGTH: float = 2000
    ENV_WIDTH: float = 2000

    def __init__(self, 
                 point_spacing: float,
                 line_spacing: float,
                 theta_agnostic=False):
        super().__init__()
        
        y_initial = SpiralWaypointSequence.BORDER_PADDING
        x_initial = SpiralWaypointSequence.BORDER_PADDING

        y_min = y_initial
        y_max = SpiralWaypointSequence.ENV_LENGTH - SpiralWaypointSequence.BORDER_PADDING
        x_min = x_initial
        x_max = SpiralWaypointSequence.ENV_WIDTH - SpiralWaypointSequence.BORDER_PADDING

        chamfer = 200

        i = 0
        j = 0
        while True:
            while True:
                x = x_min
                y = min(y_min + chamfer + point_spacing*j, y_max - chamfer)
                if y == y_max - chamfer:
                    target_heading = pi/4
                else:
                    target_heading = pi/2

                waypoint = Waypoint(
                    x, y, heading=target_heading
                )
                self.waypoints.append(waypoint)

                if y == y_max - chamfer:
                    break

                j += 1

            y_min = y_min + line_spacing
            
            while True:
                x = min(x_min + chamfer + point_spacing*i, x_max - chamfer)
                y = y_max
                if x == x_max - chamfer:
                    target_heading = -pi/4
                else:
                    target_heading = 0

                waypoint = Waypoint(
                    x, y, heading=target_heading
                )
                self.waypoints.append(waypoint)

                if x == x_max - chamfer:
                    break
            
                i += 1
    
            x_min = x_min + line_spacing
            i = 0
            j = 0

            while True:
                x = x_max
                y = max(y_max - chamfer - point_spacing*j, y_min + chamfer)
                if y == y_min + chamfer:
                    target_heading = -3*pi/4
                else:
                    target_heading = -pi/2

                waypoint = Waypoint(
                    x, y, heading=target_heading
                )
                self.waypoints.append(waypoint)

                if y == y_min + chamfer:
                    break

                j += 1
            
            y_max = y_max - line_spacing
            
            while True:
                x = max(x_max - chamfer - point_spacing*i, x_min + chamfer)
                y = y_min
                if x == x_min + chamfer:
                    target_heading = 3 * pi/4
                else:
                    target_heading = pi

                waypoint = Waypoint(
                    x, y, heading=target_heading
                )
                self.waypoints.append(waypoint)

                if x == x_min + chamfer:
                    break
            
                i += 1

            x_max = x_max - line_spacing
            i = 0
            j = 0

            if (x_max - x_min) < line_spacing or (y_max - y_min) < line_spacing:
                break
    
        self.repeat_waypoints = self.waypoints.copy()
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
        
        self.repeat_waypoints = self.waypoints.copy()

class MockLocalisationWaypointSequence(WaypointSequence):
    '''
    A waypoint to force the robot into a circle if using mock localisation
    '''
    def __init__(self):
        super().__init__()

        self.waypoints.append(Waypoint(500, 500, heading=None))
        self.repeat_waypoints = self.waypoints.copy()