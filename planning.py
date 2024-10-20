'''
Holds functionality associated with mission planning.

Classes:
    Pathplanner(): a class which plans and executes missions
    RobotGeometry(): an uninstantiated clsas which defines the geometry of the environment and robot
    Waypoint(): a class representing a point for the robot to navigate to
    DepositWaypoint(): the specific waypoint representing the deposit location
    DepositHelperWaypoint(): the specific waypoint representing the intermediate waypoint to the deposit
    WaypointSequece(): an abstract class representing a sequence of waypoints (a plan)
        SnakeWaypointSequence(): a type of plan which snakes back and forwards along a grid
        SpiralWaypointSequence(): a type of plan which spirals from the outside in
        RectangleWaypointSequence(): a type of plan which draws out a retangle
        ShiftingWindowSequence(): a type of plan which paths out a rectangle which shifts across the map
        StraightineWaypointSequence(): a type of plan which moves in a straight line from the deposit
        MockLocalisationWaypointSequence(): a type of plan compatible with mock localisation.
    Debogger(): an abstract class which detects bogging and attempts to unbog the robot
        NoDebogger(): a class with debogging disabled
        ActiveDebogger(): a class with debogging enabled
'''

from __future__ import annotations
import math
from typing import *
from abc import ABC, abstractmethod
from math import pi, atan2
import time

from controllers import *
from robot import *
from extraction import *

class Pathplanner():
    '''
    A class to hold the pathplanning algorithm. 
    '''
    def __init__(self):
        # interacting classes
        self.waypoints: WaypointSequence = None
        self.controller: LineFollowerController = None
        self.extraction_strategy: ExtractionModes = None
        self.debog_strategy: Debogger = None
        self.robot: Robot = None

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

        self.update_controller_path = True # flags that the controller needs to update its path
        self.stopFlag = False # flags that the pathplanning should be paused

    def set_extraction_strategy(self, strategy: ExtractionModes):
        '''
        Sets the extraction strategy to be used by the planner

        Parameters:
            strategy (ExtractionModes): the strategy to use
        '''
        self.extraction_strategy = strategy

    def set_debogging_strategy(self, strategy: Debogger):
        '''
        Sets the debog strategy to be used by the panner

        Parameters:
            strategy (Debogger): the strategy to use
        '''
        self.debog_strategy: Debogger = strategy

    def set_waypoints(self, waypoints: WaypointSequence):
        '''
        Set the waypoint sequence the planner will be following.

        Parameters:
            waypoints (WaypointSequence): the sequence to follow
        '''

        self.waypoints = waypoints
        self.current_waypoint = self.waypoints.get_current_waypoint() # updates current waypoint

        # log information about the first waypoint
        print("---- FIRST WAYPOINT ----")
        print(f"(Next waypoint is at: {self.current_waypoint.coords}, {self.current_waypoint.heading})")

    def set_controller(self, controller: Controller):
        '''
        Sets the controller used for the planning.

        Parameters:
            controller (Controller): the controller to use
        '''
        self.controller = controller

    def set_robot(self, robot: Robot):
        '''
        Links the robot which will be used for communication.

        Parameters:
            robot (Robot): the robot to link
        '''
        self.robot = robot

    def update_robot_position(self, current_x: float, current_y: float, current_theta: float):
        '''
        Update the planner's knowledge of the robot. Moves to the next waypoint if necessary.

        Parameters:
            current_x (float): the value to update the x coord as
            current_y (float): the value to update the y coord as
            current_theta (float): the value to update the angle as
        '''
        self.current_x = current_x
        self.current_y = current_y
        self.current_theta = current_theta

        # detects debogging and attempts to debog
        if self.debog_strategy.is_bogged(current_x, current_y, current_theta):
            print("Bogging Detected")
            self.debog_strategy.attempt_debog(self)
        elif self.debog_strategy.has_unbogged(current_x, current_y, current_theta):
            print("Unbog detected")
            self.debog_strategy.stop_debog(self)

        if self.current_waypoint is None:
            # at the end of the sequence
            return

        if self.controller.has_reached_goal() and not self.stopFlag:
            # has reached the waypoint so can move to the next one

            if self.current_waypoint.stopFlag: # stops pathplanning if flag set
                self.extraction_strategy.disable_extraction()
                self.debog_strategy.disable_debogger()
                self.stopFlag = True
            if self.current_waypoint.suspendFlag: # stops extraction if flag set
                self.extraction_strategy.disable_extraction()
            if self.current_waypoint.resumeFlag: # resumes extraction if flag set
                self.extraction_strategy.enable_extraction()

            # update the previous and current waypoint
            self.previous_waypoint = self.current_waypoint.coords
            self.current_waypoint = self.waypoints.move_to_next_waypoint()
            self.update_controller_path = True

            # restart the waypoints if at the end
            if self.current_waypoint is None:
                print("---- FINISHED PATH. RESTARTING ----")
                self.waypoints.reset_waypoints()
                self.current_waypoint = self.waypoints.get_current_waypoint()

            print("---- MOVE TO NEXT WAYPOINT ----")
            print(f"(Next waypoint is at: {self.current_waypoint.coords}, {self.current_waypoint.heading})")

    def controller_step(self):
        '''
        Use the controller to calculate the desired movements. 
        Does not return but instead sets self.desired_velocity and self.desired_angular
        '''
        if self.current_waypoint is None: # stop the robot if there are no more waypoints
            self.desired_velocity = 0
            self.desired_angular = 0
        else: # else, get the control actions
            if self.update_controller_path: # update controller path if necessary
                self.controller.set_path(
                    self.previous_waypoint,
                    self.current_waypoint.coords,
                    self.current_waypoint.heading
                    )
                self.update_controller_path = False

            def spin_start_func(): # define function to call when spin controller starts
                self.extraction_strategy.pause_extraction()
                self.debog_strategy.pause_debogger()

            def spin_stop_func(): # define function to call when spin controller stops
                self.extraction_strategy.unpause_extraction()
                self.debog_strategy.unpause_debogger()

            # get the desired movements from controller
            self.desired_velocity, self.desired_angular = self.controller.get_control_action(
                self.current_x,
                self.current_y,
                self.current_theta,
                on_spin_start=spin_start_func,
                on_spin_end=spin_stop_func
                )

            # if pathplanning has paused, stop the robot
            if self.stopFlag:
                self.desired_angular = 0
                self.desired_velocity = 0

    def add_emergency(self):
        '''
        Plans to the emergency high ground.
        '''

        # sets the old waypoint as the current potiiosn
        self.previous_waypoint = (self.current_x, self.current_y)

        # updates the waypoint sequence to include the emergency
        self.waypoints.plan_to_emergency(self.current_x, self.current_y, self.current_theta)

        # update the class variables
        self.current_waypoint = self.waypoints.get_current_waypoint()
        self.update_controller_path = True

        # disable extraction
        self.extraction_strategy.disable_extraction()

    def add_delivery(self):
        '''
        Plans to the deposit area.
        '''

        # sets the old waypoint as the current position
        self.previous_waypoint = (self.current_x, self.current_y)

        # updates the waypoint sequene to include the deposit
        self.waypoints.plan_to_deposit(self.current_x, self.current_y, self.current_theta)

        # update the class variables
        self.current_waypoint = self.waypoints.get_current_waypoint()
        self.update_controller_path = True

        # disable extraction
        self.extraction_strategy.disable_extraction()

    def signal_delivery_start(self):
        '''
        Signals to the robot to start the delivery sequence
        '''

        # sends the required command through the robot
        self.robot.send_control_command("command=0")

    def resume_plan(self):
        '''
        Resumes the path plan.
        '''

        # resumes path plan
        self.stopFlag = False
        self.debog_strategy.enable_debogger()

        # pauses the debogger for the predefined time
        if isinstance(self.debog_strategy, ActiveDebogger):
            self.debog_strategy.last_debog_time = time.time()

class RobotGeometry():
    '''
    A class to represent the physical bounds of the robot.
    '''

    # the robot's physical dimensions
    WIDTH: float = 400
    LENGTH: float = 176
    RADIUS: float = math.sqrt(WIDTH**2 + LENGTH**2)/2

    PADDING: float = 50 # padding for the robot when pathplanning
    EMERGENCY_PADDING: float = 30 # padding for the robot when moving to high ground

    # the digger's physical dimensions
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

        # ensures the waypoint is within the bounds of the environment
        assert self.x <= 2000 and self.x >= 0
        assert self.y <= 2000 and self.y >= 0
        assert self.heading is None or (self.heading >= -pi and self.heading <= pi)

    def __str__(self):
        return f"{self.x}, {self.y}, {self.heading}"

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
    HELPER_PADDING: float = 10 # the extra padding to add on to the helper waypooint

    def __init__(self):
        super().__init__(
            DepositWaypoint.DEPOSIT_X,
            DepositWaypoint.DEPOSIT_Y,
            DepositWaypoint.DEPOSIT_HEADING,
            stopMovement=True
        )

    @staticmethod
    def redefine_deposit(x: float, y: float, theta: float):
        '''
        Redefines the location of the deposit position and helper position.

        Parameters:
            x (float): the new x position for the deposit
            y (float): the new y position for the deposit
            theta (float): the new angle for the deposit
        '''

        print("Dynamically setting the delivery waypoint...")

        # calcualte how far back the helper waypoint needs to be
        backwards_length = DepositWaypoint.DEPOSIT_SIZE/2 + RobotGeometry.RADIUS

        # calculate the helper position
        new_helper_x = x - backwards_length*math.cos(theta)
        new_helper_y = y - backwards_length*math.sin(theta)

        try:
            # ensure the helper is within the bounds of the environment
            # assert new_helper_x <= 2000 and new_helper_x >= 0
            # assert new_helper_y <= 2000 and new_helper_y >= 0

            # ensure the helper is not too close to the edge
            new_helper_x = max(new_helper_x, RobotGeometry.RADIUS + DepositWaypoint.HELPER_PADDING)
            new_helper_y = max(new_helper_y, RobotGeometry.RADIUS + DepositWaypoint.HELPER_PADDING)

            # update the deposit waypoint
            DepositWaypoint.DEPOSIT_HEADING = theta
            DepositWaypoint.DEPOSIT_X = x
            DepositWaypoint.DEPOSIT_Y = y

            # update the helper waypoint
            DepositHelperWaypoint.DEPOSIT_HELPER_X = new_helper_x
            DepositHelperWaypoint.DEPOSIT_HELPER_Y = new_helper_y
        except AssertionError:
            print("Dynamic setting failed... reverting")

class DepositHelperWaypoint(Waypoint):
    '''
    The specific waypoint of being just underneath the deposit chamber.
    '''

    # parameters of waypoint
    DEPOSIT_HELPER_X: float = DepositWaypoint.DEPOSIT_X - (DepositWaypoint.DEPOSIT_SIZE/2 + RobotGeometry.RADIUS)*math.cos(DepositWaypoint.DEPOSIT_HEADING)
    DEPOSIT_HELPER_Y: float = DepositWaypoint.DEPOSIT_Y - (DepositWaypoint.DEPOSIT_SIZE/2 + RobotGeometry.RADIUS)*math.sin(DepositWaypoint.DEPOSIT_HEADING)

    def __init__(self, heading=None):
        super().__init__(
            DepositHelperWaypoint.DEPOSIT_HELPER_X,
            DepositHelperWaypoint.DEPOSIT_HELPER_Y,
            heading = heading if heading is not None else DepositWaypoint.DEPOSIT_HEADING,
            vel = None
        )

class WaypointSequence(ABC):
    '''
    A sequence of waypoints
    '''
    def __init__(self):
        self.waypoints: list[Waypoint] = list() # the list of waypoints
        self.nonrepeat_waypoints: list[Waypoint] = list() # a copy of the list
        self.repeat_waypoints: list[Waypoint] = list() # the list of waypoints when restarting the run
        self.dynamic_aim = False # whether aiming from the deposit is permitted

    def aim_enable(self):
        '''
        Enable aim assist for the plan. This overwrites the default plan to go in a straight line, if the initial robot position is at an angle different to expected
        '''
        self.dynamic_aim = True

    def aim_assist_on(self, x: float, y: float, theta: float):
        '''
        Recalculates the path with the aim assist on.

        Parameters:
            x (float): the current x position
            y (float): the current y position
            theta (float): the current angle
        '''
        straight_aim = StraightLineWaypointSequence()
        straight_aim.aim_line(x, y, theta)
        self.waypoints = straight_aim.waypoints.copy()

    def aim_assist_off(self):
        '''
        Recalculates the path with the aim assist off.
        '''
        self.nonrepeat_waypoints.copy()

    def plan_to_deposit(self, current_x: float, current_y: float, current_theta: float):
        '''
        Plans to the deposit chamber.
            1. Adds a waypoint at the current position but with corrected angle
            1. Adds a waypoint to just below the deposit chamber
            2. Adds a waypoint to next to the deposit chamber
            3. Adds a waypoint to the current position
        
        Parameters:
            current_x (float): the current x position of the robot
            current_y (float): the current y position of the robot
            current_that (float): the current angle of the robot
        '''

        # calculate the return angle after going to the helper waypoint
        return_angle = atan2(
            current_y - DepositHelperWaypoint.DEPOSIT_HELPER_Y,
            current_x - DepositHelperWaypoint.DEPOSIT_HELPER_X)

        # calculate the attack angle to go towards the helper waypoint
        return_angle_2 = atan2(
            - (current_y - DepositHelperWaypoint.DEPOSIT_HELPER_Y),
            - (current_x - DepositHelperWaypoint.DEPOSIT_HELPER_X))

        distance_to_return_angle = wrap_to_pi(return_angle - current_theta)
        distance_to_return_angle_2 = wrap_to_pi(return_angle_2 - current_theta)

        immediate_rotate_angle = return_angle if abs(distance_to_return_angle_2) > abs(distance_to_return_angle) else return_angle_2

        # insert the return waypoint
        self.waypoints.insert(0, Waypoint(current_x, current_y, current_theta, resumeExtraction=True))

        # insert the deposit waypoint, sandwiched by the helpers
        self.waypoints.insert(0, DepositHelperWaypoint(heading=immediate_rotate_angle)) ### coming out of deposit
        self.waypoints.insert(0, DepositWaypoint()) ##### deposit
        self.waypoints.insert(0, DepositHelperWaypoint()) ### coming into deposit

        # insert the spin waypoint to do immediately
        self.waypoints.insert(0, Waypoint(current_x, current_y, immediate_rotate_angle))

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
        if ((current_theta < pi/4) and (current_theta > -pi/4)) or \
            ((current_theta > 3*pi/4) and (current_theta < -3*pi/4)):
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
            emergency_waypoint = Waypoint(
                RobotGeometry.LENGTH/2 + RobotGeometry.EMERGENCY_PADDING, 
                current_y, 
                -pi if (current_theta < -pi/2 or current_theta > pi/2) else 0
                )
        elif emergency_side == right_emergency:
            emergency_waypoint = Waypoint(
                2000 - RobotGeometry.LENGTH/2 - RobotGeometry.EMERGENCY_PADDING, 
                current_y, 
                0 if (current_theta < pi/2 and current_theta > -pi/2) else -pi
                )
        elif emergency_side == up_emergency:
            emergency_waypoint = Waypoint(
                current_x, 
                2000 - RobotGeometry.LENGTH/2 - RobotGeometry.EMERGENCY_PADDING, 
                pi/2 if (current_theta > 0) else -pi/2
                )
        else:
            emergency_waypoint = Waypoint(
                current_x, 
                RobotGeometry.LENGTH/2 + RobotGeometry.EMERGENCY_PADDING, 
                -pi/2 if (current_theta < 0) else pi/2
                )

        # add waypoint
        self.waypoints.insert(0, Waypoint(current_x, current_y, current_theta, resumeExtraction=True))
        self.waypoints.insert(0, emergency_waypoint)

        # add immediate rotation waypoint
        self.waypoints.insert(0, Waypoint(current_x, current_y, emergency_waypoint.heading))

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
        '''
        Trigger the planner to restart the waypoint sequence.
        '''
        self.waypoints = self.repeat_waypoints.copy()

class SnakeWaypointSequence(WaypointSequence):
    '''
    A sequence of waypoints based on a snake search.
    '''

    BORDER_PADDING: float = RobotGeometry.RADIUS + RobotGeometry.PADDING
    ENV_LENGTH: float = 2000
    ENV_WIDTH: float = 2000

    def __init__(self,
                 points_per_line: int = 2,
                 snake_spacing = RobotGeometry.DIGGER_WIDTH,
                 theta_agnostic=False):
        super().__init__()

        # calculate the y coordinates
        y_spacing: float = (SnakeWaypointSequence.ENV_LENGTH - 2*SnakeWaypointSequence.BORDER_PADDING)/(points_per_line - 1)
        y_coords: list[float] = [SnakeWaypointSequence.BORDER_PADDING + i*y_spacing for i in range(points_per_line)]
        reversed_y_coords: list[float] = y_coords[::-1]

        # calculate the x coordinates
        num_lines: int = math.ceil((SnakeWaypointSequence.ENV_WIDTH - 2*SnakeWaypointSequence.BORDER_PADDING)/(snake_spacing))
        x_coords: list[float] = [min(SnakeWaypointSequence.BORDER_PADDING + i*snake_spacing, SnakeWaypointSequence.ENV_WIDTH - SnakeWaypointSequence.BORDER_PADDING) for i in range(num_lines)]

        for i, x in enumerate(x_coords):
            for j, y in enumerate(reversed_y_coords if i % 2 else y_coords): # toggle the direction of each line
                if (j != len(y_coords) - 1): # if not at the end
                    target_heading = pi/2 if not i % 2 else -pi/2
                else:
                    target_heading = 0

                waypoint = Waypoint(
                    x, y,
                    heading = None if theta_agnostic else target_heading,
                )
                self.waypoints.append(waypoint)

        self.repeat_waypoints = self.waypoints.copy()
        self.waypoints.pop(0)
        self.nonrepeat_waypoints = self.waypoints.copy()

class SpiralWaypointSequence(WaypointSequence):
    '''
    A sequence of waypoints based on a snake search.
    '''

    BORDER_PADDING: float = RobotGeometry.RADIUS + RobotGeometry.PADDING
    ENV_LENGTH: float = 2000
    ENV_WIDTH: float = 2000

    def __init__(self,
                 line_spacing: float,
                 point_spacing: float = 2000,
                 chamfer_size: Optional[float] = 200):
        super().__init__()

        # define the initial position
        y_initial = SpiralWaypointSequence.BORDER_PADDING
        x_initial = SpiralWaypointSequence.BORDER_PADDING

        # dynamically update the bounding box of the spiral
        y_min = y_initial - line_spacing
        y_max = SpiralWaypointSequence.ENV_LENGTH - SpiralWaypointSequence.BORDER_PADDING
        x_min = x_initial
        x_max = SpiralWaypointSequence.ENV_WIDTH - SpiralWaypointSequence.BORDER_PADDING

        # iterators for moving vertically (j) and horizontally (i)
        i = 0
        j = 0
        while True:
            while True:
                x = x_min
                
                # move along the left edge until reaching the top
                y = min(y_min + chamfer_size + point_spacing*j, y_max - chamfer_size)

                # cut corner if at the end
                if y == y_max - chamfer_size:
                    target_heading = pi/4 if chamfer_size else 0
                else:
                    target_heading = pi/2

                # if no chamfers, do not add the first waypoint
                if chamfer_size or j != 0:
                    waypoint = Waypoint(x, y, heading=target_heading)
                    self.waypoints.append(waypoint)

                # stop moving along the edge when we reach the top
                if y == y_max - chamfer_size:
                    break

                j += 1

            # shift the spiral inwards
            y_min = y_min + line_spacing

            while True:
                y = y_max

                # move along the top edge until reaching the top
                x = min(x_min + chamfer_size + point_spacing*i, x_max - chamfer_size)

                # cut corner if at end
                if x == x_max - chamfer_size:
                    target_heading = -pi/4 if chamfer_size else -pi/2
                else:
                    target_heading = 0

                # if no chamfers, do not add the first waypoint
                if chamfer_size or i != 0:
                    waypoint = Waypoint(x, y, heading=target_heading)
                    self.waypoints.append(waypoint)

                # stop moving along the edge if we reach the right side
                if x == x_max - chamfer_size:
                    break

                i += 1

            # shift spiral inwards
            x_min = x_min + line_spacing

            # reset iterators
            i = 0
            j = 0

            while True:
                x = x_max

                # move along right edge until reaching the bottom
                y = max(y_max - chamfer_size - point_spacing*j, y_min + chamfer_size)

                # cut corner if at end
                if y == y_min + chamfer_size:
                    target_heading = -3*pi/4 if chamfer_size else pi
                else:
                    target_heading = -pi/2

                # if no chamfers, do not add the first waypoint
                if chamfer_size or j != 0:
                    waypoint = Waypoint(x, y, heading=target_heading)
                    self.waypoints.append(waypoint)

                # stop moving along the edge if we reach the bottom
                if y == y_min + chamfer_size:
                    break

                j += 1

            # shift spiral inwards
            y_max = y_max - line_spacing

            while True:
                y = y_min

                # move along bottom edge until reaching the left
                x = max(x_max - chamfer_size - point_spacing*i, x_min + chamfer_size)

                # cut corner if at end
                if x == x_min + chamfer_size:
                    target_heading = 3 * pi/4 if chamfer_size else pi/2
                else:
                    target_heading = pi

                # if no chamfers, do not add first waypoint
                if chamfer_size or i != 0:
                    waypoint = Waypoint(x, y, heading=target_heading)
                    self.waypoints.append(waypoint)

                # stop moving along the edge if we reach the left
                if x == x_min + chamfer_size:
                    break

                i += 1

            # shift spiral inwards
            x_max = x_max - line_spacing

            # reset iterators
            i = 0
            j = 0

            # stop before the spiral gets too tight
            if (x_max - x_min) < 2*line_spacing or (y_max - y_min) < 2*line_spacing:
                break

        # copy waypoints over
        self.repeat_waypoints = self.waypoints.copy()
        if chamfer_size:
            self.waypoints.pop(0)
        self.nonrepeat_waypoints = self.waypoints.copy()

class RectangleWaypointSequence(WaypointSequence):
    '''
    A sequence of waypoints which is just a rectangle
    '''

    def __init__(self, length_x, length_y, origin_x, origin_y, num_loops, angle_agnostic=False):
        super().__init__()

        for _ in range(num_loops):
            # add all four corners
            self.waypoints.append(Waypoint(
                origin_x,
                origin_y + length_y,
                heading=None if angle_agnostic else 0))
            self.waypoints.append(Waypoint(
                origin_x + length_x,
                origin_y + length_y,
                heading=None if angle_agnostic else -pi/2))
            self.waypoints.append(Waypoint(
                origin_x + length_x,
                origin_y,
                heading=None if angle_agnostic else pi))
            self.waypoints.append(Waypoint(
                origin_x,
                origin_y,
                heading=None if angle_agnostic else pi/2))

        self.repeat_waypoints = self.waypoints.copy()
        self.nonrepeat_waypoints = self.waypoints.copy()

class ShiftingWindowSequence(WaypointSequence):
    '''
    A waypoint sequence which consists of a rectangle which keeps shifting over
    '''

    BORDER_PADDING: float = RobotGeometry.RADIUS + RobotGeometry.PADDING # the padding around the edge
    ENV_LENGTH: float = 2000 # the length of the environment
    ENV_WIDTH: float = 2000 # the width of the environment

    def __init__(self, line_spacing):
        super().__init__()

        # the min and max y coord
        min_y = ShiftingWindowSequence.BORDER_PADDING
        max_y = ShiftingWindowSequence.ENV_LENGTH - ShiftingWindowSequence.BORDER_PADDING

        # the position of the origin, dynamically updated as the window shifts
        origin_x = ShiftingWindowSequence.BORDER_PADDING

        while origin_x < 1000: # while not beyond half way
            # add the four corners
            self.waypoints.append(Waypoint(origin_x, min_y, pi/2))
            self.waypoints.append(Waypoint(origin_x, max_y, 0))
            self.waypoints.append(Waypoint(origin_x + 1000 - ShiftingWindowSequence.BORDER_PADDING, max_y, -pi/2))
            self.waypoints.append(Waypoint(origin_x + 1000 - ShiftingWindowSequence.BORDER_PADDING, min_y, pi))

            # shift the window over
            origin_x += line_spacing

        # set the waypoints
        self.repeat_waypoints = self.waypoints.copy()
        self.waypoints.pop(0)
        self.nonrepeat_waypoints = self.waypoints.copy()

class StraightLineWaypointSequence(WaypointSequence):
    '''
    A sequence of waypoints which moves in a straight line from the deposit.
    '''
    def __init__(self):
        super().__init__()

        self.waypoints.append(Waypoint(0, 0, 0))

    def aim_line(self, current_x: float, current_y: float, current_theta: float):
        '''
        Sets the waypoint to be straight forward.

        Parameters:
            current_x (float): the current x position
            current_y (float): the current y position
            current_theta (float): the current angle
        '''
        self.waypoints = []

        # calculate the coordinates of the end of the line
        endpoint_x = max(min(2000, current_x + 2000*math.cos(current_theta)), 0)
        endpoint_y = max(min(2000, current_y + 2000*math.sin(current_theta)), 0)

        # redefine the waypoint set
        self.waypoints = [Waypoint(endpoint_x, endpoint_y, current_theta)]
        self.repeat_waypoints = self.waypoints.copy()
        self.nonrepeat_waypoints = self.waypoints.copy()

class MockLocalisationWaypointSequence(WaypointSequence):
    '''
    A waypoint to force the robot into a circle if using mock localisation
    '''
    def __init__(self):
        super().__init__()

        # add waypoint
        self.waypoints.append(Waypoint(500, 500, heading=None))
        self.repeat_waypoints = self.waypoints.copy()
        self.nonrepeat_waypoints = self.waypoints.copy()

class Debogger(ABC):
    '''
    A class which attempts to debog the robot.
    '''
    def __init__(self) -> None:
        self.paused = False
        self.enabled = False

    @abstractmethod
    def is_bogged(self, current_x: float, current_y: float, current_theta: float) -> bool:
        '''
        Determines whether the robot is currently bogged

        Parameters:
            current_x (float): the current x coord
            current_y (float): the current y coord
            current_theta (float): the current angle
        Returns:
            True if the robot is bogged
        '''

    @abstractmethod
    def has_unbogged(self, current_x: float, current_y: float, current_theta: float) -> bool:
        '''
        Determines whether the robot has unbogged itself

        Parameters:
            current_x (float): the current x coord
            current_y (float): the current y coord
            current_theta (float): the current angle
        Returns:
            True if the robot is unbogged 
        '''

    @abstractmethod
    def attempt_debog(self, planner: Pathplanner):
        '''
        Attemps to modify the plan to debog the robot.

        Parameters:
            planner (Pathplanner): the plan to modify.
        '''

    @abstractmethod
    def stop_debog(self, planner: Pathplanner):
        '''
        Stops all pathplanning
        '''

    @abstractmethod
    def pause_debogger(self):
        '''
        Pauses the debogger.
        '''

    @abstractmethod
    def unpause_debogger(self):
        '''
        Resumes the debogger.
        '''

    def disable_debogger(self):
        '''
        Disables the debogger. A stricter condition than pausing.
        '''
        self.enabled = False

    def enable_debogger(self):
        '''
        Enables the debogger. A stricter condition than pausing.
        '''
        self.enabled = True

    @abstractmethod
    def delay(self, delay_time: float):
        '''
        Delay the debogger to only activate after a certain time.

        Parameters:
            delay_time (float): the time in seconds to delay forward.
        '''

class NoDebogger(Debogger):
    '''
    A class representing debogging being disabled.
    '''

    def is_bogged(self, current_x: float, current_y: float, current_theta: float) -> bool:
        return False
    
    def has_unbogged(self, current_x: float, current_y: float, current_theta: float) -> bool:
        return False

    def attempt_debog(self, planner: Pathplanner):
        pass

    def stop_debog(self, planner: Pathplanner):
        pass

    def pause_debogger(self):
        pass

    def unpause_debogger(self):
        pass

    def delay(self, delay_time: float):
        pass

    def disable_debogger(self):
        pass

    def enable_debogger(self):
        pass

class ActiveDebogger(Debogger):
    '''
    A class representing debogging being enabled.
    '''

    EPSILON_X: float # the distance the robot needs to stay within to be considered bogged [mm]
    EPSILON_THETA: float # the angle the robot needs to stay within to be considered bogged [rad]
    PATIENCE: float # the timeout to consider the robot bogged [s]
    REVERSE_DISTANCE: float # how far back the robot should reverse when bogged [mm]
    ANGLE_DEVIATION: float # the angle deviation the robot should make after reversing [rad]
    DEBOG_EPSILON_THETA: float # the distance the robot needs to move to be considered unbogged [rad]
    DEBOG_EPSILON_X: float # the distance the robot needs to move to be considered unbogged [mm]

    def __init__(self,
                 epsilon_x: float=30,
                 epsilon_theta: float=10,
                 debog_epsilon_x: float=60,
                 debog_epsilon_theta: float=10,
                 patience: float=2,
                 reverse_distance: float=400,
                 angle_deviation: float=pi/6):
        '''
        Parameters:
            epsilon_x (float): the distance the robot needs to stay within to be considered bogged [mm]
            epsilon_theta (float): the angle the robot needs to stay within to be considered bogged [rad]
            patience (float): the timeout to consider the robot bogged [s]
            reverse_distance (float): how far back the robot should reverse when bogged [mm]
            angle_deviation (float): the angle deviation the robot should make after reversing [rad]
        '''
        super().__init__()

        # update parameters of the debogger
        ActiveDebogger.EPSILON_X = epsilon_x
        ActiveDebogger.EPSILON_THETA = epsilon_theta
        ActiveDebogger.PATIENCE = patience
        ActiveDebogger.REVERSE_DISTANCE = reverse_distance
        ActiveDebogger.ANGLE_DEVIATION = angle_deviation

        ActiveDebogger.DEBOG_EPSILON_X = debog_epsilon_x
        ActiveDebogger.DEBOG_EPSILON_THETA = debog_epsilon_theta

        # the reference position to consider bogging from
        self.last_debog_position: tuple[float, float, float] = None

        self.last_debog_time: float = None # the timestamp to consider bogging from
        self.debogger_enabled: bool = True # whether bogging is disabled or not
        self.debogger_pause_time: float = None # the time that the bogging was disabled at

        self.bogged_start_position: tuple[float, float, float] = None # starting position of bog

    def is_bogged(self, current_x: float, current_y: float, current_theta: float) -> bool:
        if self.bogged_start_position is not None:
            return False

        # update parameters if this is the first debog cycle
        if self.last_debog_position is None:
            self.last_debog_position = (current_x, current_y, current_theta)
        if self.last_debog_time is None:
            self.last_debog_time = time.time()

        # calculate how far the robot has moved from the reference
        debog_x, debog_y, debog_theta = self.last_debog_position
        x_deviation = math.sqrt((current_x - debog_x)**2 + (current_y - debog_y)**2)
        theta_deviation = abs(wrap_to_pi(current_theta - debog_theta))

        if (x_deviation > ActiveDebogger.EPSILON_X or theta_deviation > ActiveDebogger.EPSILON_THETA):
            # we have not bogged, update the reference time and position
            self.last_debog_time = time.time()
            self.last_debog_position = (current_x, current_y, current_theta)
            return False
        elif time.time() - self.last_debog_time > ActiveDebogger.PATIENCE:
            # we have bogged
            return True
        return False
    
    def has_unbogged(self, current_x: float, current_y: float, current_theta: float) -> bool:
        if self.bogged_start_position is None:
            return False
        debog_x, debog_y, debog_theta = self.bogged_start_position
        x_deviation = max(abs(current_x - debog_x), abs(current_y - debog_y))
        theta_deviation = abs(wrap_to_pi(current_theta - debog_theta))

        if (x_deviation > ActiveDebogger.DEBOG_EPSILON_X or theta_deviation > ActiveDebogger.DEBOG_EPSILON_THETA):
            # then we have unbogged
            return True
        else:
            return False

    def stop_debog(self, planner: Pathplanner):
        print("Unbogged. End position is", (planner.current_x, planner.current_y, planner.current_theta))
        self.bogged_start_position = None
        planner.extraction_strategy.unpause_extraction()

    def attempt_debog(self, planner: Pathplanner):
        self.bogged_start_position = (planner.current_x, planner.current_y, planner.current_theta)
        print("Detected a bog. Starting position is", self.bogged_start_position)
        planner.extraction_strategy.pause_extraction()
        # # determine which direction is reversing
        # reverse = -1 if planner.desired_velocity > 0 else 1
        # print("Reversing: ", reverse)

        # # calculate where to reverse to, ensuring to stay within the bounds of the environment
        # new_x = min(max(planner.current_x + reverse*math.cos(planner.current_theta)*ActiveDebogger.REVERSE_DISTANCE, RobotGeometry.RADIUS), 2000 - RobotGeometry.RADIUS)
        # new_y = min(max(planner.current_y + reverse*math.sin(planner.current_theta)*ActiveDebogger.REVERSE_DISTANCE, RobotGeometry.RADIUS), 2000 - RobotGeometry.RADIUS)

        # # the robot can deviate in either direction after reversing
        # theta_option_one = wrap_to_pi(planner.current_theta + ActiveDebogger.ANGLE_DEVIATION)
        # theta_option_two = wrap_to_pi(planner.current_theta - ActiveDebogger.ANGLE_DEVIATION)

        # # calculate where both deviation options will lead
        # distance_option_one = (abs(1000 - (new_x + math.cos(theta_option_one))))**2 + (abs(1000 - (new_y + math.sin(theta_option_one))))**2
        # distance_option_two = (abs(1000 - (new_x + math.cos(theta_option_two))))**2 + (abs(1000 - (new_y + math.sin(theta_option_two))))**2

        # # choose the deviation option that is further away from the edge
        # new_heading = theta_option_one if distance_option_one < distance_option_two else theta_option_two

        # # set the debog waypoint
        # new_waypoint = Waypoint(new_x, new_y, heading=new_heading)
        # planner.waypoints.waypoints.insert(0, new_waypoint)
        # planner.current_waypoint = new_waypoint
        # planner.previous_waypoint = (planner.current_x, planner.current_y)

        # print("Current position: ", (planner.current_x, planner.current_y))
        # print("New waypoint: ", planner.current_waypoint.coords)

        # # update debog parameters
        # self.last_debog_time = time.time()
        # self.last_debog_position = (planner.current_x, planner.current_y, planner.current_theta)

        # # flag to update the controller path
        # planner.update_controller_path = True

    def pause_debogger(self):
        '''
        Pause the debogger
        '''
        if not self.paused and self.enabled:
            self.paused = True
            self.debogger_pause_time = time.time()

    def unpause_debogger(self):
        '''
        Resume the debogger
        '''
        # shift the debog time to account for time already measured
        if self.paused and self.enabled:
            if self.debogger_pause_time is not None and self.last_debog_time is not None:
                self.last_debog_time = time.time() - (self.debogger_pause_time - self.last_debog_time)

    def delay(self, delay_time):
        self.last_debog_time = time.time() + delay_time
