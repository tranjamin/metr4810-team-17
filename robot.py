from math import sin, cos, atan2, pi, copysign
import socket
import requests
import numpy as np
from abc import ABC, abstractmethod
from typing import *

# control signal addresses
ROBOT_PWM_ADDRESS: str = "lhs={}&rhs={}"
ROBOT_CONTROL_ADDRESS: str = "control?command={}"

# maximum pwm to send
PWM_MAX: float = 100.0

class Robot(ABC):
    '''
    A robot which executes actions.
    '''

    LOCALISATION_DPS = 3 # the decimal precision to send localisation commands in
    LOCALISATION_WL = 7 # the total word length of a localisation parameter
    
    def __init__(self, ip: str):
        '''
        Parameters:
            ip (str): the ip address to the robot is listening on
        '''
        self.ip = ip
        self.pwm_left = 0
        self.pwm_right = 0

    def send_control_action(self, v: float, omega: float, do_print: bool =False):
        '''
        Control the robot's movements

        Parameters:
            v (float): the forward linear velocity
            omega (float): the counterclockwise angular velocity
            do_print (bool): prints out the v and omega sent
        '''

        # transform the input parameters to left and right PWMs
        mapping_matrix = np.matrix([[1, -1], [1, 1]])
        control_vector = mapping_matrix * np.array([[v],[omega]])
        if do_print:
            print(f"v: {v}, omega: {omega}")

        # set the pwm
        pwm_left, pwm_right = np.ravel(control_vector).tolist()
        self.set_pwm(pwm_left, pwm_right, do_print=do_print)

    def set_pwm(self, left: float, right: float, do_print=False):
        '''
        Sends the required PWM signals to the robot

        Parameters:
            left (float): the left PWM to send
            right (float): the right PWM to send
            do_print (bool): prints out the request sent
        '''
        # round and saturate pwms
        self.pwm_left = round(max(min(left, PWM_MAX), -PWM_MAX), Robot.LOCALISATION_DPS)
        self.pwm_right = round(max(min(right, PWM_MAX), -PWM_MAX), Robot.LOCALISATION_DPS)

        # format to 7dp
        command = ROBOT_PWM_ADDRESS.format(
            str(self.pwm_left).zfill(Robot.LOCALISATION_WL), 
            str(self.pwm_right).zfill(Robot.LOCALISATION_WL))
        if do_print:
            print(command)
        
        # send command
        self.send_command(command)

    @abstractmethod
    def send_command(self, command: str):
        '''
        Sends a localisation command to the robot

        Parameters:
            command (str): the request string to send
        '''
        pass

    @abstractmethod
    def send_control_command(self, command: str):
        '''
        Send a control command of the robot.

        Parameters:
            command (str): the request string to send
        '''
        pass
  
class RobotTCP(Robot):
    '''
    A type of robot which sends requests over TCP
    '''

    LOCALISATION_ENDPOINT: str = "localisation" # the endpoint to send localisation commands to
    CONTROL_ENDPOINT: str = "control" # the endpoint to send control commands to

    def send_command(self, command: str):
        try:
            url = f"http://{self.ip}/{RobotTCP.LOCALISATION_ENDPOINT}?{command}"
            requests.get(url, timeout=(0.005, 0.001))
        except requests.exceptions.ReadTimeout: 
            pass
        except requests.exceptions.Timeout:
            pass
    
    def send_control_command(self, command: str):
        try:
            url = f"http://{self.ip}/{RobotTCP.CONTROL_ENDPOINT}?{command}"
            requests.get(url, timeout=(0.8, 0.8))
        except requests.exceptions.ReadTimeout: 
            pass
        except requests.exceptions.Timeout:
            pass

class RobotUDP(Robot):
    '''
    A type of robot which sends requests over UDP
    '''

    UDP_PORT: int = 80 # the port to send the UDP request over
    LOCALISATION_PREFIX: str = "L" # the first character to signify a localisation command
    CONTROL_PREFIX: str = "C" # the first character to signify a control command
    ENCODING: str = "utf-8" # the encoding to use

    def __init__(self, ip: str):
        super().__init__(ip)

        # initialise socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send_command(self, command: str):
        try:
            self.sock.sendto(bytes(RobotUDP.LOCALISATION_PREFIX + command, RobotUDP.ENCODING), (self.ip, RobotUDP.UDP_PORT))
        except OSError:
            print("Failed to send command")
    
    def send_control_command(self, command: str):
        try:
            self.sock.sendto(bytes(RobotUDP.CONTROL_PREFIX + command, RobotUDP.ENCODING), (self.ip, RobotUDP.UDP_PORT))
        except OSError:
            print("Failed to send command")

class Controller(ABC):
    '''
    A class which controls the movement of a robot from waypoint to waypint.
    '''
    def __init__(self):
        self.reached_goal = False # whether the goal waypoint has been reached
        self.p0: np.ndarray # the starting coordinate
        self.p1: np.ndarray # the end coordinate
        self.theta_target: float # the target angle, counterclockwise from positive x

    def set_path(self, p0: tuple[float, float], p1: tuple[float, float], theta_target: float):
        """
        Register a new path with this controller.

        Parameters:
            p0 (tuple[float, float]): initial point on line
            p1 (tuple[float, float]): goal position
            theta_target (float): final orientation
        """
        self.p0 = np.array(p0)
        self.p1 = np.array(p1)
        self.theta_target = theta_target
        self.reached_goal = False

    def has_reached_goal(self) -> bool:
        '''
        Determines whether the controller goal has been reached

        Returns:
            (bool): whether the goal has been reached
        '''
        return self.reached_goal

    @abstractmethod
    def get_control_action(self, x: float, y: float, theta: float, **kwargs) -> tuple[float, float]:
        '''
        Calculates the required linear and angular velocity to move along the controller path

        Parameters:
            x (float): the current x position
            y (float): the current y position
            theta (float): the current angle, counterclockwise from positive x

        Returns:
            (v, omega) (tuple[float, float]): the desired linear and angular velocity
        '''
        pass

class SpinController(Controller):
    '''
    A controller which spins on the spot
    '''
    def __init__(self, k_angle: float, k_v: float, angle_tolerance: float):
        '''
        Parameters:
            k_angle: the angle proportional gain
            k_v: the velocity proportional gain
            angle_tolerance: the acceptable closeness to the target angle
        '''
        self.k_angle = k_angle
        self.k_v = k_v
        self.angle_tolerance = angle_tolerance
        super().__init__()
    
    def get_control_action(self, x: float, y: float, theta: float) -> tuple[float, float]:
        r = np.array([x, y]) # robot position vector

        # calculate actual angle difference (now to target_theta)
        wr_hat = np.array([cos(self.theta_target), sin(self.theta_target)])
        heading = np.array([cos(theta), sin(theta)])

        # sin from cross product
        sin_angle_error = heading[0] * wr_hat[1] - heading[1] * wr_hat[0]

        # cos from dot product
        cos_angle_error = np.dot(heading, wr_hat)

        # calculate the wrapped angle
        angle_error = atan2(sin_angle_error, cos_angle_error)
        
        omega = copysign(self.k_angle, angle_error)

        # ensure we move in the correct direction
        r_to_goal = self.p1 - r
        r_to_goal = r_to_goal / np.linalg.norm(r_to_goal)
        v = self.k_v * np.dot(wr_hat, heading)

        if abs(angle_error) < self.angle_tolerance:
            # stop moving if goal has been reached
            self.reached_goal = True
            return 0, 0
        return v, omega

class FowardController(Controller):
    """
    Controller to make robot track along a straight path
    """

    def __init__(self, k_angle: float, k_v: float, w: float, 
                 goal_tolerance: float, reversing_allowed: bool=True):
        '''
        Parameters:
            k_angle (float): the angle proportional gain
            k_v (float): the velocity proportional gain
            w (float): the line equation parameter
            goal_tolerance (float): the acceptable closeness to the target angle
            reversing_allowed (bool): whether moving backwards is allowed
        '''
        self.k_angle = k_angle
        self.k_v = k_v
        self.w = w
        self.goal_tolerance = goal_tolerance
        self.reversing_allowed = reversing_allowed
        super().__init__()

    def get_control_action(self, x: float, y: float, theta: float) -> tuple[float, float]:
        r = np.array([x, y]) # robot position vector
        d = self.p1 - self.p0
        k = 1/np.linalg.norm(d)**2 * np.dot(d, r - self.p0)
        
        # location of closest point on path to the robot
        s = k*d + self.p0

        # virtual waypoint
        W = s + self.w * (self.p1 - s)

        r_to_waypoint = W - r

        # calculate actual angle difference

        wr_hat = r_to_waypoint / np.linalg.norm(r_to_waypoint)
        heading = np.array([cos(theta), sin(theta)])

        # sin from cross product
        sin_angle_error = heading[0] * wr_hat[1] - heading[1] * wr_hat[0]

        # cos from dot product
        cos_angle_error = np.dot(heading, wr_hat)

        # calculate the wrapped angle
        angle_error = atan2(sin_angle_error, cos_angle_error)

        if self.reversing_allowed:
            other_possibility = -wrapToPi(pi - angle_error)
            if abs(other_possibility) < abs(angle_error):
                # it would be easier to reverse
                angle_error = other_possibility
        
        omega = self.k_angle * angle_error

        # ensure we move in the correct direction
        v = self.k_v * np.dot(wr_hat, heading)
        if not self.reversing_allowed:
            # wait until we're facing in the correct direction before moving
            v = max(v, 0)

        # check if goal has been reached
        if np.linalg.norm(r - self.p1) < self.goal_tolerance or self.reached_goal:
            self.reached_goal = True
            # if goal has been reached, don't move
            return 0, 0

        return v, omega

class LineFollowerController(Controller):
    '''
    Class to rpresent a controller that guides a robot along both parts
    of a straight line path
    1. The straight section
    2. The final spin to the desired end orientation
    '''
    def __init__(self, forward_controller: FowardController, spin_controller: SpinController):
        '''
        Parameters:
            forward_controller (ForwardController): the forward controller object to use
            spin_controller (SpinController): the spin controller object to use
        '''
        self.forward_controller = forward_controller
        self.spin_controller = spin_controller
        super().__init__()
        self.phase_2 = False # whether we are in the spin part of the controller
        self.theta_agnostic = False # if we care about target angle or not

    def set_path(self, p0: tuple[float, float], p1: tuple[float, float], theta_target: float):
        # set the forward and spin paths
        self.forward_controller.set_path(p0, p1, theta_target)
        self.spin_controller.set_path(p0, p1, theta_target)

        # reset to linear phase
        self.phase_2 = False

        # make angle agnostic if there is no target angle
        self.theta_agnostic = theta_target is None

        super().set_path(p0, p1, theta_target)

    def get_control_action(self, x: float, y: float, theta: float, 
                           on_spin_start: Optional[Callable] = None,
                           on_spin_end: Optional[Callable] = None) -> tuple[float, float]:
        '''
        Calculates the required linear and angular velocity to move along the controller path

        Parameters:
            x (float): the current x position
            y (float): the current y position
            theta (float): the current angle, counterclockwise from positive x
            on_spin_start (Callable): an optional function to call when starting to spin
            on_spin_end (Callable): an optional function to call when finishing the spin

        Returns:
            (v, omega) (tuple[float, float]): the desired linear and angular velocity
        '''

        if not self.phase_2 and not self.forward_controller.has_reached_goal():
            # keep progressing along the foward path
            return self.forward_controller.get_control_action(x, y, theta)
        
        if self.spin_controller.has_reached_goal() or (self.forward_controller.has_reached_goal() and self.theta_agnostic):
            # goal has been reached
            print("CONTROLLER REACHED GOAL")
            self.reached_goal = True

            if on_spin_end is not None:
                on_spin_end()
        if not self.phase_2 and not self.theta_agnostic:
            # move to spin controller
            print("SWITCHING TO SPIN CONTROLLER")
            self.phase_2 = True

            if on_spin_start is not None:
                on_spin_start()

        if self.phase_2:
            # progress along spin controller
            return self.spin_controller.get_control_action(x, y, theta)
        else:
            # stop if goal has been reached
            return 0, 0

def wrapToPi(angle: float) -> float:
    '''
    Wraps an angle to the range -pi/2, pi/2

    Parameters:
        angle (float): the angle, in radians
    Returns:
        (float): the wrapped angle, in radians
    '''
    return (angle + pi) % (2 * pi) - pi