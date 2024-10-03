import requests
import numpy as np
from math import sin, cos, atan2, pi
import socket


ROBOT_PWM_ADDRESS = "lhs={}&rhs={}"
ROBOT_LED_ADDRESS = "ledtest?led={}"
ROBOT_CONTROL_ADDRESS = "control?command={}"

PWM_MAX = 100 # can be float when sent

class Robot:
    def __init__(self, ip: str):
        self.ip = ip
        self.pwm_left = 0
        self.pwm_right = 0

    def send_control_action(self, v: float, omega: float, do_print=False):
        mapping_matrix = np.matrix([[1, 1], [1, -1]])
        control_vector = mapping_matrix * np.array([[v],[omega]])
        if do_print:
            print(f"v: {v}, omega: {omega}")
        ul, ur = np.ravel(control_vector).tolist()
        self.set_pwm(ul, ur, do_print=do_print)
    
    def set_pwm(self, left: int, right: int, do_print=False):
        self.pwm_left = round(max(min(left, 100.0), -100.0), 3)
        self.pwm_right = round(max(min(right, 100.0), -100.0), 3)
        # fill with zeros to match code on the pico
        if do_print:
            print(ROBOT_PWM_ADDRESS.format(str(self.pwm_left).zfill(7), str(self.pwm_right).zfill(7)))
        self.send_command(ROBOT_PWM_ADDRESS.format(str(self.pwm_left).zfill(7), str(self.pwm_right).zfill(7)))
    
    def control_function(self, command):
        self.send_command(ROBOT_CONTROL_ADDRESS.format(command))

    def send_command(self, command: int):
        try:
            # print("sending robot command")
            url = f"http://{self.ip}/{command}"
            requests.get(url, timeout=(0.005, 0.001))
            # Hack from: https://stackoverflow.com/questions/27021440/python-requests-dont-wait-for-request-to-finish
            # timeout means we ignore any response from the pico
        except requests.exceptions.ReadTimeout: 
            pass
        except requests.exceptions.Timeout:
            pass

class RobotUDP(Robot):
    def __init__(self, ip: str):
        super().__init__(ip)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send_command(self, command: int):
        self.sock.sendto(bytes(command, "utf-8"), (self.ip, 80))

class Controller:
    def __init__(self):
        self.reached_goal = False
        pass

    def set_path(self, p0: tuple[float, float], p1: tuple[float, float],
                 theta_target: float):
        """Register a new path with this controller.

        Args:
            p0 (tuple[float, float]): initial point on line
            p1 (tuple[float, float]): goal position
            theta_target (float): final orientation
        """
        self.p0 = np.array(p0)
        self.p1 = np.array(p1)
        self.theta_target = theta_target
        self.reached_goal = False

    def has_reached_goal(self):
        return self.reached_goal

    def get_control_action(self, x: float, y: float, theta: float):
        pass

class SpinController(Controller):
    def __init__(self, k_angle, k_v, angle_tolerance: float):
        self.k_angle = k_angle
        self.k_v = k_v
        self.angle_tolerance = angle_tolerance
        super().__init__()
    
    def get_control_action(self, x: float, y: float, theta: float):
        p0 = self.p0
        p1 = self.p1
        theta_target = self.theta_target
        r = np.array([x, y]) # robot position vector

        # calculate actual angle difference (now to target_theta)
        wr_hat = np.array([cos(theta_target), sin(theta_target)])
        heading = np.array([cos(theta), sin(theta)])

        # sin from cross product
        sin_angle_error = heading[0] * wr_hat[1] - heading[1] * wr_hat[0]

        # cos from dot product
        cos_angle_error = np.dot(heading, wr_hat)

        # we use this method instead of subtracting the two angles since
        # this avoids dumb behaviour around the wrap around point
        angle_error = atan2(sin_angle_error, cos_angle_error)
        
        omega = self.k_angle * angle_error

        # ensure we move in the correct direction
        r_to_goal = p1 - r
        r_to_goal = r_to_goal / np.linalg.norm(r_to_goal)
        v = self.k_v * np.dot(wr_hat, heading)

        if abs(angle_error) < self.angle_tolerance:
            self.reached_goal = True
            return 0, 0
        return v, omega

class FowardController(Controller):
    """Controller to make robot track along a straight path
    """
    def __init__(self, k_angle: float, k_v: float, w: float, goal_tolerance: float, reversing_allowed=True):
        self.k_angle = k_angle
        self.k_v = k_v
        self.w = w
        self.goal_tolerance = goal_tolerance
        self.reversing_allowed = reversing_allowed
        super().__init__()
    
    def get_control_action(self, x: float, y: float, theta: float) -> tuple[float, float]:
        """Generate control action to follow a waypoint that receeds along
        this controller's path. Allows for reversing it this controller's
        reversing_allowed flag has been set to true.

        Args:
            x (float): robot x location
            y (float): robot y location
            theta (float): robot heading angle

        Returns:
            tuple[float, float]: tuple of v, omega control actions
        """
        p0 = self.p0
        p1 = self.p1
        w = self.w
        r = np.array([x, y]) # robot position vector
        d = p1 - p0
        k = 1/np.linalg.norm(d)**2 * np.dot(d, r - p0)
        
        # location of closest point on path to the robot
        s = k*d + p0

        # virtual waypoint
        W = s + w * (p1 - s)

        r_to_waypoint = W - r

        # calculate actual angle difference

        wr_hat = r_to_waypoint / np.linalg.norm(r_to_waypoint)
        heading = np.array([cos(theta), sin(theta)])

        # sin from cross product
        sin_angle_error = heading[0] * wr_hat[1] - heading[1] * wr_hat[0]

        # cos from dot product
        cos_angle_error = np.dot(heading, wr_hat)

        # we use this method instead of subtracting the two angles since
        # this avoids dumb behaviour around the wrap around point
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
        if np.linalg.norm(r - p1) < self.goal_tolerance or self.reached_goal:
            self.reached_goal = True
            # if goal has been reached, don't move
            return 0, 0

        return v, omega

class LineFollowerController(Controller):
    """Class to rpresent a controller that guides a robot along both parts
    of a straight line path
    1. The straight section
    2. The final spin to the desired end orientation
    """
    def __init__(self, forward_controller: FowardController,
                 spin_controller: SpinController):
        self.forward_controller = forward_controller
        self.spin_controller = spin_controller
        super().__init__()
        self.phase_2 = False
        self.theta_agnostic = False

    def set_path(self, p0: tuple[float, float], p1: tuple[float, float], theta_target: float):
        self.forward_controller.set_path(p0, p1, theta_target)
        self.spin_controller.set_path(p0, p1, theta_target)
        self.phase_2 = False
        self.theta_agnostic = theta_target is None
        super().set_path(p0, p1, theta_target)

    def get_control_action(self, x: float, y: float, theta: float, stop_extraction: callable = None):
        if not self.phase_2 and not self.forward_controller.has_reached_goal():
            # keep progressing along the foward path
            return self.forward_controller.get_control_action(x, y, theta)
        if self.spin_controller.has_reached_goal() or (self.forward_controller.has_reached_goal() and self.theta_agnostic):
            print("CONTROLLER REACHED GOAL")
            self.reached_goal = True
        if not self.phase_2 and not self.theta_agnostic:
            print("SWITCHING TO SPIN CONTROLLER")
            self.phase_2 = True
            if stop_extraction is not None:
                stop_extraction()
        if self.phase_2:
            # print("DOING PHASE 2")
            return self.spin_controller.get_control_action(x, y, theta)
        else:
            return 0, 0

def wrapToPi(angle):
    return (angle + pi) % (2 * pi) - pi



