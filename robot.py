'''
Holds functionality associated with interfacing over the communications channel

Classes:
    Robot(): an abstract base class for different robots.
        RobotTCP(): a robot which communicates via TCP
        RobotUDP(): a robot which communicates via UDP
        RobotMixed(): a robot which communicates via both TCP and UDP
'''

from abc import ABC, abstractmethod
from math import copysign
import socket
import requests
import numpy as np

class Robot(ABC):
    '''
    Represents the robot being communicated to.
    '''

    # control signal addresses
    ROBOT_PWM_ADDRESS: str = "lhs={}&rhs={}"
    ROBOT_CONTROL_ADDRESS: str = "control?command={}"

    LOCALISATION_DPS = 3 # the decimal precision to send localisation commands in
    LOCALISATION_WL = 7 # the total word length of a localisation parameter

    def __init__(self, ip: str, min_pwm: float=0, max_pwm: float=100, **kwargs):
        '''
        Parameters:
            ip (str): the ip address to the robot is listening on
            min_pwm (float): the lower pwm cutoff. Values below this will be saturated
            max_pwm (float): the upper pwm cutoff. Values above this will be saturated
        '''

        # ensure inputs are valid
        assert min_pwm >= 0.0 and min_pwm <= 100.0
        assert max_pwm >= 0.0 and max_pwm <= 100.0

        self.ip = ip
        self.pwm_left = 0
        self.pwm_right = 0

        self.max_pwm = max_pwm
        self.min_pwm = min_pwm

    def send_control_action(self, v: float, omega: float, do_print: bool=False):
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

    def set_pwm(self, left: float, right: float, do_print: bool=False):
        '''
        Sends the required PWM signals to the robot

        Parameters:
            left (float): the left PWM to send
            right (float): the right PWM to send
            do_print (bool): prints out the request sent
        '''
        # rescale PWM to be within correct range
        abs_left = self.min_pwm + abs(left)/100*(self.max_pwm - self.min_pwm)
        abs_right = self.min_pwm + abs(right)/100*(self.max_pwm - self.min_pwm)
        self.pwm_left = copysign(abs_left, left)
        self.pwm_right = copysign(abs_right, right)

        # round PWM to required precision
        self.pwm_left = round(self.pwm_left, Robot.LOCALISATION_DPS)
        self.pwm_right = round(self.pwm_right, Robot.LOCALISATION_DPS)

        # format to 7dp
        command = Robot.ROBOT_PWM_ADDRESS.format(
            str(self.pwm_left).zfill(Robot.LOCALISATION_WL),
            str(self.pwm_right).zfill(Robot.LOCALISATION_WL)
            )
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

    @abstractmethod
    def send_control_command(self, command: str):
        '''
        Send a control command of the robot.

        Parameters:
            command (str): the request string to send
        '''

class RobotTCP(Robot):
    '''
    A type of robot which sends requests over TCP
    '''

    LOCALISATION_ENDPOINT: str = "localisation" # the endpoint to send localisation commands to
    CONTROL_ENDPOINT: str = "control" # the endpoint to send control commands to

    # the timeouts for sending the TCP packet for localisation
    REQUEST_TIMEOUT_TIME_LOC = 0.005
    RESPONSE_TIMEOUT_TIME_LOC = 0.001

    # the timeouts for sending the TCP packet for control
    REQUEST_TIMEOUT_TIME_CTRL = 2
    RESPONSE_TIMEOUT_TIME_CTRL = 2

    def send_command(self, command: str):
        try: # try sending the command
            url = f"http://{self.ip}/{RobotTCP.LOCALISATION_ENDPOINT}?{command}"
            requests.get(url, timeout=(RobotTCP.REQUEST_TIMEOUT_TIME_LOC, RobotTCP.RESPONSE_TIMEOUT_TIME_LOC))
        except (requests.exceptions.ReadTimeout, requests.exceptions.Timeout):
            pass

    def send_control_command(self, command: str):
        try: # try sending the command
            url = f"http://{self.ip}/{RobotTCP.CONTROL_ENDPOINT}?{command}"
            requests.get(url, timeout=(RobotTCP.REQUEST_TIMEOUT_TIME_CTRL, RobotTCP.RESPONSE_TIMEOUT_TIME_CTRL))
        except requests.exceptions.ReadTimeout:
            print("Timed out on read")
        except requests.exceptions.Timeout:
            print("Timed out on something else")

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
        try: # try to send command
            self.sock.sendto(bytes(RobotUDP.LOCALISATION_PREFIX + command, RobotUDP.ENCODING), (self.ip, RobotUDP.UDP_PORT))
        except OSError:
            print("Failed to send command")

    def send_control_command(self, command: str):
        try: # try to send command
            self.sock.sendto(bytes(RobotUDP.CONTROL_PREFIX + command, RobotUDP.ENCODING), (self.ip, RobotUDP.UDP_PORT))
        except OSError:
            print("Failed to send command")

class RobotMixed(Robot):
    '''
    A robot which sends localisation commands via UDP and control commands via TCP
    '''
    def __init__(self, ip: str):
        self.udp_robot = RobotUDP(ip)
        self.tcp_robot = RobotTCP(ip)

    def send_control_command(self, command: str):
        self.tcp_robot.send_control_command(command)

    def send_control_action(self, v: float, omega: float, do_print: bool = False):
        self.udp_robot.send_control_action(v, omega, do_print)

    def send_command(self, command: str):
        self.tcp_robot.send_command(command)
