from abc import ABC, abstractmethod
from math import copysign
import socket
import requests
import numpy as np

# control signal addresses
ROBOT_PWM_ADDRESS: str = "lhs={}&rhs={}"
ROBOT_CONTROL_ADDRESS: str = "control?command={}"

class Robot(ABC):
    '''
    A robot which executes actions.
    '''

    LOCALISATION_DPS = 3 # the decimal precision to send localisation commands in
    LOCALISATION_WL = 7 # the total word length of a localisation parameter

    def __init__(self, ip: str, min_pwm=0, max_pwm=100):
        '''
        Parameters:
            ip (str): the ip address to the robot is listening on
        '''
        self.ip = ip
        self.pwm_left = 0
        self.pwm_right = 0

        self.max_pwm = max_pwm
        self.min_pwm = min_pwm

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
        abs_left = self.min_pwm + abs(left)/100*(self.max_pwm - self.min_pwm)
        abs_right = self.min_pwm + abs(right)/100*(self.max_pwm - self.min_pwm)

        self.pwm_left = copysign(abs_left, left)
        self.pwm_right = copysign(abs_right, right)

        self.pwm_left = round(self.pwm_left, Robot.LOCALISATION_DPS)
        self.pwm_right = round(self.pwm_right, Robot.LOCALISATION_DPS)

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
            requests.get(url, timeout=(2, 2))
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
        try:
            self.sock.sendto(bytes(RobotUDP.LOCALISATION_PREFIX + command, RobotUDP.ENCODING), (self.ip, RobotUDP.UDP_PORT))
        except OSError:
            print("Failed to send command")

    def send_control_command(self, command: str):
        try:
            self.sock.sendto(bytes(RobotUDP.CONTROL_PREFIX + command, RobotUDP.ENCODING), (self.ip, RobotUDP.UDP_PORT))
        except OSError:
            print("Failed to send command")
