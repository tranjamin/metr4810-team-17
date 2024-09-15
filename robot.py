import requests

ROBOT_PWM_ADDRESS = "localisation?lhs={}&rhs={}"
ROBOT_LED_ADDRESS = "ledtest?led={}"
ROBOT_CONTROL_ADDRESS = "control?command={}"

class Robot:
    def __init__(self, ip: str):
        self.ip = ip
        self.pwm_left = 0
        self.pwm_right = 0

    def set_pwm(self, left: int, right: int):
        self.pwm_left = left
        self.pwm_right = right
        self.send_command(ROBOT_PWM_ADDRESS.format(left, right))
    
    def control_function(self, command):
        self.send_command(ROBOT_CONTROL_ADDRESS.format(command))

    def send_command(self, command: int):
        try:
            print("sending robot command")
            url = f"http://{self.ip}/{command}"
            requests.get(url, timeout=(0.1, 0.0000000001))
            # Hack from: https://stackoverflow.com/questions/27021440/python-requests-dont-wait-for-request-to-finish
            # timeout means we ignore any response from the pico
        except requests.exceptions.ReadTimeout: 
            pass