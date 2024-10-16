import numpy as np
import cv2 as cv
import argparse
import json
from sys import platform
import subprocess

from robot import *
from planning import *
from fmi import *
from localisation import *

import ctypes
ctypes.windll.shcore.SetProcessDpiAwareness(2)

ROBOT_WIFI_SSID = "METR4810 Team 17"
ROBOT_WIFI_PASSWORD = ""
WIFI_CONNECT_CMD = "netsh wlan connect name={0} ssid={0}"

ROBOT_STARTED = False

class Config():
    '''
    A class to hold all methods related to loading in configuration files.
    '''

    def __init__(self, filename: str):
        '''
        Parameters:
            filename (str): the filename and path of the config file
        '''
        self.config_file = json.load(open(filename))

    def load_localiser(self):
        # load in localiser from config file
        config_localiser: str = self.config_file["localisation"]
        config_localiser_args: dict = config_localiser["args"]
        localiser: Localisation =  eval(config_localiser["localisation-class"])(**config_localiser_args)
        assert isinstance(localiser, Localisation)

        return localiser
    
    def load_pathplanning(self):
        # load in controllers from config file
        controller_config = self.config_file["controllers"]
        forward_controller = FowardController(**controller_config["forward-controller"])
        spin_controller = SpinController(**controller_config["spin-controller"])
        controller = LineFollowerController(forward_controller, spin_controller)

        # load in pathplanner from config file
        plan = Pathplanner()
        plan.set_controller(controller)

        # load in extraction mode from config file
        extraction_mode: str = self.config_file["extraction"]["type"]
        plan.set_extraction_strategy(eval(f"ExtractionStrategies.{extraction_mode.upper()}"))

        # load in debogger
        if self.config_file["debogger"]["enabled"]:
            plan.set_debogging_strategy(DeboggingStrategies.ENABLED)
        else:
            plan.set_debogging_strategy(DeboggingStrategies.NONE)
        
        # pause debogger until started
        plan.pause_debogger()

        # load in extraction mode from config file
        pathplanner_class: WaypointSequence = eval(self.config_file["pathplan"]["reference-class"])
        pathplanner_kwargs = self.config_file["pathplan"]["args"]
        plan.set_waypoints(pathplanner_class(**pathplanner_kwargs))

        return plan

    def load_robot(self):
        robot_config = self.config_file["robot"]
        robot_class = eval(robot_config["robot-class"])
        robot_comms: Robot = robot_class(**robot_config["args"])

        return robot_comms

def connect_wifi():
    if platform == "win32":
        k = subprocess.run(WIFI_CONNECT_CMD.format(ROBOT_WIFI_SSID),
                           capture_output=True, text=True).stdout
        print("Connecting to Wifi:")
        print(k)


def main(configfile, camera):
    global ROBOT_STARTED

    # open config file
    cfg = Config(configfile)

    # configure camera input and window size
    cap = cv.VideoCapture(camera, cv.CAP_DSHOW)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, int(720))
    cap.set(cv.CAP_PROP_FRAME_WIDTH, int(1280))

    # exit if camera cannot be opened
    if not cap.isOpened():
        print("Cannot open camera")
        exit(1)

    # set up localisation
    localiser = cfg.load_localiser()
    localiser.setup()

    plan = cfg.load_pathplanning()
    robot_comms = cfg.load_robot()

    plan.set_robot(robot_comms)
    
    # Main loop
    while True:
        ret, img = cap.read()
        if not ret:
            break

        v, omega = 0, 0

        positions, angles = localiser.get_position(img)

        # draw current waypoint on screen
        localiser.annotate_xy(img, plan.current_waypoint.x, plan.current_waypoint.y)

        ### PATH PLANNING
        not_none = lambda x: x is not None
        if all([not_none(e) for e in positions]):
            x, _, y, _, _, _ = np.ravel(positions).tolist()
            theta, _, _, _, _, _ = np.ravel(angles).tolist()

            plan.update_robot_position(x, y, theta)
            if ROBOT_STARTED:
                plan.controller_step()

                v = plan.desired_velocity
                omega = plan.desired_angular
                
                robot_comms.send_control_action(v, omega, do_print=True)


            # draw control info on screen
            labels = ["x", "y", "theta"]
            for index, val in enumerate([x, y, theta*180/pi]):
                cv.putText(img, '{}: {:.3f}'.format(labels[index], val),
                            (500, 50 + 50*index),
                            cv.FONT_HERSHEY_PLAIN,
                            2,
                            (0, 0, 255),
                            4)

        # draw control info on screen
        labels = ["v", "omega"]
        for index, val in enumerate([v, omega]):
            cv.putText(img, '{}: {:.3f}'.format(labels[index], val),
                        (1000, 50 + 50*index),
                        cv.FONT_HERSHEY_PLAIN,
                        2,
                        (0, 0, 255),
                        4)
    


        cv.imshow('frame', img)
        key = cv.waitKey(1)
        if key == ord('q'): # stop robot and exit
            plan.signal_extraction_stop()
            break
        elif key == ord('d') and ROBOT_STARTED: # return to delivery
            plan.add_delivery()
        elif key == ord('e') and ROBOT_STARTED: # go to high ground
            plan.add_emergency()
        elif key == ord('f') and ROBOT_STARTED: # start depositing bean
            plan.signal_delivery_start()
        elif key == ord('s'): # start robot sending
            positions, angles = localiser.get_position(img)
            x, _, y, _, _, _ = np.ravel(positions).tolist()
            theta, _, _, _, _, _ = np.ravel(angles).tolist()

            plan.unpause_debogger()

            if all([not_none(e) for e in positions]):
                print("Dynamically setting the delivery waypoint...")
                new_deposit_x = x
                new_deposit_y = y
                new_deposit_theta = theta

                backwards_length = DepositWaypoint.DEPOSIT_SIZE/2 + RobotGeometry.RADIUS

                new_helper_x = x - backwards_length*math.cos(theta)
                new_helper_y = y - backwards_length*math.sin(theta)

                try:
                    assert new_helper_x <= 2000 and new_helper_x >= 0
                    assert new_helper_y <= 2000 and new_helper_y >= 0

                    new_helper_x = max(new_helper_x, RobotGeometry.RADIUS + 10)
                    new_helper_y = max(new_helper_y, RobotGeometry.RADIUS + 10)
                    DepositWaypoint.DEPOSIT_HEADING = new_deposit_theta
                    DepositWaypoint.DEPOSIT_X = new_deposit_x
                    DepositWaypoint.DEPOSIT_Y = new_deposit_y
                    DepositHelperWaypoint.DEPOSIT_HELPER_X = new_helper_x
                    DepositHelperWaypoint.DEPOSIT_HELPER_Y = new_helper_y
                except AssertionError:
                    print("Dynamic setting failed... reverting")

            ROBOT_STARTED = True
            plan.extractionFlag = True
            plan.signal_extraction_start()
        elif key == ord('k'): # allow extraction
            plan.extraction_allowed = True
            plan.signal_extraction_execute()
        elif key == ord('m'): # manually extraction
            plan.extraction_allowed = False
            plan.signal_extraction_stop()
        elif key == ord('w'):
            connect_wifi()
            

    cap.release()
    cv.destroyAllWindows()

    # stop the robot
    robot_comms.send_control_action(0,0, True)
    localiser.deinit()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--filename", "-f", default="config/configSnake.json")
    parser.add_argument("--camera", "-c", default=2)
    args = parser.parse_args()
    print(f"Reading file {args.filename} and camera {args.camera}")

    main(args.filename, int(args.camera))