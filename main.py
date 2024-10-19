import numpy as np
import cv2 as cv
import argparse
import json
from sys import platform
import subprocess
from enum import Enum

from robot import *
from planning import *
from fmi import *
from localisation import *
from config import *

import pandas as pd

import ctypes
ctypes.windll.shcore.SetProcessDpiAwareness(2)

ROBOT_WIFI_SSID = "METR4810 Team 17"
ROBOT_WIFI_PASSWORD = ""
WIFI_CONNECT_CMD = 'netsh wlan connect name="{0}" ssid="{0}"'

ROBOT_STARTED = False

# Extraction configuration
SCOOP_DURATION = 2  # seconds to finish scooping
SCOOP_INTERVAL = 2  # seconds between each scoop

array_times = []
array_x = []
array_y = []
array_theta = []
array_v = []
array_omega = []

# Deposit configuration

class State(Enum):
    WAIT = 0
    TRAVERSAL = 1
    SCOOPING = 2
    INITIATE_SCOOP = 3
    TRAVERSAL_TO_DEPOSIT = 4
    DEPOSIT = 5
    EMERGENCY = 6


def connect_wifi():
    if platform == "win32":
        k = subprocess.run(WIFI_CONNECT_CMD.format(ROBOT_WIFI_SSID),
                           capture_output=True, text=True).stdout
        print("Connecting to Wifi:")
        print(k)

def move_robot(x, y, theta, plan: Pathplanner, robot_comms: RobotUDP):
        plan.update_robot_position(x, y, theta)
        plan.controller_step()
        v = plan.desired_velocity
        omega = plan.desired_angular
        robot_comms.send_control_action(v, omega, do_print=False)


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
    plan.extraction_strategy.attach_agents(robot_comms)
    plan.extraction_strategy.reset_extraction()
    robot_state = State.WAIT
    
    # Main loop
    while True:
        # LOCALISE THE ROBOT
        # (this always runs regardless of state)
        # read in image
        ret, img = cap.read()
        if not ret:
            break

        v, omega = 0, 0

        positions, angles = localiser.get_position(img)
        x, y, theta = 0, 0, 0
        not_none = lambda x: x is not None
        if all([not_none(e) for e in positions]):
            x, _, y, _, _, _ = np.ravel(positions).tolist()
            theta, _, _, _, _, _ = np.ravel(angles).tolist()
        # draw current waypoint on screen
        localiser.annotate_xy(img, plan.current_waypoint.x, plan.current_waypoint.y)
        
        ### PATH PLANNING
        # set to zero so printouts will work even if nothing has been sent
        v, omega = 0, 0
        
        # STATE MACHINE
        match robot_state:
            case State.WAIT:
                pass
            case State.TRAVERSAL:
                # ACTIONS
                plan.update_robot_position(x, y, theta)
                plan.controller_step()
                v = plan.desired_velocity
                omega = plan.desired_angular
                robot_comms.send_control_action(v, omega, do_print=False)

                plan.extraction_strategy.spin()

                # TRANSITIONS

            case State.INITIATE_SCOOP:
                pass
            case State.SCOOPING:
                # display message on image
                cv.putText(img, "SCOOPING",
                           (1000, 600),
                           cv.FONT_HERSHEY_PLAIN,
                           2,
                           (0, 0, 255),
                           4)
            
            case State.TRAVERSAL_TO_DEPOSIT:
                plan.update_robot_position(x, y, theta)
                plan.controller_step()
                v = plan.desired_velocity
                omega = plan.desired_angular
                robot_comms.send_control_action(v, omega, do_print=False)

                # TRANSITIONS
                if plan.stopFlag:
                    # have reached the deposit zone
                    robot_state = State.DEPOSIT

            case State.DEPOSIT:
                # do timers or something
                robot_state = State.TRAVERSAL
                pass

            case State.EMERGENCY:
                # just want to move, don't extract
                plan.update_robot_position(x, y, theta)
                plan.controller_step()
                v = plan.desired_velocity
                omega = plan.desired_angular
                robot_comms.send_control_action(v, omega, do_print=False)

                if plan.controller.has_reached_goal() and plan.controller.phase_2:
                    robot_state = State.TRAVERSAL  # maybe need separate waiting to restart state?


        array_times.append(time.time())
        array_x.append(x)
        array_y.append(y)
        array_theta.append(theta)
        array_omega.append(omega)
        array_v.append(v)
        
        # draw position info on screen
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
            plan.extraction_strategy.disable_extraction()
            break
        elif key == ord('d') and ROBOT_STARTED:  # return to delivery
            plan.add_delivery()
        elif key == ord('e') and ROBOT_STARTED:  # go to high ground
            plan.add_emergency()
        elif key == ord('f') and ROBOT_STARTED:  # start depositing bean
            plan.signal_delivery_start()
        elif key == ord('s'): # start robot sending
            positions, angles = localiser.get_position(img)
            x, _, y, _, _, _ = np.ravel(positions).tolist()
            theta, _, _, _, _, _ = np.ravel(angles).tolist()

            old_extraction_time = time.time() + 2

            plan.debog_strategy.unpause_debogger()

            if all([not_none(e) for e in positions]):
                DepositWaypoint.redefine_deposit(x, y, theta)
                
                if isinstance(plan.waypoints, StraightLineWaypointSequence):
                    plan.waypoints.aim_line(x, y, theta)
                    plan.set_waypoints(plan.waypoints)

                if plan.waypoints.dynamic_aim:
                    if theta > 2*pi/3: # then we go normal
                        plan.waypoints.aim_assist_off()
                    else:  # then we go to aim assist
                        plan.waypoints.aim_assist_on(x, y, theta)
                        DepositHelperWaypoint.DEPOSIT_HELPER_X = DepositWaypoint.DEPOSIT_X
                        DepositHelperWaypoint.DEPOSIT_HELPER_Y = DepositWaypoint.DEPOSIT_Y
                    plan.set_waypoints(plan.waypoints)
    

            ROBOT_STARTED = True
            plan.extractionFlag = True
            old_extraction_time = time.time()
            robot_state = State.TRAVERSAL
            plan.extraction_strategy.enable_extraction()
        elif key == ord('k'): # allow extraction
            plan.extraction_strategy.enable_extraction()
        elif key == ord('m'): # manually extraction
            plan.extraction_strategy.disable_extraction()
        elif key == ord('w'):
            connect_wifi()
            
    # deinit camera and cv
    cap.release()
    cv.destroyAllWindows()

    # stop the robot and localisation
    robot_comms.send_control_action(0, 0, True)
    localiser.deinit()


if __name__ == "__main__":
    # load in command line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--filename", "-f", default="config/configSimSpiral.json")
    parser.add_argument("--camera", "-c", default=2)
    args = parser.parse_args()
    print(f"Reading file {args.filename} and camera {args.camera}")

    # run main function
    main(args.filename, int(args.camera))

    df = pd.DataFrame({"t": array_times, "x": array_x, "y": array_y, "theta": array_theta, "v": array_v, "omega": array_omega})
    df.to_csv("log.csv")