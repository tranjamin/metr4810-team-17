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

import ctypes
ctypes.windll.shcore.SetProcessDpiAwareness(2)

ROBOT_WIFI_SSID = "METR4810 Team 17"
ROBOT_WIFI_PASSWORD = ""
WIFI_CONNECT_CMD = 'netsh wlan connect name="{0}" ssid="{0}"'

ROBOT_STARTED = False

# Extraction configuration
SCOOP_DURATION = 2  # seconds to finish scooping
SCOOP_INTERVAL = 20000  # seconds between each scoop

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
        robot_comms.send_control_action(v, omega, do_print=True)


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
    robot_tcp = RobotTCP("192.168.4.1")

    plan.set_robot(robot_comms)
    robot_state = State.WAIT

    old_extraction_time = None

    scoop_entry_time = 0
    
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
                robot_comms.send_control_action(v, omega, do_print=True)

                # TRANSITIONS
                if time.time() - old_extraction_time > SCOOP_INTERVAL and not plan.controller.phase_2 :
                    robot_state = State.INITIATE_SCOOP

            case State.INITIATE_SCOOP:
                robot_comms.send_control_action(0, 0, do_print=True)
                robot_tcp.send_control_command("command=9")
                scoop_entry_time = time.time()
                robot_state = State.SCOOPING
            case State.SCOOPING:
                # display message on image
                cv.putText(img, "SCOOPING",
                           (1000, 600),
                           cv.FONT_HERSHEY_PLAIN,
                           2,
                           (0, 0, 255),
                           4)

                if time.time() - scoop_entry_time > SCOOP_DURATION:
                    old_extraction_time = time.time()
                    robot_state = State.TRAVERSAL
            
            case State.TRAVERSAL_TO_DEPOSIT:
                plan.update_robot_position(x, y, theta)
                plan.controller_step()
                v = plan.desired_velocity
                omega = plan.desired_angular
                robot_comms.send_control_action(v, omega, do_print=True)

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
                robot_comms.send_control_action(v, omega, do_print=True)

                if plan.controller.has_reached_goal() and plan.controller.phase_2:
                    robot_state = State.TRAVERSAL  # maybe need separate waiting to restart state?



       

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
            plan.signal_extraction_stop()
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
            old_extraction_time = time.time()
            robot_state = State.TRAVERSAL
            plan.signal_extraction_start()
        elif key == ord('k'): # allow extraction
            plan.extraction_allowed = True
            plan.signal_extraction_execute()
        elif key == ord('m'): # manually extraction
            plan.extraction_allowed = False
            plan.signal_extraction_stop()
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