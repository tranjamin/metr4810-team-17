"""
The main file for the client-side.
"""

import numpy as np
import cv2 as cv
import argparse
from sys import platform
import subprocess
from enum import Enum
import pandas as pd
import ctypes
from math import pi

from robot import *
from planning import *
from fmi import *
from localisation import *
from config import *

# correct the resolution of the camera window
ctypes.windll.shcore.SetProcessDpiAwareness(2)

# Wi-Fi parameters
ROBOT_WIFI_SSID = "METR4810 Team 17"
ROBOT_WIFI_PASSWORD = ""
WIFI_CONNECT_CMD = 'netsh wlan connect name="{0}" ssid="{0}"'

# arrays to log data
array_times = []
array_x = []
array_y = []
array_theta = []
array_v = []
array_omega = []


class State(Enum):
    """
    An enum to store the states for the main FSM
    """

    WAIT = 0
    TRAVERSAL = 1


def connect_wifi():
    """
    Connect to the Wi-Fi
    """
    if platform == "win32":
        k = subprocess.run(
            WIFI_CONNECT_CMD.format(ROBOT_WIFI_SSID), capture_output=True,
            text=True
        ).stdout
        print("Connecting to Wifi:")
        print(k)


def main(configfile, camera):

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

    # set up pathplanner and robot
    plan = cfg.load_pathplanning()
    robot_comms = cfg.load_robot()

    plan.set_robot(robot_comms)
    plan.extraction_strategy.attach_agents(robot_comms)
    plan.extraction_strategy.reset_extraction()

    # state in the FSM
    robot_state = State.WAIT

    # Main loop
    while True:
        # read in an image
        ret, img = cap.read()
        if not ret:
            break

        v, omega = 0, 0  # parameters of the robot movement
        x, y, theta = 0, 0, 0  # parameters of the robot position

        # get the position according to the localisation
        positions, angles = localiser.get_position(img)
        if all([e is not None for e in positions]):
            x, _, y, _, _, _ = np.ravel(positions).tolist()
            theta, _, _, _, _, _ = np.ravel(angles).tolist()

        # draw current waypoint on screen
        localiser.annotate_xy(img, plan.current_waypoint.x, plan.current_waypoint.y)

        # state machine
        match robot_state:
            case State.WAIT:
                pass
            case State.TRAVERSAL:
                # get robot position
                plan.update_robot_position(x, y, theta)

                # step the controller
                plan.controller_step()

                # get desired position
                v = plan.desired_velocity
                omega = plan.desired_angular

                # send information to robot
                robot_comms.send_control_action(v, omega, do_print=False)

                # iterate the extraction strategy
                plan.extraction_strategy.spin()

        # update logs
        array_times.append(time.time())
        array_x.append(x)
        array_y.append(y)
        array_theta.append(theta)
        array_omega.append(omega)
        array_v.append(v)

        # draw position info on screen
        labels = ["x", "y", "theta"]
        for index, val in enumerate([x, y, theta * 180 / pi]):
            cv.putText(
                img,
                "{}: {:.3f}".format(labels[index], val),
                (500, 50 + 50 * index),
                cv.FONT_HERSHEY_PLAIN,
                2,
                (0, 0, 255),
                4,
            )

        # draw control info on screen
        labels = ["v", "omega"]
        for index, val in enumerate([v, omega]):
            cv.putText(
                img,
                "{}: {:.3f}".format(labels[index], val),
                (1000, 50 + 50 * index),
                cv.FONT_HERSHEY_PLAIN,
                2,
                (0, 0, 255),
                4,
            )

        # show the image
        cv.imshow("frame", img)

        key = cv.waitKey(1)
        if key == ord("q"):  # stop robot and exit
            plan.extraction_strategy.disable_extraction()
            break
        elif key == ord('d') and robot_state == State.TRAVERSAL:  # return to delivery
            plan.robot.send_control_command("command=3")
            time.sleep(5)
            plan.add_delivery()
        elif key == ord("e") and robot_state == State.TRAVERSAL:  # go to high ground
            plan.add_emergency()
        elif (
            key == ord("f") and robot_state == State.TRAVERSAL
        ):  # start depositing bean
            plan.signal_delivery_start()  # start delivery
            plan.debog_strategy.delay(22)  # resume debogging after 22 seconds
        elif key == ord("s"):  # start robot sending
            # get starting position
            positions, angles = localiser.get_position(img)
            x, _, y, _, _, _ = np.ravel(positions).tolist()
            theta, _, _, _, _, _ = np.ravel(angles).tolist()

            if all([e is not None for e in positions]):
                # redefine deposit to be the current position
                DepositWaypoint.redefine_deposit(x, y, theta)

                # if using a point and fire, dynamically set the waypoint
                if isinstance(plan.waypoints, StraightLineWaypointSequence):
                    plan.waypoints.aim_line(x, y, theta)
                    plan.set_waypoints(plan.waypoints)

                # if using aim assist
                if plan.waypoints.dynamic_aim:
                    if theta > 2 * pi / 3:  # do the normal pathplanning
                        plan.waypoints.aim_assist_off()
                    else:  # use a point and fire approach
                        plan.waypoints.aim_assist_on(x, y, theta)

                        # redefine the helper waypoint to not exist
                        DepositHelperWaypoint.DEPOSIT_HELPER_X = (
                            DepositWaypoint.DEPOSIT_X
                        )
                        DepositHelperWaypoint.DEPOSIT_HELPER_Y = (
                            DepositWaypoint.DEPOSIT_Y
                        )

                    # update waypoints
                    plan.set_waypoints(plan.waypoints)

            robot_state = State.TRAVERSAL  # update robot state

            # enable extraction and debogging
            plan.extraction_strategy.enable_extraction()
            plan.debog_strategy.enable_debogger()
        elif key == ord("w"):  # connect to the wifi
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
    parser.add_argument("--filename", "-f",
                        default="config/config.json")
    parser.add_argument("--camera", "-c", default=0)
    args = parser.parse_args()
    print(f"Reading file {args.filename} and camera {args.camera}")

    # run main function
    main(args.filename, int(args.camera))

    # save log messages
    df = pd.DataFrame(
        {
            "t": array_times,
            "x": array_x,
            "y": array_y,
            "theta": array_theta,
            "v": array_v,
            "omega": array_omega,
        }
    )

    df.to_csv("log.csv")
