import numpy as np
import cv2 as cv
import time
from math import pi
import argparse
import json

from robot import *
from planning import *
from fmi import *
from localisation import *

import matplotlib.pyplot as plt

import ctypes
ctypes.windll.shcore.SetProcessDpiAwareness(2)

def main(configfile, camera):
    CONFIG_FILE = json.load(open(configfile))

    cap = cv.VideoCapture(camera, cv.CAP_DSHOW) # set to 2 to select external webcam
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, int(720)) # seems locked to 720p
    cap.set(cv.CAP_PROP_FRAME_WIDTH, int(1280)) # seems locked to 720p

    if not cap.isOpened():
        print("Cannot open camera")
        exit()

    config_localiser = CONFIG_FILE["localisation"]
    config_localiser_args = config_localiser["args"]
    config_localiser_class = eval(config_localiser["localisation-class"])
    localiser: Localisation = config_localiser_class(**config_localiser_args)

    localiser.setup()

    ### Set up controllers
    controller_config = CONFIG_FILE["controllers"]
    forward_controller = FowardController(**controller_config["forward-controller"])
    spin_controller = SpinController(**controller_config["spin-controller"])
    controller = LineFollowerController(forward_controller, spin_controller)

    plan = Pathplanner()
    plan.set_controller(controller)

    pathplanner_class: WaypointSequence = eval(CONFIG_FILE["pathplan"]["reference-class"])
    pathplanner_kwargs = CONFIG_FILE["pathplan"]["args"]
    plan.set_waypoints(pathplanner_class(**pathplanner_kwargs))

    robot_config = CONFIG_FILE["robot"]
    robot_class: Robot = eval(robot_config["robot-class"])
    robot_comms = robot_class(**robot_config["args"])
    
    # Main loop
    while True:
        ret, img = cap.read()
        if not ret:
            break

        
        ### LOCALISATION FINISHED
        positions, angles = localiser.get_position(img)

        ### PATH PLANNING
        v, omega = 0, 0
        not_none = lambda x: x is not None
        if all([not_none(e) for e in positions]):
            x, _, y, _, _, _ = np.ravel(positions).tolist()
            theta, _, _, _, _, _ = np.ravel(angles).tolist()

            plan.update_robot_position(x, y, theta)
            plan.controller_step()

            v = plan.desired_velocity
            omega = plan.desired_angular
            
            robot_comms.send_control_action(v, omega, do_print=False)

            # draw control info on screen
            labels = ["x", "y"]
            for index, val in enumerate([x, y]):
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
        if key == ord('q'):
            break
        elif key == ord('d'):
            print("D")
            plan.previous_waypoint = (plan.current_x, plan.current_y)
            plan.waypoints.plan_to_deposit(x, y, theta)
            plan.current_waypoint = plan.waypoints.get_current_waypoint()
            plan.update_controller_path = True


    cap.release()
    cv.destroyAllWindows()

    # stop the robot
    robot_comms.send_control_action(0,0, True)
    localiser.deinit()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--filename", "-f", default="config.json")
    parser.add_argument("--camera", "-c", default=0)
    args = parser.parse_args()
    print(f"Reading file {args.filename} and camera {args.camera}")

    main(args.filename, args.camera)