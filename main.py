import numpy as np
import cv2 as cv
import argparse
import json

from robot import *
from planning import *
from fmi import *
from localisation import *

import ctypes
ctypes.windll.shcore.SetProcessDpiAwareness(2)

ROBOT_STARTED = False

def main(configfile, camera):
    global ROBOT_STARTED

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
    plan.set_extraction_strategy(ExtractionStrategies.PERIODIC)
    plan.set_debogging_strategy(DeboggingStrategies.ENABLED)

    pathplanner_class: WaypointSequence = eval(CONFIG_FILE["pathplan"]["reference-class"])
    pathplanner_kwargs = CONFIG_FILE["pathplan"]["args"]
    plan.set_waypoints(pathplanner_class(**pathplanner_kwargs))

    robot_config = CONFIG_FILE["robot"]
    robot_class = eval(robot_config["robot-class"])
    robot_comms: Robot = robot_class(**robot_config["args"])

    plan.set_robot(robot_comms)
    
    # Main loop
    while True:
        ret, img = cap.read()
        if not ret:
            break

        v, omega = 0, 0
        ### LOCALISATION FINISHED
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
                
                robot_comms.send_control_action(v, omega, do_print=False)


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
            ROBOT_STARTED = True
            plan.extractionFlag = True
            plan.signal_extraction_start()
        elif key == ord('k'): # allow extraction
            plan.extraction_allowed = True
            plan.signal_extraction_start()
        elif key == ord('m'): # manually extraction
            plan.extraction_allowed = False
            plan.signal_extraction_stop()
            

    cap.release()
    cv.destroyAllWindows()

    # stop the robot
    robot_comms.send_control_action(0,0, True)
    localiser.deinit()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--filename", "-f", default="configSnake.json")
    parser.add_argument("--camera", "-c", default=2)
    args = parser.parse_args()
    print(f"Reading file {args.filename} and camera {args.camera}")

    main(args.filename, int(args.camera))