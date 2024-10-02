import numpy as np
import cv2 as cv
import time
from math import pi
from robot import Robot, RobotUDP, LineFollowerController, FowardController, SpinController
from planning import *
from fmi import *

import matplotlib.pyplot as plt

def main():
    cap = cv.VideoCapture(0, cv.CAP_DSHOW) # set to 2 to select external webcam
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, int(720)) # seems locked to 720p
    cap.set(cv.CAP_PROP_FRAME_WIDTH, int(1280)) # seems locked to 720p

    if not cap.isOpened():
        print("Cannot open camera")
        exit()

    localiser = RobotSim()
    localiser.init("tank_sim.fmu")
    localiser.setup()

    ### Set up controllers
    forward_controller = FowardController(k_angle=5,
                                       k_v=200,
                                       w=0.5,
                                       goal_tolerance=5,
                                       reversing_allowed=True)

    # warning when tuning
    spin_controller = SpinController(k_angle=5,
                                     k_v=0,
                                     angle_tolerance=0.05
                                     )
    controller = LineFollowerController(forward_controller, spin_controller)

    plan = Pathplanner()
    plan.set_controller(controller)
    # waypoints = RectangleWaypointSequence(1000, 1000, 500, 500, 10, angle_agnostic=False)
    waypoints = SnakeWaypointSequence(theta_agnostic=False)
    plan.set_waypoints(waypoints)

    robot_comms = RobotSim()

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
    main()