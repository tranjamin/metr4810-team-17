import numpy as np
import cv2 as cv
import time
from math import pi
from robot import Robot, RobotUDP, LineFollowerController, FowardController, SpinController
from planning import *
from fmi import *

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

    # path_segments = [
    #     [(0.3, 0.5), (1.3, 0.5), pi/2],
    #     [(1.3, 0.5), (1.3, 1.5), pi],
    #     [(1.3, 1.5), (0.3, 1.5), -pi/2],
    #     [(0.3, 1.5), (0.3, 0.5), 0],
    #     ]
    
    # path_segments = [
    #     [(0.073, 0.436), (1.553, 0.432), pi/2],
    #     [(1.553, 0.432), (1.564, 1.997), pi],
    #     [(1.564, 1.997), (0.093, 1.930), -pi/2],
    #     [(0.093, 1.930), (0.073, 0.436), 0],
    #     ]
    # current_segment = 0

    ### Set up controllers
    forward_controller = FowardController(k_angle=100,
                                       k_v=60,
                                       w=0.5,
                                       goal_tolerance=0.05,
                                       reversing_allowed=True)

    # warning when tuning
    spin_controller = SpinController(k_angle=8*20,
                                     k_v=0,
                                     angle_tolerance=0.2
                                     )
    controller = LineFollowerController(forward_controller, spin_controller)

    plan = Pathplanner()
    plan.set_controller(controller)
    waypoints = RectangleWaypointSequence(1, 1, 0.3, 0.5, 10)
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

            print("X, Y, Theta: ", x, y, theta)

            x = x /1000
            y = y/1000

            plan.update_robot_position(x, y, theta)
            plan.controller_step()

            v = plan.desired_velocity
            omega = plan.desired_angular
            
            robot_comms.send_control_action(v, omega, do_print=True)

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
        if cv.waitKey(1) == ord('q'):
            break

    for name in localiser.stats.keys():
        array = localiser.stats[name]
        print(f"{name}: {np.mean(array)} (std: {np.std(array)})")
    cap.release()
    cv.destroyAllWindows()

    # stop the robot
    robot_comms.send_control_action(0,0, True)
    localiser.deinit()


if __name__ == "__main__":
    main()