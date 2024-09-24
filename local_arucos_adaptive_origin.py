import numpy as np
import cv2 as cv
from scipy.spatial.transform import Rotation as R
from estimators import RigidBodyTracker
import time
from math import pi
from robot import Robot, RobotUDP, LineFollowerController, FowardController, SpinController
from robotpy_apriltag import AprilTagDetector
from planning import *

MARKER_SIZE = 100 #97 # mm
ORIGIN_X_DELTA = -59
ORIGIN_Y_DELTA = 146
RVECS = 0
TVECS = 1
CORNERS = 2

LAST_ROBOT_COMMUNICATE = 0
EULER_ORDER = 'ZYX' # determines conversion from angles to rotations

# I believe the above is correctas it
# 1. specifies yaw, then pitch, then roll
# 2. uses intrinsic rotations (subsequent rotations are done about the axes that
#    have already been rotated)


class MarkerCollection:
    """
    Class to represent an object we want to track that has a variety of markers
    rigidly connected to it.
    """
    def __init__(self) -> None:
        # dictionary to contain mapping from Aruco id to object points
        self._points: dict[int, np.ndarray] = {}
        self.last_tvecs = None
        self.last_rvecs = None

    def register_marker(self, id: int, marker_size: float, tvec: np.ndarray,
                        rotation: R, reference_tvec: np.ndarray = None,
                        reference_rotation: R = None):
        """Adds the given marker to this collection. The marker is specified by
        its size (in same units as camera calibration matrices, probably mm),
        and the location of the marker relative to this collection's origin.

        Alternatively, if reference_tvec and reference_rotation are provided,
        the tvec and rotation arguments are interpreted as being relative to
        these.

        Args:
            id (int): aruco id of the marker
            marker_size (float): size of physical marker
            tvec (np.ndarray): vector location of marker's top left corner in
                collection frame (three-dimensional)
            rotation (Rotation): rotation from origin (or reference rotation)
                to marker frame
            reference_tvec (np.ndarray, optional): Optional reference from which to define tvec.
                Origin -> reference. Defaults to None.
            reference_rotation (Rotation, optional): Optional reference from which to define rotation.
                Origin -> reference. Defaults to None.
        """
        base_points = np.array([[0, 0, 0],
                              [marker_size, 0, 0],
                              [marker_size, marker_size, 0],
                              [0, marker_size, 0]], dtype=np.float32)

        # Base points are defined in a frame centred at the top left corner of
        # the marker, with the x axis pointing to the top right corner and the
        # y axis pointing to the bottom left corner

        # Note if changing this, I think the base points must be defined in
        # clockwise order starting from the top left (to match with
        # Aruco.detectMarkers())
        return_val = tvec, rotation
        actual_points = rotation.apply(base_points)
        actual_points = np.add(actual_points, tvec)

        if reference_rotation is not None and reference_tvec is not None:
            actual_points = reference_rotation.apply(actual_points)
            actual_points = np.add(actual_points, reference_tvec)
            return_val = np.add(reference_tvec, tvec), reference_rotation * rotation
        
        if (reference_rotation is not None) ^ (reference_tvec is not None):
            # one provided but not the other
            raise Exception("One reference provided but not the other")
        
        self._points[id] = actual_points
        return return_val

    def estimate_pose(self, corners_list: np.ndarray, ids: np.ndarray,
                      camera_matrix: cv.UMat, dist_coeffs: cv.UMat) -> tuple[bool, np.ndarray, np.ndarray]:
        """Estimate the pose of this collection's frame given a list of corners
        and corresponding marker ids where at least some of the markers detected
        are part of this collection. Uses cv.solvePnp for pose estimation, so
        requires camera matrix and distortion coefficients provided by camera
        calibration.

        Optionally, annotates the given image with the pose

        Args:
            corners_list (np.ndarray): list of corners returned by
                cv.Aruco.detectMarkers
            ids (np.ndarray): ids returned by cv.Aruco.detectMarkers
            camera_matrix (cv.UMat): camera matrix from calibration 
            dist_coeffs (cv.UMat): distortion coefficients from calibration 

        Returns:
            tuple[bool, np.ndarray, np.ndarray]: validity (whether collection
            was found), rvecs, tvecs
        """

        # step 1, figure out which markers in the image are from this
        # Collection, and build the list of object points

        # find relevant corners
        corner_collection = [corners
                             for index, corners in enumerate(corners_list)
                             if int(ids[index][0]) in self._points.keys()]
        if not corner_collection:
            # list is empty
            return False, None, None
        # this list comprehension makes a list of the corner lists
        relevant_corners = np.concatenate(corner_collection)

        # find relevant object points

        objpts_collection = [self._points[id[0]]
                             for id in ids if int(id[0]) in self._points.keys()]

        relevant_object_points = np.concatenate(objpts_collection)

        # have tvecs changed a ton since the last measurement?

        _, rvecs, tvecs = cv.solvePnP(relevant_object_points,
                                      relevant_corners,
                                      camera_matrix,
                                      dist_coeffs,
                                      flags=cv.SOLVEPNP_SQPNP)

        MAX_TVEC_CHANGE = 100 # have somehow moved 10cm between measurements

        use_guess = False
        if self.last_rvecs is not None and self.last_tvecs is not None and \
            MAX_TVEC_CHANGE > np.linalg.norm(tvecs - self.last_tvecs):
            use_guess = True
            _, rvecs, tvecs = cv.solvePnP(relevant_object_points,
                                        relevant_corners,
                                        camera_matrix,
                                        dist_coeffs,
                                        rvec=self.last_rvecs,
                                        tvec=self.last_tvecs,
                                        useExtrinsicGuess=use_guess,
                                        flags=cv.SOLVEPNP_ITERATIVE)
        
        # rvecs, tvecs = cv.solvePnPRefineLM(relevant_object_points,
        #                                    relevant_corners,
        #                                    camera_matrix,
        #                                    dist_coeffs,
        #                                    rvecs,
        #                                    tvecs)
        self.last_rvecs = rvecs
        self.last_tvecs = tvecs
        return True, rvecs, tvecs

    def annotate(self, corners_list: np.ndarray, ids: np.ndarray,
                 img: cv.typing.MatLike, color: tuple[int, int, int],
                 thickness: int = 10):
        """Annotates the markers in the given image that are relevant to this
        collection by drawing boxes around them in the given colour and
        thickness.

        Args:
            corners_list (np.ndarray): corners from cv.Aruco.detectMarkers
            ids (np.ndarray): ids from cv.Aruco.detectMarkers
            img (cv.typing.MatLike): image to be annotated
            color (tuple[int, int, int]): (blue, green, red) colour
            thickness (int): line thickness
        """
        for index, corners in enumerate(corners_list):
            if int(ids[index][0]) in self._points.keys():
                pts = np.array(corners, dtype=np.int32)
                cv.polylines(img, [pts], True, color, thickness)

class PIController:
    def __init__(self, Kp, Ki, control_saturation):
        self.Kp = Kp
        self.Ki = Ki
        self.control_saturation = control_saturation
        self.error_accumulated = 0
    
    def output(self, error: float) -> int:

        control_output = self.Kp * error + self.Ki * (self.error_accumulated + error)
        
        if control_output > self.control_saturation:
            return self.control_saturation
        
        # not saturated, therefore accumulate integral
        self.error_accumulated += error
        return control_output

def get_relative_pose(rvec_camera_to_origin, p_origin_camera_frame,
                      rvec_camera_to_target, p_target_camera_frame) -> tuple[R, np.ndarray]:

    position_delta_camera_frame = p_target_camera_frame - p_origin_camera_frame
    rot_camera_to_origin = R.from_rotvec(rvec_camera_to_origin.ravel())
    rot_camera_to_target = R.from_rotvec(rvec_camera_to_target.ravel())
    rot_origin_to_target = rot_camera_to_origin.inv() * rot_camera_to_target
    position_delta_origin_frame = rot_camera_to_origin.inv().apply(position_delta_camera_frame.ravel())
    
    return rot_origin_to_target, position_delta_origin_frame

def get_frame_image_coords(camera_matrix: cv.typing.MatLike,
                           dist_coeffs: cv.typing.MatLike,
                           frame_rvec: cv.typing.MatLike,
                           p_frame_camera_frame: cv.typing.MatLike) -> tuple[int, int]:
    """Returns the x, y position in image coordinates of the origin of the
    specified frame. Useful for drawing lines on images.

    Args:
        camera_matrix (cv.typing.MatLike): camera matrix from calibration
        dist_coeffs (cv.typing.MatLike): distortion coeffs from calibration
        frame_rvec (cv.typing.MatLike): rvec to frame (from pose estimation)
        p_frame_camera_frame (cv.typing.MatLike): tvec to frame (from pose estimation)

    Returns:
        tuple[int, int]: x, y location in image (pixel coordinates) 
    """
    point, _ = cv.projectPoints(np.array([0, 0, 0], dtype=np.float32),
                                frame_rvec, p_frame_camera_frame,
                                camera_matrix, dist_coeffs)
    x_origin, y_origin = np.round(point.ravel()).astype(int).tolist()
    return x_origin, y_origin

def process_image(img, camera_matrix, dist_coeffs, origin: MarkerCollection, target: MarkerCollection, detector: AprilTagDetector, logger):
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    stuff = detector.detect(gray)
    if not stuff:
        return False, None, None
    corners_list = []
    ids = []
    # dumb stuff to put data in similar (but not same) order as would normally
    # be given by OpenCV aruco detector module
    for detection in stuff:
        corners_list.append([[detection.getCorner(i).x,detection.getCorner(i).y] for i in range(3,-1,-1)])
        ids.append([detection.getId()])
    # corners_list, ids, _ = stuff

    # find origin
    origin_found, rvec_camera_to_origin, p_origin_camera_frame = origin.estimate_pose(corners_list, ids, camera_matrix, dist_coeffs)
    if not origin_found:
        return False, None, None
    # print(f"Origin : {p_origin_camera_frame}")
    origin.annotate(corners_list, ids, img, (0, 255, 255), 10)
    cv.drawFrameAxes(img, camera_matrix, dist_coeffs, rvec_camera_to_origin, p_origin_camera_frame, 50,4)

    target_found, rvec_camera_to_target, p_target_camera_frame = target.estimate_pose(corners_list, ids, camera_matrix, dist_coeffs)
    if not target_found:
        return False, None, None
    # print(f"Target : {p_target_camera_frame}")
    target.annotate(corners_list, ids, img, (255, 255, 0), 10)
    cv.drawFrameAxes(img, camera_matrix, dist_coeffs, rvec_camera_to_target, p_target_camera_frame, 50,4)

    rot_relative, tvec_relative = get_relative_pose(rvec_camera_to_origin, p_origin_camera_frame, rvec_camera_to_target, p_target_camera_frame)

    yaw, pitch, roll = rot_relative.as_euler(EULER_ORDER ,degrees=True)

    dist = np.linalg.norm(tvec_relative)

    # Draw data onto image
    x_origin, y_origin = get_frame_image_coords(camera_matrix, dist_coeffs, rvec_camera_to_origin, p_origin_camera_frame)
    x, y = get_frame_image_coords(camera_matrix, dist_coeffs, rvec_camera_to_target, p_target_camera_frame)
    # x_mid, y_mid = ((x_origin*0 + x) // 2, (y_origin*0 + y) // 2)
    x_mid, y_mid = (x, y)
    # cv.line(img, (x_origin, y_origin), (x, y), (255, 0, 0), 2)
    # cv.putText(img, 'x: {:.3f}'.format(tvec_relative[0]), (x_mid, y_mid), cv.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 4)
    # cv.putText(img, 'y: {:.3f}'.format(tvec_relative[1]), (x_mid, y_mid+50), cv.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 4)
    # cv.putText(img, 'yaw: {:.3f}'.format(yaw), (x_mid, y_mid+100), cv.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 4)
    # cv.putText(img, 'dist: {:.3f}'.format(dist), (x_mid, y_mid+150), cv.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 4)

    # log the values for statistics

    #         if int(id[0]) not in logger:
    #             logger[int(id[0])] = [[],[],[]]
    #         for i in [0, 1, 2]:
    #             logger[int(id[0])][i].append(position_delta_origin_frame[i][0])
    return True, rot_relative, tvec_relative


def main():
    cap = cv.VideoCapture(0, cv.CAP_DSHOW) # set to 2 to select external webcam
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, int(720)) # seems locked to 720p
    cap.set(cv.CAP_PROP_FRAME_WIDTH, int(1280)) # seems locked to 720p

    if not cap.isOpened():
        print("Cannot open camera")
        exit()

    camera_matrix = np.load('camera_matrix.npy')
    dist_coeffs = np.load('dist_coeffs.npy')
    logger = {}

    # define objects to be tracked
    origin = MarkerCollection()
    target = MarkerCollection()

    april_inner = 47 #85/9 * 5 # size of internal square
    target.register_marker(8, april_inner, [-73,0,0], R.from_euler(EULER_ORDER, [0, 0, -180], degrees=True))
    target.register_marker(9, april_inner, [25.5,0,0], R.from_euler(EULER_ORDER, [0, 0, -180], degrees=True))

    origin.register_marker(5, april_inner, [-69,24,0], R.from_euler(EULER_ORDER, [0, 0, 0], degrees=True))
    origin.register_marker(4, april_inner, [0,22.5,24], R.from_euler(EULER_ORDER, [0, -90, 0], degrees=True))
    #                        reference_tvec=[0,0,0],
    #                        reference_rotation=R.identity())
    origin.register_marker(2, april_inner, [-25,0,24], R.from_euler(EULER_ORDER, [180, 0, 90], degrees=True))
    #                        reference_tvec=[0,0,0],
    #                        reference_rotation=R.identity())
    # origin.register_marker(3, april_inner, [94.5,24,24], R.from_euler(EULER_ORDER, [0,-90, 0], degrees=True),
    #                        reference_tvec=[0,0,0],
    #                        reference_rotation=R.identity())
    # target.register_marker(10, april_inner, [0,0,0], R.identity())

    R_dist = 0.05
    Q_dist = np.array([[1, 0], [0, 1]])
    R_angle = 0.01
    Q_angle = np.array([[1, 0], [0, 1]])

    robot = RigidBodyTracker(Q_dist, R_dist, Q_angle, R_angle)
    last_robot_communicate = 0
    last_measurement_time = None
    robot_comm_dt = 0
    command = True

    path_segments = [
        [(0.3, 0.3), (0.6, 0.3), pi/2],
        [(0.6, 0.3), (0.6, 0.6), pi],
        [(0.6, 0.6), (0.3, 0.6), -pi/2],
        [(0.3, 0.6), (0.3, 0.3), 0],
        ]
    
    current_segment = 0

    ### Set up controllers
    forward_controller = FowardController(k_angle=5,
                                       k_v=0.2,
                                       w=0.5,
                                       goal_tolerance=0.01,
                                       reversing_allowed=False)

    # warning when tuning
    spin_controller = SpinController(k_angle=3,
                                     k_v=0.2,
                                     angle_tolerance=0.2
                                     )
    controller = LineFollowerController(forward_controller, spin_controller)
    p0, p1, theta_target = path_segments[current_segment]
    controller.set_path(p0, p1, theta_target)
    print(f"Currently moving along segment {current_segment}\n From {p0} to {p1} \nFinal orientation desired: {theta_target}")

    stats = {"x": [], "y": [], "z": [], "yaw": [], "pitch": [], "roll": []}

    config = AprilTagDetector.Config()
    config.debug = False
    config.decodeSharpening = 0.25
    config.numThreads = 4 # default 1
    config.quadDecimate = 1.0
    config.quadSigma = 0.0
    config.refineEdges = True
    detector = AprilTagDetector()
    detector.setConfig(config)
    detector.addFamily("tagStandard41h12")

    robot_comms = RobotUDP("192.168.4.1")

    # Main loop
    while True:
        ret, img = cap.read()
        if not ret:
            break

        ### UPDATE LOCALISATION
        # gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        valid, rot_relative, tvec_relative = process_image(img, camera_matrix, dist_coeffs, origin, target, detector, logger)
        measurement_time = time.time()

        dt = 0
        if last_measurement_time:
            dt = measurement_time - last_measurement_time
            # print(dt)
        # always run predict step
        robot.predict_estimate(dt)
        if valid:
            robot.update_estimate(rot_relative, tvec_relative, EULER_ORDER)
        
        last_measurement_time = measurement_time

        positions, angles = robot.predict_estimate(0)
        

        ### LOCALISATION FINISHED

        ### PATH PLANNING

        # check if goal was reached

        v, omega = 0, 0
        not_none = lambda x: x is not None
        if all([not_none(e) for e in positions]):
            x, _, y, _, _, _ = np.ravel(positions).tolist()
            theta, _, _, _, _, _ = np.ravel(angles).tolist()
            x = x /1000
            y = y/1000
            if controller.has_reached_goal():
                current_segment += 1
                current_segment %= len(path_segments)
                p0, p1, theta_target = path_segments[current_segment]
                controller.set_path(p0, p1, theta_target)
                print(f"Currently moving along segment {current_segment}\n From {p0} to {p1} \nFinal orientation desired: {theta_target}")
            
            v, omega = controller.get_control_action(x, y, theta)
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

        # if time.time() - last_robot_communicate > robot_comm_dt:
        #     if command:
        #         robot_communicate(1)
        #     else:
        #         robot_communicate(2)
        #     command = not command
        #     last_robot_communicate = time.time()

        # log raw data (not from kalman filter)
        labels = ["x", "y", "z", "yaw", "pitch", "roll"]
        if rot_relative and all(tvec_relative):
            raw_rotations = rot_relative.as_euler(EULER_ORDER, degrees=True)
            for index, item in enumerate(np.concatenate((tvec_relative, raw_rotations))):
                if item is None:
                    continue
                x = item.ravel().tolist()[0]
                stats[labels[index]].append(x)

                cv.putText(img, '{}: {:.3f}'.format(labels[index], x),
                        (50, 50 + 50*index),
                        cv.FONT_HERSHEY_PLAIN,
                        2,
                        (0, 255, 0),
                        4)

        cv.imshow('frame', img)
        if cv.waitKey(1) == ord('q'):
            break

    # print out the statistics for each marker we saw
    # only valid for stationary markers
    # for id in logger.keys():
    # print(f"STATS FOR MARKER {id}")
    # for i, letter in enumerate(["x", "y", "z"]):
    #     print(letter)
    #     print(f"Standard Deviation {np.std(logger[id][i])}")
    #     print(f"Mean {np.mean(logger[id][i])}")
    #     print(f"Max {np.max(logger[id][i])}")
    #     print(f"Min {np.min(logger[id][i])}")
    for name in stats.keys():
        array = stats[name]
        print(f"{name}: {np.mean(array)} (std: {np.std(array)})")
    cap.release()
    cv.destroyAllWindows()

    # stop the robot
    robot_comms.send_control_action(0,0, True)


if __name__ == "__main__":
    main()