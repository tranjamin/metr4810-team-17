import cv2 as cv
import numpy as np
from robotpy_apriltag import AprilTagDetector
from scipy.spatial.transform import Rotation as R
from estimators import RigidBodyTracker
import time

MARKER_SIZE = 100 #97 # mm
ORIGIN_X_DELTA = -59
ORIGIN_Y_DELTA = 146
RVECS = 0
TVECS = 1
CORNERS = 2
LAST_ROBOT_COMMUNICATE = 0
EULER_ORDER = 'ZYX' # determines conversion from angles to rotations

class Localisation():
    def deinit(self):
        pass

    def setup(self):
        pass

    def get_position(self):
        pass


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
        if self.last_rvecs is not None and self.last_tvecs is not None: # and \
            # MAX_TVEC_CHANGE > np.linalg.norm(tvecs - self.last_tvecs):
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

class MockLocalisation(Localisation):
    def get_position(self, img):
        return (0, 0, 0, 0, 0, 0), (0, 0, 0, 0, 0, 0)

class CameraLocalisation(Localisation):
    def __init__(self):
        self.origin: MarkerCollection
        self.target: MarkerCollection
        self.stats = {"x": [], "y": [], "z": [], "yaw": [], "pitch": [], "roll": []}
    
    def setup(self):
        camera_matrix = np.load('camera_matrix.npy')
        dist_coeffs = np.load('dist_coeffs.npy')
        logger = {}

        # define objects to be tracked
        origin = MarkerCollection()
        target = MarkerCollection()

        # april_inner_bigger = 95 * 5 / 9
        # april_border = 95 * 2 / 9
        SIZE_1 = 107
        SIZE_2 = 100
        SIZE_3 = 90

        inner_size_factor = 5/9
        border_factor = 2/9
        mid_distance = 171.5
        left_box_width = 129.5 # mm, in the y direction
        right_box_width = 109.5 # mm, in the y direction
        mid_distance = 173.0375 # mm, between each acrylic plate
        left_box_depth = 169.5
        right_box_depth = 159
        right_box_lower = 5 # mm lower than horizontal plane of left box

        left_box_height = 109 # mm in z direction
        right_box_height = left_box_height

        t_to_left = [-left_box_depth / 2, mid_distance/2 + left_box_width, 0] 
        r_to_left = R.from_euler(EULER_ORDER, [-90, 0, -90], degrees=True)

        t_to_right = [right_box_depth / 2, -mid_distance/2 - right_box_width, -right_box_lower] 
        r_to_right = R.from_euler(EULER_ORDER, [90, 0, -90], degrees=True)

        target.register_marker(14,
                            SIZE_1 * inner_size_factor,
                            [left_box_width - border_factor * SIZE_1, left_box_height - border_factor * SIZE_1, 0],
                            R.from_euler(EULER_ORDER, [180,0,0], degrees=True),
                            reference_tvec=t_to_left,
                            reference_rotation=r_to_left
                            )

        target.register_marker(17,
                            SIZE_2 * inner_size_factor,
                            [102.5 - border_factor * SIZE_2, 0, 134.5 - border_factor * SIZE_2],
                            R.from_euler(EULER_ORDER, [90, 90, 0], degrees=True),
                            reference_tvec=t_to_left,
                            reference_rotation=r_to_left
                            )

        target.register_marker(5,
                            SIZE_1 * inner_size_factor,
                            [0, left_box_height - border_factor * SIZE_1, 138 - border_factor * SIZE_1],
                            R.from_euler(EULER_ORDER, [-90, 0, -90], degrees=True),
                            reference_tvec=t_to_left,
                            reference_rotation=r_to_left
                            )

        target.register_marker(12,
                            SIZE_1 * inner_size_factor,
                            [left_box_width - border_factor * SIZE_1, left_box_height - border_factor * SIZE_1, left_box_depth],
                            R.from_euler(EULER_ORDER, [-90, 0, 180], degrees=True),
                            reference_tvec=t_to_left,
                            reference_rotation=r_to_left
                            )

        # RIGHT SIDE
        target.register_marker(13,
                            SIZE_1 * inner_size_factor,
                            [right_box_width - border_factor * SIZE_1, right_box_height - border_factor * SIZE_1, 0],
                            R.from_euler(EULER_ORDER, [180,0,0], degrees=True),
                            reference_tvec=t_to_right,
                            reference_rotation=r_to_right
                            )

        target.register_marker(18,
                            SIZE_3 * inner_size_factor,
                            [93 - border_factor * SIZE_3, 0, 125 - border_factor * SIZE_3],
                            R.from_euler(EULER_ORDER, [90, 90, 0], degrees=True),
                            reference_tvec=t_to_right,
                            reference_rotation=r_to_right
                            )

        target.register_marker(15,
                            SIZE_1 * inner_size_factor,
                            [0, right_box_height - border_factor * SIZE_1, 133 - border_factor * SIZE_1],
                            R.from_euler(EULER_ORDER, [-90, 0, -90], degrees=True),
                            reference_tvec=t_to_right,
                            reference_rotation=r_to_right
                            )

        target.register_marker(16,
                            SIZE_1 * inner_size_factor,
                            [right_box_width - border_factor * SIZE_1, right_box_height - border_factor * SIZE_1, right_box_depth],
                            R.from_euler(EULER_ORDER, [-90, 0, 180], degrees=True),
                            reference_tvec=t_to_right,
                            reference_rotation=r_to_right
                            )

        # mid_distance = 171.5
        # left_box_width = 130
        # left_box_depth = 170

        # t_to_left = [left_box_depth / 2, -mid_distance/2 - left_box_width, 0] 
        # r_to_left = R.from_euler(EULER_ORDER, [90, 0, -90], degrees=True)
        # t_to_right = [-left_box_depth / 2, mid_distance/2 + left_box_width, 16] 
        # r_to_right = R.from_euler(EULER_ORDER, [-90, 0, -90], degrees=True)


        # target.register_marker(18,
        #                        april_inner_bigger,
        #                        [15+april_border,0,32.5+april_border],
        #                        R.from_euler(EULER_ORDER, [-90,-90,0], degrees=True),
        #                        reference_tvec=t_to_left,
        #                        reference_rotation=r_to_left
        #                        )
        
        # target.register_marker(16,
        #                        april_inner_bigger,
        #                        [0,4.5+april_border,33+april_border],
        #                        R.from_euler(EULER_ORDER, [90,0,90], degrees=True),
        #                        reference_tvec=t_to_left,
        #                        reference_rotation=r_to_left
        #                        )

        # target.register_marker(17,
        #                        april_inner_bigger,
        #                        [24+april_border,4+april_border,0],
        #                        R.from_euler(EULER_ORDER, [0,0,0], degrees=True),
        #                        reference_tvec=t_to_left,
        #                        reference_rotation=r_to_left
        #                        )

        # target.register_marker(15,
        #                        april_inner_bigger,
        #                        [24+april_border,4+april_border,169.5],
        #                        R.from_euler(EULER_ORDER, [90,0,180], degrees=True),
        #                        reference_tvec=t_to_left,
        #                        reference_rotation=r_to_left
        #                        )


        # target.register_marker(14,
        #                        april_inner_bigger,
        #                        [15+april_border,0,32.5+april_border],
        #                        R.from_euler(EULER_ORDER, [-90,-90,0], degrees=True),
        #                        reference_tvec=t_to_right,
        #                        reference_rotation=r_to_right
        #                        )
        
        # target.register_marker(5,
        #                        april_inner_bigger,
        #                        [0,4.5+april_border + april_inner_bigger,left_box_depth - 33 - april_border],
        #                        R.from_euler(EULER_ORDER, [-90,0,-90], degrees=True),
        #                        reference_tvec=t_to_right,
        #                        reference_rotation=r_to_right
        #                        )

        # target.register_marker(13,
        #                        april_inner_bigger,
        #                        [24+april_border,4+april_border,0],
        #                        R.from_euler(EULER_ORDER, [0,0,0], degrees=True),
        #                        reference_tvec=t_to_right,
        #                        reference_rotation=r_to_right
        #                        )

        # target.register_marker(12,
        #                        april_inner_bigger,
        #                        [24+april_border,4+april_border,169.5],
        #                        R.from_euler(EULER_ORDER, [90,0,180], degrees=True),
        #                        reference_tvec=t_to_right,
        #                        reference_rotation=r_to_right
        #                        )
        # Paper
        # target.register_marker(8, april_inner_bigger, [-73,0,0], R.from_euler(EULER_ORDER, [0, 0, -180], degrees=True))
        # target.register_marker(9, april_inner, [25.5,0,0], R.from_euler(EULER_ORDER, [0, 0, -180], degrees=True))


        # Dog food box:
        april_inner = 47 #85/9 * 5 # size of internal square
        # origin.register_marker(5, april_inner, [-69,24,0], R.from_euler(EULER_ORDER, [0, 0, 0], degrees=True))

        # THESE TWO
        origin.register_marker(4, april_inner, [0,22.5,24], R.from_euler(EULER_ORDER, [0, -90, 0], degrees=True))
        #                        reference_tvec=[0,0,0],
        #                        reference_rotation=R.identity())
        origin.register_marker(2, april_inner, [-25,0,24], R.from_euler(EULER_ORDER, [180, 0, 90], degrees=True))
        #                        reference_tvec=[0,0,0],
        #                        reference_rotation=R.identity())

        # NOT THESE
        # origin.register_marker(3, april_inner, [94.5,24,24], R.from_euler(EULER_ORDER, [0,-90, 0], degrees=True),
        #                        reference_tvec=[0,0,0],
        #                        reference_rotation=R.identity())
        # target.register_marker(10, april_inner, [0,0,0], R.identity())

        R_dist = 0.05
        Q_dist = np.array([[1, 0], [0, 1]])
        R_angle = 0.01
        Q_angle = np.array([[1, 0], [0, 1]])

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

        self.robot_tracker = RigidBodyTracker(Q_dist, R_dist, Q_angle, R_angle)
        self.last_measurement_time = None

        self.origin = origin
        self.target = target
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.logger = {}
        self.last_measurement_time = None
        self.detector = detector
        self.config = config
        self.stats = {"x": [], "y": [], "z": [], "yaw": [], "pitch": [], "roll": []}

    def get_position(self, img):
        ### UPDATE LOCALISATION
        # gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        valid, rot_relative, tvec_relative = process_image(img, self.camera_matrix, self.dist_coeffs, self.origin, self.target, self.detector, self.logger)
        measurement_time = time.time()

        dt = 0
        if self.last_measurement_time:
            dt = measurement_time - self.last_measurement_time
            # print(dt)
        # always run predict step
        self.robot_tracker.predict_estimate(dt)
        if valid:
            self.robot_tracker.update_estimate(rot_relative, tvec_relative, EULER_ORDER)
        
        self.last_measurement_time = measurement_time

        positions, angles = self.robot_tracker.predict_estimate(0)

        # log raw data (not from kalman filter)
        stats = self.stats
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


        return positions, angles
