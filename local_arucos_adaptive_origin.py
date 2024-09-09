import numpy as np
import cv2 as cv
from scipy.spatial.transform import Rotation as R

MARKER_SIZE = 100 #97 # mm
ORIGIN_X_DELTA = -59
ORIGIN_Y_DELTA = 146
RVECS = 0
TVECS = 1
CORNERS = 2

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

        actual_points = rotation.apply(base_points)
        actual_points = np.add(actual_points, tvec)

        if reference_rotation is not None and reference_tvec is not None:
            actual_points = reference_rotation.apply(actual_points)
            actual_points = np.add(actual_points, reference_tvec)
        
        if (reference_rotation is not None) ^ (reference_tvec is not None):
            # one provided but not the other
            raise Exception("One reference provided but not the other")
        
        self._points[id] = actual_points

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
        corner_collection = [corners[0]
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

        _, rvecs, tvecs = cv.solvePnP(relevant_object_points,
                                      relevant_corners,
                                      camera_matrix,
                                      dist_coeffs,
                                      False,
                                      cv.SOLVEPNP_SQPNP)
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
                pts = np.array(corners[0], dtype=np.int32)
                cv.polylines(img, [pts], True, color, thickness)


        

# copied from https://stackoverflow.com/questions/75750177/solve-pnp-or-estimate-pose-single-markers-which-is-better
def my_estimatePoseSingleMarkers(corners, marker_size, mtx, distortion):
    '''
    This will estimate the rvec and tvec for each of the marker corners detected by:
       corners, ids, rejectedImgPoints = detector.detectMarkers(image)
    corners - is an array of detected corners for each detected marker in the image
    marker_size - is the size of the detected markers
    mtx - is the camera matrix
    distortion - is the camera distortion matrix
    RETURN list of rvecs, tvecs, and trash (so that it corresponds to the old estimatePoseSingleMarkers())
    '''
    marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, -marker_size / 2, 0],
                              [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
    trash = []
    rvecs = []
    tvecs = []
    
    for c in corners:
        nada, R, t = cv.solvePnP(marker_points, c, mtx, distortion, False, cv.SOLVEPNP_IPPE_SQUARE)
        rvecs.append(R)
        tvecs.append(t)
        trash.append(nada)
    return rvecs, tvecs, trash

def solve_origin_pose(marker_size, corners, camera_matrix, dist_coeffs):
    """
    Returns rvec and tvec corresponding to origin
    """
    origin_marker_points = np.array([
        [-marker_size / 2, marker_size / 2, 0],
        [marker_size / 2, marker_size / 2, 0],
        [marker_size / 2, -marker_size / 2, 0],
        [-marker_size / 2, -marker_size / 2, 0],
        [-marker_size / 2 + ORIGIN_X_DELTA, marker_size / 2 + ORIGIN_Y_DELTA, 0],
        [marker_size / 2 + ORIGIN_X_DELTA, marker_size / 2 + ORIGIN_Y_DELTA, 0],
        [marker_size / 2 + ORIGIN_X_DELTA, -marker_size / 2 + ORIGIN_Y_DELTA, 0],
        [-marker_size / 2 + ORIGIN_X_DELTA, -marker_size / 2 + ORIGIN_Y_DELTA, 0]
        ], dtype=np.float32)

    nada, R, t = cv.solvePnP(origin_marker_points, corners, camera_matrix, dist_coeffs, False, cv.SOLVEPNP_IPPE)
    return R, t


origin = MarkerCollection()
origin.register_marker(0, MARKER_SIZE, [0, 0, 0], R.identity())
origin.register_marker(3, MARKER_SIZE, [-59, -146, 0], R.identity())

target = MarkerCollection()
target.register_marker(4, 90.5, [-90.5, 53.5, 0], R.from_euler(EULER_ORDER, [90, 0, 180], degrees=True))
# demonstrate specifying second relative to first
target.register_marker(5, 90.5, [-53.5, -132.5, 0], R.identity(), reference_rotation=R.from_euler(EULER_ORDER, [90, 0, 180], degrees=True), reference_tvec=[-90.5, 53.5, 0])

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

def process_image(img, camera_matrix, dist_coeffs, logger):
    dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_100)
    detector = cv.aruco.ArucoDetector(dictionary)
    corners_list, ids, _ = detector.detectMarkers(img)

    # find origin
    origin_found, rvec_camera_to_origin, p_origin_camera_frame = origin.estimate_pose(corners_list, ids, camera_matrix, dist_coeffs)
    if not origin_found:
        return False, None, None
    origin.annotate(corners_list, ids, img, (0, 255, 255), 10)
    cv.drawFrameAxes(img, camera_matrix, dist_coeffs, rvec_camera_to_origin, p_origin_camera_frame, 50,4)

    target_found, rvec_camera_to_target, p_target_camera_frame = target.estimate_pose(corners_list, ids, camera_matrix, dist_coeffs)
    if not target_found:
        return False, None, None
    target.annotate(corners_list, ids, img, (255, 255, 0), 10)

    rot_relative, tvec_relative = get_relative_pose(rvec_camera_to_origin, p_origin_camera_frame, rvec_camera_to_target, p_target_camera_frame)

    yaw, pitch, roll = rot_relative.as_euler('zyx',degrees=True)

    dist = np.linalg.norm(tvec_relative)

    # Draw data onto image
    x_origin, y_origin = get_frame_image_coords(camera_matrix, dist_coeffs, rvec_camera_to_origin, p_origin_camera_frame)
    x, y = get_frame_image_coords(camera_matrix, dist_coeffs, rvec_camera_to_target, p_target_camera_frame)
    # x_mid, y_mid = ((x_origin*0 + x) // 2, (y_origin*0 + y) // 2)
    x_mid, y_mid = (x, y)
    cv.line(img, (x_origin, y_origin), (x, y), (255, 0, 0), 2)
    cv.putText(img, 'x: {:.3f}'.format(tvec_relative[0]), (x_mid, y_mid), cv.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 4)
    cv.putText(img, 'y: {:.3f}'.format(tvec_relative[1]), (x_mid, y_mid+50), cv.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 4)
    cv.putText(img, 'yaw: {:.3f}'.format(yaw), (x_mid, y_mid+100), cv.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 4)
    cv.putText(img, 'dist: {:.3f}'.format(dist), (x_mid, y_mid+150), cv.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 4)

    # log the values for statistics

    #         if int(id[0]) not in logger:
    #             logger[int(id[0])] = [[],[],[]]
    #         for i in [0, 1, 2]:
    #             logger[int(id[0])][i].append(position_delta_origin_frame[i][0])


def main():
    cap = cv.VideoCapture(3, cv.CAP_DSHOW) # set to 2 to select external webcam
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, int(720)) # seems locked to 720p
    cap.set(cv.CAP_PROP_FRAME_WIDTH, int(1280)) # seems locked to 720p

    if not cap.isOpened():
        print("Cannot open camera")
        exit()

    camera_matrix = np.load('camera_matrix.npy')
    dist_coeffs = np.load('dist_coeffs.npy')
    logger = {}
    while True:
        ret, img = cap.read()
        if not ret:
            break

        process_image(img, camera_matrix, dist_coeffs, logger)
        cv.imshow('frame', img)
        if cv.waitKey(1) == ord('q'):
            break

    # print out the statistics for each marker we saw
    # only valid for stationary markers
    for id in logger.keys():
        print(f"STATS FOR MARKER {id}")
        for i, letter in enumerate(["x", "y", "z"]):
            print(letter)
            print(f"Standard Deviation {np.std(logger[id][i])}")
            print(f"Mean {np.mean(logger[id][i])}")
            print(f"Max {np.max(logger[id][i])}")
            print(f"Min {np.min(logger[id][i])}")
    cap.release()
    cv.destroyAllWindows()


if __name__ == "__main__":
    main()