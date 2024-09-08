import numpy as np
import cv2 as cv
from scipy.spatial.transform import Rotation as R

MARKER_SIZE = 100 #97 # mm
ORIGIN_X_DELTA = -59
ORIGIN_Y_DELTA = 146
RVECS = 0
TVECS = 1
CORNERS = 2


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

def process_image(img, camera_matrix, dist_coeffs, logger):
    dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_100)
    detector = cv.aruco.ArucoDetector(dictionary)
    (corners_list, ids, rejected) = detector.detectMarkers(img)
    ORIGIN_ID_1 = 0
    ORIGIN_ID_2 = 3

    if ids is None:
        return
    # step 1: find the origin constellation

    if ORIGIN_ID_1 in ids and ORIGIN_ID_2 in ids:
        # origin was detected
        first_marker_index = int(np.where(ids==ORIGIN_ID_1)[0][0])
        x_origin, y_origin = np.mean(corners_list[first_marker_index][0], axis=0).astype(np.int64)
        second_marker_index = int(np.where(ids==ORIGIN_ID_2)[0][0])
        first_corners = corners_list[first_marker_index]
        second_corners = corners_list[second_marker_index]
        all_corners = np.concatenate((first_corners[0], second_corners[0]))

        # draw green boxes around origin markers
        pts = np.array(first_corners,dtype=np.int32)
        cv.polylines(img, pts, True, (0, 255, 0), 10)
        pts = np.array(second_corners,dtype=np.int32)
        cv.polylines(img, pts, True, (0, 255, 0), 10)

        origin_rvec, p_origin_camera_frame = solve_origin_pose(MARKER_SIZE, all_corners, camera_matrix, dist_coeffs) 
        R_camera_to_origin, _ = cv.Rodrigues(origin_rvec)
        point = cv.drawFrameAxes(img, camera_matrix, dist_coeffs, origin_rvec, p_origin_camera_frame, 50,4)
    else:
        return

    # Origin found, let's see if there are any other markers visible,
    # Then find their positions in the origin frame


    if corners_list:
        markers = []
        rvecs = []
        tvecs = []
        found_origin = False
        for corners, id in zip(corners_list, ids):
            if id == ORIGIN_ID_1 or id == ORIGIN_ID_2:
                continue
            # marker is not one of the origin ones

            pts = np.array(corners,dtype=np.int32)
            cv.polylines(img, pts, True, (0, 0, 255), 10)
            markerLength = 88 # wow I love magic, MARKER_SIZE
            rvec, tvec, _ = my_estimatePoseSingleMarkers(corners, markerLength, camera_matrix, dist_coeffs)
            p_marker_camera_frame = tvec[0]
            rvec_camera_to_marker = rvec[0]
            position_delta_camera_frame = p_marker_camera_frame - p_origin_camera_frame
            R_camera_to_marker, _ = cv.Rodrigues(rvec_camera_to_marker)
            R_origin_to_marker = R_camera_to_origin.T @ R_camera_to_marker
            position_delta_origin_frame = R_camera_to_origin.T @ position_delta_camera_frame
            r = R.from_matrix(R_origin_to_marker)
            yaw, pitch, roll = r.as_euler('zyx',degrees=True)

            dist = np.linalg.norm(position_delta_origin_frame)
            # Draw data onto image
            x, y = np.mean(corners[0], axis=0).astype(np.int64)
            # x_mid, y_mid = ((x_origin*0 + x) // 2, (y_origin*0 + y) // 2)
            x_mid, y_mid = (x, y)
            cv.line(img, (x_origin, y_origin), (x, y), (255, 0, 0), 2)
            cv.putText(img, '{:.3f}'.format(position_delta_origin_frame[0][0]), (x_mid, y_mid), cv.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 4)
            cv.putText(img, '{:.3f}'.format(position_delta_origin_frame[1][0]), (x_mid, y_mid+50), cv.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 4)
            cv.putText(img, '{:.3f}'.format(yaw), (x_mid, y_mid+100), cv.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 4)
            cv.putText(img, '{:.3f}'.format(dist), (x_mid, y_mid+150), cv.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 4)

            # log the values for statistics

            if int(id[0]) not in logger:
                logger[int(id[0])] = [[],[],[]]
            for i in [0, 1, 2]:
                logger[int(id[0])][i].append(position_delta_origin_frame[i][0])

                

            # markers.append((id, rvec, tvec, corners))
            # if id == ORIGIN_ID:
            #     found_origin = True
            #     p_origin_camera_frame = tvec[0]
            #     rvec_camera_to_origin = rvec[0]
            #     origin_corners = markers[0][CORNERS]
            #     R_camera_to_origin, _ = cv.Rodrigues(rvec_camera_to_origin)
            #     x_origin, y_origin = np.mean(corners_list[0][0], axis=0).astype(np.int64)
            #     point = cv.drawFrameAxes(img, camera_matrix, dist_coeffs, rvec[0], tvec[0], 50,4)
        # if len(ids) >= 2 and found_origin:
        #     # find other markers relative to origin
        #     for id, rvec, tvec, corners in markers:
        #         if id == ORIGIN_ID:
        #             continue
        #         p_marker_camera_frame = tvec[0]
        #         rvec_camera_to_marker = rvec[0]
        #         position_delta_camera_frame = p_marker_camera_frame - p_origin_camera_frame
        #         R_camera_to_marker, _ = cv.Rodrigues(rvec_camera_to_marker)
        #         R_origin_to_marker = R_camera_to_origin.T @ R_camera_to_marker
        #         position_delta_origin_frame = R_camera_to_origin.T @ position_delta_camera_frame

        #         dist = np.linalg.norm(position_delta_origin_frame)
        #         # Draw data onto image
        #         x, y = np.mean(corners_list[1][0], axis=0).astype(np.int64)
        #         x_mid, y_mid = ((x_origin + x) // 2, (y_origin + y) // 2)
        #         cv.line(img, (x_origin, y_origin), (x, y), (255, 0, 0), 2)
        #         cv.putText(img, '{:.3f}'.format(position_delta_origin_frame[0][0]), (x_mid, y_mid), cv.FONT_HERSHEY_PLAIN, 2.5, (0, 255, 0), 4)
        #         cv.putText(img, '{:.3f}'.format(position_delta_origin_frame[1][0]), (x_mid, y_mid+50), cv.FONT_HERSHEY_PLAIN, 2.5, (0, 255, 0), 4)
        #         cv.putText(img, '{:.3f}'.format(position_delta_origin_frame[2][0]), (x_mid, y_mid+100), cv.FONT_HERSHEY_PLAIN, 2.5, (0, 255, 0), 4)
        #         cv.putText(img, '{:.3f}'.format(dist), (x_mid, y_mid+150), cv.FONT_HERSHEY_PLAIN, 2.5, (0, 255, 0), 4)
            


                
            # p1 = tvecs[0][0]#[0] Let's start by NOT IGNORING THE TWO OTHER COMPONENTS...
            # p2 = tvecs[1][0]#[0]
            # # print(f"1 --> {np.linalg.norm(p1)}")
            # # print(f"2 --> {np.linalg.norm(p2)}")
            # r1 = rvecs[0][0]
            # r2 = rvecs[1][0]
            # R_camera_to_1, _ = cv.Rodrigues(r1)
            # R_camera_to_2, _ = cv.Rodrigues(r2)
            # p1c = -R1.T @ p1
            # p2c = -R2.T @ p2

            # for i in [0,1]:
            #     point = cv.drawFrameAxes(img, camera_matrix, dist_coeffs, rvecs[i][0], tvecs[i][0], 50,4)
            # dist = np.linalg.norm(p1 - p2)
            # vector = p1 - p2
            # distances.append(dist)
        

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