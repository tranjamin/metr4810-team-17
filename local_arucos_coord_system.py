import numpy as np
import cv2 as cv

MARKER_SIZE = 100 # mm
cap = cv.VideoCapture(3, cv.CAP_DSHOW) # set to 2 to select external webcam
cap.set(cv.CAP_PROP_FRAME_HEIGHT, int(720)) # seems locked to 720p
cap.set(cv.CAP_PROP_FRAME_WIDTH, int(1280)) # seems locked to 720p

if not cap.isOpened():
    print("Cannot open camera")
    exit()

RVECS = 0
TVECS = 1
CORNERS = 2

camera_matrix = np.load('camera_matrix.npy')
dist_coeffs = np.load('dist_coeffs.npy')
distances = []

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

while True:
    ret, img = cap.read()
    if not ret:
        break

    dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_100)
    detector = cv.aruco.ArucoDetector(dictionary)
    (corners_list, ids, rejected) = detector.detectMarkers(img)
    ORIGIN_ID_1 = 0
    ORIGIN_ID_2 = 3
    if corners_list:
        markers = []
        rvecs = []
        tvecs = []
        found_origin = False
        for corners, id in zip(corners_list, ids):
            pts = np.array(corners,dtype=np.int32)
            cv.polylines(img, pts, True, (0, 0, 255), 10)
            markerLength = MARKER_SIZE
            rvec, tvec, _ = my_estimatePoseSingleMarkers(corners, markerLength, camera_matrix, dist_coeffs)
            markers.append((id, rvec, tvec, corners))
            if id == ORIGIN_ID:
                found_origin = True
                p_origin_camera_frame = tvec[0]
                rvec_camera_to_origin = rvec[0]
                origin_corners = markers[0][CORNERS]
                R_camera_to_origin, _ = cv.Rodrigues(rvec_camera_to_origin)
                x_origin, y_origin = np.mean(corners_list[0][0], axis=0).astype(np.int64)
                point = cv.drawFrameAxes(img, camera_matrix, dist_coeffs, rvec[0], tvec[0], 50,4)
        if len(ids) >= 2 and found_origin:
            # find other markers relative to origin
            for id, rvec, tvec, corners in markers:
                if id == ORIGIN_ID:
                    continue
                p_marker_camera_frame = tvec[0]
                rvec_camera_to_marker = rvec[0]
                position_delta_camera_frame = p_marker_camera_frame - p_origin_camera_frame
                R_camera_to_marker, _ = cv.Rodrigues(rvec_camera_to_marker)
                R_origin_to_marker = R_camera_to_origin.T @ R_camera_to_marker
                position_delta_origin_frame = R_camera_to_origin.T @ position_delta_camera_frame

                dist = np.linalg.norm(position_delta_origin_frame)
                # Draw data onto image
                x, y = np.mean(corners_list[1][0], axis=0).astype(np.int64)
                x_mid, y_mid = ((x_origin + x) // 2, (y_origin + y) // 2)
                cv.line(img, (x_origin, y_origin), (x, y), (255, 0, 0), 2)
                cv.putText(img, '{:.3f}'.format(position_delta_origin_frame[0][0]), (x_mid, y_mid), cv.FONT_HERSHEY_PLAIN, 2.5, (0, 255, 0), 4)
                cv.putText(img, '{:.3f}'.format(position_delta_origin_frame[1][0]), (x_mid, y_mid+50), cv.FONT_HERSHEY_PLAIN, 2.5, (0, 255, 0), 4)
                cv.putText(img, '{:.3f}'.format(position_delta_origin_frame[2][0]), (x_mid, y_mid+100), cv.FONT_HERSHEY_PLAIN, 2.5, (0, 255, 0), 4)
                cv.putText(img, '{:.3f}'.format(dist), (x_mid, y_mid+150), cv.FONT_HERSHEY_PLAIN, 2.5, (0, 255, 0), 4)
            


                
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
            
    cv.imshow('frame', img)
    if cv.waitKey(1) == ord('q'):
        break

print(f"Standard deviation {np.std(distances)}")
print(f"Mean {np.mean(distances)}")
print(f"Max {np.max(distances)}")
print(f"Min {np.min(distances)}")


cap.release()
cv.destroyAllWindows()


