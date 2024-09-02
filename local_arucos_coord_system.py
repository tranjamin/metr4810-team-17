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

    if corners_list:
        markers = {}
        rvecs = []
        tvecs = []
        for corners, id in zip(corners_list, ids):
            pts = np.array(corners,dtype=np.int32)
            cv.polylines(img, pts, True, (0, 0, 255), 10)
            markerLength = MARKER_SIZE # 28 mm
            rvec, tvec, _ = my_estimatePoseSingleMarkers(corners, markerLength, camera_matrix, dist_coeffs)
            markers[id] = (rvec, tvec)
        if len(ids) == 2:
            # setup origin at marker 0
            p_origin_camera_frame = markers[0][TVECS][0]
            rvec_camera_to_origin = markers[0][RVECS][0]
            
            # find other markers relative to origin
            for id, vecs in markers:
                if id == 0:
                    continue
                p_marker_camera_frame = vecs[TVECS][0]
                rvec_camera_to_marker = vecs[RVECS][0]
                
            p1 = tvecs[0][0]#[0] Let's start by NOT IGNORING THE TWO OTHER COMPONENTS...
            p2 = tvecs[1][0]#[0]
            # print(f"1 --> {np.linalg.norm(p1)}")
            # print(f"2 --> {np.linalg.norm(p2)}")
            r1 = rvecs[0][0]
            r2 = rvecs[1][0]
            R_camera_to_1, _ = cv.Rodrigues(r1)
            R_camera_to_2, _ = cv.Rodrigues(r2)
            p1c = -R1.T @ p1
            p2c = -R2.T @ p2

            for i in [0,1]:
                point = cv.drawFrameAxes(img, camera_matrix, dist_coeffs, rvecs[i][0], tvecs[i][0], 50,4)
            dist = np.linalg.norm(p1 - p2)
            vector = p1 - p2
            distances.append(dist)
            x1, y1 = np.mean(corners_list[0][0], axis=0).astype(np.int64)
            x2, y2 = np.mean(corners_list[1][0], axis=0).astype(np.int64)
            x3, y3 = ((x1 + x2) // 2, (y1 + y2) // 2)
            cv.line(img, (x1, y1), (x2, y2), (255, 0, 0), 2)
            cv.putText(img, '{:.3f}'.format(vector[0][0]), (x3, y3), cv.FONT_HERSHEY_PLAIN, 2.5, (0, 255, 0), 4)
            cv.putText(img, '{:.3f}'.format(vector[1][0]), (x3, y3+50), cv.FONT_HERSHEY_PLAIN, 2.5, (0, 255, 0), 4)
            cv.putText(img, '{:.3f}'.format(vector[2][0]), (x3, y3+100), cv.FONT_HERSHEY_PLAIN, 2.5, (0, 255, 0), 4)
            cv.putText(img, '{:.3f}'.format(dist), (x3, y3+150), cv.FONT_HERSHEY_PLAIN, 2.5, (0, 255, 0), 4)
            

    cv.imshow('frame', img)
    if cv.waitKey(1) == ord('q'):
        break

print(f"Standard deviation {np.std(distances)}")
print(f"Mean {np.mean(distances)}")
print(f"Max {np.max(distances)}")
print(f"Min {np.min(distances)}")


cap.release()
cv.destroyAllWindows()


