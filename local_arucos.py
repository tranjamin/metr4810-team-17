import numpy as np
import cv2 as cv

cap = cv.VideoCapture(2, cv.CAP_DSHOW)

if not cap.isOpened():
    print("Cannot open camera")
    exit()

camera_matrix = np.load('mtx.npy')
dist_coeffs = np.load('dist.npy')
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
        rvecs = []
        tvecs = []
        for corners, id in zip(corners_list, ids):
            pts = np.array(corners,dtype=np.int32)
            cv.polylines(img, pts, True, (0, 0, 255), 10)
            markerLength = 0.147 # 28 mm
            rvec, tvec, _ = my_estimatePoseSingleMarkers(corners, markerLength, camera_matrix, dist_coeffs)
            rvecs.append(rvec)
            tvecs.append(tvec)
        if len(ids) == 2:
            p1 = tvecs[0][0][0]
            p2 = tvecs[1][0][0]
            dist = np.linalg.norm(p1 - p2)
            x1, y1 = np.mean(corners_list[0][0], axis=0).astype(np.int64)
            x2, y2 = np.mean(corners_list[1][0], axis=0).astype(np.int64)
            x3, y3 = ((x1 + x2) // 2, (y1 + y2) // 2)
            cv.line(img, (x1, y1), (x2, y2), (255, 0, 0), 2)
            cv.putText(img, '{:.3f}'.format(dist), (x3, y3), cv.FONT_HERSHEY_PLAIN, 5, (0, 255, 0), 4)
            



    cv.imshow('frame', img)
    if cv.waitKey(1) == ord('q'):
        break

cap.release()
cv.destroyAllWindows()


