import numpy as np
import cv2 as cv
 
# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
 
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*8,3), np.float32)
objp[:,:2] = np.mgrid[0:8,0:6].T.reshape(-1,2)*24
print(objp)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.


# Below code taken from Miguel's OpenCV UQ Mars talk in 2023
# https://github.com/uqmars/workshop-resources/blob/master/CV/2023/4_camera_calibration.py

cap = cv.VideoCapture(0) # set to 2 to select external webcam
cap.set(cv.CAP_PROP_AUTOFOCUS, 0)
focus = 255  # min: 0, max: 255, increment:5
cap.set(28, focus)

if not cap.isOpened():
    print("Cannot open camera")
    exit()

while True:
    ret, img = cap.read()
    
    if not ret:
        break

    cv.imshow('img', img)
    c = cv.waitKey(1)
    if c == ord('q'):
        break
    if c == ord(' '):    
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        ret, corners = cv.findChessboardCorners(gray, (8, 6), None)
    
        if ret:
            objpoints.append(objp)
            # edited this to use refined corners as in OpenCV tutorial
            # https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
            corners_draw = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners_draw)
            cv.drawChessboardCorners(img, (8, 6), corners_draw, ret)
            
            cv.imshow('img', img)
            cv.waitKey(1)

ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

np.save("ret.npy", ret)
np.save("mtx.npy", mtx)
np.save("dist.npy", dist)
np.save("rvecs.npy", rvecs)
np.save("tvecs.npy", tvecs)

cv.destroyAllWindows()

