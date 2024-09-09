import numpy as np
import cv2 as cv

IMAGE_FOLDER = "c270_calibration"

criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
image_counter = 0
# Below code taken from Miguel's OpenCV UQ Mars talk in 2023
# https://github.com/uqmars/workshop-resources/blob/master/CV/2023/4_camera_calibration.py

cap = cv.VideoCapture(3, cv.CAP_DSHOW) # set to 2 to select external webcam
cap.set(cv.CAP_PROP_FRAME_HEIGHT, int(720)) # seems locked to 720p
cap.set(cv.CAP_PROP_FRAME_WIDTH, int(1280))
print(f"{cap.get(cv.CAP_PROP_FRAME_WIDTH)}, {cap.get(cv.CAP_PROP_FRAME_HEIGHT)}")
cap.set(cv.CAP_PROP_AUTOFOCUS, 0)
# focus = 255  # min: 0, max: 255, increment:5
# cap.set(28, focus)


if not cap.isOpened():
    print("Cannot open camera")
    exit()

while True:
    ret, img = cap.read()
    
    if not ret:
        break

    cv.imshow('img', img)
    # gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # ret, corners = cv.findChessboardCorners(gray, (8,6), None)
    # if ret:
    #     corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
    #     cv.drawChessboardCorners(gray, (8,6), corners2, ret)
    cv.imshow('img', img)
    c = cv.waitKey(1)
    if c == ord('q'):
        break
    if c == ord(' '):    
        cv.imwrite(f"{IMAGE_FOLDER}\\{image_counter}.jpg", img)
        image_counter += 1

cv.destroyAllWindows()