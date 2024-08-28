import numpy as np
import cv2 as cv

IMAGE_FOLDER = "tom_iphone_calibration"

image_counter = 0
# Below code taken from Miguel's OpenCV UQ Mars talk in 2023
# https://github.com/uqmars/workshop-resources/blob/master/CV/2023/4_camera_calibration.py

cap = cv.VideoCapture(0) # set to 2 to select external webcam
# cap.set(cv.CAP_PROP_FRAME_HEIGHT, int(720)) # seems locked to 720p
# cap.set(cv.CAP_PROP_FRAME_WIDTH, int(1280))
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
    c = cv.waitKey(1)
    if c == ord('q'):
        break
    if c == ord(' '):    
        cv.imwrite(f"{IMAGE_FOLDER}\\{image_counter}.jpg", img)
        image_counter += 1

cv.destroyAllWindows()