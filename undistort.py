import cv2
import numpy as np
import depthai as dai
from depthai_sdk import OakCamera, RecordType

camRes = dai.MonoCameraProperties.SensorResolution.THE_800_P
camSocket = dai.CameraBoardSocket.LEFT
ispScale = (1,2)

calibData = dai.Device().readCalibration()

M1 = np.array(calibData.getCameraIntrinsics(camSocket, resizeWidth=1280, resizeHeight=800))
d1 = np.array(calibData.getDistortionCoefficients(camSocket))
R1 = np.identity(3)
mapX, mapY = cv2.initUndistortRectifyMap(M1, d1, R1, M1, [1280, 800], cv2.CV_32FC1)


leftRect = cv2.VideoWriter(
    "leftRect.avi", cv2.VideoWriter_fourcc(*'FMP4'), 2, (1280, 800))
rightRect = cv2.VideoWriter(
    "rightRect.avi", cv2.VideoWriter_fourcc(*'FMP4'), 2, (1280, 800))

left = cv2.VideoCapture("left.mp4")
right = cv2.VideoCapture("right.mp4")
i = 1
while True:
    print(i)
    i = i+1
    # Capture frame-by-frame
    ret, leftFrame = left.read()
    ret, rightFrame = right.read()
    # if frame is read correctly ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    # Our operations on the frame come here

    leftFrame = cv2.remap(leftFrame, mapX, mapY,  cv2.INTER_LINEAR, cv2.BORDER_CONSTANT, None)
    rightFrame = cv2.remap(rightFrame, mapX, mapY, cv2.INTER_LINEAR, cv2.BORDER_CONSTANT, None)

    leftRect.write(leftFrame)
    rightRect.write(rightFrame)

    if cv2.waitKey(1) == ord('q'):
        break


