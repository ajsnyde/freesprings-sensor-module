import cv2
import numpy as np
import depthai as dai
import pickle
from os.path import exists

camRes = dai.MonoCameraProperties.SensorResolution.THE_800_P
camSocket = dai.CameraBoardSocket.LEFT
ispScale = (1,2)

# read existing camera properties from file if exists, otherwise get from camera.
# This is nice when processing on another machine without the cam hooked up.
if exists("cameraIntrinsics") and exists("cameraDistortionCoefficients"):
    with open("cameraIntrinsics", "rb") as f:
        cameraIntrinsics = pickle.load(f)
    with open("cameraDistortionCoefficients", "rb") as f:
        cameraDistortionCoefficients = pickle.load(f)
else:
    calibData = dai.Device().readCalibration()
    with open("cameraIntrinsics", "wb") as f:
        cameraIntrinsics = calibData.getCameraIntrinsics(camSocket, resizeWidth=1280, resizeHeight=800)
        pickle.dump(cameraIntrinsics, f)
    with open("cameraDistortionCoefficients", "wb") as f:
        cameraDistortionCoefficients = calibData.getDistortionCoefficients(camSocket)
        pickle.dump(cameraDistortionCoefficients, f)


M1 = np.array(cameraIntrinsics)
d1 = np.array(cameraDistortionCoefficients)
R1 = np.identity(3)
mapX, mapY = cv2.initUndistortRectifyMap(M1, d1, R1, M1, [1280, 800], cv2.CV_32FC1)


leftRect = cv2.VideoWriter(
    "leftRect.avi", cv2.VideoWriter_fourcc(*'FMP4'), 120, (1280, 800))
rightRect = cv2.VideoWriter(
    "rightRect.avi", cv2.VideoWriter_fourcc(*'FMP4'), 120, (1280, 800))

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


