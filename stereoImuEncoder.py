#!/usr/bin/env python3

import cv2
import depthai as dai
import time
import math
import numpy as np
from depthai_sdk import OakCamera, RecordType

resolution = (1280, 800)
def getMesh(calibData):
    M1 = np.array(calibData.getCameraIntrinsics(dai.CameraBoardSocket.LEFT, resolution[0], resolution[1]))
    d1 = np.array(calibData.getDistortionCoefficients(dai.CameraBoardSocket.LEFT))
    R1 = np.array(calibData.getStereoLeftRectificationRotation())
    M2 = np.array(calibData.getCameraIntrinsics(dai.CameraBoardSocket.RIGHT, resolution[0], resolution[1]))
    d2 = np.array(calibData.getDistortionCoefficients(dai.CameraBoardSocket.RIGHT))
    R2 = np.array(calibData.getStereoRightRectificationRotation())
    mapXL, mapYL = cv2.initUndistortRectifyMap(M1, d1, R1, M2, resolution, cv2.CV_32FC1)
    mapXR, mapYR = cv2.initUndistortRectifyMap(M2, d2, R2, M2, resolution, cv2.CV_32FC1)

    meshCellSize = 16
    meshLeft = []
    meshRight = []

    for y in range(mapXL.shape[0] + 1):
        if y % meshCellSize == 0:
            rowLeft = []
            rowRight = []
            for x in range(mapXL.shape[1] + 1):
                if x % meshCellSize == 0:
                    if y == mapXL.shape[0] and x == mapXL.shape[1]:
                        rowLeft.append(mapYL[y - 1, x - 1])
                        rowLeft.append(mapXL[y - 1, x - 1])
                        rowRight.append(mapYR[y - 1, x - 1])
                        rowRight.append(mapXR[y - 1, x - 1])
                    elif y == mapXL.shape[0]:
                        rowLeft.append(mapYL[y - 1, x])
                        rowLeft.append(mapXL[y - 1, x])
                        rowRight.append(mapYR[y - 1, x])
                        rowRight.append(mapXR[y - 1, x])
                    elif x == mapXL.shape[1]:
                        rowLeft.append(mapYL[y, x - 1])
                        rowLeft.append(mapXL[y, x - 1])
                        rowRight.append(mapYR[y, x - 1])
                        rowRight.append(mapXR[y, x - 1])
                    else:
                        rowLeft.append(mapYL[y, x])
                        rowLeft.append(mapXL[y, x])
                        rowRight.append(mapYR[y, x])
                        rowRight.append(mapXR[y, x])
            if (mapXL.shape[1] % meshCellSize) % 2 != 0:
                rowLeft.append(0)
                rowLeft.append(0)
                rowRight.append(0)
                rowRight.append(0)

            meshLeft.append(rowLeft)
            meshRight.append(rowRight)

    meshLeft = np.array(meshLeft)
    meshRight = np.array(meshRight)

    return meshLeft, meshRight

print("Creating Stereo Depth pipeline")
pipeline = dai.Pipeline()

camLeft = pipeline.create(dai.node.MonoCamera)
camRight = pipeline.create(dai.node.MonoCamera)
stereo = pipeline.create(dai.node.StereoDepth)
xoutLeft = pipeline.create(dai.node.XLinkOut)
xoutRight = pipeline.create(dai.node.XLinkOut)
xoutDisparity = pipeline.create(dai.node.XLinkOut)
xoutDepth = pipeline.create(dai.node.XLinkOut)
LeftRectified = pipeline.create(dai.node.XLinkOut)
RightRectified = pipeline.create(dai.node.XLinkOut)

camLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
camRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)
res = (
    dai.MonoCameraProperties.SensorResolution.THE_800_P
)
for monoCam in (camLeft, camRight):  # Common config
    monoCam.setResolution(res)
    monoCam.setFps(120.0)


stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
stereo.setRectifyEdgeFillColor(0)  # Black, to better see the cutout
stereo.setSubpixel(True)

xoutLeft.setStreamName("left")
xoutRight.setStreamName("right")
xoutDisparity.setStreamName("disparity")
xoutDepth.setStreamName("depth")
LeftRectified.setStreamName('rectifiedLeft')
RightRectified.setStreamName('rectifiedRight')

camLeft.out.link(stereo.left)
camRight.out.link(stereo.right)
stereo.syncedLeft.link(xoutLeft.input)
stereo.syncedRight.link(xoutRight.input)
stereo.disparity.link(xoutDisparity.input)
stereo.rectifiedLeft.link(LeftRectified.input)
stereo.rectifiedRight.link(RightRectified.input)

device = dai.Device()
calibData = device.readCalibration()
leftMesh, rightMesh = getMesh(calibData)

# Create pipeline
pipeline = dai.Pipeline()

imu = pipeline.create(dai.node.IMU)
xlinkOut = pipeline.create(dai.node.XLinkOut)
# enable ACCELEROMETER_RAW at 500 hz rate
imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 400)
# enable GYROSCOPE_RAW at 400 hz rate
imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 400)
# it's recommended to set both setBatchReportThreshold and setMaxBatchReports to 20 when integrating in a pipeline with a lot of input/output connections
# above this threshold packets will be sent in batch of X, if the host is not blocked and USB bandwidth is available
imu.setBatchReportThreshold(1)
# maximum number of IMU packets in a batch, if it's reached device will block sending until host can receive it
# if lower or equal to batchReportThreshold then the sending is always blocking on device
# useful to reduce device's CPU load  and number of lost packets, if CPU load is high on device side due to multiple nodes
imu.setMaxBatchReports(10)

xlinkOut.setStreamName('IMU')

# Link plugins IMU -> XLINK
imu.out.link(xlinkOut.input)

VideoEncoderLeft = pipeline.create(dai.node.VideoEncoder)
VideoEncoderRight = pipeline.create(dai.node.VideoEncoder)

ve1Out = pipeline.create(dai.node.XLinkOut)
ve3Out = pipeline.create(dai.node.XLinkOut)

ve1Out.setStreamName('ve1Out')
ve3Out.setStreamName('ve3Out')

# Setting to 26fps will trigger error
VideoEncoderLeft.setDefaultProfilePreset(120, dai.VideoEncoderProperties.Profile.H264_MAIN)
VideoEncoderRight.setDefaultProfilePreset(120, dai.VideoEncoderProperties.Profile.H264_MAIN)

stereo.rectifiedLeft.link(VideoEncoderLeft.input)
stereo.rectifiedRight.link(VideoEncoderRight.input)


VideoEncoderLeft.bitstream.link(ve1Out.input)
VideoEncoderRight.bitstream.link(ve3Out.input)
# Connect to device and start pipeline

with device:
    device.startPipeline(pipeline)


    def timeDeltaToMilliS(delta) -> float:
        return delta.total_seconds()*1000

    # Output queue for imu bulk packets
    imuQueue = device.getOutputQueue("IMU", maxSize=200, blocking=True)
    baseTs = None

    # Output queues will be used to get the encoded data from the output defined above
    outQ1 = device.getOutputQueue('ve1Out', maxSize=130, blocking=True)
    outQ3 = device.getOutputQueue('ve3Out', maxSize=130, blocking=True)

    # Processing loop
    with open('mono1.h264', 'wb') as fileMono1H264, open('mono2.h264', 'wb') as fileMono2H264, open('imuData.csv', 'w') as imuFile:
        print("Press Ctrl+C to stop encoding...")
                              
        while True:
            try:
                while imuQueue.has():
                    imuData = imuQueue.get()  # blocking call, will wait until a new data has arrived
                    imuPackets = imuData.packets
                    for imuPacket in imuPackets:
                        acceleroValues = imuPacket.acceleroMeter
                        gyroValues = imuPacket.gyroscope

                        acceleroTs = acceleroValues.getTimestampDevice()
                        gyroTs = gyroValues.getTimestampDevice()
                        if baseTs is None:
                            baseTs = acceleroTs if acceleroTs < gyroTs else gyroTs
                        acceleroTs = timeDeltaToMilliS(acceleroTs - baseTs)
                        gyroTs = timeDeltaToMilliS(gyroTs - baseTs)

                        imuF = "{:.06f}"
                        tsF = "{:.03f}"

                        imuFile.write(f"{tsF.format(acceleroTs)},{imuF.format(acceleroValues.x)},{imuF.format(acceleroValues.y)},{imuF.format(acceleroValues.z)},{imuF.format(gyroValues.x)},{imuF.format(gyroValues.y)},{imuF.format(gyroValues.z)}\n")

                # Empty each queue
                while outQ1.has():
                    outQ1.get().getData().tofile(fileMono1H264)

                while outQ3.has():
                    outQ3.get().getData().tofile(fileMono2H264)
            except KeyboardInterrupt:
                break

    print("To view the encoded data, convert the stream file (.h264/.h265) into a video file (.mp4), using commands below:")
    cmd = "ffmpeg -framerate 120 -i {} -c copy {}"
    print(cmd.format("mono1.h264", "mono1.mp4"))
    print(cmd.format("mono2.h264", "mono2.mp4"))
