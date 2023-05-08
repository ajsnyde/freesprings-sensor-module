#!/usr/bin/env python3

import cv2
import depthai as dai
import time
import math
import ms5837
import numpy as np
from gpiozero import LED

green = LED(17)
red = LED(27)

red.blink()
green.blink()

# depth sensor
sensor = ms5837.MS5837_30BA() # Default I2C bus is 1 (Raspberry Pi 3)

# We must initialize the sensor before reading it
if not sensor.init():
    print("Sensor could not be initialized")
    exit(1)

# We have to read values from sensor to update pressure and temperature
if not sensor.read():
    print("Sensor read failed!")
    exit(1)

# Create pipeline
pipeline = dai.Pipeline()

# Define sources and outputs

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

monoLeft = pipeline.create(dai.node.MonoCamera)
monoLeft.setFps(120)

monoRight = pipeline.create(dai.node.MonoCamera)
monoRight.setFps(120)
ve1 = pipeline.create(dai.node.VideoEncoder)
ve3 = pipeline.create(dai.node.VideoEncoder)

ve1Out = pipeline.create(dai.node.XLinkOut)
ve3Out = pipeline.create(dai.node.XLinkOut)

ve1Out.setStreamName('ve1Out')
ve3Out.setStreamName('ve3Out')

# Properties
monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

# Setting to 26fps will trigger error
ve1.setDefaultProfilePreset(120, dai.VideoEncoderProperties.Profile.H264_MAIN)
ve3.setDefaultProfilePreset(120, dai.VideoEncoderProperties.Profile.H264_MAIN)

# Linking
monoLeft.out.link(ve1.input)
monoRight.out.link(ve3.input)

ve1.bitstream.link(ve1Out.input)
ve3.bitstream.link(ve3Out.input)

# Connect to device and start pipeline
with dai.Device(pipeline) as dev:

    def timeDeltaToMilliS(delta) -> float:
        return delta.total_seconds()*1000

    # Output queue for imu bulk packets
    imuQueue = dev.getOutputQueue("IMU", maxSize=200, blocking=True)
    baseTs = None
    startSystemTime = None
    # Output queues will be used to get the encoded data from the output defined above
    outQ1 = dev.getOutputQueue('ve1Out', maxSize=130, blocking=True)
    outQ3 = dev.getOutputQueue('ve3Out', maxSize=130, blocking=True)

    red.off()
    green.on()
    # Processing loop
    with open('left.h264', 'wb') as fileMono1H264, open('right.h264', 'wb') as fileMono2H264, open('imuData.csv', 'w') as imuFile, open('depthSensorData.csv', 'w') as depthSensorFile:
        print("Press Ctrl+C to stop encoding...")
        depthSensorFile.write("timeMillis,mbar,meters,celsius,fahrenheit\n")
                              
        while True:
            try:
                if sensor.read():
                    depthSensorFile.write(("%0.3f,%0.1f,%0.3f,%0.2f,%0.2f\n") % (
                        time.time(),
                        sensor.pressure(), # Default is mbar (no arguments)
                        sensor.depth(),
                        sensor.temperature(), # Default is degrees C (no arguments)
                        sensor.temperature(ms5837.UNITS_Farenheit))) # Request Farenheit
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
                        if startSystemTime is None:
                            startSystemTime = time.time()
                        acceleroTs = timeDeltaToMilliS(acceleroTs - baseTs)
                        gyroTs = timeDeltaToMilliS(gyroTs - baseTs)
                        
                        systemTimeDelta = time.time() - startSystemTime

                        imuF = "{:.06f}"
                        tsF = "{:.03f}"

                        imuFile.write(f"{tsF.format(systemTimeDelta)},{tsF.format(acceleroTs)},{imuF.format(acceleroValues.x)},{imuF.format(acceleroValues.y)},{imuF.format(acceleroValues.z)},{imuF.format(gyroValues.x)},{imuF.format(gyroValues.y)},{imuF.format(gyroValues.z)}\n")

                # Empty each queue
                while outQ1.has():
                    outQ1.get().getData().tofile(fileMono1H264)

                while outQ3.has():
                    outQ3.get().getData().tofile(fileMono2H264)
            except KeyboardInterrupt:
                red.blink()
                green.off()
                break

    print("To view the encoded data, convert the stream file (.h264/.h265) into a video file (.mp4), using commands below:")
    cmd = "ffmpeg -framerate 120 -i {} -c copy {}"
    print(cmd.format("left.h264", "left.mp4"))
    print(cmd.format("right.h264", "right.mp4"))
