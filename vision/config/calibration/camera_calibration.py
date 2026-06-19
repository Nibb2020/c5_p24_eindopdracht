#!/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np
import pickle

# =====================================================
# README
# =====================================================
#
#   This program is intended as a calibration program for the OAK-D ai camera, 
#   and is to be used with a aruco checkerboard to calibrate said camera. 
#   This needs to be done whenever said camera gets moved.
#
# =====================================================
# CONFIG
# =====================================================

CHECKERBOARD = (9, 6)

SQUARE_SIZE_MM = 25.0

SAVE_FILE = "camera_calibration.pkl"

# =====================================================
# OBJECT POINTS
# =====================================================

objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)

objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

objp *= SQUARE_SIZE_MM

objpoints = []
imgpoints = []

# =====================================================
# DEPTHAI PIPELINE
# =====================================================

pipeline = dai.Pipeline()

cam = pipeline.createColorCamera()

cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_4_K)

cam.setPreviewSize(1280, 1280)
cam.setInterleaved(False)

xout = pipeline.createXLinkOut()
xout.setStreamName("rgb")

cam.preview.link(xout.input)

# =====================================================
# MAIN
# =====================================================

with dai.Device(pipeline, maxUsbSpeed=dai.UsbSpeed.HIGH) as device:

    q = device.getOutputQueue("rgb", maxSize=4, blocking=False)

    print()
    print("c = capture")
    print("q = calibrate and quit")
    print()

    image_count = 0

    while True:

        frame = q.get().getCvFrame()
        display = frame.copy()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

        if ret:
            cv2.drawChessboardCorners(display, CHECKERBOARD, corners, ret)

        cv2.imshow("Calibration", display)
        key = cv2.waitKey(1)

        if key == ord('c'):
            if ret:
                objpoints.append(objp)

                refined = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
                imgpoints.append(refined)
                image_count += 1

                print(f"Captured image {image_count}")

            else:
                print("Checkerboard not detected")

        elif key == ord('q'):
            break

# =====================================================
# CALIBRATION
# =====================================================

print()
print("Calculating calibration...")
print()

ret, camera_matrix, dist_coeffs, rvecs, tvecs = (cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None))

print()
print("Camera Matrix:")
print(camera_matrix)

print()
print("Distortion:")
print(dist_coeffs)

data = {"camera_matrix": camera_matrix, "dist_coeffs": dist_coeffs}

with open(SAVE_FILE, "wb") as f:
    pickle.dump(data, f)

print()
print("Saved calibration")
print(SAVE_FILE)

cv2.destroyAllWindows()