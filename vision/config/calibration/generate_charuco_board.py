#!/usr/bin/env python3

import cv2
import numpy as np
from pathlib import Path

# =====================================================
# CONFIG
# =====================================================

SQUARES_X = 7
SQUARES_Y = 5

SQUARE_LENGTH = 0.03   # meters (belangrijk!)
MARKER_LENGTH = 0.022  # meters

DICT = cv2.aruco.DICT_4X4_50

OUTPUT_FILE = "charuco_board1.png"

PIXELS_X = 42000
PIXELS_Y = 30000

# =====================================================
# CREATE ARUCO DICT
# =====================================================

aruco_dict = cv2.aruco.getPredefinedDictionary(DICT)

# =====================================================
# CREATE CHARUCO BOARD
# =====================================================

board = cv2.aruco.CharucoBoard((SQUARES_X, SQUARES_Y), SQUARE_LENGTH, MARKER_LENGTH, aruco_dict)

# =====================================================
# GENERATE IMAGE (NEW API)
# =====================================================

img = np.zeros((PIXELS_Y, PIXELS_X), dtype=np.uint8)
img = cv2.aruco.drawPlanarBoard(board, (PIXELS_X, PIXELS_Y), int(5), 1, 1)

# =====================================================
# SAVE
# =====================================================

cv2.imwrite(OUTPUT_FILE, img)

print("Saved:", Path(OUTPUT_FILE).resolve())