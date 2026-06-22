# =========================================================
# Imports
# =========================================================

import math  # Nodig voor numerieke checks
import numpy as np  # Nodig voor matrixchecks

from config.config import *  # Importeer projectconfiguratie
from project_interfaces.msg import ObjectData  # Test ObjectData import
from project_interfaces.msg import ObjectDataArray  # Test ObjectDataArray import
from project_interfaces.srv import GetObjectData  # Test GetObjectData import

# =========================================================
# Interface Tests
# =========================================================

def test_project_interfaces_can_import():

    object_msg = ObjectData()  # Maak ObjectData testmessage
    array_msg = ObjectDataArray()  # Maak ObjectDataArray testmessage
    service_type = GetObjectData  # Lees servicetype

    assert object_msg is not None  # Controleer ObjectData bestaat
    assert array_msg is not None  # Controleer ObjectDataArray bestaat
    assert service_type is not None  # Controleer GetObjectData bestaat

# =========================================================
# Camera Config Tests
# =========================================================

def test_camera_resolution_is_valid():

    assert isinstance(RGB_WIDTH, int)  # Controleer breedte type
    assert isinstance(RGB_HEIGHT, int)  # Controleer hoogte type
    assert RGB_WIDTH > 0  # Controleer positieve breedte
    assert RGB_HEIGHT > 0  # Controleer positieve hoogte

def test_camera_matrix_shape_is_valid():

    camera_matrix = np.array(CAMERA_MATRIX, dtype=np.float32)  # Zet cameramatrix om naar NumPy
    dist_coeffs = np.array(DIST_COEFFS, dtype=np.float32)  # Zet distortion om naar NumPy

    assert camera_matrix.shape == (3, 3)  # Controleer 3x3 cameramatrix
    assert dist_coeffs.size >= 5  # Controleer minimaal vijf distortioncoëfficiënten
    assert camera_matrix[0, 0] > 0.0  # Controleer fx positief
    assert camera_matrix[1, 1] > 0.0  # Controleer fy positief

# =========================================================
# YOLO Config Tests
# =========================================================

def test_yolo_class_count_matches_names():

    assert YOLO_NUM_CLASSES == len(YOLO_CLASS_NAMES)  # Controleer aantal klasses
    assert len(YOLO_CLASS_MAPPING) == len(YOLO_CLASS_NAMES)  # Controleer mappinglengte
    assert YOLO_DEFAULT_CONFIDENCE >= 0.0  # Controleer minimale confidence
    assert YOLO_DEFAULT_CONFIDENCE <= 1.0  # Controleer maximale confidence

def test_model_path_is_configured():

    assert isinstance(MODEL_PATH, str)  # Controleer modelpad type
    assert len(MODEL_PATH) > 0  # Controleer modelpad niet leeg

# =========================================================
# ArUco Config Tests
# =========================================================

def test_aruco_config_is_valid():

    assert isinstance(ARUCO_MARKER_ID, int)  # Controleer marker-ID type
    assert ARUCO_MARKER_ID >= 0  # Controleer marker-ID positief
    assert ARUCO_SIZE_M > 0.0  # Controleer markermaat positief
    assert ARUCO_DICTIONARY is not None  # Controleer ArUco dictionary aanwezig

# =========================================================
# Depth ROI Config Tests
# =========================================================

def test_depth_roi_config_is_valid():

    assert MIN_DEPTH_MM > 0  # Controleer minimale depth
    assert MAX_DEPTH_MM > MIN_DEPTH_MM  # Controleer depthbereik
    assert ROI_SHRINK_FACTOR > 0.0  # Controleer ROI factor ondergrens
    assert ROI_SHRINK_FACTOR <= 1.0  # Controleer ROI factor bovengrens
    assert DEPTH_BAND_MM > 0  # Controleer depthband
    assert MIN_VALID_DEPTH_PIXELS > 0  # Controleer minimum aantal pixels

# =========================================================
# Robot Filter Config Tests
# =========================================================

def test_robot_filter_bounds_are_valid():

    assert ROBOT_MIN_X_M < ROBOT_MAX_X_M  # Controleer X-bereik
    assert ROBOT_MIN_Y_M < ROBOT_MAX_Y_M  # Controleer Y-bereik
    assert ROBOT_MIN_Z_M < ROBOT_MAX_Z_M  # Controleer Z-bereik
    assert ROBOT_MIN_CONFIDENCE >= 0.0  # Controleer robotconfidence ondergrens
    assert ROBOT_MIN_CONFIDENCE <= 1.0  # Controleer robotconfidence bovengrens
    assert len(ROBOT_ALLOWED_CLASS_IDS) > 0  # Controleer toegestane klasses