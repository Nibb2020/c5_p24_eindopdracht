import cv2 
import numpy as np

# ==================================================
# Debug
# ==================================================

DEBUG_PUBLISH_CONTINUOUS = False  # True = live marked image constant publiceren, False = alleen publiceren bij service request
DEBUG_DRAW_ARUCO_LIVE = True  # True = ArUco tekenen in debugbeeld, False = live debugbeeld zonder ArUco-overlay

DEBUG_LOG_DEVICE_INFO = True  # Print OAK-D device info bij opstarten
DEBUG_LOG_SYNC = True  # Print synced packet sequence nummers
DEBUG_LOG_FRAME_INFO = True  # Print framevormen en aantal ruwe detecties
DEBUG_LOG_ARUCO_POSE = True  # Print ArUco rvec/tvec/reprojection details
DEBUG_LOG_TRANSFORM_POINTS = True  # Print point_camera en point_marker
DEBUG_LOG_SAMPLE_RESULT = True  # Print alleen eindresultaat per service request
DEBUG_LOG_REJECTED_SAMPLES = True  # Print geweigerde samples tijdens stabiele meting

# ==================================================
# Camera
# ==================================================

RGB_WIDTH = 640  # Breedte van RGB/NN-frame
RGB_HEIGHT = 640  # Hoogte van RGB/NN-frame
USB_SPEED = "HIGH"  # Gewenste USB-snelheid als tekstwaarde
USE_DEVICE_CALIBRATION = False  # Gebruik OAK-D EEPROM-calibratie wanneer mogelijk

# ==================================================
# ROS
# ==================================================

SERVICE_NAME = "/vision/voorwerp_data"  # Service naam voor objectaanvragen
UI_TOPIC = "/vision/object_data_ui"  # Topic voor alle objecten richting UI
MARKED_IMAGE_TOPIC = "/vision/marked_foto"  # Topic voor live/gemarkeerde afbeelding
LITE6_RESULT_TOPIC = "/vision/object_data_result"  # Topic voor beste object richting Lite6/debug

# ==================================================
# Reconnect
# ==================================================

RECONNECT_INTERVAL = 1.0  # Wachttijd tussen reconnectpogingen in seconden
RECONNECT_ATTEMPTS = 5    # Aantal pogingen per reconnectbatch

# ==================================================
# Watchdog
# ==================================================

WATCHDOG_TIMEOUT_SEC = 3.0  # Maximale tijd zonder frame voordat reconnect wordt gestart
WATCHDOG_INTERVAL_SEC = 0.5 # Interval tussen watchdogchecks

# ==================================================
# Depth ROI
# ==================================================

MIN_DEPTH_MM = 100          # Minimale geldige depthwaarde in mm
MAX_DEPTH_MM = 2000         # Maximale geldige depthwaarde in mm
ROI_SHRINK_FACTOR = 0.65    # Factor waarmee bbox-ROI wordt verkleind
DEPTH_BAND_MM = 80          # Depthband rondom mediaanwaarde voor PCA-masker
MIN_VALID_DEPTH_PIXELS = 20 # Minimum aantal geldige depthpixels

# ==================================================
# Object Measurement Filtering
# ==================================================

OBJECT_SAMPLE_COUNT = 3  # Aantal geldige objectmetingen per service-call
OBJECT_SAMPLE_TIMEOUT_SEC = 4  # Maximale zoektijd voor alle metingen samen
OBJECT_MAX_POSITION_STD_M = 0.002  # Maximale standaarddeviatie in meters voor betrouwbare XYZ-meting
OBJECT_MAX_YAW_STD_RAD = 0.08  # Maximale yaw-spreiding in radialen voor betrouwbare rotatiemeting

# ==================================================
# Dataset
# ==================================================

SAVE_DATASET_ON_REQUEST = False  # Sla datasetopname op bij succesvolle objectrequest
DATASET_FOLDER = "/home/student/c5_p24_eindproject_ws/src/c5_p24_eindopdracht/vision/dataset"  # Datasetbasispad

# ==================================================
# Model
# ==================================================
Version = "V0.5"
MODEL_PATH = f"/home/student/c5_p24_eindproject_ws/src/c5_p24_eindopdracht/vision/models/{Version}/best_openvino_2022.1_3shave.blob"  # Pad naar YOLO-model

# ==================================================
# YOLO
# ==================================================

USE_YOLO = True                 # Zet YOLO tijdelijk uit
YOLO_NUM_CLASSES = 4            # Aantal YOLO-klassen
YOLO_IOU_THRESHOLD = 0.5        # IOU threshold voor YOLO post-processing
YOLO_DEFAULT_CONFIDENCE = 0.85  # Standaard confidencegrens

YOLO_CLASS_MAPPING = {          # Mapping van klassenaam naar class-ID
    "Bootje": 0,
    "Dino": 1,
    "Olifantje": 2,
    "Smiley": 3,
}

YOLO_CLASS_NAMES = {  # Mapping van class-ID naar klassenaam
    0: "schip",
    1: "dino",
    2: "olifant",
    3: "smiley",
}

# ==================================================
# ArUco World Calibration
# ==================================================

ARUCO_MARKER_ID = 0  # ID van de vaste world-reference marker
ARUCO_SIZE_M = 0.07085  # Fysieke markermaat in meters 356
ARUCO_DICTIONARY = cv2.aruco.DICT_4X4_50  # ArUco dictionary voor markerherkenning

ARUCO_MAX_REPROJECTION_ERROR_PX = 3.0  # Maximale toegestane reprojection error voor ArUco-pose in pixels
ARUCO_USE_POSE_VALIDATION = True  # True = ArUco-pose controleren op reprojection error en sprongen

ARUCO_WORLD_X = 0.0712       # World X-positie van markerorigin
ARUCO_WORLD_Y = 0.277       # World Y-positie van markerorigin
ARUCO_WORLD_Z = 0.08      # World Z-positie van markerorigin

CAMERA_MATRIX = np.array([
    [919.80036341, 0.00000000, 321.10036797],
    [0.00000000, 918.88387367, 311.29307596],
    [0.00000000, 0.00000000, 1.00000000],
], dtype=np.float32)

DIST_COEFFS = np.array([0.1502961560, -0.9671998692, -0.0037365286, 0.0022325322, 3.8777281358], dtype=np.float32)

ARUCO_TO_ROBOT_ROTATION = np.array(
    [
        [0.0, 1.0, 0.0],
        [-1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    ],
    dtype=np.float64
)

# ==================================================
# Robot Filtering
# ==================================================

ROBOT_FILTER_ENABLED = False  # Schakel robotfiltering aan of uit
ROBOT_ALLOWED_CLASS_IDS = [0, 1, 2, 3,]  # Klasses die robot mag oppakken

ROBOT_MIN_X_M = 0.736   # Minimum world-X voor robotbereik
ROBOT_MAX_X_M = 0.855    # Maximum world-X voor robotbereik
ROBOT_MIN_Y_M = 0.2520   # Minimum world-Y voor robotbereik
ROBOT_MAX_Y_M = 0.4875    # Maximum world-Y voor robotbereik
ROBOT_MIN_Z_M = 0.0883   # Minimum world-Z voor robotbereik
ROBOT_MAX_Z_M = 0.12    # Maximum world-Z voor robotbereik

ROBOT_MIN_CONFIDENCE = 0.85  # Minimum confidence voor robotselectie