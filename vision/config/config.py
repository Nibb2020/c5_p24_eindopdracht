import cv2 

# ==================================================
# Camera
# ==================================================

RGB_WIDTH = 640  # Breedte van RGB/NN-frame
RGB_HEIGHT = 640  # Hoogte van RGB/NN-frame
USB_SPEED = "HIGH"  # Gewenste USB-snelheid als tekstwaarde
USE_DEVICE_CALIBRATION = True  # Gebruik OAK-D EEPROM-calibratie wanneer mogelijk

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
RECONNECT_ATTEMPTS = 5  # Aantal pogingen per reconnectbatch

# ==================================================
# Watchdog
# ==================================================

WATCHDOG_TIMEOUT_SEC = 3.0  # Maximale tijd zonder frame voordat reconnect wordt gestart
WATCHDOG_INTERVAL_SEC = 0.5  # Interval tussen watchdogchecks

# ==================================================
# Depth ROI
# ==================================================

MIN_DEPTH_MM = 100  # Minimale geldige depthwaarde in mm
MAX_DEPTH_MM = 2000  # Maximale geldige depthwaarde in mm
ROI_SHRINK_FACTOR = 0.65  # Factor waarmee bbox-ROI wordt verkleind
DEPTH_BAND_MM = 80  # Depthband rondom mediaanwaarde voor PCA-masker
MIN_VALID_DEPTH_PIXELS = 20  # Minimum aantal geldige depthpixels

# ==================================================
# Dataset
# ==================================================

SAVE_DATASET_ON_REQUEST = True  # Sla datasetopname op bij succesvolle objectrequest
DATASET_FOLDER = "/home/student/c5_p24_eindproject_ws/src/c5_p24_eindopdracht/vision/dataset"  # Datasetbasispad

# ==================================================
# Model
# ==================================================

MODEL_PATH = "/home/student/c5_p24_eindproject_ws/src/c5_p24_eindopdracht/vision/models/YOLOv6_Nano-R2_COCO_512x288.rvc2.tar.xz"  # Pad naar YOLO-model

# ==================================================
# YOLO
# ==================================================

USE_YOLO = False  # Zet YOLO tijdelijk uit
YOLO_NUM_CLASSES = 5  # Aantal YOLO-klassen
YOLO_IOU_THRESHOLD = 0.5  # IOU threshold voor YOLO post-processing
YOLO_DEFAULT_CONFIDENCE = 0.85  # Standaard confidencegrens

YOLO_CLASS_MAPPING = {  # Mapping van klassenaam naar class-ID
    "Hooi": 0,
    "Kannon": 1,
    "Schatkist Rood": 2,
    "Krat": 3,
    "Schatkist Blauw": 4,
}

YOLO_CLASS_NAMES = {  # Mapping van class-ID naar klassenaam
    0: "Hooi",
    1: "Kannon",
    2: "Schatkist Rood",
    3: "Krat",
    4: "Schatkist Blauw",
}

# ==================================================
# ArUco World Calibration
# ==================================================

ARUCO_MARKER_ID = 0  # ID van de vaste world-reference marker
ARUCO_SIZE_M = 0.07368  # Fysieke markermaat in meters
ARUCO_DICTIONARY = cv2.aruco.DICT_4X4_50  # ArUco dictionary voor markerherkenning

ARUCO_WORLD_X = 0.0  # World X-positie van markerorigin
ARUCO_WORLD_Y = 0.0  # World Y-positie van markerorigin
ARUCO_WORLD_Z = 0.0  # World Z-positie van markerorigin

CAMERA_MATRIX = [  # Fallback cameramatrix wanneer EEPROM-calibratie faalt
    [1200.0, 0.0, 640.0],
    [0.0, 1200.0, 640.0],
    [0.0, 0.0, 1.0]
]

DIST_COEFFS = [0.0, 0.0, 0.0, 0.0, 0.0]  # Fallback distortioncoëfficiënten

# ==================================================
# Robot Filtering
# ==================================================

ROBOT_FILTER_ENABLED = True  # Schakel robotfiltering aan of uit
ROBOT_ALLOWED_CLASS_IDS = [0, 1, 2, 3, 4]  # Klasses die robot mag oppakken

ROBOT_MIN_X_M = -0.35  # Minimum world-X voor robotbereik
ROBOT_MAX_X_M = 0.35  # Maximum world-X voor robotbereik
ROBOT_MIN_Y_M = -0.35  # Minimum world-Y voor robotbereik
ROBOT_MAX_Y_M = 0.35  # Maximum world-Y voor robotbereik
ROBOT_MIN_Z_M = -0.05  # Minimum world-Z voor robotbereik
ROBOT_MAX_Z_M = 0.30  # Maximum world-Z voor robotbereik

ROBOT_MIN_CONFIDENCE = 0.85  # Minimum confidence voor robotselectie