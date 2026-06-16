# ==================================================
# Camera
# ==================================================

RGB_WIDTH = 1280
RGB_HEIGHT = 1280
USB_SPEED = "HIGH"

# ==================================================
# ROS
# ==================================================

SERVICE_NAME = "/object_data"
UI_TOPIC = "/object_data_ui"
MARKED_IMAGE_TOPIC = "/marked_foto"
LITE6_RESULT_TOPIC = "/object_data_result"

# ==================================================
# Reconnect
# ==================================================

RECONNECT_INTERVAL = 1.0
RECONNECT_ATTEMPTS = 5

# ==================================================
# Dataset
# ==================================================

DATASET_FOLDER = ("/home/student/c5_p24_eindproject_ws/src/c5_p24_eindopdracht/vision/dataset")

# ==================================================
# Model
# ==================================================

MODEL_PATH = ("/home/student/c5_p24_eindproject_ws/src/c5_p24_eindopdracht/vision/models/yolov8n.rvc2.tar.xz")

# ==================================================
# ArUco
# ==================================================

ARUCO_MARKER_ID = 0
ARUCO_SIZE_MM = 50.0

# ==================================================
# YOLO
# ==================================================

YOLO_NUM_CLASSES = 4
YOLO_IOU_THRESHOLD = 0.5
YOLO_DEFAULT_CONFIDENCE = 0.85

# ==================================================
# Class Mapping
# ==================================================

YOLO_CLASS_MAPPING = {
    "Hooi": 0,
    "Kannon": 1,
    "Schatkist Rood": 2,
    "Krat": 3,
    "Schatkist Blauw": 4,
}

YOLO_CLASS_NAMES = {
    0: "Hooi",
    1: "Kannon",
    2: "Schatkist Rood",
    3: "Krat",
    4: "Schatkist Blauw",
}