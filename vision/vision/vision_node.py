#!/usr/bin/env python3  # Gebruik Python 3 als interpreter

"""
=========================================================
Vision Node
=========================================================

Package:
    vision

ROS2:
    Jazzy

DepthAI:
    2.29.0

Responsibilities:
    - OAK-D connection
    - USB HIGH enforcement
    - Device information logging
    - Auto reconnect
    - RGB stream
    - Service interface
    - UI publisher
    - Future:
        * Stereo depth
        * YOLOv8n
        * ArUco calibration
        * PCA yaw
        * Dataset logging
        * Lite 6 integration

Author:
    Hessel / ChatGPT
=========================================================

cd ~/c5_p24_eindproject_ws
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
source install/setup.bash

python3 -m colcon build \
  --packages-select vision \
  --symlink-install \
  --allow-overriding project_interfaces

source install/setup.bash
ros2 launch vision vision.launch.py

"""

# =========================================================
# Imports
# =========================================================

import rclpy  # ROS2 Python client library
from rclpy.node import Node  # Basisclass voor een ROS2 node

from sensor_msgs.msg import Image  # ROS2 Image message voor camerabeelden

from cv_bridge import CvBridge  # Conversie tussen OpenCV images en ROS Image messages

import cv2  # OpenCV voor beeldverwerking
import numpy as np  # NumPy voor matrix- en arraybewerkingen

from config.config import *  # Importeer alle projectinstellingen uit config.py

import depthai as dai  # DepthAI API voor OAK-D pipeline en device

import uuid  # Voor unieke object- en dataset-ID's
import time  # Voor timing, watchdog en timeouts
import math  # Voor yaw- en quaternionberekeningen
import json  # Voor dataset metadata-opslag
import threading  # Voor reconnect- en watchdogthreads
from pathlib import Path  # Voor veilige bestandspaden

from project_interfaces.srv import GetObjectData  # Service-interface voor objectaanvragen
from project_interfaces.msg import ObjectData  # Message voor één gedetecteerd object
from project_interfaces.msg import ObjectDataArray  # Message voor meerdere gedetecteerde objecten

# =========================================================
# Vision Node
# =========================================================

class VisionNode(Node):  # Hoofdnode voor vision, camera, detectie en service-afhandeling

    # =====================================================
    # Constructor
    # =====================================================

    def __init__(self):
        super().__init__("vision_node")  # Initialiseer ROS2 node met naam vision_node

        # ROS
        self.bridge = CvBridge()  # Maak CvBridge aan voor OpenCV <-> ROS Image conversie
        self.object_ui_pub = self.create_publisher(ObjectDataArray, UI_TOPIC, 10)  # Publisher voor alle objecten richting UI
        self.object_L6_pub = self.create_publisher(ObjectData, LITE6_RESULT_TOPIC, 10)  # Publisher voor beste object richting Lite6/debug
        self.marked_image_pub = self.create_publisher(Image, MARKED_IMAGE_TOPIC, 10)  # Publisher voor gemarkeerd camerabeeld
        self.object_service = self.create_service(GetObjectData, SERVICE_NAME, self.object_request_callback)  # Service voor objectdata-aanvragen

        # Runtime Variables
        self.device = None  # Huidige DepthAI-device instantie
        self.pipeline = None  # Huidige DepthAI-pipeline instantie
        self.rgb_queue = None  # Queue voor live RGB-previewframes
        self.depth_queue = None  # Queue voor depthframes
        self.detection_queue = None  # Queue voor YOLO-detecties
        self.nn_frame_queue = None  # Queue voor het frame dat daadwerkelijk door YOLO is verwerkt
        self.camera_control_queue = None  # Queue voor camera control zoals AWB
        self.running = False  # Houdt bij of de camera actief is
        self.latest_frame = None  # Laatst ontvangen liveframe
        self.confidence_threshold = YOLO_DEFAULT_CONFIDENCE  # Huidige confidencegrens
        self.use_hardware_awb = True  # Bepaalt of hardware auto white balance aan staat
        self.last_rgb_frame_time = time.monotonic()  # Tijdstip van het laatst ontvangen frame
        self.watchdog_running = False  # Houdt bij of de watchdog actief is
        self.reconnect_lock = threading.Lock()  # Lock om dubbele reconnects te voorkomen
        self.processing_lock = threading.Lock()  # Lock om gelijktijdige queueverwerking te voorkomen

        # Dataset
        self.base_path = Path(DATASET_FOLDER)  # Basispad voor datasetopslag

        # ArUco
        self.camera_position = None  # Gereserveerd voor camerapositie, momenteel niet actief gebruikt
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICTIONARY)  # Laad ingestelde ArUco dictionary
        if hasattr(cv2.aruco, 'DetectorParameters_create'):  # Controleer oude OpenCV ArUco API
            self.aruco_params = cv2.aruco.DetectorParameters_create()  # Maak parameters via oude API
        else:  # Gebruik nieuwe OpenCV ArUco API
            self.aruco_params = cv2.aruco.DetectorParameters()  # Maak parameters via nieuwe API 
        self.camera_matrix = np.array(CAMERA_MATRIX, dtype=np.float32)  # Zet fallback cameramatrix om naar NumPy
        self.dist_coeffs = np.array(DIST_COEFFS, dtype=np.float32)  # Zet fallback distortioncoëfficiënten om naar NumPy
        self.rvec = None  # Laatst berekende marker-naar-camera rotatievector
        self.tvec = None  # Laatst berekende marker-naar-camera translatievector
        self.world_calibrated = False  # Houdt bij of world calibration geldig is
        self.aruco_marker_id = ARUCO_MARKER_ID  # ID van de vaste referentiemarker
        self.aruco_size_m = ARUCO_SIZE_M  # Fysieke markermaat in meters

        # Initialize Camera
        self.initialize_camera()  # Maak verbinding met de OAK-D
        self.set_awb_mode(self.use_hardware_awb)  # Stel auto white balance in
        self.timer = self.create_timer(0.03, self.main_loop)  # Start timer voor live preview / watchdog update

    # =====================================================
    # Device Information
    # =====================================================

    def print_device_information(self):
        try:
            self.get_logger().info("========== DEVICE INFO ==========")  # Print header voor device-info
            self.get_logger().info(f"MXID: {self.device.getMxId()}")  # Print unieke OAK-D identifier
            self.get_logger().info(f"USB Speed: {self.device.getUsbSpeed()}")  # Print actuele USB-snelheid
            self.get_logger().info(f"Connected Cameras:")  # Print tekstregel voor camera-overzicht
            cameras = self.device.getConnectedCameraFeatures()  # Lees aangesloten camera's uit
            for cam in cameras:  # Loop door alle camera features
                self.get_logger().info(str(cam))  # Print camera feature informatie
            self.get_logger().info("=================================")  # Print footer voor device-info
        except Exception as ex:
            self.get_logger().error(str(ex))  # Log fout bij ophalen van device-info

    # =====================================================
    # Device Calibration
    # =====================================================

    def load_device_calibration(self):
        try:
            calib_data = self.device.readCalibration()  # Lees EEPROM-calibratie uit de OAK-D
            intrinsics = calib_data.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, RGB_WIDTH, RGB_HEIGHT)  # Lees intrinsics voor RGB-outputformaat
            distortion = calib_data.getDistortionCoefficients(dai.CameraBoardSocket.CAM_A)  # Lees distortioncoëfficiënten van RGB-camera
            self.camera_matrix = np.array(intrinsics, dtype=np.float32)  # Zet intrinsics om naar NumPy matrix
            self.dist_coeffs = np.array(distortion, dtype=np.float32).reshape(-1, 1)  # Zet distortion om naar OpenCV-vorm
            self.get_logger().info("Loaded OAK-D EEPROM camera calibration")  # Log dat EEPROM-calibratie geladen is
            self.get_logger().info(f"Camera matrix: {self.camera_matrix}")  # Log cameramatrix
            self.get_logger().info(f"Dist coeffs: {self.dist_coeffs.flatten()}")  # Log distortioncoëfficiënten
            return True  # Meld succesvolle calibratielaadactie
        except Exception as ex:
            self.get_logger().warn(f"Failed to load device calibration: {ex}")  # Log fallback naar configwaarden
            self.camera_matrix = np.array(CAMERA_MATRIX, dtype=np.float32)  # Gebruik fallback cameramatrix
            self.dist_coeffs = np.array(DIST_COEFFS, dtype=np.float32).reshape(-1, 1)  # Gebruik fallback distortion
            return False  # Meld mislukte calibratielaadactie

    # =====================================================
    # Auto White Balance
    # =====================================================

    def set_awb_mode(self, use_hardware_awb: bool):
        if self.camera_control_queue is None:  # Controleer of controlqueue beschikbaar is
            return  # Stop wanneer camera nog niet klaar is
        try:
            ctrl = dai.CameraControl()  # Maak nieuw DepthAI camera control object
            if use_hardware_awb:  # Controleer of hardware-AWB aan moet
                ctrl.setAutoWhiteBalanceMode(dai.CameraControl.AutoWhiteBalanceMode.AUTO)  # Zet AWB op AUTO
                self.get_logger().info("Hardware AWB enabled")  # Log dat AWB aan staat
            else:
                ctrl.setAutoWhiteBalanceMode(dai.CameraControl.AutoWhiteBalanceMode.OFF)  # Zet AWB uit
                self.get_logger().info("Hardware AWB disabled")  # Log dat AWB uit staat
            self.camera_control_queue.send(ctrl)  # Verstuur controlcommand naar de camera
        except Exception as ex:
            self.get_logger().warn(f"Failed to set AWB mode: {ex}")  # Log fout bij AWB-instelling

    # =====================================================
    # Pipeline
    # =====================================================

    def create_pipeline(self):
        self.get_logger().info('Creating DepthAI pipeline...')  # Log dat de pipeline wordt opgebouwd

        pipeline = dai.Pipeline()  # Maak een nieuwe DepthAI pipeline aan

        cam_rgb = pipeline.create(dai.node.ColorCamera)  # Maak de RGB-camera node aan
        left = pipeline.create(dai.node.MonoCamera)  # Maak de linker mono-camera node aan
        right = pipeline.create(dai.node.MonoCamera)  # Maak de rechter mono-camera node aan
        stereo = pipeline.create(dai.node.StereoDepth)  # Maak de stereo-depth node aan
        control_in = pipeline.create(dai.node.XLinkIn)  # Maak een host-inputstream voor camera control aan
        xout_rgb = pipeline.create(dai.node.XLinkOut)  # Maak een host-outputstream voor RGB aan
        xout_depth = pipeline.create(dai.node.XLinkOut)  # Maak een host-outputstream voor depth aan

        cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)  # Selecteer de RGB-camera op CAM_A
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_4_K)  # Zet RGB-sensor op 4K
        cam_rgb.setInterleaved(False)  # Gebruik planar output in plaats van interleaved output
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)  # Gebruik BGR-volgorde voor OpenCV
        cam_rgb.setPreviewSize(RGB_WIDTH, RGB_HEIGHT)  # Zet previewformaat vanuit config.py
        cam_rgb.setPreviewKeepAspectRatio(True)  # Behoud aspect ratio bij het maken van de preview

        left.setBoardSocket(dai.CameraBoardSocket.CAM_B)  # Selecteer linker mono-camera op CAM_B
        right.setBoardSocket(dai.CameraBoardSocket.CAM_C)  # Selecteer rechter mono-camera op CAM_C
        left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)  # Zet links op 800p
        right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)  # Zet rechts op 800p

        stereo.setDefaultProfilePreset(  # Stel het stereo-depth profiel in
            dai.node.StereoDepth.PresetMode.HIGH_ACCURACY  # Gebruik nauwkeuriger depthprofiel
        )
        stereo.setLeftRightCheck(True)  # Verwijder inconsistente stereo-matches
        stereo.setSubpixel(True)  # Gebruik subpixel-disparity voor nauwkeurigere depth
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)  # Align depth op de RGB-camera
        stereo.setOutputSize(RGB_WIDTH, RGB_HEIGHT)  # Zet depth-output gelijk aan RGB-previewformaat

        control_in.setStreamName('camera_control')  # Geef de camera-control inputstream een naam
        xout_rgb.setStreamName('rgb')  # Geef de RGB-outputstream een naam
        xout_depth.setStreamName('depth')  # Geef de depth-outputstream een naam

        control_in.out.link(cam_rgb.inputControl)  # Koppel host-controlstream aan RGB-camera control-input
        left.out.link(stereo.left)  # Koppel linker mono-camera aan stereo-depth linker input
        right.out.link(stereo.right)  # Koppel rechter mono-camera aan stereo-depth rechter input
        cam_rgb.preview.link(xout_rgb.input)  # Koppel RGB-preview naar host-outputstream
        stereo.depth.link(xout_depth.input)  # Koppel depth-output naar host-outputstream

        if USE_YOLO:  # Bouw YOLO alleen als er later een geldig .blob model beschikbaar is
            detection_nn = pipeline.create(dai.node.YoloDetectionNetwork)  # Maak YOLO detection node aan
            xout_det = pipeline.create(dai.node.XLinkOut)  # Maak detection-outputstream aan
            xout_nn_frame = pipeline.create(dai.node.XLinkOut)  # Maak NN-frame passthrough-outputstream aan

            detection_nn.setConfidenceThreshold(0.01)  # Zet lage NN-threshold; filteren gebeurt later
            detection_nn.setNumClasses(YOLO_NUM_CLASSES)  # Zet aantal YOLO-klassen vanuit config.py
            detection_nn.setCoordinateSize(4)  # Gebruik vier bbox-coördinaten
            detection_nn.setAnchors([])  # Gebruik lege anchors volgens huidige YOLO-config
            detection_nn.setAnchorMasks({})  # Gebruik lege anchor masks volgens huidige YOLO-config
            detection_nn.setIouThreshold(YOLO_IOU_THRESHOLD)  # Zet IOU/NMS-threshold vanuit config.py
            detection_nn.setBlobPath(MODEL_PATH)  # Laad het DepthAI v2 .blob modelbestand

            xout_det.setStreamName('detections')  # Geef detection-outputstream een naam
            xout_nn_frame.setStreamName('nn_frame')  # Geef NN-frame stream een naam

            cam_rgb.preview.link(detection_nn.input)  # Koppel RGB-preview aan YOLO-input
            detection_nn.out.link(xout_det.input)  # Koppel YOLO-detecties naar host-outputstream
            detection_nn.passthrough.link(xout_nn_frame.input)  # Koppel YOLO-passthrough frame naar host
        else:
            self.get_logger().warn('YOLO disabled: running RGB/depth only')  # Log tijdelijke testmodus

        return pipeline  # Geef de complete pipeline terug

    # =====================================================
    # Initialize Camera
    # =====================================================

    def initialize_camera(self):
        while rclpy.ok():  # Blijf proberen zolang ROS actief is
            for attempt in range(RECONNECT_ATTEMPTS):  # Probeer per batch een vast aantal pogingen
                try:
                    self.get_logger().info(f"Connecting OAK-D ({attempt + 1}/{RECONNECT_ATTEMPTS})")  # Log verbindingspoging
                    self.pipeline = self.create_pipeline()  # Bouw nieuwe pipeline
                    self.device = dai.Device(self.pipeline, maxUsbSpeed=dai.UsbSpeed.HIGH)  # Open device met maximaal USB HIGH
                    self.print_device_information()  # Print deviceinformatie
                    if USE_DEVICE_CALIBRATION:  # Controleer of EEPROM-calibratie gebruikt moet worden
                        self.load_device_calibration()  # Laad devicecalibratie

                    self.rgb_queue = self.device.getOutputQueue("rgb", maxSize=4, blocking=False)  # Maak RGB queue
                    self.depth_queue = self.device.getOutputQueue("depth", maxSize=4, blocking=False)  # Maak depth queue
                    
                    if USE_YOLO:  # Alleen detection queues maken als YOLO actief is
                        self.detection_queue = self.device.getOutputQueue('detections', maxSize=4, blocking=False)  # Maak detectiequeue
                        self.nn_frame_queue = self.device.getOutputQueue('nn_frame', maxSize=4, blocking=False)  # Maak NN-frame queue
                    else:
                        self.detection_queue = None  # Geen detection queue zonder YOLO
                        self.nn_frame_queue = None  # Geen NN-frame queue zonder YOLO
            
                    self.camera_control_queue = self.device.getInputQueue("camera_control")  # Maak camera control queue
                    self.running = True  # Zet node op actief
                    self.last_rgb_frame_time = time.monotonic()  # Reset watchdogtimer
                    self.get_logger().info("Camera connected")  # Log succesvolle verbinding
                    self.watchdog_running = True  # Zet watchdog actief

                    if not hasattr(self, "watchdog_thread") or not self.watchdog_thread.is_alive():  # Start alleen als watchdog nog niet loopt
                        self.watchdog_thread = threading.Thread(target=self.watchdog_loop, daemon=True)  # Maak watchdogthread
                        self.watchdog_thread.start()  # Start watchdogthread
                    return  # Stop initialize na succesvolle verbinding
                except Exception as ex:
                    self.get_logger().warn(f"Connection failed: {ex}")  # Log mislukte verbinding
                    try:
                        if self.device is not None:  # Controleer of half-open device bestaat
                            self.device.close()  # Sluit half-open device
                    except Exception:
                        pass  # Negeer fouten bij sluiten
                    self.device = None  # Wis ongeldig device
                    time.sleep(RECONNECT_INTERVAL)  # Wacht voor volgende poging
            self.get_logger().error(f"Unable to connect after {RECONNECT_ATTEMPTS} attempts; continuing retries")  # Log mislukte batch
            time.sleep(RECONNECT_INTERVAL)  # Wacht voor nieuwe batch

    # =====================================================
    # ArUco
    # =====================================================

    def estimate_aruco_pose(self, frame):

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Zet BGR beeld om naar grayscale

        corners, ids, _ = cv2.aruco.detectMarkers(  # Detecteer ArUco markers met oude OpenCV API
            gray,  # Grayscale invoerbeeld
            self.aruco_dict,  # ArUco dictionary uit config
            parameters=self.aruco_params  # Detectieparameters
        )

        if ids is None:  # Controleer of er markers gevonden zijn
            self.world_calibrated = False  # Zet world calibration uit
            return frame, False  # Geef frame terug zonder geldige pose

        for i, marker_id in enumerate(ids.flatten()):  # Loop door gevonden marker-ID's

            if int(marker_id) != self.aruco_marker_id:  # Controleer of dit de gewenste marker is
                continue  # Sla andere markers over

            marker_corners = corners[i].reshape((4, 2)).astype(np.float32)  # Zet corners naar OpenCV formaat

            half_size = self.aruco_size_m / 2.0  # Bereken halve markermaat

            object_points = np.array(  # Definieer markerhoeken in markercoördinaten
                [
                    [-half_size, half_size, 0.0],  # Linksboven
                    [half_size, half_size, 0.0],  # Rechtsboven
                    [half_size, -half_size, 0.0],  # Rechtsonder
                    [-half_size, -half_size, 0.0],  # Linksonder
                ],
                dtype=np.float32  # Gebruik float32 voor OpenCV
            )

            success, rvec, tvec = cv2.solvePnP(  # Bereken markerpose t.o.v. camera
                object_points,  # 3D markerpunten
                marker_corners,  # 2D beeldpunten
                self.camera_matrix,  # Cameramatrix
                self.dist_coeffs,  # Distortioncoëfficiënten
                flags=cv2.SOLVEPNP_IPPE_SQUARE  # Goede solvePnP methode voor vierkante markers
            )

            if not success:  # Controleer solvePnP resultaat
                self.world_calibrated = False  # Zet world calibration uit
                return frame, False  # Geef foutstatus terug

            self.rvec = rvec  # Sla rotatievector op
            self.tvec = tvec  # Sla translatievector op
            self.world_calibrated = True  # Markeer world calibration als geldig

            cv2.aruco.drawDetectedMarkers(frame, corners, ids)  # Teken gevonden markers

            cv2.drawFrameAxes(  # Teken assen op marker
                frame,  # Beeld waarop getekend wordt
                self.camera_matrix,  # Cameramatrix
                self.dist_coeffs,  # Distortioncoëfficiënten
                self.rvec,  # Rotatievector
                self.tvec,  # Translatievector
                self.aruco_size_m  # Aslengte
            )

            self.get_logger().info(
                f'ArUco ID {marker_id}: x={float(tvec[0]):.3f} m, y={float(tvec[1]):.3f} m, z={float(tvec[2]):.3f} m'
            )

            return frame, True  # Geef frame en successtatus terug

        self.world_calibrated = False  # Geen juiste marker gevonden
        return frame, False  # Geef frame zonder geldige calibration terug

    # =====================================================
    # ROI Depth To Camera XYZ
    # =====================================================

    def estimate_xyz_from_roi(self, depth_frame, bbox):
        x_min, y_min, x_max, y_max = bbox  # Lees bbox uit
        depth_height, depth_width = depth_frame.shape[:2]  # Lees depthformaat uit
        x_min = max(0, min(x_min, depth_width - 1))  # Clamp links
        x_max = max(0, min(x_max, depth_width - 1))  # Clamp rechts
        y_min = max(0, min(y_min, depth_height - 1))  # Clamp boven
        y_max = max(0, min(y_max, depth_height - 1))  # Clamp onder

        if x_max <= x_min or y_max <= y_min:  # Controleer geldige ROI
            return None  # Stop bij ongeldige ROI

        center_x = int((x_min + x_max) / 2)  # Bepaal ROI-midden X
        center_y = int((y_min + y_max) / 2)  # Bepaal ROI-midden Y
        roi_width = x_max - x_min  # Bereken ROI-breedte
        roi_height = y_max - y_min  # Bereken ROI-hoogte
        shrunk_width = int(roi_width * ROI_SHRINK_FACTOR)  # Verklein ROI-breedte
        shrunk_height = int(roi_height * ROI_SHRINK_FACTOR)  # Verklein ROI-hoogte
        roi_x_min = max(0, center_x - shrunk_width // 2)  # Nieuwe ROI-links
        roi_x_max = min(depth_width, center_x + shrunk_width // 2)  # Nieuwe ROI-rechts
        roi_y_min = max(0, center_y - shrunk_height // 2)  # Nieuwe ROI-boven
        roi_y_max = min(depth_height, center_y + shrunk_height // 2)  # Nieuwe ROI-onder
        roi_depth = depth_frame[roi_y_min:roi_y_max, roi_x_min:roi_x_max]  # Pak depth-ROI
        valid_depth = roi_depth[(roi_depth > MIN_DEPTH_MM) & (roi_depth < MAX_DEPTH_MM)]  # Filter ongeldige depthwaarden

        if valid_depth.size < MIN_VALID_DEPTH_PIXELS:  # Controleer genoeg geldige depthpixels
            return None  # Stop bij te weinig depth

        z_mm = float(np.median(valid_depth))  # Bepaal mediaan depth in millimeters
        z_m = z_mm / 1000.0  # Zet depth om naar meters
        fx = float(self.camera_matrix[0, 0])  # Lees fx uit cameramatrix
        fy = float(self.camera_matrix[1, 1])  # Lees fy uit cameramatrix
        cx_camera = float(self.camera_matrix[0, 2])  # Lees cx uit cameramatrix
        cy_camera = float(self.camera_matrix[1, 2])  # Lees cy uit cameramatrix
        x_m = ((center_x - cx_camera) * z_m / fx)  # Bereken camera-X
        y_m = ((center_y - cy_camera) * z_m / fy)  # Bereken camera-Y
        return (x_m, y_m, z_m, z_mm, center_x, center_y)  # Geef XYZ, depth en ROI-midden terug

    # =====================================================
    # PCA Yaw From Depth ROI
    # =====================================================

    def estimate_yaw_from_depth_roi(self, depth_frame, bbox, z_mm):
        x_min, y_min, x_max, y_max = bbox  # Lees bbox uit
        depth_height, depth_width = depth_frame.shape[:2]  # Lees depthformaat uit
        x_min = max(0, min(x_min, depth_width - 1))  # Clamp links
        x_max = max(0, min(x_max, depth_width - 1))  # Clamp rechts
        y_min = max(0, min(y_min, depth_height - 1))  # Clamp boven
        y_max = max(0, min(y_max, depth_height - 1))  # Clamp onder

        if x_max <= x_min or y_max <= y_min:  # Controleer geldige ROI
            return 0.0  # Geef neutrale yaw terug

        roi_depth = depth_frame[y_min:y_max, x_min:x_max]  # Pak depth binnen bbox
        mask = ((roi_depth > z_mm - DEPTH_BAND_MM) & (roi_depth < z_mm + DEPTH_BAND_MM))  # Maak dieptemasker rond objectdiepte
        ys, xs = np.where(mask)  # Zoek geldige pixels in masker

        if xs.size < 20:  # Controleer genoeg punten voor PCA
            return 0.0  # Geef neutrale yaw terug

        points = np.column_stack((xs.astype(np.float32), ys.astype(np.float32)))  # Bouw 2D-puntenwolk
        mean = np.mean(points, axis=0)  # Bereken gemiddelde punt
        centered_points = points - mean  # Centreer puntenwolk
        covariance = np.cov(centered_points, rowvar=False)  # Bereken covariantiematrix
        eigenvalues, eigenvectors = np.linalg.eig(covariance)  # Bereken eigenwaarden en eigenvectoren
        principal_axis = eigenvectors[:, np.argmax(eigenvalues)]  # Kies hoofdas met grootste variantie
        yaw = math.atan2(float(principal_axis[1]), float(principal_axis[0]))  # Bereken beeldvlak-yaw
        return yaw  # Geef yaw in radialen terug

    # =====================================================
    # Image Yaw To World Yaw
    # =====================================================

    def image_yaw_to_world_yaw(self, center_x, center_y, z_m, image_yaw):
        fx = float(self.camera_matrix[0, 0])  # Lees fx
        fy = float(self.camera_matrix[1, 1])  # Lees fy
        cx_camera = float(self.camera_matrix[0, 2])  # Lees cx
        cy_camera = float(self.camera_matrix[1, 2])  # Lees cy
        step_px = 30.0  # Pixelstap om richting te projecteren
        x2_px = center_x + step_px * math.cos(image_yaw)  # Bereken tweede beeldpunt X
        y2_px = center_y + step_px * math.sin(image_yaw)  # Bereken tweede beeldpunt Y
        p1_x = ((center_x - cx_camera) * z_m / fx)  # Bereken punt 1 camera-X
        p1_y = ((center_y - cy_camera) * z_m / fy)  # Bereken punt 1 camera-Y
        p2_x = ((x2_px - cx_camera) * z_m / fx)  # Bereken punt 2 camera-X
        p2_y = ((y2_px - cy_camera) * z_m / fy)  # Bereken punt 2 camera-Y
        w1_x, w1_y, _ = self.transform_to_world(p1_x, p1_y, z_m)  # Transformeer punt 1 naar world
        w2_x, w2_y, _ = self.transform_to_world(p2_x, p2_y, z_m)  # Transformeer punt 2 naar world
        return math.atan2(w2_y - w1_y, w2_x - w1_x)  # Bereken world-yaw

    # =====================================================
    # Process Frame To Objects
    # =====================================================

    def process_frame_to_objects(self, frame, depth_frame, detections):
        object_list = []  # Lijst met geldige objecten
        best_object = None  # Beste pickbare object
        best_confidence = 0.0  # Hoogste confidence van pickbaar object
        dataset_frame = frame.copy()  # Ruwe kopie voor datasetopslag

        if depth_frame.shape[:2] != frame.shape[:2]:  # Controleer of depthformaat gelijk is aan frameformaat
            depth_frame = cv2.resize(depth_frame, (frame.shape[1], frame.shape[0]), interpolation=cv2.INTER_NEAREST)  # Resize depth naar frameformaat

        for detection in detections:  # Loop door YOLO-detecties
            confidence = float(detection.confidence)  # Lees confidence
            if confidence < self.confidence_threshold:  # Filter lage confidence
                continue  # Sla detectie over

            x_min = int(detection.xmin * frame.shape[1])  # Bereken bbox links
            x_max = int(detection.xmax * frame.shape[1])  # Bereken bbox rechts
            y_min = int(detection.ymin * frame.shape[0])  # Bereken bbox boven
            y_max = int(detection.ymax * frame.shape[0])  # Bereken bbox onder
            bbox = (x_min, y_min, x_max, y_max)  # Bouw bbox tuple
            xyz_result = self.estimate_xyz_from_roi(depth_frame, bbox)  # Bereken camera XYZ uit depth-ROI

            if xyz_result is None:  # Controleer of depth geldig is
                continue  # Sla object zonder geldige depth over

            camera_x, camera_y, camera_z, z_mm, center_x, center_y = xyz_result  # Pak spatial resultaat uit
            image_yaw = self.estimate_yaw_from_depth_roi(depth_frame, bbox, z_mm)  # Bereken PCA-yaw in beeldvlak
            world_x, world_y, world_z = self.transform_to_world(camera_x, camera_y, camera_z)  # Transformeer camera XYZ naar world XYZ
            world_yaw = self.image_yaw_to_world_yaw(center_x, center_y, camera_z, image_yaw)  # Transformeer beeld-yaw naar world-yaw

            obj = {  # Bouw intern objectrecord
                "object_id": str(uuid.uuid4()),
                "class": int(detection.label),
                "confidence": confidence,
                "x": world_x,
                "y": world_y,
                "z": world_z,
                "camera_x": camera_x,
                "camera_y": camera_y,
                "camera_z": camera_z,
                "yaw": world_yaw,
                "image_yaw": image_yaw,
                "bbox": bbox,
                "robot_pickable": False
            }

            obj["robot_pickable"] = self.is_robot_pickable(obj)  # Bepaal of object geschikt is voor robot
            object_list.append(obj)  # Voeg object toe aan objectlijst

            if obj["robot_pickable"] and confidence > best_confidence:  # Kies beste pickbare object
                best_confidence = confidence  # Update hoogste confidence
                best_object = obj  # Update beste object

            class_name = YOLO_CLASS_NAMES.get(int(detection.label), f"class_{int(detection.label)}")  # Zoek klassenaam
            color = (0, 255, 0) if obj["robot_pickable"] else (0, 165, 255)  # Groen is pickbaar, oranje is gefilterd
            cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), color, 2)  # Teken boundingbox
            cv2.putText(frame, f"{class_name} {confidence:.2f}", (x_min, y_min - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)  # Teken label
            self.draw_yaw_axis(frame, bbox, image_yaw)  # Teken PCA-richting

        return (object_list, best_object, dataset_frame, frame)  # Geef objecten, beste object, datasetframe en gemarkeerd frame terug

    # =====================================================
    # Draw PCA Yaw Axis
    # =====================================================

    def draw_yaw_axis(self, frame, bbox, yaw):
        x_min, y_min, x_max, y_max = bbox  # Lees bbox uit
        center_x = int((x_min + x_max) / 2)  # Bereken bboxcentrum X
        center_y = int((y_min + y_max) / 2)  # Bereken bboxcentrum Y
        axis_length = int(min(x_max - x_min, y_max - y_min) * 0.4)  # Bepaal aslengte
        end_x = int(center_x + axis_length * math.cos(yaw))  # Bereken eindpunt X
        end_y = int(center_y + axis_length * math.sin(yaw))  # Bereken eindpunt Y
        cv2.line(frame, (center_x, center_y), (end_x, end_y), (255, 0, 0), 2)  # Teken yaw-as

    # =====================================================
    # Process Object Request
    # =====================================================

    def process_object_request(self, timeout_sec):
        packet_result = self.get_synced_packets(timeout_sec)  # Haal synced packets op
        if packet_result is None:  # Controleer of packetset beschikbaar is
            self.get_logger().warn("Object request failed: no synced packets available")  # Log ontbrekende packets
            return None  # Stop zonder object

        nn_frame_packet, detection_packet, depth_packet = packet_result  # Pak packetset uit
        frame = nn_frame_packet.getCvFrame()  # Haal NN-frame op
        depth_frame = depth_packet.getFrame()  # Haal depthframe op
        frame, calibrated = self.estimate_aruco_pose(frame)  # Bepaal ArUco world pose

        if not calibrated:  # Controleer of world calibration gelukt is
            self.get_logger().warn("Object request rejected: ArUco marker not detected")  # Log ontbrekende ArUco marker
            return None  # Stop zonder object

        object_list, best_object, dataset_frame, marked_frame = self.process_frame_to_objects(frame, depth_frame, detection_packet.detections)  # Verwerk detecties naar objecten

        if len(object_list) == 0:  # Controleer of er objecten over zijn
            self.get_logger().warn("Object request failed: no valid objects with depth")  # Log geen geldige objecten
            return None  # Stop zonder object

        if SAVE_DATASET_ON_REQUEST:  # Controleer dataset trigger
            self.save_dataset_sample(dataset_frame, object_list)  # Sla datasetopname op

        self.publish_object_array(object_list)  # Publiceer alle objecten naar UI

        if best_object is None:  # Controleer of robotgeschikt object bestaat
            self.get_logger().warn("Object request failed: no robot-pickable object")  # Log geen robotobject
            return None  # Stop zonder robotobject

        self.marked_image_pub.publish(self.bridge.cv2_to_imgmsg(marked_frame, encoding="bgr8"))  # Publiceer gemarkeerd beeld
        return best_object  # Geef beste object terug

    # =====================================================
    # Publish Object Array
    # =====================================================

    def publish_object_array(self, object_list):
        ui_msg = ObjectDataArray()  # Maak ObjectDataArray message
        ui_msg.objects = []  # Initialiseer objectlijst
        for obj in object_list:  # Loop door interne objectrecords
            object_msg = self.convert_to_ros_msg(obj)  # Converteer object naar ROS-message
            ui_msg.objects.append(object_msg)  # Voeg objectmessage toe
        self.object_ui_pub.publish(ui_msg)  # Publiceer array naar UI-topic

    # =====================================================
    # Transform To World
    # =====================================================

    def transform_to_world(self, x, y, z):
        if self.tvec is None or self.rvec is None:  # Controleer of ArUco-pose beschikbaar is
            raise RuntimeError("Cannot transform point: ArUco world calibration is unavailable")  # Geef fout bij ontbrekende ArUco-pose

        rotation_marker_to_camera, _ = cv2.Rodrigues(self.rvec)  # Zet OpenCV rvec om naar rotatiematrix van marker naar camera
        rotation_camera_to_marker = rotation_marker_to_camera.T  # Inverteer rotatie zodat camera naar marker wordt
        translation_camera_to_marker = -rotation_camera_to_marker @ self.tvec  # Inverteer translatie zodat camera naar marker wordt

        point_camera = np.array(  # Maak objectpunt in camera-coördinaten
            [[x], [y], [z]],  # Gebruik kolomvector met x, y, z
            dtype=np.float64  # Gebruik float64 voor stabiele matrixberekeningen
        )

        point_marker = rotation_camera_to_marker @ point_camera + translation_camera_to_marker  # Transformeer objectpunt van camera-frame naar ArUco-frame

        rotation_robot_aruco = np.array(ARUCO_TO_ROBOT_ROTATION, dtype=np.float64)  # Laad vaste rotatie van ArUco-frame naar robot-base-frame uit config
        translation_robot_aruco = np.array(  # Maak translatie van ArUco-frame naar robot-base-frame
            [[ARUCO_WORLD_X], [ARUCO_WORLD_Y], [ARUCO_WORLD_Z]],  # Gebruik gemeten markercentrumpositie t.o.v. robot base
            dtype=np.float64  # Gebruik float64 voor stabiele matrixberekeningen
        )

        point_robot = rotation_robot_aruco @ point_marker + translation_robot_aruco  # Transformeer objectpunt van ArUco-frame naar robot-base-frame

        return (  # Geef positie terug als gewone Python floats
            float(point_robot[0, 0]),  # Geef robot-base X terug
            float(point_robot[1, 0]),  # Geef robot-base Y terug
            float(point_robot[2, 0])  # Geef robot-base Z terug
        )

    # =====================================================
    # Reconnect Camera
    # =====================================================

    def reconnect_camera(self):
        if not self.reconnect_lock.acquire(blocking=False):  # Controleer of reconnect al loopt
            return  # Stop wanneer al een reconnect actief is

        try:
            self.get_logger().warn("Camera disconnected")  # Log camera disconnect
            self.running = False  # Stop normale processing
            self.watchdog_running = False  # Stop watchdog
            try:
                if self.device is not None:  # Controleer bestaand device
                    self.device.close()  # Sluit device
            except Exception:
                pass  # Negeer sluitfouten

            self.device = None  # Wis device
            self.pipeline = None  # Wis pipeline
            self.rgb_queue = None  # Wis RGB queue
            self.depth_queue = None  # Wis depth queue
            self.detection_queue = None  # Wis detection queue
            self.nn_frame_queue = None  # Wis NN-frame queue
            self.camera_control_queue = None  # Wis control queue
            self.initialize_camera()  # Maak opnieuw verbinding
            self.set_awb_mode(self.use_hardware_awb)  # Herstel AWB instelling
        finally:
            self.reconnect_lock.release()  # Geef reconnectlock vrij

    # =====================================================
    # Synced Packet Fetch
    # =====================================================

    def get_synced_packets(self, timeout_sec):
        end_time = time.monotonic() + timeout_sec  # Bepaal timeouttijdstip
        nn_frames = {}  # Buffer voor NN-frames
        detections = {}  # Buffer voor detectiepakketten
        depths = {}  # Buffer voor depthpakketten
        latest_depth_packet = None  # Laatste depthpacket als fallback

        while time.monotonic() < end_time:  # Loop tot timeout
            nn_frame_packet = self.nn_frame_queue.tryGet()  # Lees NN-frame packet
            detection_packet = self.detection_queue.tryGet()  # Lees detection packet
            depth_packet = self.depth_queue.tryGet()  # Lees depth packet

            if nn_frame_packet is not None:  # Controleer NN-frame
                nn_frames[nn_frame_packet.getSequenceNum()] = nn_frame_packet  # Buffer NN-frame op sequence number
                self.last_rgb_frame_time = time.monotonic()  # Update watchdogtijd

            if detection_packet is not None:  # Controleer detection packet
                detections[detection_packet.getSequenceNum()] = detection_packet  # Buffer detecties op sequence number

            if depth_packet is not None:  # Controleer depth packet
                depths[depth_packet.getSequenceNum()] = depth_packet  # Buffer depth op sequence number
                latest_depth_packet = depth_packet  # Update fallback depthpacket

            exact_sequences = set(nn_frames.keys()).intersection(detections.keys()).intersection(depths.keys())  # Zoek exacte match tussen frame/detecties/depth
            if len(exact_sequences) > 0:  # Controleer of exacte match bestaat
                sequence = max(exact_sequences)  # Gebruik nieuwste match
                return (nn_frames[sequence], detections[sequence], depths[sequence])  # Geef exacte packetset terug

            frame_detection_sequences = set(nn_frames.keys()).intersection(detections.keys())  # Zoek frame/detectie match
            if len(frame_detection_sequences) > 0 and latest_depth_packet is not None:  # Controleer fallback met laatste depth
                sequence = max(frame_detection_sequences)  # Gebruik nieuwste frame/detectie match
                return (nn_frames[sequence], detections[sequence], latest_depth_packet)  # Geef fallback packetset terug

            time.sleep(0.005)  # Beperk CPU-belasting

        return None  # Geef None terug bij timeout

    # =====================================================
    # Watchdog Loop
    # =====================================================

    def watchdog_loop(self):
        self.get_logger().info("DepthAI watchdog started")  # Log watchdogstart
        while self.watchdog_running:  # Loop zolang watchdog actief is
            try:
                frame_age = time.monotonic() - self.last_rgb_frame_time  # Bereken leeftijd van laatste frame
                if self.running and frame_age > WATCHDOG_TIMEOUT_SEC:  # Controleer stale stream
                    self.get_logger().warn("DepthAI watchdog detected a stale RGB stream")  # Log stale stream
                    reconnect_thread = threading.Thread(target=self.reconnect_camera, daemon=True)  # Maak reconnectthread
                    reconnect_thread.start()  # Start reconnectthread
                    return  # Stop huidige watchdog
                time.sleep(WATCHDOG_INTERVAL_SEC)  # Wacht tot volgende watchdogcheck
            except Exception as ex:
                self.get_logger().warn(f"Watchdog error: {ex}")  # Log watchdogfout
                time.sleep(WATCHDOG_INTERVAL_SEC)  # Voorkom snelle foutlus

    # =====================================================
    # Object Request Callback
    # =====================================================

    def object_request_callback(self, request, response):
        with self.processing_lock:  # Blokkeer gelijktijdige objectaanvragen
            if not self.running or self.device is None:  # Controleer camera actief
                response.success = False  # Markeer response mislukt
                return response  # Geef response terug

            self.confidence_threshold = float(request.confidence_threshold)  # Lees confidencegrens uit request
            self.get_logger().info("Object request received")  # Log nieuwe aanvraag
            best_object = self.process_object_request(timeout_sec=2.0)  # Verwerk één objectaanvraag

            if best_object is None:  # Controleer of object gevonden is
                response.success = False  # Markeer response mislukt
                return response  # Geef response terug

            best_object_msg = self.convert_to_ros_msg(best_object)  # Converteer beste object naar ROS-message
            self.object_L6_pub.publish(best_object_msg)  # Publiceer beste object op debugtopic
            response.success = True  # Markeer response geslaagd
            response.object = best_object_msg  # Vul response objectveld
            return response  # Geef response terug

    # =====================================================
    # Live Preview Loop
    # =====================================================

    def vision_processing_loop(self):
        if not self.running or self.device is None:  # Controleer camera actief
            return  # Stop zonder camera

        if self.rgb_queue is None:  # Controleer RGB queue
            return  # Stop zonder queue

        rgb_packet = self.rgb_queue.tryGet()  # Lees live RGB-frame
        if rgb_packet is None:  # Controleer frame beschikbaar
            return  # Stop zonder frame

        self.last_rgb_frame_time = time.monotonic()  # Update watchdogtijd
        frame = rgb_packet.getCvFrame()  # Haal OpenCV-frame op
        self.latest_frame = frame.copy()  # Bewaar laatste frame
        self.marked_image_pub.publish(self.bridge.cv2_to_imgmsg(frame, encoding="bgr8"))  # Publiceer live preview

    # =====================================================
    # Robot Filtering
    # =====================================================

    def is_robot_pickable(self, obj):
        if not ROBOT_FILTER_ENABLED:  # Controleer of robotfilter uit staat
            return True  # Accepteer object direct

        if int(obj["class"]) not in ROBOT_ALLOWED_CLASS_IDS:  # Controleer toegestane klasse
            return False  # Weiger object met verboden klasse

        if float(obj["confidence"]) < ROBOT_MIN_CONFIDENCE:  # Controleer minimale robotconfidence
            return False  # Weiger object met lage confidence

        if not (ROBOT_MIN_X_M <= obj["x"] <= ROBOT_MAX_X_M):  # Controleer X-bereik
            self.get_logger().warn("Found object was out of bounds (X) and will be ignored")  # Log X buiten bereik
            return False  # Weiger object buiten X-bereik

        if not (ROBOT_MIN_Y_M <= obj["y"] <= ROBOT_MAX_Y_M):  # Controleer Y-bereik
            self.get_logger().warn("Found object was out of bounds (Y) and will be ignored")  # Log Y buiten bereik
            return False  # Weiger object buiten Y-bereik

        if not (ROBOT_MIN_Z_M <= obj["z"] <= ROBOT_MAX_Z_M):  # Controleer Z-bereik
            self.get_logger().warn("Found object was out of bounds (Z) and will be ignored")  # Log Z buiten bereik
            return False  # Weiger object buiten Z-bereik

        return True  # Object is geschikt voor robot

    # =====================================================
    # ROS Message Vullen
    # =====================================================

    def convert_to_ros_msg(self, obj):
        msg = ObjectData()  # Maak ObjectData message
        msg.object_class = YOLO_CLASS_NAMES.get(obj["class"], f"class_{obj['class']}")  # Vul objectklasse
        msg.object_id = obj["object_id"]  # Vul stabiel object-ID
        msg.confidence = float(obj["confidence"])  # Vul confidencewaarde
        msg.transform.header.stamp = self.get_clock().now().to_msg()  # Vul timestamp
        msg.transform.header.frame_id = "world"  # Zet parent frame op world
        msg.transform.child_frame_id = f"detected_object_{obj['object_id']}"  # Zet child frame op uniek objectframe
        msg.transform.transform.translation.x = float(obj["x"])  # Vul X-positie
        msg.transform.transform.translation.y = float(obj["y"])  # Vul Y-positie
        msg.transform.transform.translation.z = float(obj["z"])  # Vul Z-positie
        yaw = float(obj["yaw"])  # Lees yawhoek
        msg.transform.transform.rotation.x = 0.0  # Vul quaternion X
        msg.transform.transform.rotation.y = 0.0  # Vul quaternion Y
        msg.transform.transform.rotation.z = math.sin(yaw / 2.0)  # Vul quaternion Z uit yaw
        msg.transform.transform.rotation.w = math.cos(yaw / 2.0)  # Vul quaternion W uit yaw
        return msg  # Geef ObjectData message terug

    # =====================================================
    # Dataset Logger
    # =====================================================

    def save_dataset_sample(self, image, detections):
        if image is None:  # Controleer of afbeelding bestaat
            return False  # Stop zonder opslag

        if len(detections) == 0:  # Controleer of detecties bestaan
            return False  # Stop zonder opslag

        uid = str(uuid.uuid4())  # Maak unieke dataset-ID
        (self.base_path / "images").mkdir(parents=True, exist_ok=True)  # Maak afbeeldingenmap aan
        (self.base_path / "labels").mkdir(parents=True, exist_ok=True)  # Maak labelmap aan
        (self.base_path / "metadata").mkdir(parents=True, exist_ok=True)  # Maak metadatamap aan
        image_file = self.base_path / "images" / f"{uid}.jpg"  # Bouw afbeeldingspad
        label_file = self.base_path / "labels" / f"{uid}.txt"  # Bouw labelpad
        metadata_file = self.base_path / "metadata" / f"{uid}.json"  # Bouw metadatapad
        image_saved = cv2.imwrite(str(image_file), image)  # Sla afbeelding op

        if not image_saved:  # Controleer of afbeelding opgeslagen is
            self.get_logger().error(f"Failed to save dataset image: {image_file}")  # Log opslagfout
            return False  # Stop bij opslagfout

        image_height, image_width = image.shape[:2]  # Lees afbeeldingsformaat
        with open(label_file, "w", encoding="utf-8") as label_handle:  # Open YOLO-labelbestand
            for detection in detections:  # Loop door detecties
                x_min, y_min, x_max, y_max = detection["bbox"]  # Lees bbox
                x_center = ((x_min + x_max) / 2.0 / image_width)  # Bereken YOLO x-center
                y_center = ((y_min + y_max) / 2.0 / image_height)  # Bereken YOLO y-center
                box_width = ((x_max - x_min) / image_width)  # Bereken YOLO breedte
                box_height = ((y_max - y_min) / image_height)  # Bereken YOLO hoogte
                class_id = int(detection["class"])  # Lees class-ID
                label_handle.write(f"{class_id} {x_center:.6f} {y_center:.6f} {box_width:.6f} {box_height:.6f}\n")  # Schrijf YOLO-labelregel

        metadata = {  # Bouw metadata dictionary
            "id": uid,
            "objects": [
                {
                    "object_id": detection["object_id"],
                    "class_id": int(detection["class"]),
                    "class_name": YOLO_CLASS_NAMES.get(int(detection["class"]), f"class_{int(detection['class'])}"),
                    "confidence": float(detection["confidence"]),
                    "x": float(detection["x"]),
                    "y": float(detection["y"]),
                    "z": float(detection["z"]),
                    "camera_x": float(detection["camera_x"]),
                    "camera_y": float(detection["camera_y"]),
                    "camera_z": float(detection["camera_z"]),
                    "yaw": float(detection["yaw"]),
                    "image_yaw": float(detection["image_yaw"]),
                    "robot_pickable": bool(detection["robot_pickable"])
                }
                for detection in detections
            ]
        }

        with open(metadata_file, "w", encoding="utf-8") as metadata_handle:  # Open metadatafile
            json.dump(metadata, metadata_handle, indent=2)  # Schrijf metadata als JSON

        self.get_logger().info(f"Dataset sample saved: {uid}")  # Log datasetopslag
        return True  # Meld succesvolle opslag

    # =====================================================
    # Main Loop
    # =====================================================

    def main_loop(self):
        if not self.running:  # Controleer of node actief is
            return  # Stop wanneer camera niet draait
        if not USE_YOLO:  # Draai tijdelijke RGB/depth preview zonder YOLO
            rgb_packet = self.rgb_queue.tryGet()  # Lees RGB-packet non-blocking

            if rgb_packet is None:  # Geen nieuw frame beschikbaar
                return  # Stop deze timer-iteratie

            frame = rgb_packet.getCvFrame()  # Haal OpenCV-frame op
            self.last_rgb_frame_time = time.monotonic()  # Update watchdogtijd

            frame, _ = self.estimate_aruco_pose(frame)  # Probeer ArUco-pose te schatten

            self.marked_image_pub.publish(  # Publiceer gemarkeerde preview
                self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            )

            return  # Stop hier; geen YOLO-verwerking

        try:
            self.vision_processing_loop()  # Voer live preview loop uit
        except Exception as ex:
            self.get_logger().error(str(ex))  # Log fout
            reconnect_thread = threading.Thread(target=self.reconnect_camera, daemon=True)  # Maak reconnectthread
            reconnect_thread.start()  # Start reconnectthread

# =========================================================
# Main
# =========================================================

def main(args=None):
    rclpy.init(args=args)  # Initialiseer ROS2
    node = VisionNode()  # Maak VisionNode aan
    try:
        rclpy.spin(node)  # Laat ROS2 node draaien
    except KeyboardInterrupt:
        pass  # Stop netjes bij Ctrl+C
    finally:
        try:
            if node.device is not None:  # Controleer of device open is
                node.device.close()  # Sluit DepthAI-device
        except Exception:
            pass  # Negeer sluitfouten
        node.destroy_node()  # Vernietig ROS2 node
        rclpy.shutdown()  # Sluit ROS2 af

if __name__ == "__main__":
    main()  # Start main wanneer bestand direct wordt uitgevoerd