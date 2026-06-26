#!/usr/bin/env python3

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
            self.aruco_params = cv2.aruco.DetectorParameters_create()  # Maak detectorparameters via oude API
        else:  # Gebruik nieuwe OpenCV ArUco API
            self.aruco_params = cv2.aruco.DetectorParameters()  # Maak detectorparameters via nieuwe API

        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX  # Verfijn markerhoeken subpixel voor stabielere pose
        self.aruco_params.cornerRefinementWinSize = 5  # Gebruik venster van 5 pixels voor corner refinement
        self.aruco_params.cornerRefinementMaxIterations = 50  # Geef corner refinement genoeg iteraties
        self.aruco_params.cornerRefinementMinAccuracy = 0.01  # Stop refinement bij hoge nauwkeurigheid
        self.camera_matrix = np.array(CAMERA_MATRIX, dtype=np.float32)  # Zet fallback cameramatrix om naar NumPy
        self.dist_coeffs = np.array(DIST_COEFFS, dtype=np.float32)  # Zet fallback distortioncoëfficiënten om naar NumPy
        self.rvec = None  # Laatst berekende marker-naar-camera rotatievector
        self.tvec = None  # Laatst berekende marker-naar-camera translatievector
        self.last_valid_rvec = None  # Bewaar laatst geaccepteerde ArUco rotatievector
        self.last_valid_tvec = None  # Bewaar laatst geaccepteerde ArUco translatievector
        self.max_aruco_translation_jump_m = 0.02  # Maximale toegestane ArUco-positiesprong in meters
        self.max_aruco_rotation_jump_rad = 0.35  # Maximale toegestane ArUco-rotatiesprong in radialen
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
            self.get_logger().info("=================================")
            self.camera_control_queue.send(ctrl)  # Verstuur controlcommand naar de camera
        except Exception as ex:
            self.get_logger().warn(f"Failed to set AWB mode: {ex}")  # Log fout bij AWB-instelling

    # =====================================================
    # Pipeline
    # =====================================================

    def create_pipeline(self):
        self.get_logger().info('Creating DepthAI pipeline...')  # Log dat de pipeline wordt opgebouwd

        pipeline = dai.Pipeline()  # Maak een nieuwe DepthAI pipeline aan
        pipeline.setOpenVINOVersion(dai.OpenVINO.Version.VERSION_2022_1)  # Forceer dezelfde OpenVINO-versie als waarmee de blob is gecompileerd

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
                    if DEBUG_LOG_DEVICE_INFO:  # Controleer of device-info in terminal gewenst is
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

        corners, ids, _ = cv2.aruco.detectMarkers(  # Detecteer ArUco markers
            gray,  # Gebruik grayscale beeld
            self.aruco_dict,  # Gebruik ingestelde ArUco dictionary
            parameters=self.aruco_params  # Gebruik ingestelde detectorparameters
        )

        if ids is None:  # Controleer of er markers gevonden zijn
            self.world_calibrated = False  # Zet world calibration uit
            return frame, False  # Geef frame terug zonder geldige pose

        for i, marker_id in enumerate(ids.flatten()):  # Loop door gevonden marker-ID's
            if int(marker_id) != self.aruco_marker_id:  # Controleer of dit de gewenste marker is
                continue  # Sla andere markers over

            marker_corners = corners[i].reshape((4, 2)).astype(np.float32)  # Zet markerhoeken naar 4x2 float32

            cv2.cornerSubPix(  # Verfijn markerhoeken nogmaals handmatig op subpixelniveau
                gray,  # Gebruik grayscale beeld
                marker_corners,  # Geef markerhoeken mee die verfijnd worden
                (5, 5),  # Gebruik zoekvenster van 5x5 pixels
                (-1, -1),  # Gebruik geen dead zone in het midden
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 0.01)  # Stopcriteria voor subpixel refinement
            )

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

            success, rvec, tvec, reprojection_error = self.choose_best_ippe_pose(  # Kies beste IPPE-pose
                object_points,  # Geef 3D markerpunten mee
                marker_corners  # Geef verfijnde 2D markerhoeken mee
            )

            if not success:  # Controleer of solvePnPGeneric gelukt is
                self.world_calibrated = False  # Zet world calibration uit
                return frame, False  # Geef foutstatus terug

            if reprojection_error > ARUCO_MAX_REPROJECTION_ERROR_PX:  # Controleer reprojection error
                self.get_logger().warn(f"Rejected ArUco pose: reprojection error {reprojection_error:.3f} px")  # Log fout
                self.world_calibrated = False  # Zet world calibration uit
                return frame, False  # Weiger onbetrouwbare pose

            if ARUCO_USE_POSE_VALIDATION and not self.accept_aruco_pose(rvec, tvec):  # Controleer of pose geen onrealistische sprong maakt
                self.world_calibrated = False  # Zet world calibration uit
                return frame, False  # Weiger springende pose

            self.rvec = rvec.copy()  # Sla geaccepteerde rotatievector op
            self.tvec = tvec.copy()  # Sla geaccepteerde translatievector op
            self.world_calibrated = True  # Markeer world calibration als geldig

            if DEBUG_LOG_ARUCO_POSE:  # Controleer of ArUco pose debug gewenst is
                self.get_logger().info(f"ArUco reprojection error: {reprojection_error:.3f} px")  # Log reprojection error
                self.get_logger().info(f"ArUco rvec: {self.rvec.flatten()}")  # Log ArUco rotatievector
                self.get_logger().info(f"ArUco tvec: {self.tvec.flatten()}")  # Log ArUco translatievector

            cv2.aruco.drawDetectedMarkers(frame, corners, ids)  # Teken gevonden markers

            cv2.drawFrameAxes(  # Teken assen op marker
                frame,  # Beeld waarop getekend wordt
                self.camera_matrix,  # Cameramatrix
                self.dist_coeffs,  # Distortioncoëfficiënten
                self.rvec,  # Rotatievector
                self.tvec,  # Translatievector
                self.aruco_size_m  # Aslengte
            )

            if DEBUG_LOG_ARUCO_POSE:  # Controleer of ArUco positie debug gewenst is
                self.get_logger().info(f'ArUco ID {marker_id}: x={float(tvec[0]):.3f} m, y={float(tvec[1]):.3f} m, z={float(tvec[2]):.3f} m') # Log marker positie

            return frame, True  # Geef frame en successtatus terug

        self.world_calibrated = False  # Geen juiste marker gevonden
        return frame, False  # Geef frame zonder geldige calibration terug

    # =====================================================
    # Validate ArUco Pose
    # =====================================================

    def accept_aruco_pose(self, rvec, tvec):
        if self.last_valid_rvec is None or self.last_valid_tvec is None:  # Controleer of er nog geen vorige pose bestaat
            self.last_valid_rvec = rvec.copy()  # Bewaar huidige rotatie als eerste geldige pose
            self.last_valid_tvec = tvec.copy()  # Bewaar huidige translatie als eerste geldige pose
            return True  # Accepteer eerste pose altijd

        translation_jump = float(np.linalg.norm(tvec - self.last_valid_tvec))  # Bereken verschil in translatie
        rotation_jump = self.calculate_rvec_rotation_difference(rvec, self.last_valid_rvec)  # Bereken fysiek rotatieverschil

        if translation_jump > self.max_aruco_translation_jump_m:  # Controleer of translatie te veel springt
            if DEBUG_LOG_REJECTED_SAMPLES:  # Controleer of rejected-sample logs gewenst zijn
                self.get_logger().warn(f"Rejected ArUco pose: translation jump {translation_jump:.3f} m")  # Log geweigerde translatie
            return False  # Weiger deze ArUco-pose

        if rotation_jump > self.max_aruco_rotation_jump_rad:  # Controleer of rotatie te veel springt
            if DEBUG_LOG_REJECTED_SAMPLES:  # Controleer of rejected-sample logs gewenst zijn
                self.get_logger().warn(f"Rejected ArUco pose: rotation jump {rotation_jump:.3f} rad")  # Log geweigerde rotatie
            return False  # Weiger deze ArUco-pose

        self.last_valid_rvec = rvec.copy()  # Bewaar geaccepteerde rotatievector
        self.last_valid_tvec = tvec.copy()  # Bewaar geaccepteerde translatievector
        return True  # Accepteer deze ArUco-pose

    # =====================================================
    # Reprojection Error
    # =====================================================

    def calculate_reprojection_error(self, object_points, image_points, rvec, tvec):  # Bereken reprojection error van een pose
        projected_points, _ = cv2.projectPoints(  # Projecteer 3D markerpunten terug naar beeldpunten
            object_points,  # Gebruik markerpunten in markercoördinaten
            rvec,  # Gebruik rotatievector van solvePnP
            tvec,  # Gebruik translatievector van solvePnP
            self.camera_matrix,  # Gebruik huidige cameramatrix
            self.dist_coeffs  # Gebruik huidige distortioncoëfficiënten
        )
        projected_points = projected_points.reshape(-1, 2)  # Zet geprojecteerde punten naar Nx2-vorm
        image_points = image_points.reshape(-1, 2)  # Zet gemeten beeldpunten naar Nx2-vorm
        errors = np.linalg.norm(projected_points - image_points, axis=1)  # Bereken pixelafstand per hoek
        mean_error = float(np.mean(errors))  # Bereken gemiddelde fout in pixels
        return mean_error  # Geef gemiddelde reprojection error terug

    # =====================================================
    # Choose Best IPPE Pose
    # =====================================================

    def choose_best_ippe_pose(self, object_points, image_points):  # Kies de beste pose uit solvePnPGeneric IPPE-oplossingen
        result = cv2.solvePnPGeneric(  # Bereken meerdere mogelijke poses voor vierkante marker
            object_points,  # Gebruik 3D markerhoeken
            image_points,  # Gebruik 2D markerhoeken
            self.camera_matrix,  # Gebruik cameramatrix
            self.dist_coeffs,  # Gebruik distortioncoëfficiënten
            flags=cv2.SOLVEPNP_IPPE_SQUARE  # Gebruik IPPE-methode voor vierkante marker
        )

        success = bool(result[0])  # Lees successtatus uit resultaat
        rvecs = result[1]  # Lees lijst met rotatievectoren
        tvecs = result[2]  # Lees lijst met translatievectoren

        if not success or len(rvecs) == 0:  # Controleer of er oplossingen zijn
            return False, None, None, None  # Geef foutstatus terug

        best_rvec = None  # Initialiseer beste rotatievector
        best_tvec = None  # Initialiseer beste translatievector
        best_error = None  # Initialiseer beste reprojection error

        for rvec_candidate, tvec_candidate in zip(rvecs, tvecs):  # Loop door alle IPPE-oplossingen
            if float(tvec_candidate[2]) <= 0.0:  # Controleer of marker voor de camera ligt
                continue  # Sla oplossing achter de camera over

            error = self.calculate_reprojection_error(  # Bereken reprojection error voor deze oplossing
                object_points,  # Geef 3D markerpunten mee
                image_points,  # Geef 2D markerpunten mee
                rvec_candidate,  # Geef kandidaatrotatie mee
                tvec_candidate  # Geef kandidaattranslatie mee
            )

            if best_error is None or error < best_error:  # Controleer of deze oplossing beter is
                best_error = error  # Bewaar laagste fout
                best_rvec = rvec_candidate.copy()  # Bewaar beste rotatievector
                best_tvec = tvec_candidate.copy()  # Bewaar beste translatievector

        if best_rvec is None or best_tvec is None:  # Controleer of er een geldige oplossing gevonden is
            return False, None, None, None  # Geef foutstatus terug

        return True, best_rvec, best_tvec, best_error  # Geef beste pose terug

    # =====================================================
    # Depth Position From ROI
    # =====================================================

    def estimate_position_from_depth_roi(self, depth_frame, bbox):
        x_min, y_min, x_max, y_max = bbox  # Lees bbox uit
        depth_height, depth_width = depth_frame.shape[:2]  # Lees depthformaat uit

        x_min = max(0, min(x_min, depth_width - 1))  # Clamp links
        x_max = max(0, min(x_max, depth_width - 1))  # Clamp rechts
        y_min = max(0, min(y_min, depth_height - 1))  # Clamp boven
        y_max = max(0, min(y_max, depth_height - 1))  # Clamp onder

        if x_max <= x_min or y_max <= y_min:  # Controleer geldige ROI
            return None  # Stop bij ongeldige ROI

        roi_width = x_max - x_min  # Bereken ROI-breedte
        roi_height = y_max - y_min  # Bereken ROI-hoogte
        bbox_center_x = int((x_min + x_max) / 2)  # Bepaal bboxcentrum X als fallback
        bbox_center_y = int((y_min + y_max) / 2)  # Bepaal bboxcentrum Y als fallback

        shrunk_width = int(roi_width * ROI_SHRINK_FACTOR)  # Verklein ROI-breedte voor depthmeting
        shrunk_height = int(roi_height * ROI_SHRINK_FACTOR)  # Verklein ROI-hoogte voor depthmeting
        roi_x_min = max(0, bbox_center_x - shrunk_width // 2)  # Nieuwe ROI-links
        roi_x_max = min(depth_width, bbox_center_x + shrunk_width // 2)  # Nieuwe ROI-rechts
        roi_y_min = max(0, bbox_center_y - shrunk_height // 2)  # Nieuwe ROI-boven
        roi_y_max = min(depth_height, bbox_center_y + shrunk_height // 2)  # Nieuwe ROI-onder

        roi_depth = depth_frame[roi_y_min:roi_y_max, roi_x_min:roi_x_max]  # Pak centrale depth-ROI
        valid_depth = roi_depth[(roi_depth > MIN_DEPTH_MM) & (roi_depth < MAX_DEPTH_MM)]  # Filter geldige depthwaarden

        if valid_depth.size < MIN_VALID_DEPTH_PIXELS:  # Controleer genoeg geldige depthpixels
            return None  # Stop bij te weinig geldige depth

        z_mm = float(np.median(valid_depth))  # Bepaal mediaan objectdiepte
        z_m = z_mm / 1000.0  # Zet depth om naar meters

        full_roi_depth = depth_frame[y_min:y_max, x_min:x_max]  # Pak volledige bbox-ROI voor zwaartepunt
        object_mask = (  # Maak depthmasker rond objectdiepte
            (full_roi_depth > z_mm - DEPTH_BAND_MM) &
            (full_roi_depth < z_mm + DEPTH_BAND_MM)
        )

        ys, xs = np.where(object_mask)  # Zoek objectpixels binnen bbox

        if xs.size >= MIN_VALID_DEPTH_PIXELS:  # Controleer of genoeg objectpixels bestaan
            center_x = int(x_min + np.mean(xs))  # Gebruik oud depth-zwaartepunt X
            center_y = int(y_min + np.mean(ys))  # Gebruik oud depth-zwaartepunt Y
        else:
            center_x = bbox_center_x  # Gebruik bboxcentrum als fallback
            center_y = bbox_center_y  # Gebruik bboxcentrum als fallback

        fx = float(self.camera_matrix[0, 0])  # Lees fx uit cameramatrix
        fy = float(self.camera_matrix[1, 1])  # Lees fy uit cameramatrix
        cx_camera = float(self.camera_matrix[0, 2])  # Lees cx uit cameramatrix
        cy_camera = float(self.camera_matrix[1, 2])  # Lees cy uit cameramatrix

        x_m = ((center_x - cx_camera) * z_m / fx)  # Bereken camera-X vanuit oud middenpunt
        y_m = ((center_y - cy_camera) * z_m / fy)  # Bereken camera-Y vanuit oud middenpunt

        return (x_m, y_m, z_m, z_mm, center_x, center_y)  # Geef positie terug

    # =====================================================
    # Classical Object Pose From ROI
    # =====================================================

    def estimate_object_axis_from_classical_roi(self, frame, bbox):
        x_min, y_min, x_max, y_max = bbox  # Lees YOLO-boundingbox uit
        image_height, image_width = frame.shape[:2]  # Lees beeldformaat

        x_min = max(0, min(x_min, image_width - 1))  # Clamp bbox-links
        x_max = max(0, min(x_max, image_width - 1))  # Clamp bbox-rechts
        y_min = max(0, min(y_min, image_height - 1))  # Clamp bbox-boven
        y_max = max(0, min(y_max, image_height - 1))  # Clamp bbox-onder

        if x_max <= x_min or y_max <= y_min:  # Controleer geldige bbox
            return None  # Stop bij ongeldige bbox

        bbox_width = x_max - x_min  # Bereken bbox-breedte
        bbox_height = y_max - y_min  # Bereken bbox-hoogte
        margin_x = int(bbox_width * 0.25)  # Maak cropmarge in X
        margin_y = int(bbox_height * 0.25)  # Maak cropmarge in Y

        crop_x_min = max(0, x_min - margin_x)  # Bepaal crop-links
        crop_x_max = min(image_width, x_max + margin_x)  # Bepaal crop-rechts
        crop_y_min = max(0, y_min - margin_y)  # Bepaal crop-boven
        crop_y_max = min(image_height, y_max + margin_y)  # Bepaal crop-onder

        crop = frame[crop_y_min:crop_y_max, crop_x_min:crop_x_max].copy()  # Pak RGB-crop
        crop_height, crop_width = crop.shape[:2]  # Lees cropformaat

        if crop_width < 10 or crop_height < 10:  # Controleer minimale cropgrootte
            return None  # Stop bij te kleine crop

        grabcut_mask = np.zeros((crop_height, crop_width), dtype=np.uint8)  # Maak GrabCut-masker
        bgd_model = np.zeros((1, 65), dtype=np.float64)  # Maak achtergrondmodel
        fgd_model = np.zeros((1, 65), dtype=np.float64)  # Maak voorgrondmodel

        rect_x = max(1, x_min - crop_x_min)  # Bbox-links relatief in crop
        rect_y = max(1, y_min - crop_y_min)  # Bbox-boven relatief in crop
        rect_w = max(2, min(x_max - x_min, crop_width - rect_x - 1))  # Bbox-breedte relatief in crop
        rect_h = max(2, min(y_max - y_min, crop_height - rect_y - 1))  # Bbox-hoogte relatief in crop
        grabcut_rect = (rect_x, rect_y, rect_w, rect_h)  # Maak GrabCut-rectangle

        try:
            cv2.grabCut(crop, grabcut_mask, grabcut_rect, bgd_model, fgd_model, 3, cv2.GC_INIT_WITH_RECT)  # Segmenteer object
        except Exception as ex:
            self.get_logger().warn(f"GrabCut failed: {ex}")  # Log fout
            return None  # Stop bij fout

        foreground_mask = (  # Maak binair voorgrondmasker
            (grabcut_mask == cv2.GC_FGD) |
            (grabcut_mask == cv2.GC_PR_FGD)
        ).astype(np.uint8)  # Zet om naar uint8

        kernel = np.ones((3, 3), np.uint8)  # Maak morphology-kernel
        foreground_mask = cv2.morphologyEx(foreground_mask, cv2.MORPH_OPEN, kernel)  # Verwijder ruis
        foreground_mask = cv2.morphologyEx(foreground_mask, cv2.MORPH_CLOSE, kernel)  # Vul kleine gaten

        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(foreground_mask, connectivity=8)  # Zoek componenten

        if num_labels <= 1:  # Controleer of objectcomponent bestaat
            return None  # Stop zonder objectcomponent

        largest_label = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))  # Kies grootste component
        object_mask = (labels == largest_label).astype(np.uint8)  # Maak objectmasker
        ys, xs = np.where(object_mask > 0)  # Zoek objectpixels

        if xs.size < MIN_VALID_DEPTH_PIXELS:  # Controleer genoeg objectpixels
            return None  # Stop bij te weinig pixels

        global_xs = xs + crop_x_min  # Zet crop-X om naar beeld-X
        global_ys = ys + crop_y_min  # Zet crop-Y om naar beeld-Y

        points = np.column_stack((global_xs.astype(np.float32), global_ys.astype(np.float32)))  # Bouw puntenwolk van objectpixels
        mean = np.mean(points, axis=0)  # Bereken gemiddelde punt
        centered_points = points - mean  # Centreer puntenwolk

        covariance = np.cov(centered_points, rowvar=False)  # Bereken covariantie
        eigenvalues, eigenvectors = np.linalg.eig(covariance)  # Bereken PCA

        sorted_indices = np.argsort(eigenvalues)[::-1]  # Sorteer eigenwaarden aflopend
        major_index = int(sorted_indices[0])  # Index langste as
        minor_index = int(sorted_indices[1])  # Index kortste as

        principal_axis = eigenvectors[:, major_index]  # Pak langste objectas

        if float(principal_axis[0]) < 0.0:  # Houd richting consistent
            principal_axis = -principal_axis  # Draai as om

        raw_object_axis_yaw = math.atan2(float(principal_axis[1]), float(principal_axis[0]))  # Bereken ruwe objectas
        raw_gripper_yaw = raw_object_axis_yaw + math.pi / 2.0  # Bereken ruwe gripperas exact haaks

        object_axis_yaw = self.normalize_axis_yaw(raw_object_axis_yaw)  # Normaliseer objectas voor opslag
        gripper_yaw = self.normalize_axis_yaw(raw_gripper_yaw)  # Normaliseer gripperas voor opslag

        eigenvalue_major = float(eigenvalues[major_index])  # Lees hoofdvariantie
        eigenvalue_minor = float(eigenvalues[minor_index])  # Lees minorvariantie
        axis_ratio = eigenvalue_major / max(eigenvalue_minor, 1e-6)  # Bereken asbetrouwbaarheid

        return (gripper_yaw, object_axis_yaw, axis_ratio)  # Geef alleen rotatie-info terug

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
        world_yaw = math.atan2(w2_y - w1_y, w2_x - w1_x)  # Bereken world-yaw
        world_yaw = self.normalize_axis_yaw(world_yaw)  # Normaliseer world-yaw als richtingloze grijpas
        return world_yaw  # Geef gestabiliseerde world-yaw terug

    # =====================================================
    # Normalize Angle
    # =====================================================

    def normalize_angle_pi(self, angle):
        while angle > math.pi:  # Controleer of hoek groter is dan pi
            angle -= 2.0 * math.pi  # Trek één volledige rotatie af
        while angle <= -math.pi:  # Controleer of hoek kleiner of gelijk is aan min pi
            angle += 2.0 * math.pi  # Tel één volledige rotatie op
        return angle  # Geef genormaliseerde hoek terug

    # =====================================================
    # Normalize Axis Yaw
    # =====================================================

    def normalize_axis_yaw(self, yaw):
        yaw = self.normalize_angle_pi(yaw)  # Normaliseer yaw eerst naar -pi tot +pi
        if yaw > math.pi / 2.0:  # Controleer of yaw boven +90 graden ligt
            yaw -= math.pi  # Trek 180 graden af omdat de grijpas richtingloos is
        if yaw <= -math.pi / 2.0:  # Controleer of yaw onder of gelijk aan -90 graden ligt
            yaw += math.pi  # Tel 180 graden op omdat de grijpas richtingloos is
        return yaw  # Geef yaw terug binnen -90 tot +90 graden

    # =====================================================
    # Mean Axis Yaw
    # =====================================================

    def mean_axis_yaw(self, yaw_values):
        if len(yaw_values) == 0:  # Controleer of er yaw-waarden zijn
            return 0.0  # Geef neutrale yaw terug

        doubled_sin_sum = 0.0  # Som van sinus van dubbele yaw
        doubled_cos_sum = 0.0  # Som van cosinus van dubbele yaw

        for yaw in yaw_values:  # Loop door alle yaw-metingen
            normalized_yaw = self.normalize_axis_yaw(float(yaw))  # Normaliseer yaw als richtingloze as
            doubled_sin_sum += math.sin(2.0 * normalized_yaw)  # Voeg sinus van dubbele yaw toe
            doubled_cos_sum += math.cos(2.0 * normalized_yaw)  # Voeg cosinus van dubbele yaw toe

        mean_doubled_yaw = math.atan2(doubled_sin_sum, doubled_cos_sum)  # Bereken gemiddelde dubbele hoek
        mean_yaw = 0.5 * mean_doubled_yaw  # Halveer terug naar gewone yaw
        mean_yaw = self.normalize_axis_yaw(mean_yaw)  # Normaliseer eindresultaat opnieuw
        return mean_yaw  # Geef gemiddelde richtingloze yaw terug

    # =====================================================
    # Axis Yaw Standard Deviation
    # =====================================================

    def axis_yaw_std(self, yaw_values, mean_yaw):
        if len(yaw_values) <= 1:  # Controleer of standaarddeviatie zinvol is
            return 0.0  # Geef nul terug bij één of geen metingen

        errors = []  # Maak lijst voor hoekfouten

        for yaw in yaw_values:  # Loop door alle yaw-metingen
            yaw_error = self.normalize_axis_yaw(float(yaw) - float(mean_yaw))  # Bereken richtingloze yaw-fout
            errors.append(yaw_error)  # Voeg fout toe aan lijst

        return float(np.std(np.array(errors, dtype=np.float64)))  # Geef standaarddeviatie van yaw terug

    # =====================================================
    # Combine Object Measurements
    # =====================================================

    def combine_object_measurements(self, measurements):
        if len(measurements) == 0:  # Controleer of er metingen zijn
            return None  # Geef None terug zonder metingen

        x_values = np.array([float(obj["x"]) for obj in measurements], dtype=np.float64)  # Verzamel X-waarden
        y_values = np.array([float(obj["y"]) for obj in measurements], dtype=np.float64)  # Verzamel Y-waarden
        z_values = np.array([float(obj["z"]) for obj in measurements], dtype=np.float64)  # Verzamel Z-waarden
        yaw_values = [float(obj["yaw"]) for obj in measurements]  # Verzamel yaw-waarden
        confidence_values = np.array([float(obj["confidence"]) for obj in measurements], dtype=np.float64)  # Verzamel confidencewaarden

        median_x = float(np.median(x_values))  # Neem mediaan van X voor robuuste positie
        median_y = float(np.median(y_values))  # Neem mediaan van Y voor robuuste positie
        median_z = float(np.median(z_values))  # Neem mediaan van Z voor robuuste positie
        mean_yaw = self.mean_axis_yaw(yaw_values)  # Neem correct hoekgemiddelde van yaw
        mean_confidence = float(np.mean(confidence_values))  # Neem gemiddelde confidence

        std_x = float(np.std(x_values))  # Bereken X-spreiding
        std_y = float(np.std(y_values))  # Bereken Y-spreiding
        std_z = float(np.std(z_values))  # Bereken Z-spreiding
        std_position = float(max(std_x, std_y, std_z))  # Neem grootste XYZ-spreiding als betrouwbaarheidsscore
        std_yaw = self.axis_yaw_std(yaw_values, mean_yaw)  # Bereken yaw-spreiding

        best_reference = max(measurements, key=lambda obj: float(obj["confidence"]))  # Gebruik hoogste confidence als basisrecord
        combined_object = best_reference.copy()  # Maak kopie van beste objectrecord

        combined_object["object_id"] = str(uuid.uuid4())  # Maak nieuw ID voor gecombineerde meting
        combined_object["x"] = median_x  # Vul gefilterde X
        combined_object["y"] = median_y  # Vul gefilterde Y
        combined_object["z"] = median_z  # Vul gefilterde Z
        combined_object["yaw"] = mean_yaw  # Vul gefilterde yaw
        combined_object["confidence"] = mean_confidence  # Vul gemiddelde confidence
        combined_object["position_std"] = std_position  # Bewaar grootste XYZ-spreiding voor debug
        combined_object["yaw_std"] = std_yaw  # Bewaar yaw-spreiding voor debug
        combined_object["sample_count"] = len(measurements)  # Bewaar aantal gebruikte metingen

        if DEBUG_LOG_SAMPLE_RESULT:  # Controleer of eindresultaat gelogd moet worden
            self.get_logger().info(  # Log gecombineerde meetkwaliteit
                f"Combined object from {len(measurements)} samples: "
                f"x={median_x:.4f}, y={median_y:.4f}, z={median_z:.4f}, yaw={mean_yaw:.4f}, "
                f"pos_std={std_position:.4f} m, yaw_std={std_yaw:.4f} rad"
            )

        if std_position > OBJECT_MAX_POSITION_STD_M:  # Controleer of positie te veel spreidt
            self.get_logger().warn(f"Object position spread too high: {std_position:.4f} m")  # Log te hoge XYZ-spreiding

        if std_yaw > OBJECT_MAX_YAW_STD_RAD:  # Controleer of yaw te veel spreidt
            self.get_logger().warn(f"Object yaw spread too high: {std_yaw:.4f} rad")  # Log te hoge yaw-spreiding

        return combined_object  # Geef gecombineerd object terug

    # =====================================================
    # Process Stable Object Request
    # =====================================================

    def process_synced_packets_to_best_object(self, nn_frame_packet, detection_packet, depth_packet, save_dataset=False):
        frame = nn_frame_packet.getCvFrame()  # Haal NN-frame op
        depth_frame = depth_packet.getFrame()  # Haal depthframe op

        if DEBUG_LOG_FRAME_INFO:  # Controleer of frame/debuginformatie gewenst is
            self.get_logger().info(f"NN frame shape: {frame.shape}")  # Log RGB/NN-frameformaat
            self.get_logger().info(f"Depth frame shape: {depth_frame.shape}")  # Log depthframeformaat
            self.get_logger().info(f"Raw detections: {len(detection_packet.detections)}")  # Log aantal ruwe YOLO-detecties

        frame, calibrated = self.estimate_aruco_pose(frame)  # Bepaal ArUco world pose

        if not calibrated and DEBUG_LOG_REJECTED_SAMPLES:  # Controleer of world calibration gelukt is
            self.get_logger().warn("Object sample rejected: ArUco marker not detected")  # Log ontbrekende ArUco marker
            return None  # Stop zonder object

        object_list, best_object, dataset_frame, marked_frame = self.process_frame_to_objects(  # Verwerk detecties naar objecten
            frame,  # Gebruik NN-frame
            depth_frame,  # Gebruik gesynchroniseerd depthframe
            detection_packet.detections  # Gebruik YOLO-detecties
        )

        if len(object_list) == 0 and DEBUG_LOG_REJECTED_SAMPLES:  # Controleer of er objecten over zijn
            self.get_logger().warn("Object sample rejected: no valid objects with depth")  # Log geen geldige objecten
            return None  # Stop zonder object

        if SAVE_DATASET_ON_REQUEST and save_dataset:  # Controleer of datasetopslag voor deze sample gewenst is
            self.save_dataset_sample(dataset_frame, object_list)  # Sla datasetopname op

        if best_object is None and DEBUG_LOG_REJECTED_SAMPLES:  # Controleer of robotgeschikt object bestaat
            self.get_logger().warn("Object sample rejected: no robot-pickable object")  # Log geen robotobject
            return None  # Stop zonder robotobject

        self.marked_image_pub.publish(self.bridge.cv2_to_imgmsg(marked_frame, encoding="bgr8"))  # Publiceer gemarkeerd beeld
        return best_object  # Geef beste object terug

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
            position_result = self.estimate_position_from_depth_roi(depth_frame, bbox)  # Gebruik oude betere positiebepaling

            if position_result is None:  # Controleer of positie geldig is
                continue  # Sla object zonder geldige positie over

            camera_x, camera_y, camera_z, z_mm, center_x, center_y = position_result  # Pak oude positie uit

            axis_result = self.estimate_object_axis_from_classical_roi(frame, bbox)  # Gebruik klassieke vision alleen voor objectas

            if axis_result is None:  # Controleer of rotatie geldig is
                image_gripper_yaw = 0.0  # Gebruik fallback-gripperhoek
                image_object_axis_yaw = 0.0  # Gebruik fallback-objectas
                axis_ratio = 0.0  # Markeer lage betrouwbaarheid
            else:
                image_gripper_yaw, image_object_axis_yaw, axis_ratio = axis_result  # Pak rotatie uit

            cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)  # Teken optisch zwaartepunt rood

            world_yaw = self.image_yaw_to_world_yaw(center_x, center_y, camera_z, image_gripper_yaw)  # Transformeer gripperrichting naar world-yaw
            world_x, world_y, world_z = self.transform_to_world(camera_x, camera_y, camera_z)  # Transformeer optisch zwaartepunt naar world-positie

            obj = {  # Bouw intern objectrecord
                "object_id": str(uuid.uuid4()),  # Maak uniek object-ID
                "class": int(detection.label),  # Bewaar klasse-ID
                "confidence": confidence,  # Bewaar confidence
                "x": world_x,  # Bewaar world-X
                "y": world_y,  # Bewaar world-Y
                "z": world_z,  # Bewaar world-Z
                "camera_x": camera_x,  # Bewaar camera-X
                "camera_y": camera_y,  # Bewaar camera-Y
                "camera_z": camera_z,  # Bewaar camera-Z
                "yaw": world_yaw,  # World-gripperyaw
                "image_yaw": image_gripper_yaw,  # Beeldvlak-gripperyaw
                "image_object_axis_yaw": image_object_axis_yaw,  # Beeldvlak-objectas
                "axis_ratio": axis_ratio,  # Betrouwbaarheid objectas
                "bbox": bbox,  # Bewaar boundingbox
                "robot_pickable": False  # Initialiseer robotfilterstatus
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
            self.draw_yaw_axis(frame, center_x, center_y, bbox, image_gripper_yaw, (0, 255, 255))  # Geel = gripperas haaks op objectas
            self.draw_yaw_axis(frame, center_x, center_y, bbox, image_object_axis_yaw, (255, 0, 0))  # Blauw = echte langste objectas

        return (object_list, best_object, dataset_frame, frame)  # Geef objecten, beste object, datasetframe en gemarkeerd frame terug

    # =====================================================
    # Draw Yaw Axis
    # =====================================================

    def draw_yaw_axis(self, frame, center_x, center_y, bbox, yaw, color):
        x_min, y_min, x_max, y_max = bbox  # Lees bbox uit
        axis_length = int(min(x_max - x_min, y_max - y_min) * 0.45)  # Bepaal halve aslengte

        dx = axis_length * math.cos(yaw)  # Bereken X-richting
        dy = axis_length * math.sin(yaw)  # Bereken Y-richting

        start_x = int(center_x - dx)  # Bereken startpunt X
        start_y = int(center_y - dy)  # Bereken startpunt Y
        end_x = int(center_x + dx)  # Bereken eindpunt X
        end_y = int(center_y + dy)  # Bereken eindpunt Y

        cv2.line(frame, (start_x, start_y), (end_x, end_y), color, 2)  # Teken volledige richtingloze as

    # =====================================================
    # Process Stable Object Request
    # =====================================================

    def process_stable_object_request(self):
        measurements = []  # Lijst met geldige objectmetingen
        end_time = time.monotonic() + OBJECT_SAMPLE_TIMEOUT_SEC  # Bepaal eindtijd voor complete sampleverzameling

        nn_frames = {}  # Buffer voor NN-frame packets
        detections = {}  # Buffer voor detection packets
        depths = {}  # Buffer voor depth packets
        used_sequences = set()  # Set met reeds gebruikte sequence-nummers

        while time.monotonic() < end_time and len(measurements) < OBJECT_SAMPLE_COUNT:  # Verzamel samples tot count of timeout
            nn_frame_packet = self.nn_frame_queue.tryGet()  # Probeer een NN-frame packet te lezen
            detection_packet = self.detection_queue.tryGet()  # Probeer een detection packet te lezen
            depth_packet = self.depth_queue.tryGet()  # Probeer een depth packet te lezen

            if nn_frame_packet is not None:  # Controleer of er een NN-frame is ontvangen
                nn_sequence = nn_frame_packet.getSequenceNum()  # Lees sequence number van NN-frame
                nn_frames[nn_sequence] = nn_frame_packet  # Sla NN-frame op onder sequence number
                self.last_rgb_frame_time = time.monotonic()  # Update watchdogtijd

            if detection_packet is not None:  # Controleer of er detecties zijn ontvangen
                detection_sequence = detection_packet.getSequenceNum()  # Lees sequence number van detecties
                detections[detection_sequence] = detection_packet  # Sla detecties op onder sequence number

            if depth_packet is not None:  # Controleer of er een depthpacket is ontvangen
                depth_sequence = depth_packet.getSequenceNum()  # Lees sequence number van depth
                depths[depth_sequence] = depth_packet  # Sla depth op onder sequence number

            exact_sequences = set(nn_frames.keys()).intersection(detections.keys()).intersection(depths.keys())  # Zoek exact gelijke sequence numbers
            available_sequences = sorted([sequence for sequence in exact_sequences if sequence not in used_sequences])  # Pak alleen ongebruikte matches

            for sequence in available_sequences:  # Loop door alle nieuwe exacte matches
                used_sequences.add(sequence)  # Markeer sequence als gebruikt

                if DEBUG_LOG_SYNC:  # Controleer of sync-debug gewenst is
                    self.get_logger().info(f"Stable sample sequence={sequence}, " f"nn={nn_frames[sequence].getSequenceNum()}, " f"det={detections[sequence].getSequenceNum()}, " f"depth={depths[sequence].getSequenceNum()}")

                best_object = self.process_synced_packets_to_best_object(  # Verwerk deze exacte packetset
                    nn_frames[sequence],  # Geef NN-frame mee
                    detections[sequence],  # Geef detecties mee
                    depths[sequence],  # Geef depth mee
                    save_dataset=False  # Sla niet elke sample apart op
                )

                if best_object is not None:  # Controleer of sample geldig is
                    measurements.append(best_object)  # Voeg geldige meting toe

                if len(measurements) >= OBJECT_SAMPLE_COUNT:  # Controleer of genoeg samples verzameld zijn
                    break  # Stop for-loop

            if len(nn_frames) > 20:  # Beperk buffergrootte
                newest_sequences = sorted(nn_frames.keys())[-20:]  # Bewaar alleen nieuwste 20 sequences
                nn_frames = {sequence: nn_frames[sequence] for sequence in newest_sequences}  # Snoei NN-buffer

            if len(detections) > 20:  # Beperk detectiebuffer
                newest_sequences = sorted(detections.keys())[-20:]  # Bewaar alleen nieuwste 20 sequences
                detections = {sequence: detections[sequence] for sequence in newest_sequences}  # Snoei detectiebuffer

            if len(depths) > 20:  # Beperk depthbuffer
                newest_sequences = sorted(depths.keys())[-20:]  # Bewaar alleen nieuwste 20 sequences
                depths = {sequence: depths[sequence] for sequence in newest_sequences}  # Snoei depthbuffer

            time.sleep(0.005)  # Wacht kort om CPU-belasting te beperken

        if len(measurements) == 0:  # Controleer of er geen enkele meting gelukt is
            self.get_logger().warn("Stable object request failed: no valid samples")  # Log geen samples
            return None  # Geef None terug

        if len(measurements) < OBJECT_SAMPLE_COUNT:  # Controleer of er minder samples dan gewenst zijn
            self.get_logger().warn(f"Stable object request has few samples: {len(measurements)}/{OBJECT_SAMPLE_COUNT}")  # Log weinig samples

        combined_object = self.combine_object_measurements(measurements)  # Combineer alle geldige metingen
        return combined_object  # Geef stabiel object terug

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

        if DEBUG_LOG_TRANSFORM_POINTS:  # Controleer of transformpunten gelogd moeten worden
            self.get_logger().info(f"point_camera: x={float(point_camera[0, 0]):.4f}, " f"y={float(point_camera[1, 0]):.4f}, "f"z={float(point_camera[2, 0]):.4f}")
            self.get_logger().info(f"point_marker: x={float(point_marker[0, 0]):.4f}, " f"y={float(point_marker[1, 0]):.4f}, " f"z={float(point_marker[2, 0]):.4f}")

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
        end_time = time.monotonic() + timeout_sec  # Bepaal het eindtijdstip van de timeout
        nn_frames = {}  # Maak buffer voor NN-frame packets
        detections = {}  # Maak buffer voor detection packets
        depths = {}  # Maak buffer voor depth packets

        while time.monotonic() < end_time:  # Blijf zoeken tot de timeout verlopen is
            nn_frame_packet = self.nn_frame_queue.tryGet()  # Probeer een NN-frame packet te lezen
            detection_packet = self.detection_queue.tryGet()  # Probeer een detection packet te lezen
            depth_packet = self.depth_queue.tryGet()  # Probeer een depth packet te lezen

            if nn_frame_packet is not None:  # Controleer of er een NN-frame is ontvangen
                nn_sequence = nn_frame_packet.getSequenceNum()  # Lees sequence number van het NN-frame
                nn_frames[nn_sequence] = nn_frame_packet  # Sla NN-frame op onder sequence number
                self.last_rgb_frame_time = time.monotonic()  # Update watchdogtijd

            if detection_packet is not None:  # Controleer of er een detection packet is ontvangen
                detection_sequence = detection_packet.getSequenceNum()  # Lees sequence number van detections
                detections[detection_sequence] = detection_packet  # Sla detections op onder sequence number

            if depth_packet is not None:  # Controleer of er een depth packet is ontvangen
                depth_sequence = depth_packet.getSequenceNum()  # Lees sequence number van depth
                depths[depth_sequence] = depth_packet  # Sla depth op onder sequence number

            exact_sequences = set(nn_frames.keys()).intersection(detections.keys()).intersection(depths.keys())  # Zoek exact gelijke sequence numbers

            if len(exact_sequences) > 0:  # Controleer of exacte synchronisatie bestaat
                sequence = max(exact_sequences)  # Pak de nieuwste exacte sequence
                if DEBUG_LOG_SYNC:  # Controleer of sync-debug gewenst is
                    self.get_logger().info(f"Synced packets sequence={sequence}, " f"nn={nn_frames[sequence].getSequenceNum()}, " f"det={detections[sequence].getSequenceNum()}, " f"depth={depths[sequence].getSequenceNum()}")
                return (nn_frames[sequence], detections[sequence], depths[sequence])  # Geef exact gesynchroniseerde packets terug

            time.sleep(0.005)  # Wacht kort om CPU-belasting te beperken

        self.get_logger().warn("No exact synced NN/detection/depth packets found")  # Log dat exacte sync niet gevonden is
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
    # Rvec Rotation Difference
    # =====================================================

    def calculate_rvec_rotation_difference(self, rvec_a, rvec_b):
        rotation_a, _ = cv2.Rodrigues(rvec_a)  # Zet eerste rvec om naar rotatiematrix
        rotation_b, _ = cv2.Rodrigues(rvec_b)  # Zet tweede rvec om naar rotatiematrix
        rotation_delta = rotation_a @ rotation_b.T  # Bereken relatieve rotatie tussen beide poses
        trace_value = float(np.trace(rotation_delta))  # Lees trace van relatieve rotatiematrix
        cosine_value = (trace_value - 1.0) / 2.0  # Zet trace om naar cosinus van rotatiehoek
        cosine_value = max(-1.0, min(1.0, cosine_value))  # Clamp voor numerieke veiligheid
        rotation_difference = math.acos(cosine_value)  # Bereken echte rotatiehoek in radialen
        return rotation_difference  # Geef fysieke rotatieverschilhoek terug

    # =====================================================
    # Object Request Callback
    # =====================================================

    def object_request_callback(self, request, response):
        with self.processing_lock:  # Blokkeer gelijktijdige objectaanvragen
            if not self.running or self.device is None:  # Controleer of camera en device actief zijn
                response.success = False  # Markeer response als mislukt
                return response  # Geef response terug

            if not USE_YOLO:  # Controleer of YOLO uit staat
                self.get_logger().warn("Object request rejected: YOLO is disabled")  # Log dat objectdetectie niet beschikbaar is
                response.success = False  # Markeer response als mislukt
                return response  # Geef response terug

            if self.nn_frame_queue is None or self.detection_queue is None or self.depth_queue is None:  # Controleer of alle benodigde queues bestaan
                self.get_logger().warn("Object request rejected: required DepthAI queues are unavailable")  # Log ontbrekende queues
                response.success = False  # Markeer response als mislukt
                return response  # Geef response terug

            self.confidence_threshold = float(request.confidence_threshold)  # Lees confidencegrens uit request
            self.get_logger().info("Object request received")  # Log nieuwe aanvraag
            best_object = self.process_stable_object_request()  # Verwerk meerdere samples tot één stabiel object
            if best_object is None:  # Controleer of object gevonden is
                response.success = False  # Markeer response als mislukt
                return response  # Geef response terug

            best_object_msg = self.convert_to_ros_msg(best_object)  # Converteer beste object naar ROS-message

            ui_msg = ObjectDataArray()  # Maak UI-array message
            ui_msg.objects = [best_object_msg]  # Zet alleen het definitieve gecombineerde object in de UI-array
            self.object_ui_pub.publish(ui_msg)  # Publiceer één UI-message per service request

            self.object_L6_pub.publish(best_object_msg)  # Publiceer beste object op debugtopic
            response.success = True  # Markeer response als geslaagd
            response.object = best_object_msg  # Vul response objectveld
            return response  # Geef response terug

    # =====================================================
    # Live Preview Loop
    # =====================================================

    def vision_processing_loop(self):
        if not self.running or self.device is None:  # Controleer of de camera actief is
            return  # Stop wanneer de camera niet actief is

        if self.rgb_queue is None:  # Controleer of de RGB queue bestaat
            return  # Stop wanneer er geen RGB queue beschikbaar is

        rgb_packet = self.rgb_queue.tryGet()  # Lees live RGB-frame non-blocking uit de queue

        if rgb_packet is None:  # Controleer of er een nieuw frame beschikbaar is
            return  # Stop wanneer er geen nieuw frame is

        self.last_rgb_frame_time = time.monotonic()  # Update watchdogtijd zodat de watchdog weet dat de camera leeft

        frame = rgb_packet.getCvFrame()  # Haal OpenCV-frame uit het DepthAI packet
        self.latest_frame = frame.copy()  # Bewaar laatste frame intern voor eventuele debug of latere inspectie

        if not DEBUG_PUBLISH_CONTINUOUS:  # Controleer of continue debugpublicatie uit staat
            return  # Stop zonder iets te publiceren

        debug_frame = frame.copy()  # Maak een kopie zodat het originele frame niet onnodig aangepast wordt

        if DEBUG_DRAW_ARUCO_LIVE:  # Controleer of ArUco-overlays in live debugbeeld gewenst zijn
            debug_frame, _ = self.estimate_aruco_pose(debug_frame)  # Teken ArUco-pose op het debugbeeld indien marker zichtbaar is

        self.marked_image_pub.publish(  # Publiceer het debugbeeld alleen wanneer DEBUG_PUBLISH_CONTINUOUS True is
            self.bridge.cv2_to_imgmsg(debug_frame, encoding="bgr8")  # Converteer OpenCV BGR-frame naar ROS Image message
        )

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
        msg.transform.transform.rotation.x = 0.0
        msg.transform.transform.rotation.y = 0.0
        msg.transform.transform.rotation.z = yaw
        msg.transform.transform.rotation.w = 0.0
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
        if not self.running:  # Controleer of de node actief is
            return  # Stop wanneer camera/node niet draait

        if self.rgb_queue is None:  # Controleer of de RGB queue beschikbaar is
            return  # Stop wanneer er geen RGB queue is

        if not USE_YOLO:  # Gebruik tijdelijke RGB/depth/ArUco modus zonder YOLO
            rgb_packet = self.rgb_queue.tryGet()  # Lees RGB-packet non-blocking

            if rgb_packet is None:  # Controleer of er een nieuw frame beschikbaar is
                return  # Stop deze timer-iteratie zonder watchdogupdate

            frame = rgb_packet.getCvFrame()  # Haal OpenCV-frame op uit DepthAI packet
            self.last_rgb_frame_time = time.monotonic()  # Update watchdogtijd omdat er een geldig RGB-frame is ontvangen
            self.latest_frame = frame.copy()  # Bewaar laatste frame intern voor debug of inspectie

            if not DEBUG_PUBLISH_CONTINUOUS:  # Controleer of continue debugpublicatie uit staat
                return  # Stop zonder marked image te publiceren

            debug_frame = frame.copy()  # Maak kopie voor tekenen/publiceren

            if DEBUG_DRAW_ARUCO_LIVE:  # Controleer of ArUco in live debugbeeld getekend moet worden
                debug_frame, _ = self.estimate_aruco_pose(debug_frame)  # Schat en teken ArUco-pose alleen in debugmodus

            self.marked_image_pub.publish(  # Publiceer marked image alleen wanneer debugpublicatie aan staat
                self.bridge.cv2_to_imgmsg(debug_frame, encoding="bgr8")  # Converteer OpenCV-frame naar ROS Image
            )

            return  # Stop hier omdat YOLO uit staat

        try:
            self.vision_processing_loop()  # Verwerk live/debug preview wanneer YOLO actief is
        except Exception as ex:
            self.get_logger().error(str(ex))  # Log foutmelding
            reconnect_thread = threading.Thread(target=self.reconnect_camera, daemon=True)  # Maak reconnectthread
            reconnect_thread.start()  # Start reconnectthread

# =========================================================
# Main
# =========================================================

def main(args=None):
    rclpy.init(args=args)  # Initialiseer ROS2
    node = None  # Initialiseer node als None zodat finally veilig blijft

    try:
        node = VisionNode()  # Maak VisionNode aan
        rclpy.spin(node)  # Laat ROS2 node draaien

    except KeyboardInterrupt:
        pass  # Stop netjes bij Ctrl+C

    finally:
        if node is not None:  # Controleer of node bestaat
            node.running = False  # Stop normale verwerking
            node.watchdog_running = False  # Stop watchdogthread

            try:
                if node.device is not None:  # Controleer of device open is
                    node.device.close()  # Sluit DepthAI-device
            except Exception:
                pass  # Negeer sluitfouten

            try:
                node.destroy_node()  # Vernietig ROS2 node
            except Exception:
                pass  # Negeer destroy-fouten tijdens shutdown

        if rclpy.ok():  # Controleer of ROS2-context nog actief is
            rclpy.shutdown()  # Sluit ROS2 alleen af als dat nog niet gebeurd is

if __name__ == "__main__":
    main()  # Start main wanneer bestand direct wordt uitgevoerd