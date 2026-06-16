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
"""

# =========================================================
# Imports
# =========================================================

# ROS2
import rclpy
from rclpy.node import Node

# Messages
from sensor_msgs.msg import Image
from std_msgs.msg import Header

# CvBridge
from cv_bridge import CvBridge

# OpenCV
import cv2
import cv2.aruco as aruco

# System config
from config.config import *

# DepthAI
import depthai as dai

# Utilities
import uuid
import time
import math
import os
import json
import threading
from pathlib import Path

# Custom Interfaces
from project_interfaces.srv import GetObjectData
from project_interfaces.srv import ObjectData
from project_interfaces.msg import ObjectDataArray

# =========================================================
# FUTURE PIPELINE
# =========================================================

# ArUco
# ↓
# Stereo Depth
# ↓
# Sync
# ↓
# YOLOv8n
# ↓
# ROI Extraction
# ↓
# Depth Filter
# ↓
# PCA Yaw
# ↓
# World Transform
# ↓
# UI Publish
# ↓
# Service Response

# =========================================================
# Vision Node
# =========================================================

class VisionNode(Node):

    def __init__(self):

        super().__init__("vision_node")

        # =================================================
        # ROS
        # =================================================

        self.bridge = CvBridge()

        self.object_ui_pub = self.create_publisher(ObjectDataArray, UI_TOPIC, 10)
        self.object_L6_pub = self.create_publisher(ObjectData, "/object_data_result", 10)
        self.marked_image_pub = self.create_publisher(Image, MARKED_IMAGE_TOPIC, 10)
        self.object_service = self.create_service(GetObjectData, SERVICE_NAME, self.object_request_callback)

        # =================================================
        # Runtime Variables
        # =================================================

        self.device = None
        self.pipeline = None
        self.rgb_queue = None
        self.depth_queue = None
        self.detection_queue = None
        self.running = False
        self.latest_frame = None
        self.confidence_threshold = 0.85
        self.publish_requested = False
        self.use_hardware_awb = True

        # =================================================
        # Initialize
        # =================================================

        self.initialize_camera()
        self.set_awb_mode(self.use_hardware_awb)
        self.timer = self.create_timer(0.03, self.main_loop)

    # =====================================================
    # Device Information
    # =====================================================

    def print_device_information(self):

        try:
            self.get_logger().info("========== DEVICE INFO ==========")
            self.get_logger().info(f"MXID: {self.device.getMxId()}")
            self.get_logger().info(f"USB Speed: {self.device.getUsbSpeed()}")
            self.get_logger().info(f"Connected Cameras:")
            
            cameras = (self.device.getConnectedCameraFeatures())
            for cam in cameras:
                self.get_logger().info(str(cam))
            self.get_logger().info("=================================")

        except Exception as ex:
            self.get_logger().error(str(ex))
    
    # =====================================================
    # Auto White Balance
    # =====================================================

    def set_awb_mode(self, use_hardware_awb: bool):

        if self.camera_control_queue is None:
            return

        try:
            ctrl = dai.CameraControl()

            if use_hardware_awb:
                ctrl.setAutoWhiteBalanceMode(dai.CameraControl.AutoWhiteBalanceMode.AUTO)
                self.get_logger().info("Hardware AWB enabled")

            else:
                ctrl.setAutoWhiteBalanceMode(dai.CameraControl.AutoWhiteBalanceMode.OFF)
                self.get_logger().info("Hardware AWB disabled")
            self.camera_control_queue.send(ctrl)

        except Exception as ex:
            self.get_logger().warn(f"Failed to set AWB mode: {ex}")

    # =====================================================
    # Pipeline
    # =====================================================

    def create_pipeline(self):
        self.get_logger().info("Creating DepthAI pipeline...")
        pipeline = dai.Pipeline()

        # ============================================================
        # RGB CAMERA (MAIN STREAM FOR YOLO)
        # ============================================================
        cam_rgb = pipeline.createColorCamera()
        cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_4_K)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

        # YOLO input size
        cam_rgb.setPreviewSize(1280, 1280)
        cam_rgb.setPreviewKeepAspectRatio(True)

        # ============================================================
        # CAMERA CONTROL INPUT
        # ============================================================

        control_in = pipeline.createXLinkIn()
        control_in.setStreamName("camera_control")
        control_in.out.link(cam_rgb.inputControl)

        # ============================================================
        # STEREO DEPTH SETUP
        # ============================================================
        left = pipeline.createMonoCamera()
        right = pipeline.createMonoCamera()

        left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
        right.setBoardSocket(dai.CameraBoardSocket.CAM_C)

        left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
        right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)

        stereo = pipeline.createStereoDepth()
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_ACCURACY)

        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)

        left.out.link(stereo.left)
        right.out.link(stereo.right)

        # ============================================================
        # YOLOV8 SPATIAL NETWORK (PLACEHOLDER FOR RVC2 MODEL)
        # ============================================================
        detection_nn = pipeline.create(dai.node.YoloDetectionNetwork)

        detection_nn.setConfidenceThreshold(0.01)  # overwritten via ROS later
        detection_nn.setNumClasses(5)
        detection_nn.setCoordinateSize(4)
        detection_nn.setAnchors([])
        detection_nn.setAnchorMasks({})
        detection_nn.setIouThreshold(0.5)

        detection_nn.setBlobPath("/home/student/c5_p24_eindproject_ws/src/c5_p24_eindopdracht/vision/models/yolov8n.rvc2.tar.xz")

        cam_rgb.preview.link(detection_nn.input)

        stereo.depth.link(detection_nn.inputDepth)

        # ============================================================
        # OUTPUT STREAMS
        # ============================================================
        xout_rgb = pipeline.createXLinkOut()
        xout_rgb.setStreamName("rgb")
        cam_rgb.preview.link(xout_rgb.input)

        xout_depth = pipeline.createXLinkOut()
        xout_depth.setStreamName("depth")
        stereo.depth.link(xout_depth.input)

        xout_det = pipeline.createXLinkOut()
        xout_det.setStreamName("detections")
        detection_nn.out.link(xout_det.input)

        # ============================================================
        # DEBUG / SYNC READY (for dataset logging)
        # ============================================================
        xout_synced = pipeline.createXLinkOut()
        xout_synced.setStreamName("synced")
        detection_nn.passthrough.link(xout_synced.input)

        return pipeline

    # =====================================================
    # INITIALIZE CAMERA
    # =====================================================

    def initialize_camera(self):

        success = False
        self.stale_counter = 0

        for attempt in range(RECONNECT_ATTEMPTS):
            try:
                self.get_logger().info(f"Connecting OAK-D ({attempt+1}/{RECONNECT_ATTEMPTS})")

                # -----------------------------
                # Build pipeline
                # -----------------------------
                self.pipeline = self.create_pipeline()

                # -----------------------------
                # Create device with pipeline
                # -----------------------------
                self.device = dai.Device(self.pipeline, maxUsbSpeed=dai.UsbSpeed.HIGH)

                # -----------------------------
                # Device info
                # -----------------------------
                self.print_device_information()

                # -----------------------------
                # Output queues
                # -----------------------------
                self.rgb_queue = self.device.getOutputQueue("rgb", maxSize=4, blocking=False)
                self.depth_queue = self.device.getOutputQueue("depth", maxSize=4, blocking=False)
                self.detection_queue = self.device.getOutputQueue("detections", maxSize=4, blocking=False)
                self.camera_control_queue = self.device.getInputQueue("camera_control")

                self.running = True
                success = True

                self.get_logger().info("Camera connected")

                # -----------------------------
                # Start watchdog once connected
                # -----------------------------
                self.watchdog_running = True

                if not hasattr(self, "watchdog_thread") or not self.watchdog_thread.is_alive():
                    self.watchdog_running = True
                    self.watchdog_thread = threading.Thread(target=self._watchdog_loop, daemon=True)
                    self.watchdog_thread.start()

            except Exception as ex:
                self.get_logger().warn(f"Connection failed: {ex}")
                time.sleep(RECONNECT_INTERVAL)

        if not success:
            raise RuntimeError("Unable to connect OAK-D")


    # =====================================================
    # RECONNECT CAMERA
    # =====================================================

    def reconnect_camera(self):

        self.get_logger().warn("Camera disconnected")
        
        self.running = False
        self.watchdog_running = False

        try:
            if self.device is not None:
                self.device.close()
        except Exception:
            pass

        self.device = None
        self.pipeline = None

        # reset queues
        self.rgb_queue = None
        self.depth_queue = None
        self.detection_queue = None

        # wait a bit before retry
        time.sleep(RECONNECT_INTERVAL)

        self.initialize_camera()


    # =====================================================
    # WATCHDOG LOOP
    # =====================================================

    def _watchdog_loop(self):

        self.get_logger().info("DepthAI watchdog started")
        last_frame_time = time.time()

        while self.watchdog_running:
            try:
                # -----------------------------
                # Check if device exists
                # -----------------------------
                if self.device is None:
                    time.sleep(1)
                    continue

                # -----------------------------
                # Check RGB queue health
                # -----------------------------
                if self.rgb_queue is not None:

                    try:
                        frame = self.rgb_queue.tryGet()

                        if frame is not None:
                            last_frame_time = time.time()
                            self.stale_counter = 0

                    except Exception:
                        self.stale_counter += 1

                # -----------------------------
                # Detect freeze condition
                # -----------------------------
                if time.time() - last_frame_time > 3.0:
                    self.stale_counter += 1

                # -----------------------------
                # Trigger reconnect
                # -----------------------------
                if self.stale_counter > 5:
                    self.get_logger().warn("Watchdog triggered reconnect (camera stale)")
                    self.reconnect_camera()
                    return
                time.sleep(1.0)

            except Exception as e:
                self.get_logger().warn(f"Watchdog error: {e}")
                time.sleep(1.0)

    def object_request_callback(self, request, response):

        # Store request parameters
        self.confidence_threshold = float(request.confidence_threshold)

        # Trigger processing flag
        self.publish_requested = True
        self.request_timestamp = self.get_clock().now()

        # Reset previous result
        self.latest_selected_object = None
        self.get_logger().info("Object request received")
        self.get_logger().info(f"Confidence: {self.confidence_threshold}")

        response.success = True
        return response

    def vision_processing_loop(self):
        # SAFETY CHECKS
        if not self.running or self.device is None:
            return

        if self.rgb_queue is None or self.detection_queue is None:
            return

        # GET DATA
        rgb_packet = self.rgb_queue.tryGet()
        det_packet = self.detection_queue.tryGet()

        if rgb_packet is None or det_packet is None:
            return

        frame = rgb_packet.getCvFrame()
        detections = det_packet.detections

        # PROCESS DETECTIONS
        object_list = []
        best_object = None
        best_confidence = 0.0

        for det in detections:
            confidence = float(det.confidence)

            # Apply dynamic ROS threshold filter
            if confidence < self.confidence_threshold:
                continue

            # Convert normalized coords → pixel coords
            x_min = int(det.xmin * frame.shape[1])
            x_max = int(det.xmax * frame.shape[1])
            y_min = int(det.ymin * frame.shape[0])
            y_max = int(det.ymax * frame.shape[0])

            cx = int((x_min + x_max) / 2)
            cy = int((y_min + y_max) / 2)

            # Depth placeholder (replace with stereo depth)
            z = getattr(det, "spatialCoordinates", None)
            if z is not None:
                x, y, z = z.x, z.y, z.z
            else:
                x, y, z = 0.0, 0.0, 0.0  # TODO depth fallback

            # Yaw estimation placeholder (PCA later)
            yaw = 0.0  # TODO PCA / contour-based orientation

            obj = {
                "class": det.label,
                "confidence": confidence,
                "x": x,
                "y": y,
                "z": z,
                "yaw": yaw,
                "bbox": (x_min, y_min, x_max, y_max),}

            object_list.append(obj)

            # BEST OBJECT SELECTION
            if confidence > best_confidence:
                best_confidence = confidence
                best_object = obj

            class_name = YOLO_CLASS_NAMES.get(det.label, f"class_{det.label}")

            # DRAW BBOX FOR UI
            cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
            cv2.putText(frame, f"{class_name} {confidence:.2f}", (x_min, y_min - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0) ,1)

        # UI PUBLISH (ALL OBJECTS)
        if len(object_list) > 0 and self.publish_requested:

            ui_msg = ObjectDataArray()
            ui_msg.objects = []  # TODO ROS message mapping

            for obj in object_list:
                ui_msg.objects.append(self.convert_to_ros_msg(obj))

            self.object_ui_pub.publish(ui_msg)

        # LITE6 SINGLE OBJECT OUTPUT
        if self.publish_requested and best_object is not None:
            lite6_msg = self.convert_to_ros_msg(best_object)
            self.object_L6_pub.publish(lite6_msg)
            self.get_logger().info(f"Sent to Lite6: {best_object['class']}" f"(conf={best_object['confidence']:.2f})")

            # lock request after sending
            self.publish_requested = False

        # MARKED IMAGE OUTPUT
        self.marked_image_pub.publish(self.bridge.cv2_to_imgmsg(frame, encoding="bgr8"))

    # =====================================================
    # ROS Message vullen
    # =====================================================

    def convert_to_ros_msg(self, obj):

        msg = ObjectData()
        msg.object_class = YOLO_CLASS_NAMES.get(obj["class"], f"class_{obj['class']}")
        msg.object_id = str(uuid.uuid4())

        msg.transform.header.stamp
        msg.transform.header.frame_id
        msg.transform.child_frame_id

        msg.transform.transform.translation.x
        msg.transform.transform.translation.y
        msg.transform.transform.translation.z

        msg.transform.transform.rotation

        return msg

    # =====================================================
    # Dataset Logger
    # =====================================================

    def save_dataset_sample(self, image, detections):
        
        uid = str(uuid.uuid4())
        base_path = Path(self.DATASET_FOLDER)

        # Checken of de folders bestaan
        (base_path / "images").mkdir(parents=True, exist_ok=True)
        (base_path / "labels").mkdir(parents=True, exist_ok=True)
        (base_path / "metadata").mkdir(parents=True, exist_ok=True)

        # Items opslaan
        image_file = base_path / "images" / f"{uid}.jpg"
        label_file = base_path / "labels" / f"{uid}.txt"
        meta_file = base_path / "metadata" / f"{uid}.json"

        # SAVE IMAGE
        cv2.imwrite(str(image_file), image)

        # SAVE LABELS (YOLO FORMAT)
        h, w = image.shape[:2]

        with open(label_file, "w") as f:
            for det in detections:

                # TODO: ensure det has bbox in pixel coords
                x_min, y_min, x_max, y_max = det["bbox"]

                x_center = ((x_min + x_max) / 2) / w
                y_center = ((y_min + y_max) / 2) / h
                bw = (x_max - x_min) / w
                bh = (y_max - y_min) / h

                class_id = YOLO_CLASS_MAPPING[det["class"]]

                f.write(f"{class_id} {x_center} {y_center} {bw} {bh}\n")

        # SAVE METADATA (FOR ROBOTICS / TF / ANALYSIS)
        metadata = {"id": uid, "objects": [{
            "class": det["class"],
            "confidence": det["confidence"],
            "x": det["x"],
            "y": det["y"],
            "z": det["z"],
            "yaw": det["yaw"]
        } for det in detections]}

        with open(meta_file, "w") as f:
            json.dump(metadata, f, indent=2)

    # =====================================================
    # Main Loop
    # =====================================================

    def main_loop(self):

        if not self.running:
            return

        try:
            self.vision_processing_loop()

        except Exception as ex:
            self.get_logger().error(str(ex))
            reconnect_thread = (threading.Thread(target=self.reconnect_camera, daemon=True))
            reconnect_thread.start()


# =========================================================
# Main
# =========================================================

def main(args=None):

    rclpy.init(args=args)
    node = VisionNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        try:
            if node.device is not None:
                node.device.close()

        except Exception:
            pass

        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()