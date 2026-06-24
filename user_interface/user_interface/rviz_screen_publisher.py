#!/usr/bin/env python3
import os
import subprocess
import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

class RvizScreenPublisher(Node):
    def __init__(self):
        super().__init__("rviz_screen_publisher")
        self.publisher = self.create_publisher(Image,"/rviz/camera_image",10)
        self.bridge = CvBridge()
        self.screenshot_path = "/tmp/rviz_screen.png"

        # Crop values for the RViz area.
        self.crop_x = 100
        self.crop_y = 100
        self.crop_width = 800
        self.crop_height = 600

        # 2 Hz is safer because gnome-screenshot starts a new process each time.
        self.timer = self.create_timer(0.5,self.capture_and_publish)
        self.get_logger().info("RViz screen publisher started")

    def capture_and_publish(self):
        try:
            subprocess.run(["gnome-screenshot","-f",self.screenshot_path],check=True,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)

            frame = cv2.imread(self.screenshot_path)
            if frame is None:
                self.get_logger().warning("Screenshot could not be read")
                return

            x1 = self.crop_x
            y1 = self.crop_y
            x2 = x1 + self.crop_width
            y2 = y1 + self.crop_height

            frame = frame[y1:y2,x1:x2]

            if frame.size == 0:
                self.get_logger().warning("RViz crop region is invalid")
                return

            frame = cv2.resize(frame,(640,480))
            message = self.bridge.cv2_to_imgmsg(frame,encoding="bgr8")
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = "rviz_screen"
            self.publisher.publish(message)

        except subprocess.CalledProcessError as exception:
            self.get_logger().error(f"Screenshot failed: {exception}")
        except Exception as exception:
            self.get_logger().error(f"RViz capture failed: {exception}")

def main(args=None):
    rclpy.init(args=args)
    node = RvizScreenPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if os.path.exists(node.screenshot_path):
            os.remove(node.screenshot_path)
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == "__main__":
    main()