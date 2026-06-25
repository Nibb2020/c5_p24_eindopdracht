#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from project_interfaces.srv import GetObjectData


class MockVision(Node):
    def __init__(self) -> None:
        super().__init__("mock_vision")

        self.service = self.create_service(
            GetObjectData,
            "/vision/voorwerp_data",
            self.vision_callback,
        )

        self.get_logger().info(
            "Mock vision service started"
        )

    def vision_callback(
        self,
        request: GetObjectData.Request,
        response: GetObjectData.Response,
    ) -> GetObjectData.Response:
        self.get_logger().info(
            f"Mock vision request received, threshold: "
            f"{request.confidence_threshold}"
        )

        response.success = True

        response.object.object_class = "dino"
        response.object.object_id = "mock_dino_1"
        response.object.confidence = 0.95

        response.object.transform.transform.translation.x = 0.30
        response.object.transform.transform.translation.y = 0.20
        response.object.transform.transform.translation.z = 0.12

        response.object.transform.transform.rotation.x = 0.0
        response.object.transform.transform.rotation.y = 0.0
        response.object.transform.transform.rotation.z = 0.0
        response.object.transform.transform.rotation.w = 1.0

        return response


def main(args=None) -> None:
    rclpy.init(args=args)

    node = MockVision()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()