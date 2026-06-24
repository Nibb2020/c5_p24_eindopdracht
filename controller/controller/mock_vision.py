#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from project_interfaces.srv import VoorwerpData


class MockVision(Node):
    def __init__(self) -> None:
        super().__init__("mock_vision")

        self.service = self.create_service(
            VoorwerpData,
            "/vision/voorwerp_data",
            self.vision_callback,
        )

        self.get_logger().info(
            "Mock vision service started"
        )

    def vision_callback(
        self,
        request: VoorwerpData.Request,
        response: VoorwerpData.Response,
    ) -> VoorwerpData.Response:
        self.get_logger().info(
            "Mock vision request received"
        )

        response.klasse = "rood"
        response.translation = [
            0.30,
            0.20,
            0.12,
        ]
        response.rotation = 0.0

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