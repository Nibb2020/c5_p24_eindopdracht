#!/usr/bin/env python3

from enum import Enum
from typing import Optional

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future

from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger

from project_interfaces.srv import Manipulator, VoorwerpData
from xarm_msgs.msg import RobotMsg


class State(Enum):
    STANDBY = "stand-by"
    RUN_VISION = "run vision"
    RUN_MANIPULATOR = "run manipulator"
    MOVING_HOME = "moving home"
    TRAINING_INFERENCE = "training/inference mode"
    RESETTING = "resetting"
    ERROR = "error"


class RobotController(Node):
    def __init__(self) -> None:
        super().__init__("robot_controller")

        self.callback_group = ReentrantCallbackGroup()

        self.current_state = State.STANDBY
        self.previous_state = State.STANDBY

        self.run_enabled = False

        self.warning_active = False
        self.warning_message = ""

        self.error_active = False
        self.error_message = ""

        self.emergency_stop_detected = False

        self.vision_request_active = False
        self.manipulator_request_active = False
        self.home_request_active = False
        self.reset_request_active = False
        self.robot_ready_after_reset = False

        self.operation_generation = 0

        self.waiting_for_restart_response = False
        self.waiting_for_robot_ready = False
        self.waiting_for_reset_home = False

        self.reset_ready_counter = 0
        self.reset_ready_required = 5

        self.reset_wait_counter = 0
        self.reset_wait_limit = 1200

        self.declare_parameter("vision_confidence", 0.7)
        self.declare_parameter("vision_awb", True)

        self.vision_confidence = self.get_parameter(
            "vision_confidence"
        ).value
        self.vision_awb = self.get_parameter(
            "vision_awb"
        ).value

        self.state_publisher = self.create_publisher(
            String,
            "/controller/state",
            10,
        )

        self.warning_publisher = self.create_publisher(
            String,
            "/controller/warning",
            10,
        )

        self.error_publisher = self.create_publisher(
            String,
            "/controller/error",
            10,
        )

        self.emergency_publisher = self.create_publisher(
            Bool,
            "/controller/emergency_stop",
            10,
        )

        self.start_subscription = self.create_subscription(
            Bool,
            "/ui/start",
            self.start_callback,
            10,
            callback_group=self.callback_group,
        )

        self.manipulator_status_subscription = self.create_subscription(
            String,
            "/manipulator/status",
            self.manipulator_status_callback,
            10,
            callback_group=self.callback_group,
        )

        self.robot_state_subscription = self.create_subscription(
            RobotMsg,
            "/xarm/robot_states",
            self.robot_state_callback,
            10,
            callback_group=self.callback_group,
        )

        self.vision_client = self.create_client(
            VoorwerpData,
            "/vision/voorwerp_data",
            callback_group=self.callback_group,
        )

        self.manipulator_client = self.create_client(
            Manipulator,
            "/manipulator/start",
            callback_group=self.callback_group,
        )

        # Change this name/type if your manipulator uses another service.
        self.move_home_client = self.create_client(
            Trigger,
            "/manipulator/move_home",
            callback_group=self.callback_group,
        )

        self.supervisor_restart_client = self.create_client(
            Trigger,
            "/system_supervisor/restart",
            callback_group=self.callback_group,
        )

        self.reset_service = self.create_service(
            Trigger,
            "/controller/reset_error",
            self.reset_error_callback,
            callback_group=self.callback_group,
        )

        self.move_home_service = self.create_service(
            Trigger,
            "/controller/move_home",
            self.move_home_ui_callback,
            callback_group=self.callback_group,
        )

        self.training_service = self.create_service(
            SetBool,
            "/controller/training_mode",
            self.training_mode_callback,
            callback_group=self.callback_group,
        )

        self.state_timer = self.create_timer(
            0.1,
            self.state_machine_loop,
            callback_group=self.callback_group,
        )

        self.publish_all_ui_data()
        self.get_logger().info("Robot controller started in stand-by")


    def state_machine_loop(self) -> None:
        if self.current_state == State.RESETTING:
            if (self.waiting_for_robot_ready or self.waiting_for_reset_home):
                self.reset_wait_counter += 1

                if self.reset_wait_counter >= self.reset_wait_limit:
                    self.reset_failed(
                        "Reset timed out while waiting for the robot "
                        "or home movement"
                    )
                    return

            if (self.waiting_for_robot_ready and self.robot_ready_after_reset and self.move_home_client.service_is_ready()):
                self.start_reset_home()

            return

        if self.current_state in (
            State.ERROR,
            State.TRAINING_INFERENCE,
            State.RUN_VISION,
            State.RUN_MANIPULATOR,
            State.MOVING_HOME,
        ):
            return

        if (
            self.current_state == State.STANDBY
            and self.run_enabled
        ):
            self.change_state(State.RUN_VISION)
            self.start_vision()

    def start_callback(self, message: Bool) -> None:
        self.run_enabled = message.data

        if self.run_enabled:
            self.get_logger().info("Automatic operation enabled")
        else:
            self.get_logger().info(
                "Stop requested. Current cycle will finish."
            )

    def start_vision(self) -> None:
        if self.current_state != State.RUN_VISION:
            return
        if self.vision_request_active:
            return
        if not self.vision_client.service_is_ready():
            self.enter_error("Vision service is unavailable")
            return

        self.vision_request_active = True
        generation = self.operation_generation

        request = VoorwerpData.Request()
        request.confidence = float(self.vision_confidence)
        request.awb = bool(self.vision_awb)

        future = self.vision_client.call_async(request)
        future.add_done_callback(
            lambda completed_future: self.vision_response_callback(
                completed_future,
                generation,
            )
        )

    def vision_response_callback(
        self,
        future: Future,
        generation: int,
    ) -> None:
        self.vision_request_active = False

        if generation != self.operation_generation:
            return
        if self.current_state != State.RUN_VISION:
            return

        try:
            response = future.result()
        except Exception as exception:
            self.enter_error(f"Vision service failed: {exception}")
            return

        klasse = response.klasse.strip()
        translation = list(response.translation)
        rotation = float(response.rotation)

        if not klasse:
            self.enter_error("Vision returned no object class")
            return

        if len(translation) != 3:
            self.enter_error(
                "Vision translation must contain x, y and z"
            )
            return

        self.change_state(State.RUN_MANIPULATOR)
        self.start_manipulator(klasse, translation, rotation)

    def start_manipulator(
        self,
        klasse: str,
        translation: list[float],
        rotation: float,
    ) -> None:
        if self.current_state != State.RUN_MANIPULATOR:
            return
        if self.manipulator_request_active:
            return
        if not self.manipulator_client.service_is_ready():
            self.enter_error("Manipulator service is unavailable")
            return

        self.manipulator_request_active = True
        generation = self.operation_generation

        request = Manipulator.Request()
        request.klasse = klasse
        request.translation = translation
        request.rotation = rotation

        future = self.manipulator_client.call_async(request)
        future.add_done_callback(
            lambda completed_future: self.manipulator_response_callback(
                completed_future,
                generation,
            )
        )

    def manipulator_response_callback(
        self,
        future: Future,
        generation: int,
    ) -> None:
        self.manipulator_request_active = False

        if generation != self.operation_generation:
            return
        if self.current_state != State.RUN_MANIPULATOR:
            return

        try:
            response = future.result()
        except Exception as exception:
            self.enter_error(f"Manipulator service failed: {exception}")
            return

        if not response.succes:
            self.enter_error("Manipulator rejected the request")
            return

        self.get_logger().info(
            "Manipulator started. Waiting for status 'klaar'."
        )

    def move_home_ui_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request

        if self.current_state != State.STANDBY:
            response.success = False
            response.message = (
                "Move home is only allowed while the controller "
                "is in stand-by"
            )
            return response

        if self.run_enabled:
            response.success = False
            response.message = (
                "Stop automatic operation before moving home"
            )
            return response

        if self.home_request_active:
            response.success = False
            response.message = "A move-home request is already active"
            return response

        if not self.move_home_client.service_is_ready():
            response.success = False
            response.message = (
                "Manipulator move-home service is unavailable"
            )
            return response

        self.change_state(State.MOVING_HOME)
        self.request_move_home(for_reset=False)

        response.success = True
        response.message = "Move-home request accepted"
        return response

    def request_move_home(self, for_reset: bool) -> None:
        if self.home_request_active:
            message = "A move-home request is already active"
            if for_reset:
                self.reset_failed(message)
            else:
                self.enter_error(message)
            return

        if not self.move_home_client.service_is_ready():
            message = "Manipulator move-home service is unavailable"
            if for_reset:
                self.reset_failed(message)
            else:
                self.enter_error(message)
            return

        self.home_request_active = True
        generation = self.operation_generation

        request = Trigger.Request()
        future = self.move_home_client.call_async(request)
        future.add_done_callback(
            lambda completed_future: self.move_home_response_callback(
                completed_future,
                generation,
                for_reset,
            )
        )

        self.get_logger().info("Move-home request sent to manipulator")

    def move_home_response_callback(
        self,
        future: Future,
        generation: int,
        for_reset: bool,
    ) -> None:
        self.home_request_active = False

        if generation != self.operation_generation:
            return

        if for_reset:
            if (
                self.current_state != State.RESETTING
                or not self.waiting_for_reset_home
            ):
                return
        elif self.current_state != State.MOVING_HOME:
            return

        try:
            response = future.result()
        except Exception as exception:
            if for_reset:
                self.reset_failed(
                    f"Move-home service failed: {exception}"
                )
            else:
                self.enter_error(
                    f"Move-home service failed: {exception}"
                )
            return

        if not response.success:
            message = (
                response.message
                or "Manipulator rejected the move-home request"
            )

            if for_reset:
                self.reset_failed(message)
            else:
                self.enter_error(message)
            return

        self.get_logger().info(
            "Move home started. Waiting for status 'klaar'."
        )

    def manipulator_status_callback(self, message: String) -> None:
        status = message.data.strip()
        status_lower = status.lower()

        if not status:
            return

        if status_lower.startswith("fout"):
            if self.current_state == State.RESETTING:
                self.reset_failed(status)
            else:
                self.enter_error(status)
            return

        if status_lower != "klaar":
            return

        if self.current_state == State.RUN_MANIPULATOR:
            self.get_logger().info(
                "Manipulator sorting operation completed"
            )
            self.change_state(State.STANDBY)
            return

        if self.current_state == State.MOVING_HOME:
            self.get_logger().info(
                "Manual move-home operation completed"
            )
            self.change_state(State.STANDBY)
            return

        if (
            self.current_state == State.RESETTING
            and self.waiting_for_reset_home
        ):
            self.get_logger().info(
                "Reset move-home operation completed"
            )
            self.complete_reset()

    @staticmethod
    def is_emergency_condition(message: RobotMsg) -> bool:
        return (
            message.state == 4
            and message.err == 2
            and message.mt_brake == 0
            and message.mt_able == 0
        )

    @staticmethod
    def is_robot_ready(message: RobotMsg) -> bool:
        return (
            message.state == 2
            and message.err == 0
            and message.mt_brake == 63
            and message.mt_able == 63
        )

    def robot_state_callback(self, message: RobotMsg) -> None:
        if self.current_state == State.RESETTING:
            if not self.waiting_for_robot_ready:
                return

            if self.is_robot_ready(message):
                self.reset_ready_counter += 1

                if self.reset_ready_counter >= self.reset_ready_required:
                    self.robot_ready_after_reset = True
            else:
                self.reset_ready_counter = 0
                self.robot_ready_after_reset = False
            return

        if self.is_emergency_condition(message):
            if not self.emergency_stop_detected:
                self.emergency_stop_detected = True
                self.publish_emergency_stop()
                self.enter_error("Emergency stop activated")
            return

        if self.current_state == State.ERROR:
            return

        if message.err != 0:
            self.enter_error(f"xArm error code: {message.err}")
            return

        if message.warn != 0:
            self.set_warning(f"xArm warning code: {message.warn}")
        else:
            self.clear_warning()

    def reset_error_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request

        if self.current_state != State.ERROR:
            response.success = False
            response.message = "Controller is not in the error state"
            return response

        if self.reset_request_active:
            response.success = False
            response.message = "Reset is already running"
            return response

        if not self.supervisor_restart_client.service_is_ready():
            response.success = False
            response.message = "System supervisor is unavailable"
            return response

        self.run_enabled = False
        self.reset_request_active = True
        self.operation_generation += 1

        self.vision_request_active = False
        self.manipulator_request_active = False
        self.home_request_active = False

        self.waiting_for_restart_response = True
        self.waiting_for_robot_ready = False
        self.waiting_for_reset_home = False
        self.robot_ready_after_reset = False

        self.reset_ready_counter = 0
        self.reset_wait_counter = 0

        self.change_state(State.RESETTING)

        generation = self.operation_generation
        future = self.supervisor_restart_client.call_async(
            Trigger.Request()
        )
        future.add_done_callback(
            lambda completed_future:
            self.supervisor_restart_response_callback(
                completed_future,
                generation,
            )
        )

        response.success = True
        response.message = "System reset started"
        return response

    def supervisor_restart_response_callback(
        self,
        future: Future,
        generation: int,
    ) -> None:
        if generation != self.operation_generation:
            return
        if self.current_state != State.RESETTING:
            return

        self.waiting_for_restart_response = False

        try:
            response = future.result()
        except Exception as exception:
            self.reset_failed(
                f"Supervisor restart failed: {exception}"
            )
            return

        if not response.success:
            self.reset_failed(
                response.message or "System restart failed"
            )
            return

        self.waiting_for_robot_ready = True
        self.reset_ready_counter = 0
        self.reset_wait_counter = 0

        self.get_logger().info(
            "Processes restarted. Waiting for the robot to report ready."
        )


    def start_reset_home(self) -> None:
        if self.current_state != State.RESETTING:
            return

        if not self.waiting_for_robot_ready:
            return

        if not self.robot_ready_after_reset:
            return

        if not self.move_home_client.service_is_ready():
            return

        self.waiting_for_robot_ready = False
        self.waiting_for_reset_home = True
        self.reset_ready_counter = 0
        self.reset_wait_counter = 0

        self.get_logger().info(
            "Robot and manipulator are ready. "
            "Moving home before completing reset."
        )

        self.request_move_home(for_reset=True)

    def complete_reset(self) -> None:
        if self.current_state != State.RESETTING:
            return

        self.reset_request_active = False
        self.waiting_for_restart_response = False
        self.waiting_for_robot_ready = False
        self.waiting_for_reset_home = False
        self.robot_ready_after_reset = False
        self.reset_ready_counter = 0
        self.reset_wait_counter = 0

        self.emergency_stop_detected = False
        self.error_active = False
        self.error_message = ""
        self.warning_active = False
        self.warning_message = ""
        self.run_enabled = False

        self.publish_emergency_stop()
        self.publish_error()
        self.publish_warning()

        self.change_state(State.STANDBY)
        self.get_logger().info(
            "System reset completed and robot is home"
        )

    def reset_failed(self, message: str) -> None:
        self.reset_request_active = False
        self.home_request_active = False
        self.waiting_for_restart_response = False
        self.waiting_for_robot_ready = False
        self.waiting_for_reset_home = False
        self.robot_ready_after_reset = False
        self.reset_ready_counter = 0
        self.reset_wait_counter = 0

        self.current_state = State.ERROR
        self.error_active = True
        self.error_message = message

        self.publish_state()
        self.publish_error()
        self.get_logger().error(message)

    def training_mode_callback(
        self,
        request: SetBool.Request,
        response: SetBool.Response,
    ) -> SetBool.Response:
        if request.data:
            if self.current_state != State.STANDBY:
                response.success = False
                response.message = (
                    "Training mode can only start from stand-by"
                )
                return response

            if self.run_enabled:
                response.success = False
                response.message = (
                    "Stop automatic operation before starting "
                    "training mode"
                )
                return response

            self.change_state(State.TRAINING_INFERENCE)
            response.success = True
            response.message = "Training mode started"
            return response

        if self.current_state != State.TRAINING_INFERENCE:
            response.success = False
            response.message = "Training mode is not active"
            return response

        self.change_state(State.STANDBY)
        response.success = True
        response.message = "Training mode stopped"
        return response

    def enter_error(self, message: str) -> None:
        self.run_enabled = False

        if self.current_state != State.ERROR:
            self.previous_state = self.current_state

        self.operation_generation += 1
        self.vision_request_active = False
        self.manipulator_request_active = False
        self.home_request_active = False
        self.reset_request_active = False
        self.waiting_for_restart_response = False
        self.waiting_for_robot_ready = False
        self.waiting_for_reset_home = False

        self.current_state = State.ERROR
        self.error_active = True
        self.error_message = message

        self.publish_state()
        self.publish_error()
        self.get_logger().error(message)

    def set_warning(self, message: str) -> None:
        if self.warning_active and self.warning_message == message:
            return

        self.warning_active = True
        self.warning_message = message
        self.publish_warning()
        self.get_logger().warning(message)

    def clear_warning(self) -> None:
        if not self.warning_active:
            return

        self.warning_active = False
        self.warning_message = ""
        self.publish_warning()

    def change_state(self, new_state: State) -> None:
        if self.current_state == new_state:
            return

        old_state = self.current_state
        self.previous_state = old_state
        self.current_state = new_state

        self.publish_state()
        self.get_logger().info(
            f"State changed: {old_state.value} -> {new_state.value}"
        )

    def publish_state(self) -> None:
        message = String()
        message.data = self.current_state.value
        self.state_publisher.publish(message)

    def publish_warning(self) -> None:
        message = String()
        message.data = (
            self.warning_message if self.warning_active else ""
        )
        self.warning_publisher.publish(message)

    def publish_error(self) -> None:
        message = String()
        message.data = self.error_message if self.error_active else ""
        self.error_publisher.publish(message)

    def publish_emergency_stop(self) -> None:
        message = Bool()
        message.data = self.emergency_stop_detected
        self.emergency_publisher.publish(message)

    def publish_all_ui_data(self) -> None:
        self.publish_state()
        self.publish_warning()
        self.publish_error()
        self.publish_emergency_stop()


def main(args=None) -> None:
    rclpy.init(args=args)

    node: Optional[RobotController] = None
    executor: Optional[MultiThreadedExecutor] = None

    try:
        node = RobotController()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
