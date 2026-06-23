#!/usr/bin/env python3

import os
import signal
import subprocess
import threading
import time
from typing import Optional

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from std_srvs.srv import Trigger


class SystemSupervisor(Node):
    def __init__(self) -> None:
        super().__init__("system_supervisor")

        self.callback_group = ReentrantCallbackGroup()
        self.process_lock = threading.Lock()

        self.system_process: Optional[subprocess.Popen] = None

        self.startup_started = False
        self.restart_active = False

        self.declare_parameter(
            "workspace_path",
            "/home/student/P4_C5_project_ws",
        )

        self.declare_parameter(
            "restart_launch_package",
            "controller",
        )

        self.declare_parameter(
            "restart_launch_file",
            "robot_and_manipulator.launch.py",
        )

        self.declare_parameter(
            "startup_check_time",
            5.0,
        )

        self.declare_parameter(
            "shutdown_timeout",
            12.0,
        )

        self.workspace_path = str(
            self.get_parameter("workspace_path").value
        )

        self.restart_launch_package = str(
            self.get_parameter(
                "restart_launch_package"
            ).value
        )

        self.restart_launch_file = str(
            self.get_parameter(
                "restart_launch_file"
            ).value
        )

        self.startup_check_time = float(
            self.get_parameter(
                "startup_check_time"
            ).value
        )

        self.shutdown_timeout = float(
            self.get_parameter(
                "shutdown_timeout"
            ).value
        )

        self.restart_service = self.create_service(
            Trigger,
            "/system_supervisor/restart",
            self.restart_callback,
            callback_group=self.callback_group,
        )

        # This timer runs once shortly after the supervisor starts.
        self.startup_timer = self.create_timer(
            1.0,
            self.automatic_start_callback,
            callback_group=self.callback_group,
        )

        self.get_logger().info(
            "System supervisor started"
        )

    def build_launch_command(self) -> list[str]:
        command = (
            "source /opt/ros/jazzy/setup.bash && "
            f"source {self.workspace_path}/install/setup.bash && "
            "exec ros2 launch "
            f"{self.restart_launch_package} "
            f"{self.restart_launch_file}"
        )

        return [
            "bash",
            "-lc",
            command,
        ]

    @staticmethod
    def process_is_running(
        process: Optional[subprocess.Popen],
    ) -> bool:
        return (
            process is not None
            and process.poll() is None
        )

    def automatic_start_callback(self) -> None:
        if self.startup_started:
            return

        self.startup_started = True
        self.startup_timer.cancel()

        # Run process startup outside the timer callback.
        thread = threading.Thread(
            target=self.automatic_start_worker,
            daemon=True,
        )
        thread.start()

    def automatic_start_worker(self) -> None:
        self.get_logger().info(
            "Automatically starting robot and manipulator"
        )

        with self.process_lock:
            success, message = self.start_system_locked()

        if success:
            self.get_logger().info(message)
        else:
            self.get_logger().error(message)

    def start_system_locked(self) -> tuple[bool, str]:
        if self.process_is_running(self.system_process):
            return True, (
                "Robot and manipulator launch is already running"
            )

        try:
            self.get_logger().info(
                "Starting robot_and_manipulator.launch.py"
            )

            self.system_process = subprocess.Popen(
                self.build_launch_command(),
                start_new_session=True,
            )

            time.sleep(self.startup_check_time)

            if not self.process_is_running(
                self.system_process
            ):
                return_code = None

                if self.system_process is not None:
                    return_code = self.system_process.poll()

                self.system_process = None

                return False, (
                    "Robot and manipulator launch stopped "
                    f"during startup, return code: {return_code}"
                )

            return True, (
                "Robot and manipulator launch started"
            )

        except Exception as exception:
            self.system_process = None

            return False, (
                f"Could not start robot system: {exception}"
            )

    def stop_system_locked(self) -> None:
        if not self.process_is_running(
            self.system_process
        ):
            self.system_process = None
            return

        process = self.system_process

        self.get_logger().warning(
            "Stopping robot and manipulator launch"
        )

        try:
            process_group = os.getpgid(process.pid)

            # Graceful ROS shutdown, similar to Ctrl+C.
            os.killpg(
                process_group,
                signal.SIGINT,
            )

            process.wait(
                timeout=self.shutdown_timeout
            )

            self.get_logger().info(
                "Robot and manipulator launch stopped"
            )

        except subprocess.TimeoutExpired:
            self.get_logger().warning(
                "Launch did not stop after SIGINT"
            )

            try:
                os.killpg(
                    os.getpgid(process.pid),
                    signal.SIGTERM,
                )

                process.wait(timeout=5.0)

            except subprocess.TimeoutExpired:
                self.get_logger().error(
                    "Forcing launch process to stop"
                )

                try:
                    os.killpg(
                        os.getpgid(process.pid),
                        signal.SIGKILL,
                    )

                    process.wait(timeout=3.0)

                except ProcessLookupError:
                    pass

        except ProcessLookupError:
            pass

        except Exception as exception:
            self.get_logger().error(
                f"Could not stop robot system: {exception}"
            )

        finally:
            self.system_process = None

    def restart_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request

        if self.restart_active:
            response.success = False
            response.message = (
                "A system restart is already active"
            )
            return response

        self.restart_active = True

        # Run restart asynchronously so the service callback does not block
        # the executor while ROS processes shut down and start again.
        thread = threading.Thread(
            target=self.restart_worker,
            daemon=True,
        )
        thread.start()

        response.success = True
        response.message = (
            "Robot and manipulator restart started"
        )

        return response

    def restart_worker(self) -> None:
        try:
            with self.process_lock:
                self.stop_system_locked()

                # Give ROS and DDS time to remove the old nodes.
                time.sleep(3.0)

                success, message = self.start_system_locked()

            if success:
                self.get_logger().info(message)
            else:
                self.get_logger().error(message)

        finally:
            self.restart_active = False

    def destroy_node(self):
        with self.process_lock:
            self.stop_system_locked()

        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)

    node: Optional[SystemSupervisor] = None
    executor: Optional[MultiThreadedExecutor] = None

    try:
        node = SystemSupervisor()

        executor = MultiThreadedExecutor(
            num_threads=4
        )

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