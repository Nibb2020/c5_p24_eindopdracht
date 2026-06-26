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

        # Reentrant callbacks allow the node to handle another callback while
        # a service or timer callback is still active.
        self.callback_group = ReentrantCallbackGroup()

        # This lock prevents startup, shutdown and restart code from changing
        # the managed process at the same time from different threads.
        self.process_lock = threading.Lock()

        # Stores the ros2 launch process started by this supervisor.
        # None means that no managed process is currently registered.
        self.system_process: Optional[subprocess.Popen] = None

        # Prevents the automatic startup timer from starting the system twice.
        self.startup_started = False

        # Prevents two restart requests from running at the same time.
        self.restart_active = False

        # ================= PARAMETERS =================
        # Workspace that contains the built ROS 2 installation.
        self.declare_parameter(
            "workspace_path",
            "/home/student/c5_p24_eindproject_ws",
        )

        # Package and launch file used to start the robot and manipulator.
        self.declare_parameter(
            "restart_launch_package",
            "controller",
        )
        self.declare_parameter(
            "restart_launch_file",
            "robot_and_manipulator.launch.py",
        )

        # Time given to the launch process to prove that it stays running.
        self.declare_parameter(
            "startup_check_time",
            5.0,
        )

        # Maximum time given to a normal SIGINT shutdown.
        self.declare_parameter(
            "shutdown_timeout",
            12.0,
        )

        # Read the declared ROS 2 parameters once and store them locally.
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

        # ================= ROS 2 COMMUNICATION =================
        # The state machine calls this service when the robot system must restart.
        self.restart_service = self.create_service(
            Trigger,
            "/system_supervisor/restart",
            self.restart_callback,
            callback_group=self.callback_group,
        )

        # This timer runs once shortly after the supervisor starts.
        # Its callback cancels it after the first execution.
        self.startup_timer = self.create_timer(
            1.0,
            self.automatic_start_callback,
            callback_group=self.callback_group,
        )

        self.get_logger().info(
            "System supervisor started"
        )

    # =====================================================
    # LAUNCH COMMAND
    # =====================================================

    def build_launch_command(self) -> list[str]:
        # Start a new shell, source ROS 2 and the workspace, then replace the
        # shell process with ros2 launch by using exec.
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
        # poll() returns None while the process is still running.
        return (
            process is not None
            and process.poll() is None
        )

    # =====================================================
    # AUTOMATIC STARTUP
    # =====================================================

    def automatic_start_callback(self) -> None:
        # Ignore any unexpected second timer callback.
        if self.startup_started:
            return

        self.startup_started = True
        self.startup_timer.cancel()

        # Process startup includes a blocking sleep, so it runs in a separate
        # thread instead of blocking the ROS 2 timer callback.
        thread = threading.Thread(
            target=self.automatic_start_worker,
            daemon=True,
        )
        thread.start()

    def automatic_start_worker(self) -> None:
        self.get_logger().info(
            "Automatically starting robot and manipulator"
        )

        # Only one thread may start, stop or restart the process at a time.
        with self.process_lock:
            success, message = self.start_system_locked()

        if success:
            self.get_logger().info(message)
        else:
            self.get_logger().error(message)

    # =====================================================
    # START MANAGED LAUNCH PROCESS
    # =====================================================

    def start_system_locked(self) -> tuple[bool, str]:
        # The caller must hold process_lock before entering this function.
        if self.process_is_running(self.system_process):
            return True, (
                "Robot and manipulator launch is already running"
            )

        try:
            self.get_logger().info(
                "Starting robot_and_manipulator.launch.py"
            )

            # start_new_session creates a separate process group. This allows
            # the supervisor to later send one signal to the full launch tree.
            self.system_process = subprocess.Popen(
                self.build_launch_command(),
                start_new_session=True,
            )

            # Give ros2 launch time to start and check that it did not exit.
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

    # =====================================================
    # STOP MANAGED LAUNCH PROCESS
    # =====================================================

    def stop_system_locked(self) -> None:
        # The caller must hold process_lock before entering this function.
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
            # Obtain the process group created by start_new_session=True.
            process_group = os.getpgid(process.pid)

            # First attempt: graceful ROS shutdown, similar to pressing Ctrl+C.
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
                # Second attempt: request normal process termination.
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
                    # Final attempt: force the full process group to stop.
                    os.killpg(
                        os.getpgid(process.pid),
                        signal.SIGKILL,
                    )

                    process.wait(timeout=3.0)

                except ProcessLookupError:
                    # The process already stopped between checking and signalling.
                    pass

        except ProcessLookupError:
            # The process already stopped before its process group was accessed.
            pass

        except Exception as exception:
            self.get_logger().error(
                f"Could not stop robot system: {exception}"
            )

        finally:
            # Forget the old process after every completed stop attempt.
            self.system_process = None

    # =====================================================
    # RESTART SERVICE
    # =====================================================

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

        # The response confirms that the restart thread was started.
        # It does not mean that the full restart has already completed.
        response.success = True
        response.message = (
            "Robot and manipulator restart started"
        )

        return response

    def restart_worker(self) -> None:
        try:
            # Keep the lock during the complete stop-wait-start sequence so
            # no other worker can change system_process halfway through.
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
            # A new restart request may be accepted after this worker finishes.
            self.restart_active = False

    # =====================================================
    # NODE SHUTDOWN
    # =====================================================

    def destroy_node(self):
        # Stop the managed launch process before destroying the supervisor.
        with self.process_lock:
            self.stop_system_locked()

        return super().destroy_node()


# =====================================================
# MAIN
# =====================================================

def main(args=None) -> None:
    rclpy.init(args=args)

    node: Optional[SystemSupervisor] = None
    executor: Optional[MultiThreadedExecutor] = None

    try:
        node = SystemSupervisor()

        # Multiple executor threads allow service and timer callbacks to run
        # while separate worker threads handle process startup and shutdown.
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
