#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from threading import Thread

from tf2_ros import Buffer, TransformListener, TransformException

from my_moveit_python import srdfGroupStates, MovegroupHelper
import tf_transformations

from project_interfaces.srv import Manipulator
from std_msgs.msg import String
import threading


class manipulatorController(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        # Robot parameters
        prefix = ""
        # arrange the joint_names alphabetically to match URDF
        self.joint_names = [
            prefix + "joint1",
            prefix + "joint2",
            prefix + "joint3",
            prefix + "joint4",
            prefix + "joint5",
            prefix + "joint6",
        ]
        self.base_link_name = "link_base"
        self.end_effector_name = "link6"
        self.group_name = "lite6"
        self.package_name = "my_uf_moveit_config"
        self.srdf_file_name = "config/uf_robot.srdf"

        # TF setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # MoveIt helpers
        self.group_states = srdfGroupStates(
            self.package_name, self.srdf_file_name, self.group_name
        )
        self.move_group = MovegroupHelper(
            self, self.joint_names, self.base_link_name, self.end_effector_name, self.group_name
        )

        # --- Create subscribers, publishers, clients, timers here ---
        #start service
        self.manipulator_start = self.create_service(Manipulator, "manipulator/start", self.start_ontvangen)

        #status publisher
        self.status_pub = self.create_publisher(String, "manipulator/status", 10)

        # interne state
        self.running = False
        self.lock = threading.Lock()

        self.get_logger().info("Lite6 manipulator node has been initialized.")

    # --- Create callback functions here ---

    # --- Motion primitives ------------------------------------------------
    def move_to_state(self, state_name: str):
        result, joint_values = self.group_states.get_joint_values(state_name)
        if not result:
            self.get_logger().error(f"Failed to get joint values for state '{state_name}'.")
            return
        self.get_logger().info(f"Moving to state '{state_name}'.")
        self.move_group.move_to_configuration(joint_values)

    def move_to_pose(self, translation, rotation):
        self.get_logger().info(f"Moving to pose: {translation}, {rotation}")
        self.move_group.move_to_pose(translation, rotation)

    def move_to_tf(self, from_frame: str, to_frame: str):
        try:
            t = self.tf_buffer.lookup_transform(
                to_frame, from_frame, rclpy.time.Time()
            )
            translation = [
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z,
            ]
            rotation = [
                t.transform.rotation.w,
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
            ]
            self.get_logger().info(f"Moving to transform: {from_frame} → {to_frame}")
            self.move_to_pose(translation, rotation)
        except TransformException as ex:
            self.get_logger().warn(f"Could not transform {to_frame} to {from_frame}: {ex}")

    # --- App sequence ----------------------------------------------------

    def start_ontvangen(self, request, response):
        self.get_logger().info("Service ontvangen")

        with self.lock:
            if self.running:
                response.success = False
                return response

            self.running = True

        # status update
        self.publish_status("START ontvangen → job wordt gestart")

        # run in aparte thread zodat service niet blokkeert
        thread = threading.Thread(target=self._run_process, daemon=True)
        thread.start()

        response.success = True
        return response
    
    def _run_process(self):

        try:
            self.publish_status("Initialisatie...")

            self.create_rate(1.0).sleep()

            self.publish_status("Robot beweging gestart")

            self.execute_app()   # jouw bestaande logica

            self.publish_status("Klaar")

        except Exception as e:
            self.publish_status(f"Fout: {str(e)}")

        finally:
            with self.lock:
                self.running = False

    def publish_status(self, msg: str):
        status = String()
        status.data = msg
        self.status_pub.publish(status)
        self.get_logger().info(msg)

    def execute_app(self):
        #self.move_to_state("home")

        #self.move_to_state("right")
        True


        

# --------------------------------------------------------------------------
# Do not modify the main function unless necessary.
# --------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)

    # Instantiate the manipulatorController node.
    # NOTE: This must be done before creating the executor to ensure callbacks are registered correctly.
    node = manipulatorController("manipulatie")

    # Create a multithreaded executor with 2 threads.
    # Allows the node to handle multiple callbacks concurrently (e.g., subscriptions, timers).
    executor = MultiThreadedExecutor(num_threads=2)

    # Add the node to the executor so it can process its callbacks.
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
