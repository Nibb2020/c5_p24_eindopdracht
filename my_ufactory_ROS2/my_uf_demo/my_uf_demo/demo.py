#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from threading import Thread

from tf2_ros import Buffer, TransformListener, TransformException

from my_moveit_python import srdfGroupStates, MovegroupHelper
import tf_transformations


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

        self.get_logger().info("Lite6 demo node has been initialized.")

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

    def execute_app(self):

        # Move through predefined joint states
        for state in ["left", "right", "home"]:
            self.move_to_state(state)

        translation = [0.5, 0.0, 0.25] # Relative to base_link    
        # RPY angles in radians
        roll = 3.1415927 # De gripper is facing downwards
        pitch = 0.0
        yaw = 0.0
        # Convert RPY to quaternion
        rotation = tf_transformations.quaternion_from_euler(roll, pitch, yaw)

        # Move to a specific pose
        self.move_to_pose(translation, rotation)

        # Move back to home position
        self.move_to_state("home")

        #-move to top position
        self.move_to_state('up')

        #-move back to home position
        self.move_to_state('home')

        # Move to resting position
        self.move_to_state("resting")

        

# --------------------------------------------------------------------------
# Do not modify the main function unless necessary.
# --------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)

    # Instantiate the manipulatorController node.
    # NOTE: This must be done before creating the executor to ensure callbacks are registered correctly.
    node = manipulatorController("demo")

    # Create a multithreaded executor with 2 threads.
    # Allows the node to handle multiple callbacks concurrently (e.g., subscriptions, timers).
    executor = MultiThreadedExecutor(num_threads=2)

    # Add the node to the executor so it can process its callbacks.
    executor.add_node(node)

    # Start the executor in a separate background thread.
    # Keeps the ROS event loop running while allowing the main thread to execute custom logic.
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    # Create a 1 Hz rate object and sleep once to allow initialization.
    # Provides time for system setup (e.g., MoveIt, TF) before running main logic.
    node.create_rate(1.0).sleep()

    # Execute the main application logic defined in the node.
    # Typically runs robot motion, computations, or control behaviors.
    node.execute_app()

    # Shutdown ROS gracefully after main logic completes.
    rclpy.shutdown()

    # Wait for the executor thread to exit cleanly before terminating the program.
    executor_thread.join()


if __name__ == "__main__":
    main()
