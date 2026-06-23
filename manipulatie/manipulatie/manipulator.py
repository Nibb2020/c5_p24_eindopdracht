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
from xarm_msgs.srv import VacuumGripperCtrl
import time
from std_srvs.srv import Trigger


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

        #move home service
        self.manipulator_move_home = self.create_service(Trigger,"manipulator/move_home",self.move_home_callback)

        # interne state
        self.running = False
        self.home_requested = False
        self.lock = threading.Lock()

        #gripper opstarten
        self.vacuum_gripper = VacuumGripper()
        self.vacuum_gripper.open()

        #self.move_to_state("up")

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

    def move_to_pose(self, translation, yaw):
        roll = 3.14159      #180 graden
        pitch = 0.1396    #8 graden
        rotation = tf_transformations.quaternion_from_euler(roll, pitch, yaw)

        self.get_logger().info(f"Moving to pose: {translation}, {rotation}")
        self.move_group.move_to_pose(translation, rotation)

    def move_to_pose_offset(self, yaw, z_offset=0.1):
        translation_z_offset = [self.translation[0], self.translation[1], self.translation[2] + z_offset]
        self.move_to_pose(translation_z_offset, yaw)

    # --- App sequence ----------------------------------------------------

    def start_ontvangen(self, request, response):
        self.get_logger().info(f"Service ontvangen, voorwerp is: {request.klasse}")
        self.klasse = request.klasse
        self.translation = request.translation
        self.translation[2] = 0.1
        self.yaw_rotation = request.rotation

        with self.lock:
            if self.running:
                response.succes = False
                return response

            self.running = True

        # status update
        self.publish_status("START ontvangen → job wordt gestart")

        # run in aparte thread zodat service niet blokkeert
        thread = threading.Thread(target=self._run_process, daemon=True)
        thread.start()

        response.succes = True
        return response

    def _run_process(self):
        try:
            self.publish_status("Initialisatie...")

            self.create_rate(1.0).sleep()

            self.publish_status("Robot beweging gestart")

            self.execute_app()

            if self.home_is_requested():
                self.publish_status("Sequence onderbroken, robot gaat naar up positie")

                self.move_to_home()

                self.publish_status("Home_klaar")
            else:
                self.publish_status("Klaar")

        except Exception as exception:
            self.publish_status(
                f"Fout: {str(exception)}"
            )

        finally:
            with self.lock:
                self.running = False
                self.home_requested = False

    #status publisher
    def publish_status(self, msg: str):
        status = String()
        status.data = msg
        self.status_pub.publish(status)
        self.get_logger().info(msg)

    #echte logica
    def execute_app(self):
        self.move_to_state("home")
        if self.home_is_requested():
            return

        self.move_to_pose_offset(   self.yaw_rotation)
        if self.home_is_requested():
            return

        self.move_to_pose(self.translation, self.yaw_rotation)
        if self.home_is_requested():
            return

        self.vacuum_gripper.close()

        if self.home_is_requested():
            return
        
        self.move_to_pose_offset(   self.yaw_rotation)
        if self.home_is_requested():
            return
        
        if self.home_is_requested():
            return

        if self.klasse == "hooi":
            self.move_to_state("drop1")
        elif self.klasse == "kanon":
            self.move_to_state("drop2")
        elif self.klasse == "rood":
            self.move_to_state("drop3")
        elif self.klasse == "blauw":
            self.move_to_state("drop4")
        else:
            self.get_logger().warn("Sorry, er is geen object van een van de vier klasse gedetecteerd")
            self.move_to_state("home")

        if self.home_is_requested():
            return

        self.vacuum_gripper.open()

        if self.home_is_requested():
            return

        self.move_to_state("up")

    


#move home functies
    def move_home_callback(self, request, response):
        self.get_logger().info("Move home service ontvangen")

        with self.lock:
            if self.running:
                self.home_requested = True
                response.success = True
                response.message = "Move home requested. Current planned movement will finish first."
                return response

            self.running = True
            self.home_requested = False

        thread = threading.Thread(target=self._run_home_process, daemon=True)
        thread.start()

        response.success = True
        response.message = "Robot is moving to up position."
        return response
    
    def _run_home_process(self):
        try:
            self.publish_status("Robot gaat naar up positie")
            self.move_to_home()
            self.publish_status("Home_klaar")

        except Exception as e:
            self.publish_status(f"Fout tijdens move_home: {str(e)}")

        finally:
            with self.lock:
                self.running = False
                self.home_requested = False
    
    def home_is_requested(self):
        with self.lock:
            return self.home_requested


    def move_to_home(self):
        self.vacuum_gripper.open()

        self.move_to_state("up")


class VacuumGripper(Node):
    def __init__(self):
        super().__init__('vacuum_gripper')

        self.gripper = self.create_client(VacuumGripperCtrl, "/xarm/set_vacuum_gripper" )
        self.request = VacuumGripperCtrl.Request()

    def close (self):
        self.get_logger().info('closing gripper')

        self.request.on = False

        self.future = self.gripper.call_async(self.request)
        time.sleep(2.0)
        return self.future

    def open (self):
        self.get_logger().info('opening gripper')
        
        self.request.on = True

        self.future = self.gripper.call_async(self.request)
        time.sleep(2.0)
        return self.future


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
