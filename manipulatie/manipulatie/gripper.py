#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from threading import Thread

from std_msgs.msg import String
import threading
from xarm_msgs.srv import VacuumGripperCtrl
import time

class VacuumGripper(Node):
    def __init__(self, node_naam):
        super().__init__(node_naam)

        self.gripper = self.create_client(VacuumGripperCtrl, "/xarm/set_vacuum_gripper" )
        self.request = VacuumGripperCtrl.Request()

    def close (self):
        self.get_logger().info('closing gripper')

        self.request.on = False

        self.future = self.gripper.call_async(self.request)
        time.sleep(1.5)
        return self.future

    def open (self):
        self.get_logger().info('opening gripper')
        
        self.request.on = True

        self.future = self.gripper.call_async(self.request)
        time.sleep(1.5)
        return self.future

    def execute_app(self):
        self.open()

        self.close()

        
    
def main(args=None):
    rclpy.init(args=args)

    # Instantiate the manipulatorController node.
    # NOTE: This must be done before creating the executor to ensure callbacks are registered correctly.
    node = VacuumGripper("Vaccum_gripper")

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
