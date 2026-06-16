#!/usr/bin/env python3

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2, MoveIt2Servo
from ament_index_python.packages import get_package_share_directory
import xml.etree.ElementTree as ET
from math import cos, sin


class srdfGroupStates(Node):
    def __init__(self, ros_package, srd_file_name, group_name):
        super().__init__('srdf_group_states')
        self.groupname = group_name

        package_path = get_package_share_directory(ros_package)
        xml_file_path = f"{package_path}/{srd_file_name}"

        self.get_logger().info(f"Reading XML file: {xml_file_path}")
        
        self.tree = ET.parse(xml_file_path)
        self.root = self.tree.getroot()

    def get_joint_values(self, name):
        right_group_state = None
        for group_state in self.root.findall("group_state"):
            if group_state.get("name") == name:
                right_group_state = group_state
                break

        # Extract joint values if the 'right' group state was found
        if right_group_state:
            joint_values= []
            joint_tmp = {}
            for joint in right_group_state.findall("joint"):
                joint_name = joint.get("name")
                joint_value = float(joint.get("value"))  # Convert the value to float
                joint_values.append(joint_value);
                joint_tmp[joint_name] = joint_value

            # Print the joint values
            #self.get_logger().info("Joint values for 'right':", joint_tmp)
            return True, joint_values
        else:
            self.get_logger().info("Group state 'right' not found in the XML.")
            return False, []

class MovegroupHelper(Node):
    def __init__(self, node, joint_names, base_link_name, end_effector_name, group_name):
        super().__init__('move_group_helper')

        # Create callback group that allows execution of callbacks in parallel without restrictions
        self.callback_group = ReentrantCallbackGroup()

        # Create MoveIt 2 interface
        self.moveit2 = MoveIt2(
            node=node,
            joint_names=joint_names,
            base_link_name=base_link_name,
            end_effector_name=end_effector_name,
            group_name=group_name,
            callback_group=self.callback_group,
        )

        self.moveit2_servo = MoveIt2Servo(
            node=node,
            frame_id=base_link_name,
            callback_group=self.callback_group,
        )


        self.moveit2.planner_id = (
            "RRTConnectkConfigDefault"
        )

        # Scale down velocity and acceleration of joints (percentage of maximum)
        self.moveit2.max_velocity = 0.5
        self.moveit2.max_acceleration = 0.5
        self.synchronous = True
        self.cancel_after_secs = 0.0

        self.cartesian = True
        self.cartesian_max_step = 0.0025
        self.cartesian_fraction_threshold = 0.0

    def wait_for_moveit_services(self, timeout_sec=10.0):
        """Wait for MoveIt2 services to become available."""
        import time
        start_time = time.time()
        
        # Access the private service clients from MoveIt2
        plan_service = self.moveit2._plan_kinematic_path_service
        cartesian_service = self.moveit2._plan_cartesian_path_service
        
        self.get_logger().info("Waiting for MoveIt2 services to become available...")
        
        # Wait for plan_kinematic_path service
        while not plan_service.service_is_ready():
            if time.time() - start_time > timeout_sec:
                self.get_logger().error(f"Timeout waiting for service '{plan_service.srv_name}'")
                return False
            time.sleep(0.1)
        
        self.get_logger().info(f"Service '{plan_service.srv_name}' is ready")
        
        # Wait for compute_cartesian_path service
        while not cartesian_service.service_is_ready():
            if time.time() - start_time > timeout_sec:
                self.get_logger().error(f"Timeout waiting for service '{cartesian_service.srv_name}'")
                return False
            time.sleep(0.1)
        
        self.get_logger().info(f"Service '{cartesian_service.srv_name}' is ready")
        self.get_logger().info("All MoveIt2 services are ready!")
        return True

    def move_to_configuration(self, joint_values):
        self.get_logger().info(f"Moving to {{joint_positions: {list(joint_values)}}}")
        self.moveit2.move_to_configuration(joint_values)
        if self.synchronous:
            # Note: the same functionality can be achieved by setting
            # `synchronous:=false` and `cancel_after_secs` to a negative value.
            self.moveit2.wait_until_executed()
        else:
            # Wait for the request to get accepted (i.e., for execution to start)
            self.get_logger().info("Current State: " + str(self.moveit2.query_state()))
            rate = self.node.create_rate(10)
            while self.moveit2.query_state() != self.MoveIt2State.EXECUTING:
                rate.sleep()

            # Get the future
            self.get_logger().info("Current State: " + str(self.moveit2.query_state()))
            future = self.moveit2.get_execution_future()

            # Cancel the goal
            if self.cancel_after_secs > 0.0:
                # Sleep for the specified time
                sleep_time = self.node.create_rate(self.cancel_after_secs)
                sleep_time.sleep()
                # Cancel the goal
                self.get_logger().info("Cancelling goal")
                self.moveit2.cancel_execution()

            # Wait until the future is done
            while not future.done():
                rate.sleep()

            # Print the result
            self.get_logger().info("Result status: " + str(future.result().status))
            self.get_logger().info("Result error code: " + str(future.result().result.error_code))

    def move_to_pose(self, position, quat_xyzw, cartesian=True, cartesian_max_step=0.0025, cartesian_fraction_threshold=0.0):
        self.get_logger().info(f"Moving to {{position: {list(position)}, quat_xyzw: {list(quat_xyzw)}}}")
        self.moveit2.move_to_pose(
            position=position,
            quat_xyzw=quat_xyzw,
            cartesian=cartesian,
            cartesian_max_step=cartesian_max_step,
            cartesian_fraction_threshold=cartesian_fraction_threshold,
        )
        if self.synchronous:
            # Note: the same functionality can be achieved by setting
            # `synchronous:=false` and `cancel_after_secs` to a negative value.
            self.moveit2.wait_until_executed()
        else:
            # Wait for the request to get accepted (i.e., for execution to start)
            self.get_logger().info("Current State: " + str(self.moveit2.query_state()))
            rate = self.node.create_rate(10)
            while self.moveit2.query_state() != self.MoveIt2State.EXECUTING:
                rate.sleep()

            # Get the future
            self.get_logger().info("Current State: " + str(self.moveit2.query_state()))
            future = self.moveit2.get_execution_future()

            # Cancel the goal
            if self.cancel_after_secs > 0.0:
                # Sleep for the specified time
                sleep_time = self.node.create_rate(self.cancel_after_secs)
                sleep_time.sleep()
                # Cancel the goal
                self.get_logger().info("Cancelling goal")
                self.moveit2.cancel_execution()

            # Wait until the future is done
            while not future.done():
                rate.sleep()

            # Print the result
            self.get_logger().info("Result status: " + str(future.result().status))
            self.get_logger().info("Result error code: " + str(future.result().result.error_code))
    
    def compute_fk(self, joint_values):
        self.get_logger().info(f"Computing FK for {{joint_positions: {list(joint_values)}}}")
        self.get_logger().info("compute_fk")
        if self.synchronous:
            retval = self.moveit2.compute_fk(joint_values)
        else:
            future = self.moveit2.compute_fk_async(joint_values)
            if future is not None:
                rate = self.node.create_rate(10)
                while not future.done():
                    rate.sleep()
                retval = self.moveit2.get_compute_fk_result(future)
        if retval is None:
            self.get_logger().info("Failed.")
            return False, None
        else:
            self.get_logger().info("Succeeded. Result: " + str(retval))
            return True, retval
        
    def compute_ik(self, position, quat_xyzw):
        self.get_logger().info(f"Computing IK for {{position: {list(position)}, quat_xyzw: {list(quat_xyzw)}}}")

        if self.synchronous:
            retval = self.moveit2.compute_ik(position, quat_xyzw)
        else:
            future = self.moveit2.compute_ik_async(position, quat_xyzw)
            if future is not None:
                rate = self.node.create_rate(10)
                while not future.done():
                    rate.sleep()
                retval = self.moveit2.get_compute_ik_result(future)
        if retval is None:
            self.get_logger().info("Failed.")
            return False, None        
        else:
            self.get_logger().info("Succeeded. Result: " + str(retval))
            return True, retval


    def _servo_circular_motion(self):
        """Move in a circular motion using Servo"""

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        self.moveit2_servo(linear=(sin(now_sec), cos(now_sec), 0.0), angular=(0.0, 0.0, 0.0))

    # Create timer for moving in a circular motion


    def move_servo(self):
        self.get_logger().info("Not implemented yet")
        #self.create_timer(0.2, self._servo_circular_motion)


