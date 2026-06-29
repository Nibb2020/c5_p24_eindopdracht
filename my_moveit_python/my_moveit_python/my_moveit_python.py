#!/usr/bin/env python3

import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from action_msgs.msg import GoalStatus
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

        #added myself
        self.node = node

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

        #parameters zelf toegevoegd:
        self.node.declare_parameter("velocity_scaling", 0.1)
        self.node.declare_parameter("acceleration_scaling", 0.1)
        self.node.declare_parameter("max_planning_attempts", 10)
        self.node.declare_parameter("planning_retry_delay_sec", 0.2)

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

    def get_max_planning_attempts(self):
        max_attempts = self.node.get_parameter("max_planning_attempts").value
        max_attempts = int(max_attempts)
        max_attempts = max(1, min(max_attempts, 10))
        return max_attempts

    def get_planning_retry_delay_sec(self):
        retry_delay = self.node.get_parameter("planning_retry_delay_sec").value
        retry_delay = float(retry_delay)
        retry_delay = max(0.0, min(retry_delay, 2.0))
        return retry_delay

    def get_current_execution_future(self):
        try:
            return self.moveit2.get_execution_future()
        except Exception:
            return None

    def execution_result_is_successful(self, future):
        if future is None:
            return False

        if not future.done():
            return False

        try:
            result = future.result()
        except Exception as exception:
            self.get_logger().warn(f"Failed to read execution result: {exception}")
            return False

        status = getattr(result, "status", None)

        if status is not None:
            if int(status) == GoalStatus.STATUS_SUCCEEDED:
                return True

            self.get_logger().warn(f"Action 'execute_trajectory' was unsuccessful: {status}.")
            return False

        result_object = getattr(result, "result", None)

        if result_object is None:
            return False

        error_code = getattr(result_object, "error_code", None)

        if error_code is None:
            return False

        error_value = getattr(error_code, "val", error_code)

        try:
            error_value = int(error_value)
        except Exception:
            return False

        if error_value == 1:
            return True

        self.get_logger().warn(f"MoveIt execution failed with error code: {error_value}")
        return False

    def execute_motion_once(self, motion_function):
        previous_future = self.get_current_execution_future()

        try:
            motion_function()
        except Exception as exception:
            self.get_logger().warn(f"MoveIt motion request failed: {exception}")
            return False, True

        try:
            wait_result = self.moveit2.wait_until_executed()
        except Exception as exception:
            self.get_logger().warn(f"MoveIt wait failed: {exception}")
            return False, False

        if isinstance(wait_result, bool):
            return wait_result, not wait_result

        current_future = self.get_current_execution_future()

        if current_future is None:
            self.get_logger().warn("MoveIt did not create an execution future. Assuming planning failed.")
            return False, True

        if previous_future is not None and current_future is previous_future:
            self.get_logger().warn("MoveIt did not create a new execution future. Assuming planning failed.")
            return False, True

        success = self.execution_result_is_successful(current_future)

        if success:
            return True, False

        return False, False

    def execute_with_planning_retries(self, description, motion_function):
        max_attempts = self.get_max_planning_attempts()
        retry_delay = self.get_planning_retry_delay_sec()

        for attempt in range(1, max_attempts + 1):
            self.get_logger().info(
                f"{description}: planning attempt {attempt}/{max_attempts}"
            )

            success, retry_allowed = self.execute_motion_once(motion_function)

            if success:
                self.get_logger().info(f"{description}: succeeded on attempt {attempt}/{max_attempts}")
                return True

            if not retry_allowed:
                self.get_logger().warn(f"{description}: failed during execution; not retrying.")
                return False

            if attempt < max_attempts:
                self.get_logger().warn(f"{description}: planning failed, retrying...")
                time.sleep(retry_delay)

        self.get_logger().error(f"{description}: failed after {max_attempts} planning attempts.")
        return False

    def move_to_configuration(self, joint_values):
        self.update_planning_speed_from_parameters()

        joint_values = list(joint_values)

        self.get_logger().info(f"Moving to {{joint_positions: {joint_values}}}")

        def motion_function():
            self.moveit2.move_to_configuration(joint_values)

        return self.execute_with_planning_retries(
            description="move_to_configuration",
            motion_function=motion_function,
        )

    def move_to_pose(
        self,
        position,
        quat_xyzw,
        cartesian=True,
        cartesian_max_step=0.0025,
        cartesian_fraction_threshold=0.0,
    ):
        self.update_planning_speed_from_parameters()

        position = list(position)
        quat_xyzw = list(quat_xyzw)

        self.get_logger().info(
            f"Moving to {{position: {position}, quat_xyzw: {quat_xyzw}}}"
        )

        def motion_function():
            self.moveit2.move_to_pose(
                position=position,
                quat_xyzw=quat_xyzw,
                cartesian=cartesian,
                cartesian_max_step=cartesian_max_step,
                cartesian_fraction_threshold=cartesian_fraction_threshold,
            )

        return self.execute_with_planning_retries(
            description="move_to_pose",
            motion_function=motion_function,
        )
    
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


    #change from me
    def update_planning_speed_from_parameters(self):
        velocity_scaling = self.node.get_parameter("velocity_scaling").value
        acceleration_scaling = self.node.get_parameter("acceleration_scaling").value

        velocity_scaling = max(0.05, min(float(velocity_scaling), 1.0))
        acceleration_scaling = max(0.05, min(float(acceleration_scaling), 1.0))

        self.moveit2.max_velocity = velocity_scaling
        self.moveit2.max_acceleration = acceleration_scaling

        self.get_logger().info(
            f"Using velocity_scaling={velocity_scaling}, "
            f"acceleration_scaling={acceleration_scaling}"
        )