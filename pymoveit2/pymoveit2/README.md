# pymoveit2 API — MoveIt2 module

Module overview
- Provides a Python interface to MoveIt 2 for planning and executing trajectories.
- Requires an rclpy Node and a JointTrajectory-based controller for execution.
- Exposes class MoveIt2, enum MoveIt2State, and utility functions.

## Enum: MoveIt2State
Represents the execution state of the MoveIt2 interface:
- `IDLE` — No motion requested or executing.
- `REQUESTING` — Motion requested but not accepted yet.
- `EXECUTING` — Motion accepted and executing.

## Class: MoveIt2
Python interface for MoveIt 2. Main responsibilities:
- Build and send planning requests (kinematic and Cartesian).
- Send execution goals to the controller (ExecuteTrajectory) or to MoveGroup action.
- Maintain cached planning scene and joint state.
- Manage goal / path constraints and collision objects.

Constructor
- Signature:
  __init__(node, joint_names, base_link_name, end_effector_name, group_name='arm',
           execute_via_moveit=False, ignore_new_calls_while_executing=False,
           callback_group=None, follow_joint_trajectory_action_name='DEPRECATED',
           use_move_group_action=False)
- Parameters:
  - node: rclpy.node.Node used for clients/publishers/subscriptions.
  - joint_names: list of joint names for the manipulator.
  - base_link_name: base frame id used for planning.
  - end_effector_name: end-effector link name for Cartesian goals.
  - group_name: MoveIt planning group (default "arm").
  - execute_via_moveit: use MoveGroup action for execution if True.
  - ignore_new_calls_while_executing: ignore incoming requests while executing if True.
  - callback_group: optional rclpy CallbackGroup.
  - follow_joint_trajectory_action_name: deprecated/action name placeholder.
  - use_move_group_action: use MoveGroup action for plan+execute if True.
- Preconditions: node valid, parameters correct types.
- Postconditions: clients/publishers/subscriptions prepared.

Properties
- planning_scene: cached moveit_msgs/PlanningScene
- joint_names, base_link_name, end_effector_name
- joint_state: current sensor_msgs/JointState (thread-safe access)
- new_joint_state_available: bool
- max_velocity, max_acceleration, num_planning_attempts, allowed_planning_time
- cartesian_avoid_collisions, cartesian jump/prismatic/revolute thresholds
- pipeline_id, planner_id

Key methods (short summary)
- query_state() -> MoveIt2State
  - Returns current interface state.
- cancel_execution()
  - Publish cancellation ("stop") for currently executing goal.
- get_execution_future() -> Optional[Future]
  - Return future for current execution goal.
- get_last_execution_error_code() -> Optional[MoveItErrorCodes]
  - Return last recorded MoveIt error code.

Planning & execution
- move_to_pose(pose=None, position=None, quat_xyzw=None, target_link=None, frame_id=None,
               tolerance_position=0.001, tolerance_orientation=0.001,
               weight_position=1.0, cartesian=False, weight_orientation=1.0,
               cartesian_max_step=0.0025, cartesian_fraction_threshold=0.0)
  - Plan & execute a pose goal (Cartesian or kinematic).
- move_to_configuration(joint_positions, joint_names=None, tolerance=0.001, weight=1.0)
  - Plan & execute a joint-space goal.
- plan(... ) -> Optional[JointTrajectory]
  - Synchronous planning; wraps plan_async and returns JointTrajectory on success.
- plan_async(... ) -> Optional[Future]
  - Asynchronous planning request; returns Future.
- get_trajectory(future, cartesian=False, cartesian_fraction_threshold=0.0) -> Optional[JointTrajectory]
  - Extract JointTrajectory from a completed planning Future.
- execute(joint_trajectory)
  - Send ExecuteTrajectory goal to controller (async).
- wait_until_executed() -> bool
  - Block until current motion completes; returns success flag.
- reset_controller(joint_state, sync=True)
  - Send dummy zero-duration trajectory to controller to reset state.

Goal & path constraints
- set_pose_goal(...), set_position_goal(...), set_orientation_goal(...)
- create_position_constraint(...), create_orientation_constraint(...)
- create_joint_constraints(...), set_joint_goal(...)
- clear_goal_constraints(), create_new_goal_constraint()
- set_path_joint_constraint(...), set_path_position_constraint(...), set_path_orientation_constraint(...)
- clear_path_constraints()

Kinematics
- compute_fk(joint_state=None, fk_link_names=None) -> Optional[PoseStamped|List[PoseStamped]]
- compute_fk_async(...)
- get_compute_fk_result(future, fk_link_names=None)
- compute_ik(position, quat_xyzw, start_joint_state=None, constraints=None, wait_for_server_timeout_sec=1.0)
- compute_ik_async(...)
- get_compute_ik_result(future)

Planning scene & collision objects
- add_collision_primitive(id, primitive_type, dimensions, pose=None, position=None, quat_xyzw=None, frame_id=None, operation=CollisionObject.ADD)
- add_collision_box / _sphere / _cylinder / _cone / add_collision_mesh
- remove_collision_object(id), remove_collision_mesh(id)
- attach_collision_object(id, link_name=None, touch_links=[], weight=0.0)
- detach_collision_object(id), detach_all_collision_objects()
- move_collision(id, position, quat_xyzw, frame_id=None)
- update_planning_scene() -> bool
- allow_collisions(id, allow) -> Optional[Future]
- process_allow_collision_future(future) -> bool
- clear_all_collision_objects() -> Optional[Future]
- cancel_clear_all_collision_objects_future(future)
- process_clear_all_collision_objects_future(future) -> bool

Internal helpers & callbacks
- __joint_state_callback(msg)
- _plan_kinematic_path(), _plan_cartesian_path(max_step=..., frame_id=None)
- _send_goal_async_move_action(), __response_callback_move_action(response), __result_callback_move_action(res)
- _send_goal_async_execute_trajectory(goal, wait_until_response=False), __response_callback_execute_trajectory(response), __result_callback_execute_trajectory(res)
- __init_move_action_goal(frame_id, group_name, end_effector)
- __init_compute_fk(), __init_compute_ik()

## Top-level utility functions
- init_joint_state(joint_names, joint_positions=None, joint_velocities=None, joint_effort=None) -> JointState
  - Create JointState with provided values or zeros.
- init_execute_trajectory_goal(joint_trajectory) -> Optional[ExecuteTrajectory.Goal]
  - Wrap JointTrajectory into ExecuteTrajectory.Goal.
- init_dummy_joint_trajectory_from_state(joint_state, duration_sec=0, duration_nanosec=0) -> JointTrajectory
  - Create single-point JointTrajectory from JointState.

---

