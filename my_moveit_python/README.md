# MyMoveIt Python Module

A comprehensive Python module for robot manipulation using MoveIt2 in ROS2. This module provides helper classes to simplify motion planning, inverse/forward kinematics computation, and servo control for robotic manipulators.

## Overview

This module contains two main classes that wrap and extend the PyMoveIt2 library functionality:

1. **`srdfGroupStates`** - SRDF parser for retrieving predefined joint configurations
2. **`MovegroupHelper`** - High-level interface for robot motion control

## Features

- ✅ Motion planning to joint configurations and end-effector poses
- ✅ Forward and inverse kinematics computation (synchronous and asynchronous)
- ✅ SRDF group state parsing and retrieval
- ✅ Cartesian path planning with collision avoidance
- ✅ Velocity and acceleration scaling
- ✅ Servo control for continuous motion (framework in place)
- ✅ ROS2 logging integration

## Dependencies

```
rclpy              - ROS2 Python client library
pymoveit2          - Python interface for MoveIt2
ament_index_python - Package discovery utilities
```

## Classes

### `srdfGroupStates`

A ROS2 Node that parses SRDF (Semantic Robot Description Format) files to retrieve predefined joint configurations for robot groups.

#### Constructor

```python
srdfGroupStates(ros_package: str, srd_file_name: str, group_name: str)
```

**Parameters:**
- `ros_package` (str): The ROS package name containing the SRDF file
- `srd_file_name` (str): Path to the SRDF file (relative to package share directory)
- `group_name` (str): The robot group name (used for logging)

#### Methods

##### `get_joint_values(name: str) -> Tuple[bool, List[float]]`

Retrieves joint values for a predefined group state from the SRDF file.

**Parameters:**
- `name` (str): The name of the group state to retrieve (e.g., "home", "ready")

**Returns:**
- `Tuple[bool, List[float]]`: A tuple containing:
  - `bool`: Success flag (True if group state found, False otherwise)
  - `List[float]`: List of joint values in radians

**Example:**
```python
srdf_parser = srdfGroupStates("my_robot_moveit_config", "config/robot.srdf", "manipulator")
success, joint_values = srdf_parser.get_joint_values("home")
if success:
    print(f"Home configuration: {joint_values}")
```

---

### `MovegroupHelper`

A high-level interface for robot motion planning and control using MoveIt2.

#### Constructor

```python
MovegroupHelper(node: Node, joint_names: List[str], base_link_name: str, 
                end_effector_name: str, group_name: str)
```

**Parameters:**
- `node` (Node): The ROS2 node instance
- `joint_names` (List[str]): List of joint names in the robot group
- `base_link_name` (str): Name of the robot's base link (reference frame)
- `end_effector_name` (str): Name of the end-effector link
- `group_name` (str): Name of the move group in MoveIt2 configuration

**Attributes:**
- `synchronous` (bool): If True, wait for motion to complete; if False, execute asynchronously. Default: `True`
- `cancel_after_secs` (float): Cancel execution after this many seconds (0.0 = no cancellation). Default: `0.0`
- `cartesian` (bool): Use Cartesian path planning for pose goals. Default: `True`
- `cartesian_max_step` (float): Maximum step size for Cartesian planning (meters). Default: `0.0025`
- `cartesian_fraction_threshold` (float): Fraction of path that must be achievable. Default: `0.0`
- `cartesian_jump_threshold` (float): Maximum allowed joint jump. Default: `0.0`
- `cartesian_avoid_collisions` (bool): Check for collisions during Cartesian planning. Default: `False`

#### Methods

##### `move_to_configuration(joint_values: List[float]) -> None`

Move the robot to a target joint configuration.

**Parameters:**
- `joint_values` (List[float]): Target joint values in radians

**Behavior:**
- In synchronous mode: Blocks until motion is complete
- In asynchronous mode: Returns immediately; motion executes in background
- Can be cancelled after specified delay via `cancel_after_secs`

**Example:**
```python
helper = MovegroupHelper(node, joint_names, "base_link", "ee_link", "manipulator")
helper.move_to_configuration([0.0, 0.5, -1.57, 0.0, 1.57, 0.0])
```

---

##### `move_to_pose(position: List[float], quat_xyzw: List[float], cartesian: bool = True, cartesian_max_step: float = 0.0025, cartesian_fraction_threshold: float = 0.0, cartesian_jump_threshold: float = 0.0, cartesian_avoid_collisions: bool = False) -> None`

Move the robot's end-effector to a target pose in Cartesian space.

**Parameters:**
- `position` (List[float]): Target position as [x, y, z] in meters
- `quat_xyzw` (List[float]): Target orientation as quaternion [x, y, z, w]
- `cartesian` (bool): Use Cartesian path planning. Default: `True`
- `cartesian_max_step` (float): Maximum step size for path interpolation
- `cartesian_fraction_threshold` (float): Minimum fraction of path that must succeed
- `cartesian_jump_threshold` (float): Maximum allowed joint jump between steps
- `cartesian_avoid_collisions` (bool): Enable collision checking during planning

**Behavior:**
- Same synchronous/asynchronous behavior as `move_to_configuration()`
- Attempts to move end-effector in a straight line (Cartesian mode)
- Falls back to joint-space planning if Cartesian path fails

**Example:**
```python
position = [0.5, 0.2, 0.3]  # x, y, z in meters
orientation = [0.0, 0.707, 0.0, 0.707]  # quaternion (x, y, z, w)
helper.move_to_pose(position, orientation, cartesian=True)
```

---

##### `compute_fk(joint_values: List[float]) -> Tuple[bool, Optional[str]]`

Compute forward kinematics for given joint values.

**Parameters:**
- `joint_values` (List[float]): Joint values in radians

**Returns:**
- `Tuple[bool, Optional[str]]`: A tuple containing:
  - `bool`: Success flag (True if computation succeeded)
  - `Optional[str]`: String representation of the end-effector pose (None if failed)

**Behavior:**
- In synchronous mode: Blocks until computation is complete
- In asynchronous mode: Uses async computation with polling

**Example:**
```python
success, pose = helper.compute_fk([0.0, 0.5, -1.57, 0.0, 1.57, 0.0])
if success:
    print(f"End-effector pose: {pose}")
else:
    print("FK computation failed")
```

---

##### `compute_ik(position: List[float], quat_xyzw: List[float]) -> Tuple[bool, Optional[str]]`

Compute inverse kinematics for a target end-effector pose.

**Parameters:**
- `position` (List[float]): Target position [x, y, z] in meters
- `quat_xyzw` (List[float]): Target orientation as quaternion [x, y, z, w]

**Returns:**
- `Tuple[bool, Optional[str]]`: A tuple containing:
  - `bool`: Success flag (True if solution found)
  - `Optional[str]`: String representation of joint solution (None if failed)

**Behavior:**
- Finds joint configuration to achieve target end-effector pose
- Returns first valid solution found
- May fail if pose is unreachable or in singularities

**Example:**
```python
position = [0.5, 0.2, 0.3]
orientation = [0.0, 0.707, 0.0, 0.707]
success, joints = helper.compute_ik(position, orientation)
if success:
    print(f"Joint solution: {joints}")
else:
    print("IK solution not found")
```

---

##### `move_servo() -> None`

Placeholder for servo-based continuous motion control.

**Status:** Not yet implemented. Framework is in place for future servo functionality.

---

## Configuration

### Motion Planner

The helper uses the RRT-Connect motion planner:
```python
self.moveit2.planner_id = "RRTConnectkConfigDefault"
```

To use a different planner, modify this attribute:
```python
helper.moveit2.planner_id = "PRMkConfigDefault"  # PRM planner
```

### Velocity and Acceleration

Scale velocity and acceleration as a percentage of maximum (0.0 to 1.0):
```python
helper.moveit2.max_velocity = 0.5        # 50% of maximum
helper.moveit2.max_acceleration = 0.5    # 50% of maximum
```

### Motion Execution Mode

```python
helper.synchronous = True   # Wait for motion completion
helper.synchronous = False  # Execute asynchronously
```

## Usage Examples

### Basic Motion Planning

```python
import rclpy
from my_moveit_python import MovegroupHelper

rclpy.init()
node = rclpy.create_node('my_robot_controller')

# Initialize helper
helper = MovegroupHelper(
    node=node,
    joint_names=["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
    base_link_name="base_link",
    end_effector_name="tool0",
    group_name="manipulator"
)

# Move to predefined configuration
helper.move_to_configuration([0.0, -1.57, 1.57, -1.57, -1.57, 0.0])

# Move to target pose
helper.move_to_pose(
    position=[0.5, 0.3, 0.4],
    quat_xyzw=[0.0, 0.707, 0.0, 0.707]
)

rclpy.shutdown()
```

### Using SRDF Group States

```python
from my_moveit_python import srdfGroupStates, MovegroupHelper

# Parse SRDF for predefined configurations
srdf_parser = srdfGroupStates(
    "my_robot_moveit_config",
    "config/my_robot.srdf",
    "manipulator"
)

# Get home configuration
success, home_joints = srdf_parser.get_joint_values("home")

if success:
    helper.move_to_configuration(home_joints)
```

### Computing Kinematics

```python
# Forward kinematics
success_fk, pose = helper.compute_fk([0.0, 0.5, -1.57, 0.0, 1.57, 0.0])

# Inverse kinematics
position = [0.6, 0.2, 0.35]
orientation = [0.0, 1.0, 0.0, 0.0]
success_ik, joints = helper.compute_ik(position, orientation)
```

## Logging

The module uses ROS2 logging for informational and debugging messages:

```
[INFO] Reading XML file: /path/to/robot.srdf
[INFO] Moving to {joint_positions: [0.0, 0.5, -1.57, ...]}
[INFO] Succeeded. Result: ...
```

All messages are prefixed with the node name for easy filtering in ROS2 logs.

## Known Limitations

- **Servo control** (`move_servo()`) is not yet implemented
- **Asynchronous mode** has limited documentation and may require debugging
- **Cartesian planning** success depends on workspace and collision environment
- **Planner performance** varies significantly based on robot configuration and scene complexity

## Future Enhancements

- [ ] Implement `move_servo()` for continuous servo control
- [ ] Add trajectory recording and playback
- [ ] Implement gripper control integration
- [ ] Add constraint-based planning (orientation constraints, etc.)
- [ ] Implement force/torque feedback control
- [ ] Add multi-goal sequencing with transitions

## Troubleshooting

### FK/IK Computation Fails

- **Cause:** Robot not properly initialized or MoveIt2 service not running
- **Solution:** Ensure `move_group` node is running and robot configuration is loaded

### Motion Planning Fails

- **Cause:** Target configuration is unreachable or in collision
- **Solution:** Check target pose validity and collision environment; adjust `cartesian_max_step` or disable `cartesian_avoid_collisions`

### Synchronous Motion Never Returns

- **Cause:** MoveIt2 planning is taking too long or failed silently
- **Solution:** Check MoveIt2 logs with `ros2 run pymoveit2 ...`; increase planner time limit

## Related Resources

- [MoveIt2 Documentation](https://moveit.picknik.ai/)
- [PyMoveIt2 GitHub](https://github.com/jspricke/pymoveit2)
- [ROS2 Documentation](https://docs.ros.org/en/jazzy/)
- [SRDF Format](http://wiki.ros.org/srdf)

## Author & License

Part of the ROS2 Industrial training workspace. Refer to the main project LICENSE for terms.
