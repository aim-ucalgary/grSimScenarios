## grSim native functions

In [ssl-simulation-protocol's proto directory](https://github.com/RoboCup-SSL/ssl-simulation-protocol/tree/master/proto) and [grSim's proto directory](https://github.com/RoboCup-SSL/grSim/tree/master/proto), you can find the supported protocol definitions for the simulation environment, which are used to communicate with the simulator via UDP.

The following files in the `helpers/` directory contain utility functions for interacting with the grSim simulator. They will be used to simplify interaction with the grSim simulator, such as sending movement commands to robots and managing their velocities.

### helpers_communication.py

| Function | Purpose |
|---|---|
| `open_socket()` | Create and configure a UDP socket. |
| `send_commands()` | Send commands for multiple robots on one team. |
| `read_feedback()` | Read robot feedback and simulator errors. |
| `close_socket()` | Release the socket. |

### helpers_movement.py

| Function | Purpose |
|---|---|
| `send_global_velocity()` | Send one field-relative velocity command. |
| `send_local_velocity()` | Send one robot-relative velocity command. |
| `hold_velocity()` | Repeat a velocity command for a duration. |
| `ramp_velocity()` | Gradually change commanded velocity. |
| `stop_robot()` | Repeat zero-velocity commands for one robot. |
| `stop_robots()` | Stop a specified group of robots. |

### helpers_actuators.py

| Function | Purpose |
|---|---|
| `kick()` | Request a kick with a speed and elevation angle. |
| `set_dribbler()` | Enable or disable the dribbler. |
| `disable_actuators()` | Clear kick requests and disable the dribbler. |

### helpers_setup.py

| Function | Purpose |
|---|---|
| `teleport_robot()` | Set robot position, heading, and initial velocity. |
| `teleport_ball()` | Set ball position and initial velocity. |
| `set_robot_present()` | Enable or remove an existing robot slot. |
| `apply_initial_state()` | Establish a documented scene configuration. |
| `reset_scene()` | Restore the scenario’s initial state. |

### helpers_observations.py

| Function | Purpose |
|---|---|
| `read_vision()` | Receive and decode simulated vision packets. |
| `get_robot_pose()` | Retrieve a robot’s observed position and heading. |
| `get_ball_state()` | Retrieve the ball’s observed state. |
| `require_fresh_state()` | Reject missing or stale observations. |
| `estimate_velocity()` | Estimate velocity from timestamped positions. |

### helpers_geometry.py

| Function | Purpose |
|---|---|
| `distance_to()` | Calculate distance to a target. |
| `bearing_to()` | Calculate the heading toward a target. |
| `angle_error()` | Calculate the shortest signed angular difference. |
| `global_to_local()` | Convert a field-relative vector to robot coordinates. |
| `local_to_global()` | Convert a robot-relative vector to field coordinates. |

### helpers_behaviors.py

| Function | Purpose |
|---|---|
| `rotate_to()` | Use heading feedback to reach an orientation. |
| `face_point()` | Rotate toward a field position. |
| `hold_pose()` | Use feedback to maintain a position and heading. |
| `follow_waypoints()` | Visit a sequence of target positions. |
| `approach_ball()` | Move toward the ball from a chosen direction. |
| `pass_to()` | Coordinate aiming and kicking toward a target. |

### helpers_execution.py

| Function | Purpose |
|---|---|
| `run_steps()` | Execute and log named scenario steps. |
| `wait_until()` | Wait for a condition with a timeout. |
| `repeat_scenario()` | Repeat a scenario from its initial state. |
| `sweep_parameters()` | Run a scenario with different parameter values. |

### helpers_validation.py

| Function | Purpose |
|---|---|
| `assert_near_position()` | Check observed position against a tolerance. |
| `assert_heading()` | Check observed heading against a tolerance. |
| `assert_stopped()` | Verify measured speed remains below a threshold. |
| `assert_ball_displacement()` | Check how far the ball moved. |

### helpers_reporting.py

| Function | Purpose |
|---|---|
| `record_run()` | Record configuration, commands, observations, and outcomes. |
| `save_results()` | Export measurements and pass/fail results. |
| `plot_trajectory()` | Plot observed robot and ball paths. |