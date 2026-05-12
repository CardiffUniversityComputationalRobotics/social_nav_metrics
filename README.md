# Social Robot Navigation Metrics Recorder

ROS 2 package for recording social navigation metrics in simulation or on a real robot. The package writes one CSV row per navigation run and can optionally count collisions against people and an OctoMap.

The package provides two nodes:

- `metrics_recorder_node`: records metrics from odometry, pedestrian states, goal status, and collision count topics.
- `collision_counter_node`: counts collision events against social agents and an OctoMap, then publishes the current count.

## Metrics Recorder Node

`metrics_recorder_node` starts measuring when `goal_available_topic` publishes `True`. It saves the current run when `save_metrics_topic` publishes `True`, resets its internal state, and waits for the next goal.

If the node is interrupted with `Ctrl+C`, it also attempts to save the current metrics before shutting down.

### Parameters

| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `clock_topic` | string | `/clock` | Simulation clock topic used when `sim` is `True`. |
| `goal_available_topic` | string | `/goal_available` | A `True` message starts a new metrics recording session. |
| `save_metrics_topic` | string | `/save_metrics` | A `True` message saves the current session and resets the recorder. |
| `goal_reached_topic` | string | `/goal_reached` | A `True` message marks the goal as reached and records the final navigation time. |
| `odom_topic` | string | `/odom` | Robot odometry topic. |
| `num_nodes_topic` | string | `/num_nodes` | Optional planner node-count topic, useful for sampling-based planners. |
| `agent_states_topic` | string | `/pedsim_simulator/simulated_agents` | Social agent states topic. |
| `collision_counter_topic` | string | `/collision_counter` | Collision count topic, normally published by `collision_counter_node`. |
| `measure_rate` | double | `5.0` | Measurement frequency in Hz. For example, `10.0` records every `0.1` seconds. |
| `sim` | bool | `True` | If `True`, elapsed time is computed from `/clock`; otherwise wall time is used. |
| `csv_dir` | string | `""` | Base directory for CSV results. |
| `approach_name` | string | `""` | Subdirectory inside `csv_dir`, usually the tested planner/approach name. |
| `csv_name` | string | `""` | CSV file name. |

The CSV directory `${csv_dir}/${approach_name}` must exist before saving.

### Subscribed Topics

| Topic | Message Type | Purpose |
| --- | --- | --- |
| `/clock` | `rosgraph_msgs/msg/Clock` | Simulation time when `sim` is enabled. |
| `/goal_available` | `std_msgs/msg/Bool` | Starts recording when `data: true`. |
| `/goal_reached` | `std_msgs/msg/Bool` | Marks the run as successful when `data: true`. |
| `/save_metrics` | `std_msgs/msg/Bool` | Saves the current run when `data: true`. |
| `/odom` | `nav_msgs/msg/Odometry` | Robot pose and velocity. |
| `/pedsim_simulator/simulated_agents` | `pedsim_msgs/msg/AgentStates` | Agent poses, orientations, and velocities. |
| `/num_nodes` | `std_msgs/msg/Int32` | Optional number of sampled planner nodes. |
| `/collision_counter` | `std_msgs/msg/Int32` | Current collision count. |

### Saved CSV Fields

The recorder writes these columns:

```text
test_number,time,goal_reached,average_sii,average_rmi,total_time,collision_counter,num_nodes,path_irregularity,acc_per_segment,path_length
```

The averaged metrics use a running average over all valid samples in the run. The recorder does not keep an unbounded array of samples, and it does not compress old samples into an unweighted average.

## Collision Counter Node

`collision_counter_node` checks whether the robot is colliding with social agents or occupied OctoMap geometry. It increments the counter once per continuous collision event, publishes the current count, and waits until the robot leaves collision before counting another event.

The node requests the OctoMap once at startup. If you do not have an OctoMap service available, run only `metrics_recorder_node` or provide collision counts from another node.

### Parameters

| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `robot_height` | double | `1.0` | Height of the robot collision cylinder. |
| `robot_radius` | double | `0.25` | Radius of the robot collision cylinder. |
| `agent_radius` | double | `0.3` | Radius of each agent collision cylinder. |
| `odom_topic` | string | `/odom` | Robot odometry topic. |
| `agent_states_topic` | string | `/pedsim_simulator/simulated_agents` | Social agent states topic. |
| `octomap_service` | string | `/octomap_full` | Service used to retrieve the OctoMap. |
| `collision_counter_topic` | string | `/collision_counter` | Published collision count topic. |
| `goal_available_topic` | string | `/goal_available` | Resets the collision counter when a `True` message is received. |

### Interfaces

| Interface | Message/Service Type | Direction |
| --- | --- | --- |
| `/odom` | `nav_msgs/msg/Odometry` | Subscriber |
| `/pedsim_simulator/simulated_agents` | `pedsim_msgs/msg/AgentStates` | Subscriber |
| `/goal_available` | `std_msgs/msg/Bool` | Subscriber |
| `/collision_counter` | `std_msgs/msg/Int32` | Publisher |
| `/octomap_full` | `octomap_msgs/srv/GetOctomap` | Client |

## Build With Colcon

Source your ROS 2 installation first. For ROS 2 Humble:

```bash
source /opt/ros/humble/setup.bash
```

Then build the package in a colcon workspace:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/CardiffUniversityComputationalRobotics/social_nav_metrics.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select social_nav_metrics
source install/setup.bash
```

You need ROS 2-compatible versions of `pedsim_msgs`, `octomap_msgs`, `octomap_server`, FCL, and `tf_transformations` available through your workspace, underlay, or system packages. This package now uses the ROS 2 colcon workflow instead of the old rosinstall/catkin dependency flow.

If colcon warns that `social_nav_metrics` already exists in an underlay and you intentionally want to override it, rebuild with:

```bash
colcon build --symlink-install --packages-select social_nav_metrics --allow-overriding social_nav_metrics
```

## Run

Launch both nodes with the example configuration:

```bash
ros2 launch social_nav_metrics metrics_recorder.launch.py
```

Use a custom parameter file:

```bash
ros2 launch social_nav_metrics metrics_recorder.launch.py metrics_config_file:=/absolute/path/to/config.yaml
```

Run only the metrics recorder:

```bash
ros2 run social_nav_metrics metrics_recorder.py --ros-args --params-file /absolute/path/to/config.yaml
```

Run only the collision counter:

```bash
ros2 run social_nav_metrics collision_counter --ros-args --params-file /absolute/path/to/config.yaml
```

## Example Configuration

ROS 2 parameter files must be grouped by node name under `ros__parameters`:

```yaml
metrics_recorder_node:
  ros__parameters:
    clock_topic: "/clock"
    goal_reached_topic: "/smf_move_base_planner/goal_reached"
    goal_available_topic: "/goal_available"
    save_metrics_topic: "/save_metrics"
    collision_counter_topic: "/collision_counter"
    num_nodes_topic: "/smf_move_base_planner/smf_num_nodes"
    agent_states_topic: "/pedsim_simulator/simulated_agents"
    odom_topic: "/odom"

    measure_rate: 10.0
    sim: True

    csv_dir: "/tmp/social_nav_metrics/results"
    approach_name: "tests"
    csv_name: "new_test.csv"

collision_counter_node:
  ros__parameters:
    odom_topic: "/odom"
    agent_states_topic: "/pedsim_simulator/simulated_agents"
    collision_counter_topic: "/collision_counter"
    goal_available_topic: "/goal_available"
    octomap_service: "/octomap_full"

    robot_radius: 0.3
    robot_height: 1.0
    agent_radius: 0.45
```

Before saving, create the output folder:

```bash
mkdir -p /tmp/social_nav_metrics/results/tests
```

## Recording Workflow

Start a run:

```bash
ros2 topic pub --once /goal_available std_msgs/msg/Bool "{data: true}"
```

Mark the goal as reached:

```bash
ros2 topic pub --once /goal_reached std_msgs/msg/Bool "{data: true}"
```

Save the current run and reset the recorder:

```bash
ros2 topic pub --once /save_metrics std_msgs/msg/Bool "{data: true}"
```

After saving, publish another `goal_available` message to begin the next run.
