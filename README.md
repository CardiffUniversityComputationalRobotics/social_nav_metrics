# Social Robot Navigation Metrics Recorder

ROS 2 package for recording social robot navigation metrics in simulation or on a real robot. The package writes one CSV row per navigation run and can optionally count collisions against people and an OctoMap separately.

The package provides two nodes:

- `metrics_recorder_node`: records metrics from odometry, pedestrian states, goal status, and collision count topics.
- `collision_counter_node`: counts collision events against social agents and an OctoMap, then publishes separate people and object collision counts.

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
| `num_nodes_topic` | string | `/num_nodes` | Optional topic carrying the number of nodes a sampling-based planner expanded in its search tree or roadmap. |
| `agent_states_topic` | string | `/pedsim_simulator/simulated_agents` | Social agent states topic. |
| `people_collision_counter_topic` | string | `/people_collision_counter` | People collision count topic, normally published by `collision_counter_node`. |
| `object_collision_counter_topic` | string | `/object_collision_counter` | Object collision count topic, normally published by `collision_counter_node`. |
| `measure_rate` | double | `5.0` | Measurement frequency in Hz. For example, `10.0` records every `0.1` seconds. |
| `robot_max_velocity` | double | `1.0` | Theoretical maximum robot speed in m/s, used by the Social Effort Index. |
| `agent_max_velocity` | double | `1.0` | Theoretical maximum person speed in m/s, used by the Social Effort Index. |
| `robot_radius` | double | `0.25` | Robot radius used by Social Effort Index and by Time-to-Collision, which tests the robot disc against each person disc. |
| `agent_radius` | double | `0.3` | Person radius used by Social Effort Index and Time-to-Collision. Together with `robot_radius` it sets the robot-person contact distance. |
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
| `/num_nodes` | `std_msgs/msg/Int32` | Optional count of nodes in the sampling-based planner's tree or roadmap. |
| `/people_collision_counter` | `std_msgs/msg/Int32` | Current people collision count. |
| `/object_collision_counter` | `std_msgs/msg/Int32` | Current object collision count. |

### Saved CSV Fields

The recorder writes these columns:

```text
test_number,time,goal_reached,average_sii,average_rmi,average_sei,average_ttc,total_time,people_collision_counter,object_collision_counter,num_nodes,path_irregularity,in_place_rotation,linear_acc_per_segment,angular_acc_per_segment,path_length
```

The sampled metrics use a running average over all valid samples in the run. The recorder does not keep an unbounded array of samples, and it does not compress old samples into an unweighted average.

| Column | Unit | Better | Meaning |
| --- | --- | --- | --- |
| `test_number` | - | - | Row counter, continued from the existing CSV. |
| `time` | - | - | Wall-clock timestamp of the save, `dd/mm/YYYY HH:MM:SS`. |
| `goal_reached` | 0 or 1 | higher | `1` if `goal_reached_topic` published `True` during the run. |
| `average_sii` | 0 to 1 | lower | Social Individual Index. Per tick, the highest `exp(-(dx**2 + dy**2) / sigma**2)` over all people, where `sigma = sqrt(2) * d_c / 2` and `d_c` is a 1.2 m proxemics distance. `1` means the robot is on top of a person. |
| `average_rmi` | index | lower | Relative Motion Index. Per tick, the highest `(2 + v_robot * cos(beta) + v_person * cos(alpha)) / distance` over all people, where the angles are each body's heading relative to the other. The numerator adds a dimensionless `2` to two velocity terms, so the result has no consistent physical unit and is only comparable between runs that use the same formula. Grows without bound as the distance goes to zero. |
| `average_sei` | 1/m | lower | Social Effort Index. Two dimensionless sigmoid factors scaling an inverse distance, hence 1/m. Unlike the other three, it is summed over every person at each tick, so it grows with crowd size. Uses `robot_max_velocity`, `agent_max_velocity`, `robot_radius` and `agent_radius`. |
| `average_ttc` | s | higher | Time-to-Collision **against social agents only**. Per tick, the shortest time until the robot and person discs (`robot_radius + agent_radius`) touch at current velocities, taken over the people on `agent_states_topic`. Static obstacles, walls and the OctoMap are **not** considered, so this never warns about driving into a wall. Saturates at 5 s, so a scene with no people averages exactly `5.0`. |
| `total_time` | s | lower | Time from `goal_available` to `goal_reached`, or to the save if the goal was never reached. |
| `people_collision_counter` | count | lower | Distinct collision events against people, from `collision_counter_node`. |
| `object_collision_counter` | count | lower | Distinct collision events against the OctoMap, from `collision_counter_node`. |
| `num_nodes` | count | lower | Average number of nodes the planner sampled into its search tree or roadmap (RRT, RRT*, PRM and similar), truncated to an integer. **These are planner search nodes.** A planning-effort measure, so it is only meaningful for sampling-based planners; it stays `0` for anyone else, because nothing publishes the topic. |
| `path_irregularity` | rad/m | lower | Total absolute heading change divided by path length, clamped to `10.0`. |
| `in_place_rotation` | rad | lower | Total yaw change accumulated while the robot turned without translating (under 1 mm between ticks). |
| `linear_acc_per_segment` | 1/s^2 | lower | Linear acceleration divided by the distance travelled that tick, each sample clamped to `30.0`. Smoothness measure. |
| `angular_acc_per_segment` | rad/(s^2 m) | lower | Angular acceleration divided by the distance travelled that tick, each sample clamped to `30.0`. Kept separate from the linear one so translational and rotational units are not mixed. |
| `path_length` | m | lower | Total distance travelled. |

Two sampling details are worth knowing when comparing runs:

- All four social metrics are computed against the people on `agent_states_topic` and nothing else. None of them sees static obstacles or the OctoMap; object collisions are covered only by `object_collision_counter`.
- Social Individual Index and Relative Motion Index are only sampled while at least one person is published. Social Effort Index and Time-to-Collision are sampled on every tick, so a scene with no people still contributes `0.0` and `5.0` samples respectively. A high `average_ttc` therefore means "no person was on a collision course", not "the run was safe".
- The proxemics distance `d_c` used by the Social Individual Index is fixed at 1.2 m in `scripts/metrics_recorder.py`; it is not a ROS parameter.

The clamps and the Time-to-Collision ceiling are the constants at the top of `scripts/metrics.py` and `scripts/utils.py`.

## Collision Counter Node

`collision_counter_node` checks whether the robot is colliding with social agents or occupied OctoMap geometry. It increments people and object counters independently once per continuous collision event, publishes both current counts, and waits until the robot leaves that collision type before counting another event of the same type.

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
| `people_collision_counter_topic` | string | `/people_collision_counter` | Published people collision count topic. |
| `object_collision_counter_topic` | string | `/object_collision_counter` | Published object collision count topic. |
| `goal_available_topic` | string | `/goal_available` | Resets the collision counter when a `True` message is received. |

### Interfaces

| Interface | Message/Service Type | Direction |
| --- | --- | --- |
| `/odom` | `nav_msgs/msg/Odometry` | Subscriber |
| `/pedsim_simulator/simulated_agents` | `pedsim_msgs/msg/AgentStates` | Subscriber |
| `/goal_available` | `std_msgs/msg/Bool` | Subscriber |
| `/people_collision_counter` | `std_msgs/msg/Int32` | Publisher |
| `/object_collision_counter` | `std_msgs/msg/Int32` | Publisher |
| `/octomap_full` | `octomap_msgs/srv/GetOctomap` | Client |

## Dependencies

This branch targets ROS 2 Jazzy.

### Binary dependencies

Every dependency except `pedsim_msgs` ships as a binary package:

```bash
sudo apt install \
  ros-jazzy-octomap ros-jazzy-octomap-msgs ros-jazzy-octomap-server \
  ros-jazzy-tf-transformations ros-jazzy-tf2 ros-jazzy-tf2-ros \
  ros-jazzy-geometry-msgs ros-jazzy-nav-msgs ros-jazzy-rosgraph-msgs \
  libfcl-dev python3-numpy python3-transforms3d
```

`CMakeLists.txt` requires FCL 0.7.0 or newer through pkg-config. Check what you have with:

```bash
pkg-config --modversion fcl
```

### Source dependencies

`pedsim_msgs` has no rosdep rule, so `rosdep install` cannot supply it. Clone the pedsim fork into the same workspace, alongside this package:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/CardiffUniversityComputationalRobotics/social_nav_metrics.git
git clone https://github.com/CardiffUniversityComputationalRobotics/pedsim_ros.git
```

Only `pedsim_msgs` is needed to build and run this package. The rest of `pedsim_ros` is what actually simulates the crowd, so you need it for real experiments but not for the quick test below.

## Build With Colcon

Source your ROS 2 installation first:

```bash
source /opt/ros/jazzy/setup.bash
```

Then build in the workspace:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to social_nav_metrics
source install/setup.bash
```

Two things are easy to get wrong here:

- Run `rosdep install` only after cloning `pedsim_ros`, otherwise it cannot resolve `pedsim_msgs`.
- Use `--packages-up-to`, not `--packages-select`. `--packages-select` skips `pedsim_msgs`, and the build then fails on the missing message package.

This package uses the ROS 2 colcon workflow instead of the old rosinstall/catkin dependency flow.

## Quick Test Without a Simulator

This verifies an install end to end in about a minute, with no Gazebo, no pedsim simulator and no OctoMap. `fake_scenario.py` publishes a robot driving a 2 m circle while one person walks back and forth across its path, which is enough to exercise every sampled metric.

Create the output folder and start the recorder with the quick-test config. That config sets `sim: False`, so no `/clock` publisher is needed:

```bash
mkdir -p /tmp/social_nav_metrics/quick_test
ros2 run social_nav_metrics metrics_recorder.py --ros-args \
  --params-file "$(ros2 pkg prefix social_nav_metrics)/share/social_nav_metrics/config/quick_test.yaml"
```

In a second terminal, start the fake robot and person:

```bash
ros2 run social_nav_metrics fake_scenario.py
```

In a third terminal, run one navigation query:

```bash
ros2 topic pub --once /goal_available std_msgs/msg/Bool "{data: true}"
sleep 10
ros2 topic pub --once /goal_reached std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /save_metrics std_msgs/msg/Bool "{data: true}"
```

The recorder logs `Metrics for test saved.`, and the row is in the CSV:

```bash
cat /tmp/social_nav_metrics/quick_test/quick_test.csv
```

```text
test_number,time,goal_reached,average_sii,average_rmi,average_sei,average_ttc,total_time,people_collision_counter,object_collision_counter,num_nodes,path_irregularity,in_place_rotation,linear_acc_per_segment,angular_acc_per_segment,path_length
1,22/09/2026 12:28:42,1,0.0068,0.7604,0.0104,5.0,11.1161,0,0,0,0.5,0,0.0,0.0,5.99952345123277
```

The social metrics and the timings shift a little from run to run, but three columns have a known answer for this scenario and are the ones to check:

- `path_irregularity` is `0.5`. The robot drives a 2 m radius circle, and total heading change over path length is exactly `1 / radius`.
- `linear_acc_per_segment` and `angular_acc_per_segment` are `0.0`, because the fake robot holds a constant linear and angular speed.
- `average_ttc` is `5.0` whenever the fake person never crosses closely enough to threaten a collision, since Time-to-Collision saturates at 5 s. It is measured against that person only, so nothing in the scene other than the pedestrian can lower it.

`people_collision_counter` and `object_collision_counter` stay `0` here: `collision_counter_node` is not part of this test because it needs an OctoMap service. Run the full launch file against a simulator that provides one to record collisions.

Instead of publishing `save_metrics`, you can press `Ctrl+C` in the recorder terminal. It saves the run in progress with `goal_reached` set to `0`.

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

`collision_counter_node` fetches the OctoMap once at startup, so start it only after the OctoMap server is serving `octomap_service`. Launch only `metrics_recorder_node` when no OctoMap is available; the collision columns then stay at `0`.

## Configuration Files

The package installs two parameter files under `share/social_nav_metrics/config`:

| File | Purpose |
| --- | --- |
| `config_example.yaml` | Full example for a real run: both nodes, simulation time, and planner-specific topic names. |
| `quick_test.yaml` | Minimal recorder-only config used by the quick test above. Wall time, default topic names. |

## Example Configuration

ROS 2 parameter files must be grouped by node name under `ros__parameters`:

```yaml
metrics_recorder_node:
  ros__parameters:
    clock_topic: "/clock"
    goal_reached_topic: "/smf_move_base_planner/goal_reached"
    goal_available_topic: "/goal_available"
    save_metrics_topic: "/save_metrics"
    people_collision_counter_topic: "/people_collision_counter"
    object_collision_counter_topic: "/object_collision_counter"
    num_nodes_topic: "/smf_move_base_planner/smf_num_nodes"
    agent_states_topic: "/pedsim_simulator/simulated_agents"
    odom_topic: "/odom"

    measure_rate: 10.0
    sim: True
    robot_max_velocity: 1.0
    agent_max_velocity: 1.0
    robot_radius: 0.3
    agent_radius: 0.45

    csv_dir: "/tmp/social_nav_metrics/results"
    approach_name: "tests"
    csv_name: "new_test.csv"

collision_counter_node:
  ros__parameters:
    odom_topic: "/odom"
    agent_states_topic: "/pedsim_simulator/simulated_agents"
    people_collision_counter_topic: "/people_collision_counter"
    object_collision_counter_topic: "/object_collision_counter"
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
