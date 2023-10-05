# Social Robot Navigation metrics recorder

This is a package that enables to record metrics for navigation involving people, both in simulation and real life.

This package is compounded of two nodes: `/metrics_recorder_node` and `/collision_counter_node`. The following image shows the topic connections with these nodes:

![](https://i.imgur.com/o3xbjtv.png)

## Metrics Recorder Node

This node is in charge of recording different metrics for social robot navigation.

### Parameters

- clock_topic (string, default: "/clock")

  In case simulation is desired, this is the topic of the time of the simulation in seconds. Commonly the topic `/clock` is used which corresponds to the time given by gazebo.

- cpu_topic (string, default: "/cpu_monitor/planner/cpu")

  Topic where the CPU usage of the used navigation system is published.

- goal_available_topic (string, default: "/goal_available")

  Topic where it is stated whether a goal has been set as a query. It is the trigger for this node to start recording the metrics.

- goal_reached_topic (string, default: "/goal_reached")

  Topic where it is stated whether the goal has been reached or not.

- odom_topic (string, default: "/odom")

  Robot odometry. Used in order to calculate some of the social robot navigation metrics.

- num_nodes_topic (string, default "/num_nodes")

  In some cases, for some planning approaches such as sampling based, it may be desired to record the amount of nodes sampled. This topic is used to record that amount of nodes.

- agent_states_topic (string, default: "/pedsim_simulator/simulated_agents")

  Topic where the states of the social agents are obtained.

- collision_counter_topic (string, default: "/collision_counter")

  Topic with the amount of collisions that the robot has encountered.

- measure_rate (double, default: 5)

  Time period in seconds between each measurement and recording.

- sim (bool, default: True)

  Whether the test is done in simulation or real-life. If `True` the time obtained from `clock_topic` is used to calculate the total time passed to complete the query and the period of measurement.

- csv_dir (string)

  The directory where the CSV to be used is located or where the new CSV will be created.

- approach_name (string)

  Name of the tested approach. This is an additional folder inside `csv_dir` with the name of the tested approach.

- csv_name (string)

  Name of the existing or to be created CSV.

### Subscribers

The name of the subscribers' topics are just defined as an example, but they may be configured using the parameters defined before.

- /clock ([rosgraph_msgs/Clock](http://docs.ros.org/en/melodic/api/rosgraph_msgs/html/msg/Clock.html))

  Time of the test when running simulation.

- /cpu_monitor/planner/cpu ([std_msgs/Float32](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html))

  CPU consumption of the navigation system.

- /num_nodes ([std_msgs/Int32](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Int32.html))

  Number of nodes sampled.

- /goal_available ([std_msgs/Bool](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html))

  If goal is available.

- /goal_reached ([std_msgs/Bool](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html))

  If goal has been reached.

- /pedsim_simulator/simulated_agents ([pedsim_msgs/AgentStates](https://github.com/CardiffUniversityComputationalRobotics/pedsim_ros/blob/noetic-devel/pedsim_msgs/msg/AgentStates.msg))

  Position, orientation and velocity of several social agents.

- /odom ([nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html))

  Odometry of the robot for the test.

- /collision_counter ([std_msgs/Int32](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Int32.html))

  Number of collisions the robot had with common obstacles and social agents.

## Collision Counter Node

This node is in charge of checking if the robot has collided with surrounding objects and social agents.

### Parameters

- robot_height (double)

  Height of the robot considered as a cylinder for collision checking.

- robot_radius (double)

  Radius of the robot considered as a cylinder for collision checking.

- agent_radius (double)

  Radius of the social agents considered as a cylinder for collision checking.

- odom_topic (string)

  Robot odometry. Used in order to do collision checking.

- agent_states_topic (string)

  Topic where the states of the social agents are obtained.

- collision_counter_topic (string)

  Topic with the amount of collisions that the robot has encountered.

- octomap_service (string)

  Topic where the states of the social agents are obtained.

### Subscribers

The name of the subscribers' topics are just defined as an example, but they may be configured using the parameters defined before.

- /odom ([nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html))

  Odometry of the robot for the test.

- /pedsim_simulator/simulated_agents ([pedsim_msgs/AgentStates](https://github.com/CardiffUniversityComputationalRobotics/pedsim_ros/blob/noetic-devel/pedsim_msgs/msg/AgentStates.msg))

  Position, orientation and velocity of several social agents.

### Publishers

- /collision_counter ([std_msgs/Int32](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Int32.html))

  Number of collisions that the robot had during one of the tests.

### Services

- octomap_service ([octomap_msgs/GetOctomap](http://docs.ros.org/en/melodic/api/octomap_msgs/html/srv/GetOctomap.html))

  The `collision_counter_node` uses this service in order to obtain the octomap and do collision checking using `fcl`.

## Demo

### Install the package

```bash
cd catkin_ws/src
git clone https://github.com/CardiffUniversityComputationalRobotics/social_nav_metrics.git
cd ..
rosinstall . social_nav_metrics/dependencies.rosinstall
catkin build
```

**Note:** When running `catkin build`, compiling the package may take some time becaude of `fcl` for Noetic.

### Deploying the nodes

Here you are provided with a `launch` file that can be used to deploy the code provided in this repo. Have in mind that it is not mandatory to run the `collision_counter` node. In this case we include it as part as our example using `octomap`, but you may replace it with any other node that counts the collisions for you and publish the number of collisions into the listening topic from the `metrics_recorder_node`.

```xml
<?xml version="1.0"?>
<launch>
    <node pkg="social_nav_metrics" type="metrics_recorder.py" name="metrics_recorder_node" output="screen">
        <rosparam command="load" file="$(find social_nav_metrics)/config/config_example.yaml"/>
    </node>

    <node pkg="social_nav_metrics" type="collision_counter" name="collision_counter_node" output="screen">
        <rosparam command="load" file="$(find social_nav_metrics)/config/config_example.yaml"/>
    </node>
</launch>
```

### Configuration file

The configuration file `config_example.xml` used in the previous presented `launch` file is showed below:

```yaml
# !TOPICS
clock_topic: "/clock"
cpu_topic: "/cpu_monitor/smf_move_base_planner/cpu"
goal_reached_topic: "/smf_move_base_planner/goal_reached"
goal_topic: "/goal_available"
collision_counter_topic: "/collision_counter"
num_nodes_topic: "/smf_move_base_planner/smf_num_nodes" # only considered if the approach is sampling based
agent_states_topic: "/pedsim_simulator/simulated_agents"
odom_topic: "/pepper/odom_groundtruth"

# octomap service
octomap_service: /smf_move_base_mapper/get_binary

#! robot and agents params
robot_radius: 0.3
robot_height: 1.0
agent_radius: 0.45

# ! measuring characteristics
measure_rate: 0.5
sim: True # whether the test is being done in simulation or real experiment

#! saving files config
csv_dir: "/home/sasm/ros/noetic/system/src/pepper_social_nav_tests/results"
approach_name: "smf_planner"
csv_name: "new_test.csv"

max_test_time: 500
```
