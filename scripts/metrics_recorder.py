#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import rclpy.qos
from nav_msgs.msg import Odometry
from pedsim_msgs.msg import AgentStates
from rosgraph_msgs.msg import Clock
from metrics import RunningAverage, measure_values
from std_msgs.msg import Bool, Int32
from utils import save_value_csv


class MetricsRecorder(Node):
    """This class manages the measurement of the included metrics for social robot navigation
    and saves the metrics on a CSV"""

    def __init__(self):
        super().__init__("metrics_recorder_node")

        #! PARAMETERS DECLARED
        self.declare_parameters(
            namespace="",
            parameters=[
                ("clock_topic", "/clock"),
                ("goal_reached_topic", "/goal_reached"),
                ("goal_available_topic", "/goal_available"),
                ("save_metrics_topic", "/save_metrics"),
                ("odom_topic", "/odom"),
                ("num_nodes_topic", "/num_nodes"),
                ("agent_states_topic", "/pedsim_simulator/simulated_agents"),
                ("collision_counter_topic", "/collision_counter"),
                ("measure_rate", 5.0),
                ("sim", True),
                ("csv_dir", ""),
                ("approach_name", ""),
                ("csv_name", ""),
            ],
        )

        # ! CONFIGS VALUES
        # ===============================================
        # ? TOPICS
        self.clock_topic_ = (
            self.get_parameter("clock_topic").get_parameter_value().string_value
        )
        self.goal_reached_topic_ = (
            self.get_parameter("goal_reached_topic").get_parameter_value().string_value
        )
        self.goal_available_topic_ = (
            self.get_parameter("goal_available_topic")
            .get_parameter_value()
            .string_value
        )
        self.save_metrics_topic_ = (
            self.get_parameter("save_metrics_topic").get_parameter_value().string_value
        )
        self.odom_topic_ = (
            self.get_parameter("odom_topic").get_parameter_value().string_value
        )
        self.num_nodes_topic = (
            self.get_parameter("num_nodes_topic").get_parameter_value().string_value
        )
        self.agent_states_topic_ = (
            self.get_parameter("agent_states_topic").get_parameter_value().string_value
        )
        self.collision_counter_topic_ = (
            self.get_parameter("collision_counter_topic")
            .get_parameter_value()
            .string_value
        )

        # ? RATE PARAM
        self.measure_rate_ = (
            self.get_parameter("measure_rate").get_parameter_value().double_value
        )
        self.sim = self.get_parameter("sim").get_parameter_value().bool_value

        # ? CSV SAVING PARAMS
        self.csv_dir_ = self.get_parameter("csv_dir").get_parameter_value().string_value
        self.approach_name_ = (
            self.get_parameter("approach_name").get_parameter_value().string_value
        )
        self.csv_name_ = (
            self.get_parameter("csv_name").get_parameter_value().string_value
        )

        # =============================

        # ! RECORDING VARIABLES
        self.measure_period_ = float(1 / self.measure_rate_)

        # POSITIONS AND ORIENTATIONS
        self.past_robot_position_ = None
        self.robot_position_ = None
        self.agent_states_ = None

        # ROBOT VELOCITIES
        self.robot_velocities_ = None
        self.past_robot_velocities_ = None

        # SOCIAL NAVIGATION COMMON METRICS
        self.rmi_ = RunningAverage()
        self.sii_ = RunningAverage()
        self.num_nodes_ = RunningAverage()
        self.collision_counter_ = 0
        self.goal_reached_ = 0
        self.current_num_nodes_ = None

        self.path_irregularity_ = RunningAverage()
        self.acceleration_per_segment_ = RunningAverage()

        self.orientation_change_ = 0
        self.path_length_ = 0

        # ! SII VARIABLES

        """d_c: is the desirable value of the distance between the robot and the 
        agents, can be around 0.45m and 1.2m according to Hall depending on the
        culture
        """
        self.d_c = 1.2
        self.sigma_p = self.d_c / 2
        self.final_sigma = math.sqrt(2) * self.sigma_p
        # GOAL FLAG
        self.goal_available_ = False

        # TIME VARIABLES
        self.total_time_ = 0.0
        self.init_query_time_ = 0.0
        self.current_time_ = 0.0
        self.last_time_ = 0
        # ================================================

        #! SUBSCRIBERS
        # ================================================
        qos_profile = QoSProfile(depth=10)

        clock_qos_profile = QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, depth=1
        )

        if self.sim:
            self.create_subscription(
                Clock, self.clock_topic_, self.clock_callback, clock_qos_profile
            )
        self.create_subscription(
            Int32, self.num_nodes_topic, self.num_nodes_callback, qos_profile
        )
        self.create_subscription(
            Bool, self.goal_available_topic_, self.goal_callback, qos_profile
        )
        self.create_subscription(
            Bool, self.goal_reached_topic_, self.goal_reached_callback, qos_profile
        )
        self.create_subscription(
            Bool, self.save_metrics_topic_, self.save_metrics_callback, qos_profile
        )
        self.create_subscription(
            AgentStates, self.agent_states_topic_, self.agents_callback, qos_profile
        )
        self.create_subscription(
            Odometry, self.odom_topic_, self.odom_callback, qos_profile
        )
        self.create_subscription(
            Int32,
            self.collision_counter_topic_,
            self.collision_counter_callback,
            qos_profile,
        )
        # ======================================================
        self.timer = self.create_timer(self.measure_period_, lambda: measure_values(self))

    # ! CALLBACKS
    # ===============================================

    def has_metrics_to_save(self):
        """Return whether there is a completed or active measurement session."""
        return (
            self.goal_available_
            or self.goal_reached_ == 1
            or self.total_time_ > 0
            or self.collision_counter_ > 0
            or self.path_length_ > 0
            or len(self.rmi_) > 0
            or len(self.sii_) > 0
            or len(self.num_nodes_) > 0
            or len(self.path_irregularity_) > 0
            or len(self.acceleration_per_segment_) > 0
        )

    def reset_metrics_state(self):
        """Reset the recorder after saving so a new goal starts a new session."""
        self.goal_available_ = False
        self.goal_reached_ = 0
        self.collision_counter_ = 0
        self.total_time_ = 0.0
        self.init_query_time_ = 0.0
        self.last_time_ = 0.0

        self.robot_position_ = None
        self.past_robot_position_ = None
        self.agent_states_ = None
        self.robot_velocities_ = None
        self.past_robot_velocities_ = None

        self.current_num_nodes_ = None

        self.rmi_ = RunningAverage()
        self.sii_ = RunningAverage()
        self.num_nodes_ = RunningAverage()
        self.path_irregularity_ = RunningAverage()
        self.acceleration_per_segment_ = RunningAverage()

        self.orientation_change_ = 0
        self.path_length_ = 0

    def save_current_metrics(self):
        """Save the current metrics if there is an active session."""
        if not self.has_metrics_to_save():
            self.get_logger().info("No metrics are available to save.")
            return False

        save_value_csv(self)
        return True

    def goal_callback(self, goal_available: Bool):
        """Receives if the goal for the navigation query is already available."""
        if not goal_available.data or self.goal_available_:
            return

        if not self.sim:
            self.init_query_time_ = time.time()
            self.last_time_ = self.init_query_time_
        else:
            self.init_query_time_ = self.current_time_
            self.last_time_ = self.current_time_

        self.goal_available_ = True
        self.get_logger().info("Goal available received. Starting metrics recording.")

    def goal_reached_callback(self, msg: Bool):
        """Receives if the robot has reached or not the goal"""
        if msg.data and self.goal_available_:
            self.goal_reached_ = 1
            if not self.sim:
                self.total_time_ = time.time() - self.init_query_time_
            else:
                self.total_time_ = self.current_time_ - self.init_query_time_

    def save_metrics_callback(self, msg: Bool):
        """Save the current metrics when requested, then reset the recorder."""
        if not msg.data:
            return

        saved_metrics = self.save_current_metrics()
        self.reset_metrics_state()

        if saved_metrics:
            self.get_logger().info(
                "Metrics saved after /save_metrics request. Waiting for a new goal."
            )
        else:
            self.get_logger().info(
                "Received /save_metrics request with no active metrics. Recorder reset."
            )

    def clock_callback(self, msg: Clock):
        """Listens to the gazebo clock time if simulation is running."""
        self.current_time_ = msg.clock.sec + msg.clock.nanosec / 1e9

    def collision_counter_callback(self, msg: Int32):
        """Listens to the amount of collisions happening by an external node."""
        self.collision_counter_ = msg.data

    def num_nodes_callback(self, msg: Int32):
        """Listens to the number of nodes sampled for the case of sampling based techniques"""
        self.current_num_nodes_ = msg.data

    def odom_callback(self, odom: Odometry):
        """Listens to the odometry of the robot."""
        self.robot_position_ = odom.pose
        self.robot_velocities_ = odom.twist

    def agents_callback(self, agents: AgentStates):
        """Listens to the states of social agents"""
        self.agent_states_ = agents.agent_states


def main(args=None):
    rclpy.init(args=args)
    metrics_recorder_node = MetricsRecorder()
    try:
        rclpy.spin(metrics_recorder_node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        metrics_recorder_node.save_current_metrics()
    finally:
        rclpy.try_shutdown()
    metrics_recorder_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
