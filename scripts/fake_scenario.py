#!/usr/bin/env python3
"""Publish a fake robot and a fake pedestrian so the recorder can be tested
without a simulator.

The robot drives a circle while one person walks back and forth across its
path, which exercises every sampled metric: the robot moves (path length,
path irregularity, acceleration) and passes close to a moving person (SII,
RMI, SEI, TTC). It publishes odometry and agent states only, so it works
with metrics_recorder_node alone, without pedsim, Gazebo or an OctoMap.
"""

import math

import rclpy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from pedsim_msgs.msg import AgentState, AgentStates
from rclpy.node import Node

PUBLISH_PERIOD = 0.05
ROBOT_SPEED = 0.5
ROBOT_PATH_RADIUS = 2.0
AGENT_SPEED = 0.4


def yaw_to_quaternion(yaw):
    return Quaternion(z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


class FakeScenario(Node):
    """Publish a circling robot and one pedestrian crossing its path."""

    def __init__(self):
        super().__init__("fake_scenario")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("odom_topic", "/odom"),
                ("agent_states_topic", "/pedsim_simulator/simulated_agents"),
                ("frame_id", "map"),
            ],
        )

        odom_topic = self.get_parameter("odom_topic").value
        agent_states_topic = self.get_parameter("agent_states_topic").value
        self.frame_id_ = self.get_parameter("frame_id").value

        self.odom_publisher_ = self.create_publisher(Odometry, odom_topic, 10)
        self.agents_publisher_ = self.create_publisher(
            AgentStates, agent_states_topic, 10
        )

        self.elapsed_time_ = 0.0
        self.create_timer(PUBLISH_PERIOD, self.publish_scenario)

        self.get_logger().info(
            f"Publishing a fake robot on {odom_topic} and one fake person on "
            f"{agent_states_topic}."
        )

    def publish_scenario(self):
        self.elapsed_time_ += PUBLISH_PERIOD
        heading = ROBOT_SPEED * self.elapsed_time_ / ROBOT_PATH_RADIUS

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.frame_id_
        odom.pose.pose.position.x = ROBOT_PATH_RADIUS * math.cos(heading)
        odom.pose.pose.position.y = ROBOT_PATH_RADIUS * math.sin(heading)
        odom.pose.pose.orientation = yaw_to_quaternion(heading + math.pi / 2)
        odom.twist.twist.linear.x = ROBOT_SPEED
        odom.twist.twist.angular.z = ROBOT_SPEED / ROBOT_PATH_RADIUS
        self.odom_publisher_.publish(odom)

        agent = AgentState()
        agent.header.stamp = odom.header.stamp
        agent.header.frame_id = self.frame_id_
        agent.id = 1
        agent.pose.position.x = ROBOT_PATH_RADIUS
        agent.pose.position.y = 1.5 * math.sin(0.3 * self.elapsed_time_)
        agent.pose.orientation = yaw_to_quaternion(math.pi)
        agent.twist.linear.x = -AGENT_SPEED

        agents = AgentStates()
        agents.header.stamp = odom.header.stamp
        agents.header.frame_id = self.frame_id_
        agents.agent_states = [agent]
        self.agents_publisher_.publish(agents)


def main(args=None):
    rclpy.init(args=args)
    fake_scenario_node = FakeScenario()
    try:
        rclpy.spin(fake_scenario_node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
