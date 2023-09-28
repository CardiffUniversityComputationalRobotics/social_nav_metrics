#!/usr/bin/env python3

import csv
from datetime import datetime
import rospy
from std_msgs.msg import Float32, Int32, Bool, Float64
import numpy as np
from rosgraph_msgs.msg import Clock
from pedsim_msgs.msg import AgentStates
from nav_msgs.msg import Odometry
import math
import tf


def import_csv(csvfilename):
    """opens and return all content from csv in an array"""
    data = []
    with open(csvfilename, "r", encoding="utf-8", errors="ignore") as scraped:
        reader = csv.reader(scraped, delimiter=",")
        row_index = 0
        for row in reader:
            if row:  # avoid blank lines
                row_index += 1
                columns = [
                    str(row_index),
                    row[0],
                    row[1],
                    row[2],
                    row[3],
                    row[4],
                    row[5],
                    row[6],
                    row[7],
                ]
                data.append(columns)
        scraped.close()
    return data


class MetricsRecorder:
    """This class manages the state of the agents based on it position and time"""

    def save_value_csv(self):
        print("About to save data")
        """saves value of the metrics recorded in a csv"""
        now = datetime.now()
        dt_string = now.strftime("%d/%m/%Y %H:%M:%S")

        last_data = None
        try:
            csv_read_data = import_csv(
                self.csv_dir_ + self.solution_type_ + "/" + self.csv_name_
            )
            last_data = csv_read_data[-1]
        except Exception as _e:
            pass
            # print(_e)

        rospy.loginfo("csv imported")
        rospy.loginfo(self.csv_dir_ + self.solution_type_ + "/" + self.csv_name_)
        rospy.loginfo(last_data)

        with open(
            self.csv_dir_ + self.solution_type_ + "/" + self.csv_name_, "a", newline=""
        ) as csvfile_write, open(
            self.csv_dir_ + self.solution_type_ + "/" + self.csv_name_,
            "r",
        ) as csvfile_read:
            reader = csv.reader(csvfile_read)
            fieldnames = [
                "test_number",
                "time",
                "goal_reached",
                "average_sii",
                "average_rmi",
                "total_time",
                "average_cpu",
                "collision_counter",
                "num_nodes",
            ]
            writer = csv.DictWriter(csvfile_write, fieldnames=fieldnames)
            try:
                if next(reader) != [
                    "test_number",
                    "time",
                    "goal_reached",
                    "average_sii",
                    "average_rmi",
                    "total_time",
                    "average_cpu",
                    "collision_counter",
                    "num_nodes",
                ]:
                    writer.writeheader()
            except:
                writer.writeheader()

            rospy.loginfo("goal_reached: " + self.goal_reached_)

            if last_data is not None:
                if (
                    self.solution_type_ != "smf_planner"
                    and self.solution_type_ != "esc_planner"
                ):
                    self.num_nodes_ = 0

                writer.writerow(
                    {
                        "test_number": int(last_data[1]) + 1,
                        "time": dt_string,
                        "goal_reached": self.goal_reached_,
                        "average_sii": round(np.average(self.sii_), 2),
                        "average_rmi": round(np.average(self.rmi_), 2),
                        "total_time": self.total_time_,
                        "average_cpu": round(np.average(self.cpu_list_), 2),
                        "collision_counter": self.collision_counter_,
                        "num_nodes": int(np.average(self.num_nodes_)),
                    }
                )
            else:
                if (
                    self.solution_type_ != "smf_planner"
                    and self.solution_type_ != "esc_planner"
                ):
                    self.num_nodes_ = 0

                writer.writerow(
                    {
                        "test_number": 1,
                        "time": dt_string,
                        "goal_reached": self.goal_reached_,
                        "average_sii": round(np.average(self.sii_), 2),
                        "average_rmi": round(np.average(self.rmi_), 2),
                        "total_time": self.total_time_,
                        "average_cpu": round(np.average(self.cpu_list_), 2),
                        "collision_counter": self.collision_counter_,
                        "num_nodes": int(np.average(self.num_nodes_)),
                    }
                )
            print("[INFO] [" + str(rospy.get_time()) + "] metrics for test saved.")
            csvfile_write.close()
            csvfile_read.close()

    def __init__(self):
        rospy.init_node("social_nav_metrics_recorder", anonymous=True)

        rospy.on_shutdown(self.save_value_csv)

        # ! RECORDING VARIABLES
        # POSITIONS
        self.robot_position_ = None
        self.agent_states_ = None

        # ROBOT VELOCITIES
        self.robot_velocities_ = None

        # SOCIAL NAVIGATION COMMON METRICS
        self.rmi_ = np.array([], dtype=np.float64)
        self.sii_ = np.array([], dtype=np.float64)
        self.collision_counter_ = 0
        self.goal_reached_ = 0

        # LAMBDA FUNCTIONS
        self.rmi_value = (
            lambda v_r, beta, v_a, alpha, x_agent, y_agent, x_robot, y_robot: (
                2 + v_r * np.cos(beta) + v_a * np.cos(alpha)
            )
            / (np.sqrt(math.pow(x_agent - x_robot, 2) + math.pow(y_agent - y_robot, 2)))
        )

        self.sii_value = lambda x_agent, y_agent, x_robot, y_robot: (
            math.pow(
                math.e,
                -(
                    math.pow(
                        (x_robot - x_agent) / (self.final_sigma),
                        2,
                    )
                    + math.pow(
                        (y_robot - y_agent) / (self.final_sigma),
                        2,
                    )
                ),
            )
        )

        self.num_nodes_ = np.array([], dtype=np.int32)

        # GOAL FLAG
        self.goal_available_ = False

        # TIME VARIABLES
        self.total_time_ = 0.0
        self.current_time_ = 0.0
        self.last_time_ = 0

        self.cpu_list_ = np.array([], dtype=np.float64)

        # ! CONFIGS VALUES
        # TOPICS
        self.clock_topic_ = rospy.get_param("~clock_topic", "/clock")
        self.cpu_topic_ = rospy.get_param("~cpu_topic", "/cpu_monitor/planner/cpu")
        self.goal_reached_topic_ = rospy.get_param(
            "~goal_reached_topic", "/goal_reached_topic"
        )
        self.goal_topic_ = rospy.get_param("~goal_topic", "/goal_topic")
        self.odom_topic_ = rospy.get_param("~odom_topic", "/odom")
        self.num_nodes_topic = rospy.get_param("~num_nodes_topic", "/num_nodes_topic")
        self.agents_states_topic_ = rospy.get_param(
            "~agents_states_topic", "/pedsim_simulator/simulated_agents"
        )

        # RATE PARAM
        self.measure_rate_ = rospy.get_param("~measure_rate", 5)
        self.measure_period_ = 1 / self.measure_rate_

        # CSV SAVING PARAMS
        self.csv_dir_ = rospy.get_param("~csv_dir")
        self.solution_type_ = rospy.get_param("~solution_type")
        self.csv_name_ = rospy.get_param("~csv_name")

        #! SUBSCRIBERS
        rospy.Subscriber(
            self.clock_topic_,
            Clock,
            self.clock_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            self.cpu_topic_,
            Float32,
            self.cpu_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            self.num_nodes_topic,
            Int32,
            self.num_nodes_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            self.goal_reached_topic_,
            Bool,
            self.num_nodes_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            self.agents_states_topic_, AgentStates, self.agents_callback, queue_size=1
        )
        rospy.Subscriber(self.odom_topic_, Odometry, self.odom_callback)
        rospy.Subscriber(self.goal_reached_topic_, Bool, self.goal_reached_callback)

    def goal_reached_callback(self, msg: Bool):
        if msg.data:
            self.goal_reached_ = True

    def clock_callback(self, msg: Clock):
        self.current_time_ = msg.clock.secs

    def cpu_callback(self, msg):
        if self.goal_available_:
            self.current_cpu_ = np.append(self.cpu_list_, msg.data)

    def collision_counter_callback(self, msg):
        self.collision_counter_ = msg.data

    def num_nodes_callback(self, msg):
        if self.goal_available_:
            self.num_nodes_ = np.append(self.num_nodes_, msg.data)

    def odom_callback(self, odom: Odometry):
        self.robot_position_ = odom.pose
        self.robot_velocities_ = odom.twist

    def agents_callback(self, agents: AgentStates):
        self.agent_states_ = agents.agent_states

    def calculate_rmi(self):
        last_rmi = 0

        for agent in self.agent_states_:
            v_r = np.sqrt(
                math.pow(self.robot_velocities_.twist.linear.x, 2)
                + math.pow(self.robot_velocities_.twist.twist.linear.y, 2)
            )

            # angle between robot orientation and vector robot-agent
            beta = math.atan2(
                agent.pose.position.y - self.robot_position_.pose.pose.position.y,
                agent.pose.position.x - self.robot_position_.pose.pose.position.x,
            )

            if beta < 0:
                beta = 2 * math.pi + beta

            quaternion = (
                self.robot_position_.pose.pose.orientation.x,
                self.robot_position_.pose.pose.orientation.y,
                self.robot_position_.pose.pose.orientation.z,
                self.robot_position_.pose.pose.orientation.w,
            )
            euler = tf.transformations.euler_from_quaternion(quaternion)
            yaw = euler[2]

            if yaw < 0:
                yaw = 2 * math.pi + yaw

            if beta > (yaw + math.pi):
                beta = abs(yaw + 2 * math.pi - beta)
            elif yaw > (beta + math.pi):
                beta = abs(beta + 2 * math.pi - yaw)
            else:
                beta = abs(beta - yaw)

            v_a = np.sqrt(
                math.pow(agent.twist.linear.x, 2) + math.pow(agent.twist.linear.y, 2)
            )

            alpha = math.atan2(
                self.robot_position_.pose.pose.position.y - agent.pose.position.y,
                self.robot_position_.pose.pose.position.x - agent.pose.position.x,
            )

            if alpha < 0:
                alpha = 2 * math.pi + alpha

            quaternion = (
                agent.pose.orientation.x,
                agent.pose.orientation.y,
                agent.pose.orientation.z,
                agent.pose.orientation.w,
            )
            euler = tf.transformations.euler_from_quaternion(quaternion)
            yaw = euler[2]

            if yaw < 0:
                yaw = 2 * math.pi + yaw

            if alpha > (yaw + math.pi):
                alpha = abs(yaw + 2 * math.pi - alpha)
            elif yaw > (alpha + math.pi):
                alpha = abs(alpha + 2 * math.pi - yaw)
            else:
                alpha = abs(alpha - yaw)

            current_rmi = self.rmi_value(
                v_r,
                beta,
                v_a,
                alpha,
                agent.pose.position.x,
                agent.pose.position.y,
                self.robot_position_.pose.pose.position.x,
                self.robot_position_.pose.pose.position.y,
            )

            if current_rmi > last_rmi:
                last_rmi = current_rmi

        return last_rmi

    def run(self):
        while not rospy.is_shutdown():
            if self.last_time_ - self.current_time_ <= self.measure_period_:
                np.append(self.cpu_list_, self.current_cpu_)

                self.last_time_ = self.current_time_


if __name__ == "__main__":
    csv_counter_saver = MetricsRecorder()
    while not rospy.is_shutdown():
        rospy.spin()
