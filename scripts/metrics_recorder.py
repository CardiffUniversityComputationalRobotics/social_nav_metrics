#!/usr/bin/env python3

import csv
from datetime import datetime
import math
import time
import tf
import rospy
from std_msgs.msg import Float32, Int32, Bool
import numpy as np
from rosgraph_msgs.msg import Clock
from pedsim_msgs.msg import AgentStates
from nav_msgs.msg import Odometry


def import_csv(csvfilename):
    """Opens and return all content from a CSV in an array"""
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
                    row[8],
                    row[9],
                    row[10],
                ]
                data.append(columns)
        scraped.close()
    return data


class MetricsRecorder:
    """This class manages the measurement of the included metrics for social robot navigation
    and saves the metrics on a CSV"""

    def save_value_csv(self):
        """Saves value of the measured metrics in a new or previously given csv"""
        rospy.loginfo("About to save test measurements.")

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
            "path_irregularity",
            "acc_per_segment",
            "path_length",
        ]

        # in case num_nodes is not considered, just make it zero
        if len(self.num_nodes_) == 0:
            self.num_nodes_ = np.append(self.num_nodes_, 0)

        now = datetime.now()
        dt_string = now.strftime("%d/%m/%Y %H:%M:%S")

        last_data = None
        try:
            csv_read_data = import_csv(
                self.csv_dir_ + "/" + self.approach_name_ + "/" + self.csv_name_
            )
            last_data = csv_read_data[-1]
        except OSError:
            rospy.logwarn("Could not open the defined CSV file")

        rospy.loginfo(
            "Provided CSV at "
            + self.csv_dir_
            + "/"
            + self.approach_name_
            + "/"
            + self.csv_name_
            + " has been imported."
        )

        with open(
            self.csv_dir_ + "/" + self.approach_name_ + "/" + self.csv_name_,
            "a",
            newline="",
            encoding="utf-8",
        ) as csvfile_write, open(
            self.csv_dir_ + "/" + self.approach_name_ + "/" + self.csv_name_,
            "r",
            encoding="utf-8",
        ) as csvfile_read:
            reader = csv.reader(csvfile_read)

            writer = csv.DictWriter(csvfile_write, fieldnames=fieldnames)
            try:
                if next(reader) != fieldnames:
                    writer.writeheader()
            except:
                writer.writeheader()

            if last_data is not None:
                last_data_index = 1
            else:
                last_data_index = int(last_data[1]) + 1

            writer.writerow(
                {
                    "test_number": last_data_index,
                    "time": dt_string,
                    "goal_reached": self.goal_reached_,
                    "average_sii": round(np.average(self.sii_), 4),
                    "average_rmi": round(np.average(self.rmi_), 4),
                    "total_time": self.total_time_,
                    "average_cpu": round(np.average(self.cpu_list_), 4),
                    "collision_counter": self.collision_counter_,
                    "num_nodes": int(np.average(self.num_nodes_)),
                    "path_irregularity": round(np.average(self.path_irregularity_), 4),
                    "acc_per_segment": round(
                        np.average(self.acceleration_per_segment_), 4
                    ),
                    "path_length": self.path_length_,
                }
            )

            rospy.loginfo("Metrics for test saved.")
            csvfile_write.close()
            csvfile_read.close()

    def __init__(self):
        rospy.init_node("social_nav_metrics_recorder", anonymous=True)

        rospy.on_shutdown(self.save_value_csv)

        # ! RECORDING VARIABLES
        # POSITIONS AND ORIENTATIONS
        self.past_robot_position_ = None
        self.robot_position_ = None
        self.agent_states_ = None

        # ROBOT VELOCITIES
        self.robot_velocities_ = None
        self.past_robot_velocities_ = None

        # SOCIAL NAVIGATION COMMON METRICS
        self.rmi_ = np.array([], dtype=np.float64)
        self.sii_ = np.array([], dtype=np.float64)
        self.num_nodes_ = np.array([], dtype=np.int32)
        self.collision_counter_ = 0
        self.goal_reached_ = 0
        self.current_cpu_ = None
        self.current_num_nodes_ = None

        self.path_irregularity_ = np.array([], dtype=np.float64)
        self.acceleration_per_segment_ = np.array([], dtype=np.float64)

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

        # ====================================================

        #! LAMBDA FUNCTIONS
        # ? RELATIVE MOTION INDEX
        self.rmi_value = (
            lambda v_r, beta, v_a, alpha, x_agent, y_agent, x_robot, y_robot: (
                2 + v_r * np.cos(beta) + v_a * np.cos(alpha)
            )
            / (np.sqrt(math.pow(x_agent - x_robot, 2) + math.pow(y_agent - y_robot, 2)))
        )

        # ? SOCIAL INDIVIDUAL INDEX
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

        # GOAL FLAG
        self.goal_available_ = False

        # TIME VARIABLES
        self.total_time_ = 0.0
        self.init_query_time_ = 0.0
        self.current_time_ = 0.0
        self.last_time_ = 0

        self.cpu_list_ = np.array([], dtype=np.float64)
        # ================================================

        # ! CONFIGS VALUES
        # ===============================================

        # ? TOPICS
        self.clock_topic_ = rospy.get_param("~clock_topic", "/clock")
        self.cpu_topic_ = rospy.get_param("~cpu_topic", "/cpu_monitor/planner/cpu")
        self.goal_reached_topic_ = rospy.get_param(
            "~goal_reached_topic", "/goal_reached"
        )
        self.goal_available_topic_ = rospy.get_param(
            "~goal_available_topic", "/goal_available"
        )
        self.odom_topic_ = rospy.get_param("~odom_topic", "/odom")
        self.num_nodes_topic = rospy.get_param("~num_nodes_topic", "/num_nodes")
        self.agents_states_topic_ = rospy.get_param(
            "~agent_states_topic", "/pedsim_simulator/simulated_agents"
        )
        self.collision_counter_topic_ = rospy.get_param(
            "~collision_counter_topic", "/collision_counter"
        )

        # ? RATE PARAM
        self.measure_rate_ = rospy.get_param("~measure_rate", 5)
        self.measure_period_ = float(1 / self.measure_rate_)
        self.sim = rospy.get_param("~sim", True)

        # ? CSV SAVING PARAMS
        self.csv_dir_ = rospy.get_param("~csv_dir")
        self.approach_name_ = rospy.get_param("~approach_name")
        self.csv_name_ = rospy.get_param("~csv_name")
        # ================================================

        #! SUBSCRIBERS
        # ================================================
        if self.sim:
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
            self.goal_available_topic_, Bool, self.goal_callback, queue_size=1
        )
        rospy.Subscriber(
            self.goal_reached_topic_,
            Bool,
            self.goal_reached_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            self.agents_states_topic_, AgentStates, self.agents_callback, queue_size=1
        )
        rospy.Subscriber(self.odom_topic_, Odometry, self.odom_callback)
        rospy.Subscriber(
            self.collision_counter_topic_, Int32, self.collision_counter_callback
        )
        # ======================================================

    # ! CALLBACKS
    # ===============================================

    def goal_callback(self, goal_available: Bool):
        """Receives if the goal for the navigation query is already available."""
        if self.init_query_time_ == 0:
            if not self.sim:
                self.init_query_time_ = time.time()
            else:
                self.init_query_time_ = self.current_time_
        self.goal_available_ = True

    def goal_reached_callback(self, msg: Bool):
        """Receives if the robot has reached or not the goal"""
        if msg.data:
            self.goal_reached_ = 1
            if not self.sim:
                self.total_time_ = time.time() - self.init_query_time_
            else:
                self.total_time_ = self.current_time_ - self.init_query_time_

    def clock_callback(self, msg: Clock):
        """Listens to the gazebo clock time if simulation is running."""
        self.current_time_ = msg.clock.secs + float(msg.clock.nsecs / 1000000000)

    def cpu_callback(self, msg):
        """Listens to the CPU power used by the navigation system"""
        if self.goal_available_:
            self.current_cpu_ = msg.data

    def collision_counter_callback(self, msg):
        """Listens to the amount of collisions happening by an external node."""
        self.collision_counter_ = msg.data

    def num_nodes_callback(self, msg):
        """Listens to the number of nodes sampled for the case of sampling based techniques"""
        self.current_num_nodes_ = msg.data

    def odom_callback(self, odom: Odometry):
        """Listens to the odometry of the robot."""
        self.robot_position_ = odom.pose
        self.robot_velocities_ = odom.twist

    def agents_callback(self, agents: AgentStates):
        """Listens to the states of social agents"""
        self.agent_states_ = agents.agent_states

    # =================================================

    # ! SOCIAL NAVIGATION SPECIFIC METRICS CALCULATIONS FUNCTIONS
    # =================================================
    def calculate_rmi(self):
        """Calculates the relative motion index according to the robot
        and surrounding social agents."""
        last_rmi = 0

        for agent in self.agent_states_:
            v_r = np.sqrt(
                math.pow(self.robot_velocities_.twist.linear.x, 2)
                + math.pow(self.robot_velocities_.twist.linear.y, 2)
            )

            beta = math.atan2(
                agent.pose.position.y - self.robot_position_.pose.position.y,
                agent.pose.position.x - self.robot_position_.pose.position.x,
            )

            if beta < 0:
                beta = 2 * math.pi + beta

            quaternion = (
                self.robot_position_.pose.orientation.x,
                self.robot_position_.pose.orientation.y,
                self.robot_position_.pose.orientation.z,
                self.robot_position_.pose.orientation.w,
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
                self.robot_position_.pose.position.y - agent.pose.position.y,
                self.robot_position_.pose.position.x - agent.pose.position.x,
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
                self.robot_position_.pose.position.x,
                self.robot_position_.pose.position.y,
            )

            if current_rmi > last_rmi:
                last_rmi = current_rmi

        return last_rmi

    def calculate_sii(self):
        """Calculates the social individual index according to the robot
        and surrounding social agents."""
        last_sii = 0
        current_sii = 0
        for agent in self.agent_states_:
            current_sii = self.sii_value(
                agent.pose.position.x,
                agent.pose.position.y,
                self.robot_position_.pose.position.x,
                self.robot_position_.pose.position.y,
            )
            if current_sii > last_sii:
                last_sii = current_sii

        return last_sii

    def calculate_path_irregularity(self):
        """Calculates path irrgularity at an specific time"""

        distance_change = math.sqrt(
            math.pow(
                self.past_robot_position_.pose.position.x
                - self.robot_position_.pose.position.x,
                2,
            )
            + math.pow(
                self.past_robot_position_.pose.position.y
                - self.robot_position_.pose.position.y,
                2,
            )
        )

        old_q = (
            self.past_robot_position_.pose.orientation.x,
            self.past_robot_position_.pose.orientation.y,
            self.past_robot_position_.pose.orientation.z,
            self.past_robot_position_.pose.orientation.w,
        )

        old_angle = tf.transformations.euler_from_quaternion(old_q)[2]

        new_q = (
            self.robot_position_.pose.orientation.x,
            self.robot_position_.pose.orientation.y,
            self.robot_position_.pose.orientation.z,
            self.robot_position_.pose.orientation.w,
        )

        new_angle = tf.transformations.euler_from_quaternion(new_q)[2]

        angle_change = abs(
            min((2 * math.pi) - abs(old_angle - new_angle), abs(old_angle - new_angle))
        )

        if distance_change < 0.001 and angle_change < 0.001:
            path_irregularity = -1
        elif distance_change < 0.001:
            distance_change = 0.001
            path_irregularity = float(angle_change / distance_change)
        else:
            path_irregularity = float(angle_change / distance_change)

        if path_irregularity > 10:
            path_irregularity = 10

        return path_irregularity

    def calculate_acc_per_segment(self):
        acceleration_x = (
            self.past_robot_velocities_.twist.linear.x
            - self.robot_velocities_.twist.linear.x
        ) / self.measure_period_
        acceleration_y = (
            self.past_robot_velocities_.twist.linear.y
            - self.robot_velocities_.twist.linear.y
        ) / self.measure_period_

        res_acceleration = math.sqrt(
            math.pow(acceleration_x, 2) + math.pow(acceleration_y, 2)
        )

        distance_change = math.sqrt(
            math.pow(
                self.past_robot_position_.pose.position.x
                - self.robot_position_.pose.position.x,
                2,
            )
            + math.pow(
                self.past_robot_position_.pose.position.y
                - self.robot_position_.pose.position.y,
                2,
            )
        )

        self.path_length_ += distance_change

        if distance_change > 0.001:
            acc_per_segment = float(res_acceleration / distance_change)
        else:
            acc_per_segment = -1

        if acc_per_segment > 30:
            acc_per_segment = 30

        return acc_per_segment

    # ========================================================

    def run(self):
        """Manages the time passed and recording of the metrics"""
        while not rospy.is_shutdown():
            if self.goal_available_:
                if not self.sim:
                    self.current_time_ = time.time()
                if self.current_time_ - self.last_time_ >= self.measure_period_:
                    if self.current_cpu_:
                        if len(self.current_cpu_) > 1000:
                            self.current_cpu_ = np.array(
                                [np.average(self.current_cpu_)], dtype=np.float64
                            )
                        self.cpu_list_ = np.append(self.cpu_list_, self.current_cpu_)
                    if self.current_num_nodes_:
                        if len(self.self.num_nodes_) > 1000:
                            self.self.num_nodes_ = np.array(
                                [np.average(self.self.num_nodes_)], dtype=np.float64
                            )
                        self.num_nodes_ = np.append(
                            self.num_nodes_, self.current_num_nodes_
                        )
                    if (
                        self.robot_velocities_
                        and self.robot_position_
                        and self.agent_states_
                    ):
                        rmi = self.calculate_rmi()
                        if len(self.rmi_) > 1000:
                            self.rmi_ = np.array(
                                [np.average(self.rmi_)], dtype=np.float64
                            )
                        self.rmi_ = np.append(self.rmi_, rmi)
                        sii = self.calculate_sii()
                        if len(self.sii_) > 1000:
                            self.sii_ = np.array(
                                [np.average(self.sii_)], dtype=np.float64
                            )
                        self.sii_ = np.append(self.sii_, sii)

                        if self.past_robot_position_ and self.past_robot_velocities_:
                            # measure path irregularity
                            path_irregularity = self.calculate_path_irregularity()

                            if path_irregularity >= 0:
                                if len(self.path_irregularity_) > 1000:
                                    self.path_irregularity_ = np.array(
                                        [np.average(self.path_irregularity_)],
                                        dtype=np.float64,
                                    )
                                self.path_irregularity_ = np.append(
                                    self.path_irregularity_, path_irregularity
                                )

                            # measure aceleration per segment
                            acc_per_segment = self.calculate_acc_per_segment()

                            if acc_per_segment >= 0:
                                if len(self.acceleration_per_segment__) > 1000:
                                    self.acceleration_per_segment_ = np.array(
                                        [np.average(self.acceleration_per_segment_)],
                                        dtype=np.float64,
                                    )
                                self.acceleration_per_segment_ = np.append(
                                    self.acceleration_per_segment_, acc_per_segment
                                )

                            self.past_robot_position_ = self.robot_position_
                            self.past_robot_velocities_ = self.robot_velocities_

                        else:
                            self.past_robot_position_ = self.robot_position_
                            self.past_robot_velocities_ = self.robot_velocities_

                    self.last_time_ = self.current_time_
            rospy.sleep(0.0001)


if __name__ == "__main__":
    csv_counter_saver = MetricsRecorder()
    csv_counter_saver.run()
