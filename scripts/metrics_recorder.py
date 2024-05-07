#!/usr/bin/env python3

import csv
from datetime import datetime
import math
import time
import tf
import rospy
from std_msgs.msg import Float32, Int32, Bool, Int32MultiArray, Float32MultiArray
import numpy as np
from rosgraph_msgs.msg import Clock
from pedsim_msgs.msg import AgentStates
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal

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
                    row[0], # Test Number
                    row[1], # Time/Date of Benchmark
                    row[2], # Goal Reached
                    row[3], # Average SII
                    row[4], # Average RMI
                    row[5], # Time Taken
                    row[6], # Average CPU
                    row[7], # Collision Counter
                    row[8], # Collision Counter Agent
                    row[9], # Collision Counter Environment
                    row[10], # Num Nodes
                    row[11], # Path Efficiency
                    row[12], # Average Linear Velocity
                    row[13], # Average Angular Velocity
                    row[14], # Average Linear Acceleration
                    row[15], # Average Angular Acceleration
                    row[16], # Num Social Spaces
                    row[17], # Num Personal Spaces
                    row[18], # Num Intimate Spaces
                    row[19] # Average SEI
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
            fieldnames = [
                "test_number",
                "time",
                "goal_reached",
                "average_sii",
                "average_rmi",
                "total_time",
                "average_cpu",
                "collision_counter",
                "collision_agent_counter",
                "collision_environment_counter",
                "num_nodes",
                "path_efficiency",
                "average_linear_velocity",
                "average_angular_velocity",
                "average_linear_acceleration",
                "average_angular_acceleration",
                "num_social_spaces",
                "num_personal_spaces",
                "num_intimate_spaces",
                "average_sei"
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
                    "collision_agent_counter",
                    "collision_environment_counter",
                    "num_nodes",
                    "path_efficiency",
                    "average_linear_velocity",
                    "average_angular_velocity",
                    "average_linear_acceleration",
                    "average_angular_acceleration",
                    "num_social_spaces",
                    "num_personal_spaces",
                    "num_intimate_spaces",
                    "average_sei"
                ]:
                    writer.writeheader()
            except:
                writer.writeheader()

            if self.num_nodes_.size == 0:
                self.num_nodes_ = "N/A"
            else:
                self.num_nodes_ = int(np.average(self.num_nodes_))

            start_point = self.points_list[0]

            for point in self.points_list:
                if point != start_point:
                    self.total_distance_ += self.euc_value(
                        point[0],
                        point[1],
                        last_point[0],
                        last_point[1]
                    )
                last_point = point

            self.path_efficiency_ = self.path_efficiency(self.waypoint_distance, self.total_distance_)

            if last_data is not None:
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
                        "collision_agent_counter": self.collision_agent_counter_,
                        "collision_environment_counter": self.collision_environment_counter_,
                        "num_nodes": self.num_nodes_,
                        "path_efficiency": round(self.path_efficiency_, 2),
                        "average_linear_velocity": round(np.average(self.velocity_linear_list_), 2),
                        "average_angular_velocity": round(np.average(self.velocity_angular_list_), 2),
                        "average_linear_acceleration": round(np.average(self.acceleration_linear), 2),
                        "average_angular_acceleration": round(np.average(self.acceleration_angular), 2),
                        "num_social_spaces": self.social_space_counter,
                        "num_personal_spaces": self.personal_space_counter,
                        "num_intimate_spaces": self.intimate_space_counter,
                        "average_sei": round(np.average(self.sei_), 2)
                    }
                )
            else:
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
                        "collision_agent_counter": self.collision_agent_counter_,
                        "collision_environment_counter": self.collision_environment_counter_,
                        "num_nodes": self.num_nodes_,
                        "path_efficiency": round(self.path_efficiency_, 2),
                        "average_linear_velocity": round(np.average(self.velocity_linear_list_), 2),
                        "average_angular_velocity": round(np.average(self.velocity_angular_list_), 2),
                        "average_linear_acceleration": round(np.average(self.acceleration_linear), 2),
                        "average_angular_acceleration": round(np.average(self.acceleration_angular), 2),
                        "num_social_spaces": self.social_space_counter,
                        "num_personal_spaces": self.personal_space_counter,
                        "num_intimate_spaces": self.intimate_space_counter,
                        "average_sei": round(np.average(self.sei_), 2)
                    }
                )
            rospy.loginfo("Metrics for test saved.")
            csvfile_write.close()
            csvfile_read.close()

    def __init__(self):
        rospy.init_node("social_nav_metrics_recorder", anonymous=True)

        rospy.on_shutdown(self.save_value_csv)

        # ! RECORDING VARIABLES
        # POSITIONS
        self.robot_position_ = None
        self.agent_states_ = None
        self.waypoint_distance = 0
        self.total_distance_ = 0
        self.path_efficiency_ = 0

        # ROBOT VELOCITIES
        self.robot_velocities_ = None

        # SOCIAL NAVIGATION COMMON METRICS
        self.rmi_ = np.array([], dtype=np.float64)
        self.sii_ = np.array([], dtype=np.float64)
        self.num_nodes_ = np.array([], dtype=np.int32)
        self.collision_counter_ = 0
        self.collision_agent_counter_ = 0
        self.collision_environment_counter_ = 0
        self.goal_reached_ = 0
        self.current_cpu_ = None
        self.current_num_nodes_ = None
        self.goal_position_ = None

        self.acceleration_linear = np.array([], dtype=np.float32)
        self.acceleration_angular = np.array([], dtype=np.float32)

        self.points_list = []

        self.intimate_space_counter = 0 # 0.45m of a person
        self.personal_space_counter = 0 # 1.2m of a person
        self.social_space_counter = 0 # 1.2m of a person)

        self.sei_ = np.array([], dtype=np.float64)
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

        # Distance of robot to agent
        self.euc_value = lambda x_1, y_1, x_2, y_2: (
            math.sqrt(
                    math.pow(
                        (x_2 - x_1),
                        2,
                    )
                    + math.pow(
                        (y_2 - y_1),
                        2,
                    )
            )
        )

        # GOAL FLAG
        self.goal_available_ = False

        # TIME VARIABLES
        self.total_time_ = 0.0
        self.init_query_time_ = 0.0
        self.current_time_ = 0.0
        self.last_time_ = 0
        self.current_nano_time = 0.0

        self.cpu_list_ = np.array([], dtype=np.float64)
        self.velocity_linear_list_ = np.array([], dtype=np.float64)
        self.velocity_angular_list_ = np.array([], dtype=np.float64)
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
        self.goal_position_topic_ = rospy.get_param(
            "~goal_position_topic", "/smf_move_base_planner/query_goal_pose_rviz"
        )
        self.acceleration_monitor_topic_ = rospy.get_param(
            "~acceleration_monitor_topic", "/acceleration_monitor"
        )

        # ? RATE PARAM
        self.measure_rate_ = rospy.get_param("~measure_rate", 5)
        self.measure_period_ = 1 / self.measure_rate_
        self.sim = rospy.get_param("~sim", True)

        # ? CSV SAVING PARAMS
        self.csv_dir_ = rospy.get_param("~csv_dir")
        self.approach_name_ = rospy.get_param("~approach_name")
        self.csv_name_ = rospy.get_param("~csv_name")

        self.robot_radius = rospy.get_param("~robot_radius", 0.3)
        self.agent_radius = rospy.get_param("~agent_radius", 0.45)

        self.robot_max_velocity = rospy.get_param("~robot_max_velocity", 0.22)
        self.agent_max_velocity = rospy.get_param("~agent_max_velocity", 0.5)
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
            self.collision_counter_topic_, Int32MultiArray, self.collision_counter_callback
        )
        rospy.Subscriber(
            self.acceleration_monitor_topic_, Float32MultiArray, self.acceleration_monitor_callback
        )
        rospy.logwarn(self.goal_position_topic_)
        if self.goal_position_topic_ == "/move_base/goal":
            rospy.Subscriber(self.goal_position_topic_, MoveBaseActionGoal, self.goal_position_callback)
        else:
            rospy.Subscriber(self.goal_position_topic_, PoseStamped, self.goal_position_callback)
        # ======================================================

    # ! CALLBACKS
    # ===============================================

    def goal_callback(self, goal_available: Bool):
        """Receives if the goal for the navigation query is already available."""
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
        self.current_time_ = msg.clock.secs
        self.current_nano_time = msg.clock.nsecs

    def cpu_callback(self, msg):
        """Listens to the CPU power used by the navigation system"""
        if self.goal_available_:
            self.current_cpu_ = msg.data

    def collision_counter_callback(self, array: Int32MultiArray):
        """Listens to the amount of collisions happening by an external node."""
        self.collision_counter_ = array.data[0]
        self.collision_agent_counter_ = array.data[1]
        self.collision_environment_counter_ = array.data[2]

    def num_nodes_callback(self, msg):
        """Listens to the number of nodes sampled for the case of sampling based techniques"""
        self.current_num_nodes_ = msg.data

    def odom_callback(self, odom: Odometry):
        """Listens to the odometry of the robot."""
        current_position = odom.pose.pose.position
        new_linear_velocity = odom.twist.twist.linear
        new_angular_velocity = odom.twist.twist.angular

        if self.robot_position_ is not None:
            # euc = self.euc_value(
            #         current_position.x,
            #         current_position.y,
            #         self.robot_position_.pose.position.x,
            #         self.robot_position_.pose.position.y
            #     )
            # self.total_distance_ += euc
            self.points_list.append([current_position.x, current_position.y])

        if self.robot_velocities_ is not None:
            lin_val = math.sqrt(math.pow(new_linear_velocity.x, 2) + math.pow(new_linear_velocity.y, 2) + math.pow(new_linear_velocity.z, 2))
            ang_val = math.sqrt(math.pow(new_angular_velocity.x, 2) + math.pow(new_angular_velocity.y, 2) + math.pow(new_angular_velocity.z, 2))
            self.velocity_linear_list_ = np.append(self.velocity_linear_list_, lin_val)
            self.velocity_angular_list_ = np.append(self.velocity_angular_list_, ang_val)

        self.robot_position_ = odom.pose
        self.robot_velocities_ = odom.twist

    def agents_callback(self, agents: AgentStates):
        """Listens to the states of social agents"""
        self.agent_states_ = agents.agent_states

    def goal_position_callback(self, pose: PoseStamped):
        """Listens to the goal position"""
        if self.goal_position_topic_ == "/move_base/goal":
            self.goal_position_ = pose.goal.target_pose
        else:
            self.goal_position_ = pose

    def acceleration_monitor_callback(self, array: Float32MultiArray):
        """Listens to the acceleration of the robot"""
        self.acceleration_linear = np.append(self.acceleration_linear, array.data[0])
        self.acceleration_angular = np.append(self.acceleration_angular, array.data[1])

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

    # ========================================================

    def calculate_sei(self):
        """Calculates the social effort index according to the robot
        and surrounding social agents"""

        last_sei = 0

        for agent in self.agent_states_:

            v_r_max = self.robot_max_velocity
            v_p_max = self.agent_max_velocity
            v_p_r = 0
            v_r_p = 0
            d = 0
            d_min = self.robot_radius + self.agent_radius

            d = self.euc_value(
                agent.pose.position.x,
                agent.pose.position.y,
                self.robot_position_.pose.position.x,
                self.robot_position_.pose.position.y,
            )

            v_r = np.sqrt(
                    math.pow(self.robot_velocities_.twist.linear.x, 2)
                    + math.pow(self.robot_velocities_.twist.linear.y, 2)
                )
            
            v_p = np.sqrt(
                    math.pow(agent.twist.linear.x, 2) + math.pow(agent.twist.linear.y, 2)
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

            v_p_r = v_r * np.cos(beta)
            v_r_p = v_p * np.cos(alpha)

            p1_1 = -( ((v_p_r - v_r_max) * (v_r_p + v_p_max)) / math.pow(v_p_r + v_r_p, 2) )

            if p1_1 >= -208 and p1_1 <= 370:
                p1_1 = math.exp(-( ((v_p_r - v_r_max) * (v_r_p + v_p_max)) / math.pow(v_p_r + v_r_p, 2)))
            else:
                p1_1 = 0

            p1 = (2 / (1 + p1_1))
            p2 = (1 / (1 + math.exp( -((10 / v_p_max) * (v_p_r - v_r_max / 4)))))
            p3 = (1 / (d + d_min))

            current_sei = p1 * p2 * p3

            if current_sei > last_sei:
                last_sei = current_sei

        return last_sei

    def personal_space(self):
        "Checks to see if the robot is within a specific personal-space range"
        self.initialize_ps_list()
        for count, agent in enumerate(self.agent_states_, 1):
            euc_distance = self.euc_value(
                agent.pose.position.x,
                agent.pose.position.y,
                self.robot_position_.pose.position.x,
                self.robot_position_.pose.position.y,
            )
            if euc_distance > 1.2 and euc_distance < 3.6:
                self.personal_space_list[(count*3)-3] = 1
            elif euc_distance > 0.45 and euc_distance < 1.2:
                self.personal_space_list[(count*3)-2] = 1
            elif euc_distance < 0.45:
                self.personal_space_list[(count*3)-1] = 1

    def initialize_ps_list(self):
        """Initialises a list to hold values of personal space ranges
        for each agent"""
        self.personal_space_list = []
        for agent in self.agent_states_:
            for i in range(3):
                self.personal_space_list.append(0)

    def personal_space_check(self):
        """Checks the list to see if there has been any changes to the
        robot being in a specified space range"""
        if self.personal_space_list_check != self.personal_space_list:
            for i in range(len(self.personal_space_list)):
                if self.personal_space_list[i] > self.personal_space_list_check[i]:
                    if (i % 3) == 0:
                        self.social_space_counter += 1
                    elif (i % 3) == 1:
                        self.personal_space_counter += 1
                    else:
                        self.intimate_space_counter += 1

    def path_efficiency(self, distance_direct, distance_actual):
        """Calculates the ratio of the distance between 2 way points 
        and the actual distance of the path used by the robot.
        Higher value is better"""

        path_efficiency = distance_direct/distance_actual

        return path_efficiency

    def run(self):
        """Manages the time passed and recording of the metrics"""

        first_iteration = False

        while not rospy.is_shutdown():
            if self.goal_available_:
                if (first_iteration == False) and (self.goal_position_ != None):
                    first_iteration = True
                    self.waypoint_distance = self.euc_value(
                        self.goal_position_.pose.position.x,
                        self.goal_position_.pose.position.y,
                        self.robot_position_.pose.position.x,
                        self.robot_position_.pose.position.y,
                    )
                    self.initialize_ps_list()
                if not self.sim:
                    self.current_time_ = time.time()
                if self.current_time_ - self.last_time_ >= self.measure_period_:
                    if self.current_cpu_:
                        self.cpu_list_ = np.append(self.cpu_list_, self.current_cpu_)
                    if self.current_num_nodes_:
                        self.num_nodes_ = np.append(
                            self.num_nodes_, self.current_num_nodes_
                        )
                    if (
                        self.robot_velocities_
                        and self.robot_position_
                        and self.agent_states_
                    ):
                        rmi = self.calculate_rmi()
                        self.rmi_ = np.append(self.rmi_, rmi)
                        sii = self.calculate_sii()
                        self.sii_ = np.append(self.sii_, sii)
                        if first_iteration == True:
                            self.personal_space_list_check = self.personal_space_list.copy()
                            self.personal_space()
                            self.personal_space_check()
                            sei = self.calculate_sei()
                            self.sei_ = np.append(self.sei_, sei)
                    self.last_time_ = self.current_time_
                    rospy.sleep(0.00001)

    # def test_run(self):
    #     first_iteration = False
    #     while not rospy.is_shutdown():
    #         if (first_iteration == False) and (self.goal_position_ != None):
    #             first_iteration = True
    #             self.waypoint_distance = self.euc_value(
    #                 self.goal_position_.pose.position.x,
    #                 self.goal_position_.pose.position.y,
    #                 self.robot_position_.pose.position.x,
    #                 self.robot_position_.pose.position.y,
    #             )
    #             self.initialize_ps_list()
    #         if self.goal_available_ == True:
    #             if (
    #                 self.robot_velocities_
    #                 and self.robot_position_
    #                 and self.agent_states_
    #             ):  
    #                 if first_iteration == True:
    #                     self.personal_space_list_check = self.personal_space_list.copy()
    #                     self.personal_space()
    #                     self.personal_space_check()
    #                     sei = self.calculate_sei()
    #                     rospy.logwarn(sei)
    #                     self.sei_ = np.append(self.sei_, sei)
    #                 rospy.sleep(0.00001)
    #         if self.goal_reached_:
    #             self.path_efficiency_ = self.path_efficiency(self.waypoint_distance, self.total_distance_)
            

if __name__ == "__main__":
    csv_counter_saver = MetricsRecorder()
    csv_counter_saver.run()
    #csv_counter_saver.test_run()
