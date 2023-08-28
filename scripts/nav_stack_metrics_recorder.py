#!/usr/bin/env python3

import csv
from datetime import datetime

import rospy
from std_msgs.msg import Float32, Int32, Bool
import numpy as np
from rosgraph_msgs.msg import Clock


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
                self.csv_dir + self.solution_type + "/" + self.csv_name
            )
            last_data = csv_read_data[-1]
        except Exception as _e:
            pass
            # print(_e)

        rospy.loginfo("csv imported")
        rospy.loginfo(self.csv_dir + self.solution_type + "/" + self.csv_name)
        rospy.loginfo(last_data)

        with open(
            self.csv_dir + self.solution_type + "/" + self.csv_name, "a", newline=""
        ) as csvfile_write, open(
            self.csv_dir + self.solution_type + "/" + self.csv_name,
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

            print("goal_reached: ", self.goal_reached)

            if last_data is not None:
                if (
                    self.solution_type != "smf_planner"
                    and self.solution_type != "esc_planner"
                ):
                    self.num_nodes = 0

                writer.writerow(
                    {
                        "test_number": int(last_data[1]) + 1,
                        "time": dt_string,
                        "goal_reached": self.goal_reached,
                        "average_sii": round(np.average(self.sii), 2),
                        "average_rmi": round(np.average(self.rmi), 2),
                        "total_time": self.total_time,
                        "average_cpu": round(np.average(self.cpu), 2),
                        "collision_counter": self.collision_counter,
                        "num_nodes": int(np.average(self.num_nodes)),
                    }
                )
            else:
                if (
                    self.solution_type != "smf_planner"
                    and self.solution_type != "esc_planner"
                ):
                    self.num_nodes = 0

                writer.writerow(
                    {
                        "test_number": 1,
                        "time": dt_string,
                        "goal_reached": self.goal_reached,
                        "average_sii": round(np.average(self.sii), 2),
                        "average_rmi": round(np.average(self.rmi), 2),
                        "total_time": self.total_time,
                        "average_cpu": round(np.average(self.cpu), 2),
                        "collision_counter": self.collision_counter,
                        "num_nodes": int(np.average(self.num_nodes)),
                    }
                )
            print("[INFO] [" + str(rospy.get_time()) + "] metrics for test saved.")
            csvfile_write.close()
            csvfile_read.close()

    def __init__(self):

        rospy.init_node("metrics_recorder", anonymous=True)

        rospy.on_shutdown(self.save_value_csv)

        # arrays for the metrics values to be stored

        self.rmi = np.array([], dtype=np.float64)

        self.sii = np.array([], dtype=np.float64)

        self.num_nodes = np.array([], dtype=np.int32)

        self.goal_reached = 0

        self.goal_available = False

        self.total_time = 0.0

        self.current_time = 0.0

        self.cpu = np.array([], dtype=np.float64)

        self.collision_counter = 0

        # ! configs values

        self.clock_topic = rospy.get_param("~clock_topic", "/clock")
        self.cpu_topic = rospy.get_param("~cpu_topic", "/cpu_monitor/planner/cpu")
        self.result_topic = rospy.get_param("~result_topic", "/result_topic")
        self.goal_topic = rospy.get_param("~goal_topic", "/goal_topic")
        self.collision_counter_topic = rospy.get_param(
            "~collision_counter_topic", "/collision_counter"
        )
        self.num_nodes_topic = rospy.get_param("~num_nodes_topic", "/num_nodes_topic")

        self.csv_dir = rospy.get_param("~csv_dir")
        self.solution_type = rospy.get_param("~solution_type")
        self.csv_name = rospy.get_param("~csv_name")

        #! subcribers

        rospy.Subscriber(
            self.clock_topic,
            Clock,
            self.clock_callback,
            queue_size=1,
        )

        rospy.Subscriber(
            self.cpu_topic,
            Float32,
            self.cpu_callback,
            queue_size=1,
        )

        if self.solution_type == "smf_planner":
            from smf_move_base_msgs.msg import Goto2DActionGoal, Goto2DActionResult

            rospy.Subscriber(
                self.result_topic,
                Goto2DActionResult,
                self.sample_goal_reached_callback,
                queue_size=1,
            )

            rospy.Subscriber(
                self.goal_topic,
                Goto2DActionGoal,
                self.sample_goal_available_callback,
                queue_size=1,
            )

        elif self.solution_type == "esc_planner":

            from esc_move_base_msgs.msg import Goto2DActionGoal, Goto2DActionResult

            rospy.Subscriber(
                self.result_topic,
                Goto2DActionResult,
                self.sample_goal_reached_callback,
                queue_size=1,
            )

            rospy.Subscriber(
                self.goal_topic,
                Goto2DActionGoal,
                self.sample_goal_available_callback,
                queue_size=1,
            )

        elif self.solution_type == "sfm_planner":
            from sfm_diff_drive.msg import SFMDriveActionResult, SFMDriveActionFeedback

            rospy.Subscriber(
                self.result_topic,
                SFMDriveActionResult,
                self.apf_goal_reached_callback,
                queue_size=1,
            )

            rospy.Subscriber(
                self.goal_topic,
                SFMDriveActionFeedback,
                self.apf_goal_available_callback,
                queue_size=1,
            )

        elif self.solution_type == "psmm_planner":
            from psmm_drive.msg import PSMMDriveActionResult, PSMMDriveActionFeedback

            rospy.Subscriber(
                self.result_topic,
                PSMMDriveActionResult,
                self.apf_goal_reached_callback,
                queue_size=1,
            )

            rospy.Subscriber(
                self.goal_topic,
                PSMMDriveActionFeedback,
                self.apf_goal_available_callback,
                queue_size=1,
            )

        elif self.solution_type == "cadrl_planner":
            from geometry_msgs.msg import PoseStamped

            rospy.Subscriber(
                self.result_topic,
                Bool,
                self.rl_goal_reached_callback,
                queue_size=1,
            )

            rospy.Subscriber(
                self.goal_topic,
                PoseStamped,
                self.rl_goal_available_callback,
                queue_size=1,
            )

        rospy.Subscriber(
            self.collision_counter_topic,
            Int32,
            self.collision_counter_callback,
            queue_size=1,
        )

        rospy.Subscriber(
            self.num_nodes_topic,
            Int32,
            self.num_nodes_callback,
            queue_size=1,
        )

    def clock_callback(self, msg):
        self.current_time = msg.clock.secs

    def sample_goal_reached_callback(self, msg):
        if msg.result.success and not self.goal_reached:
            self.goal_reached = 1
            self.total_time = abs(self.current_time - self.total_time)

    def sample_goal_available_callback(self, msg):
        if msg.goal:
            if not self.goal_available:
                self.goal_available = True
                current_time_msg = rospy.wait_for_message("clock", Clock)
                self.total_time = current_time_msg.clock.secs

    def apf_goal_reached_callback(self, msg):
        if msg.result.result == "waypoint reached" and not self.goal_reached:
            self.goal_reached = 1
            self.total_time = abs(self.current_time - self.total_time)

    def apf_goal_available_callback(self, msg):
        if msg.feedback.feedback == "robot moving":
            if not self.goal_available:
                self.goal_available = True
                current_time_msg = rospy.wait_for_message("clock", Clock)
                self.total_time = current_time_msg.clock.secs

    def rl_goal_reached_callback(self, msg):
        if msg.data and not self.goal_reached:
            self.goal_reached = 1
            self.total_time = abs(self.current_time - self.total_time)

    def rl_goal_available_callback(self, msg):
        if not self.goal_available:
            self.goal_available = True
            current_time_msg = rospy.wait_for_message("clock", Clock)
            self.total_time = current_time_msg.clock.secs

    def cpu_callback(self, msg):
        if self.goal_available:
            self.cpu = np.append(self.cpu, msg.data)

    def collision_counter_callback(self, msg):
        self.collision_counter = msg.data

    def num_nodes_callback(self, msg):
        if self.goal_available:
            self.num_nodes = np.append(self.num_nodes, msg.data)


if __name__ == "__main__":
    csv_counter_saver = MetricsRecorder()
    while not rospy.is_shutdown():
        rospy.spin()
