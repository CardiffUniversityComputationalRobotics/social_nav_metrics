#!/usr/bin/env python3

import rospy
import math
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray

class Acceleration:

    def __init__(self):
        self.current_linear_velocity = 0
        self.last_linear_velocity = 0
        self.current_angular_velocity = 0
        self.last_angular_velocity = 0
        self.robot_velocities = None

        self.linear_acceleration = 0
        self.angular_acceleration = 0

        self.last_time = 0

        rospy.init_node("acceleration_monitor", anonymous=True)

        self.pub = rospy.Publisher('acceleration_monitor', Float32MultiArray, queue_size=10)

        self.odom_topic_ = rospy.get_param("~odom_topic", "/ground_truth")

        rospy.Subscriber(self.odom_topic_, Odometry, self.odom_callback)


    def odom_callback(self, odom: Odometry):
        new_linear_velocity = odom.twist.twist.linear
        new_angular_velocity = odom.twist.twist.angular

        if self.robot_velocities is not None:
            self.current_linear_velocity = math.sqrt(math.pow(new_linear_velocity.x, 2) + math.pow(new_linear_velocity.y, 2) + math.pow(new_linear_velocity.z, 2))
            self.current_angular_velocity = math.sqrt(math.pow(new_angular_velocity.x, 2) + math.pow(new_angular_velocity.y, 2) + math.pow(new_angular_velocity.z, 2))
            self.calculate_acceleration()
        
        self.robot_velocities = odom.twist
        self.last_linear_velocity = self.current_linear_velocity
        self.last_angular_velocity = self.current_angular_velocity
        

    def calculate_acceleration(self):
        current_time = rospy.get_time()
        dt = current_time - self.last_time
        if dt > 0 and self.last_linear_velocity != 0 and self.last_angular_velocity != 0:
            self.msg = Float32MultiArray()
            linear_acceleration = abs(self.current_linear_velocity - self.last_linear_velocity) / dt
            angular_acceleration = abs(self.current_angular_velocity - self.last_angular_velocity) / dt

            self.msg.data = [linear_acceleration, angular_acceleration]
            self.pub.publish(self.msg)

        self.last_time = current_time

    def run(self):
        while not rospy.is_shutdown():
            rospy.sleep(0.00001)


if __name__ == "__main__":
    acceleration = Acceleration()
    acceleration.run()