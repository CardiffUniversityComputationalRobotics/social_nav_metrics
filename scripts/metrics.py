import math
import time

import numpy as np
from tf_transformations import euler_from_quaternion

MAX_STORED_SAMPLES = 1000


def _safe_average(values, default=0.0):
    if len(values) == 0:
        return default
    return float(np.average(values))


def _append_metric_value(values, value):
    if len(values) > MAX_STORED_SAMPLES:
        values = np.array([np.average(values)], dtype=np.float64)
    return np.append(values, value)


def _normalize_angle(angle):
    if angle < 0:
        angle = 2 * math.pi + angle
    return angle


def _relative_angle(reference_angle, yaw):
    if reference_angle > (yaw + math.pi):
        return abs(yaw + 2 * math.pi - reference_angle)
    if yaw > (reference_angle + math.pi):
        return abs(reference_angle + 2 * math.pi - yaw)
    return abs(reference_angle - yaw)


def calculate_rmi(recorder):
    """Calculate the relative motion index for the robot and nearby agents."""
    last_rmi = 0

    v_r = np.sqrt(
        math.pow(recorder.robot_velocities_.twist.linear.x, 2)
        + math.pow(recorder.robot_velocities_.twist.linear.y, 2)
    )

    robot_quaternion = (
        recorder.robot_position_.pose.orientation.x,
        recorder.robot_position_.pose.orientation.y,
        recorder.robot_position_.pose.orientation.z,
        recorder.robot_position_.pose.orientation.w,
    )
    robot_yaw = _normalize_angle(euler_from_quaternion(robot_quaternion)[2])

    for agent in recorder.agent_states_:
        beta = math.atan2(
            agent.pose.position.y - recorder.robot_position_.pose.position.y,
            agent.pose.position.x - recorder.robot_position_.pose.position.x,
        )
        beta = _relative_angle(_normalize_angle(beta), robot_yaw)

        v_a = np.sqrt(
            math.pow(agent.twist.linear.x, 2) + math.pow(agent.twist.linear.y, 2)
        )

        alpha = math.atan2(
            recorder.robot_position_.pose.position.y - agent.pose.position.y,
            recorder.robot_position_.pose.position.x - agent.pose.position.x,
        )

        agent_quaternion = (
            agent.pose.orientation.x,
            agent.pose.orientation.y,
            agent.pose.orientation.z,
            agent.pose.orientation.w,
        )
        agent_yaw = _normalize_angle(euler_from_quaternion(agent_quaternion)[2])
        alpha = _relative_angle(_normalize_angle(alpha), agent_yaw)

        distance = np.sqrt(
            math.pow(
                agent.pose.position.x - recorder.robot_position_.pose.position.x, 2
            )
            + math.pow(
                agent.pose.position.y - recorder.robot_position_.pose.position.y, 2
            )
        )

        current_rmi = (2 + v_r * np.cos(beta) + v_a * np.cos(alpha)) / distance

        if current_rmi > last_rmi:
            last_rmi = current_rmi

    return last_rmi


def calculate_sii(recorder):
    """Calculate the social individual index for the robot and nearby agents."""
    last_sii = 0

    for agent in recorder.agent_states_:
        current_sii = math.pow(
            math.e,
            -(
                math.pow(
                    (recorder.robot_position_.pose.position.x - agent.pose.position.x)
                    / recorder.final_sigma,
                    2,
                )
                + math.pow(
                    (recorder.robot_position_.pose.position.y - agent.pose.position.y)
                    / recorder.final_sigma,
                    2,
                )
            ),
        )
        if current_sii > last_sii:
            last_sii = current_sii

    return last_sii


def calculate_path_irregularity(recorder):
    """Calculate path irregularity at a specific time."""
    distance_change = math.sqrt(
        math.pow(
            recorder.past_robot_position_.pose.position.x
            - recorder.robot_position_.pose.position.x,
            2,
        )
        + math.pow(
            recorder.past_robot_position_.pose.position.y
            - recorder.robot_position_.pose.position.y,
            2,
        )
    )

    old_q = (
        recorder.past_robot_position_.pose.orientation.x,
        recorder.past_robot_position_.pose.orientation.y,
        recorder.past_robot_position_.pose.orientation.z,
        recorder.past_robot_position_.pose.orientation.w,
    )
    old_angle = euler_from_quaternion(old_q)[2]

    new_q = (
        recorder.robot_position_.pose.orientation.x,
        recorder.robot_position_.pose.orientation.y,
        recorder.robot_position_.pose.orientation.z,
        recorder.robot_position_.pose.orientation.w,
    )
    new_angle = euler_from_quaternion(new_q)[2]

    angle_change = abs(
        min((2 * math.pi) - abs(old_angle - new_angle), abs(old_angle - new_angle))
    )

    if distance_change < 0.001 and angle_change < 0.001:
        path_irregularity = -1
    elif distance_change < 0.001:
        path_irregularity = float(angle_change / 0.001)
    else:
        path_irregularity = float(angle_change / distance_change)

    if path_irregularity > 10:
        path_irregularity = 10

    return path_irregularity


def calculate_acc_per_segment(recorder):
    """Calculate acceleration per travelled segment."""
    acceleration_x = (
        recorder.past_robot_velocities_.twist.linear.x
        - recorder.robot_velocities_.twist.linear.x
    ) / recorder.measure_period_
    acceleration_y = (
        recorder.past_robot_velocities_.twist.linear.y
        - recorder.robot_velocities_.twist.linear.y
    ) / recorder.measure_period_

    res_acceleration = math.sqrt(
        math.pow(acceleration_x, 2) + math.pow(acceleration_y, 2)
    )

    distance_change = math.sqrt(
        math.pow(
            recorder.past_robot_position_.pose.position.x
            - recorder.robot_position_.pose.position.x,
            2,
        )
        + math.pow(
            recorder.past_robot_position_.pose.position.y
            - recorder.robot_position_.pose.position.y,
            2,
        )
    )

    recorder.path_length_ += distance_change

    if distance_change > 0.001:
        acc_per_segment = float(res_acceleration / distance_change)
    else:
        acc_per_segment = -1

    if acc_per_segment > 30:
        acc_per_segment = 30

    return acc_per_segment


def measure_values(recorder):
    """Manage elapsed time and record the measured metrics."""
    if not recorder.goal_available_:
        return

    if not recorder.sim:
        recorder.current_time_ = time.time()

    if recorder.current_time_ - recorder.last_time_ < recorder.measure_period_:
        return

    if recorder.current_num_nodes_ is not None:
        recorder.num_nodes_ = _append_metric_value(
            recorder.num_nodes_, recorder.current_num_nodes_
        )

    if recorder.robot_velocities_ and recorder.robot_position_ and recorder.agent_states_:
        recorder.rmi_ = _append_metric_value(recorder.rmi_, calculate_rmi(recorder))
        recorder.sii_ = _append_metric_value(recorder.sii_, calculate_sii(recorder))

        if recorder.past_robot_position_ and recorder.past_robot_velocities_:
            path_irregularity = calculate_path_irregularity(recorder)
            if path_irregularity >= 0:
                recorder.path_irregularity_ = _append_metric_value(
                    recorder.path_irregularity_, path_irregularity
                )

            acc_per_segment = calculate_acc_per_segment(recorder)
            if acc_per_segment >= 0:
                recorder.acceleration_per_segment_ = _append_metric_value(
                    recorder.acceleration_per_segment_, acc_per_segment
                )

        recorder.past_robot_position_ = recorder.robot_position_
        recorder.past_robot_velocities_ = recorder.robot_velocities_

    recorder.last_time_ = recorder.current_time_
