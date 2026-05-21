import math
import sys
import time

import numpy as np
from tf_transformations import euler_from_quaternion

MIN_SEGMENT_DISTANCE = 0.001
MIN_ANGLE_CHANGE = 0.001
SEI_DENOMINATOR_EPS = 1e-9
TTC_MAX = 10.0
TTC_EPS = 1e-9
MAX_ACC_PER_SEGMENT = 30.0


class RunningAverage:
    """Track an average without storing samples or an ever-growing sum."""

    __slots__ = ("count", "mean")

    def __init__(self):
        self.count = 0
        self.mean = 0.0

    def __len__(self):
        return min(self.count, sys.maxsize)

    def add(self, value):
        value = float(value)
        if not math.isfinite(value):
            return

        new_count = self.count + 1
        try:
            sample_weight = 1.0 / new_count
        except OverflowError:
            self.count = new_count
            return

        self.mean = math.fsum(
            (
                self.mean * (1.0 - sample_weight),
                value * sample_weight,
            )
        )
        self.count = new_count

    def average(self, default=0.0):
        if self.count == 0:
            return default
        return self.mean

def _append_metric_value(values, value):
    if hasattr(values, "add"):
        values.add(value)
        return values
    return np.append(values, value)


def _sigmoid(value):
    if math.isnan(value):
        return 0.0
    if value >= 0:
        z = math.exp(-value)
        return 1.0 / (1.0 + z)

    z = math.exp(value)
    return z / (1.0 + z)


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


def calculate_sei(recorder):
    """Calculate the social effort index for the robot and nearby agents."""
    if recorder.robot_max_velocity_ <= 0 or recorder.agent_max_velocity_ <= 0:
        return -1

    robot_speed = math.hypot(
        recorder.robot_velocities_.twist.linear.x,
        recorder.robot_velocities_.twist.linear.y,
    )

    robot_quaternion = (
        recorder.robot_position_.pose.orientation.x,
        recorder.robot_position_.pose.orientation.y,
        recorder.robot_position_.pose.orientation.z,
        recorder.robot_position_.pose.orientation.w,
    )
    robot_yaw = _normalize_angle(euler_from_quaternion(robot_quaternion)[2])

    d_min = max(recorder.robot_radius_ + recorder.agent_radius_, SEI_DENOMINATOR_EPS)
    total_sei = 0.0

    for agent in recorder.agent_states_:
        beta = math.atan2(
            agent.pose.position.y - recorder.robot_position_.pose.position.y,
            agent.pose.position.x - recorder.robot_position_.pose.position.x,
        )
        beta = _relative_angle(_normalize_angle(beta), robot_yaw)
        robot_velocity_toward_agent = robot_speed * math.cos(beta)

        agent_speed = math.hypot(agent.twist.linear.x, agent.twist.linear.y)

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
        agent_velocity_toward_robot = agent_speed * math.cos(alpha)

        distance = math.hypot(
            agent.pose.position.x - recorder.robot_position_.pose.position.x,
            agent.pose.position.y - recorder.robot_position_.pose.position.y,
        )

        velocity_sum = robot_velocity_toward_agent + agent_velocity_toward_robot
        denominator = max(math.pow(velocity_sum, 2), SEI_DENOMINATOR_EPS)

        isolation_value = (
            (robot_velocity_toward_agent - recorder.robot_max_velocity_)
            * (agent_velocity_toward_robot + recorder.agent_max_velocity_)
        ) / denominator

        p1 = 2.0 * _sigmoid(isolation_value)
        p2 = _sigmoid(
            (10.0 / recorder.agent_max_velocity_)
            * (robot_velocity_toward_agent - recorder.robot_max_velocity_ / 4.0)
        )
        p3 = 1.0 / (distance + d_min)

        current_sei = p1 * p2 * p3
        if math.isfinite(current_sei):
            total_sei += current_sei

    return total_sei


def calculate_ttc(recorder):
    """Calculate the minimum time-to-collision against nearby agents."""
    if recorder.agent_states_ is None:
        return -1

    if not recorder.agent_states_:
        return TTC_MAX

    robot_speed = math.hypot(
        recorder.robot_velocities_.twist.linear.x,
        recorder.robot_velocities_.twist.linear.y,
    )

    robot_quaternion = (
        recorder.robot_position_.pose.orientation.x,
        recorder.robot_position_.pose.orientation.y,
        recorder.robot_position_.pose.orientation.z,
        recorder.robot_position_.pose.orientation.w,
    )
    robot_yaw = euler_from_quaternion(robot_quaternion)[2]
    robot_vx = robot_speed * math.cos(robot_yaw)
    robot_vy = robot_speed * math.sin(robot_yaw)

    collision_radius = max(
        recorder.robot_radius_ + recorder.agent_radius_,
        TTC_EPS,
    )
    min_ttc = TTC_MAX

    for agent in recorder.agent_states_:
        agent_speed = math.hypot(agent.twist.linear.x, agent.twist.linear.y)

        agent_quaternion = (
            agent.pose.orientation.x,
            agent.pose.orientation.y,
            agent.pose.orientation.z,
            agent.pose.orientation.w,
        )
        agent_yaw = euler_from_quaternion(agent_quaternion)[2]
        agent_vx = agent_speed * math.cos(agent_yaw)
        agent_vy = agent_speed * math.sin(agent_yaw)

        px = agent.pose.position.x - recorder.robot_position_.pose.position.x
        py = agent.pose.position.y - recorder.robot_position_.pose.position.y
        vx = agent_vx - robot_vx
        vy = agent_vy - robot_vy

        a = vx * vx + vy * vy
        b = 2.0 * (px * vx + py * vy)
        c = px * px + py * py - collision_radius * collision_radius

        if c <= 0.0:
            current_ttc = 0.0
        elif a < TTC_EPS:
            current_ttc = TTC_MAX
        else:
            discriminant = b * b - 4.0 * a * c
            if discriminant < 0.0:
                current_ttc = TTC_MAX
            else:
                sqrt_discriminant = math.sqrt(discriminant)
                t1 = (-b - sqrt_discriminant) / (2.0 * a)
                t2 = (-b + sqrt_discriminant) / (2.0 * a)
                future_times = [
                    collision_time
                    for collision_time in (t1, t2)
                    if collision_time >= 0.0
                ]
                current_ttc = min(future_times) if future_times else TTC_MAX

        if math.isfinite(current_ttc):
            min_ttc = min(min_ttc, current_ttc, TTC_MAX)

    return min_ttc


def update_path_irregularity(recorder, distance_change):
    """Accumulate heading changes for the path irregularity metric."""
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
        math.atan2(math.sin(new_angle - old_angle), math.cos(new_angle - old_angle))
    )

    if distance_change < MIN_SEGMENT_DISTANCE and angle_change < MIN_ANGLE_CHANGE:
        return

    recorder.orientation_change_ += angle_change


def calculate_acc_per_segment(recorder, distance_change):
    """Calculate linear and angular acceleration per travelled segment."""
    past_robot_time = getattr(recorder, "past_robot_time_", None)
    if past_robot_time is None:
        return -1

    dt = recorder.current_time_ - past_robot_time
    if dt <= 0:
        return -1

    acceleration_x = (
        recorder.robot_velocities_.twist.linear.x
        - recorder.past_robot_velocities_.twist.linear.x
    ) / dt
    acceleration_y = (
        recorder.robot_velocities_.twist.linear.y
        - recorder.past_robot_velocities_.twist.linear.y
    ) / dt
    angular_acceleration_z = (
        recorder.robot_velocities_.twist.angular.z
        - recorder.past_robot_velocities_.twist.angular.z
    ) / dt

    res_acceleration = math.sqrt(
        math.pow(acceleration_x, 2)
        + math.pow(acceleration_y, 2)
        + math.pow(angular_acceleration_z, 2)
    )

    if distance_change > MIN_SEGMENT_DISTANCE:
        acc_per_segment = float(res_acceleration / distance_change)
    else:
        acc_per_segment = -1

    if acc_per_segment > MAX_ACC_PER_SEGMENT:
        acc_per_segment = MAX_ACC_PER_SEGMENT

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

    has_robot_position = recorder.robot_position_ is not None
    has_robot_velocity = recorder.robot_velocities_ is not None

    if has_robot_velocity and has_robot_position and recorder.agent_states_ is not None:
        sei = calculate_sei(recorder)
        if sei >= 0:
            recorder.sei_ = _append_metric_value(recorder.sei_, sei)

        ttc = calculate_ttc(recorder)
        if ttc >= 0:
            recorder.ttc_ = _append_metric_value(recorder.ttc_, ttc)

        if recorder.agent_states_:
            recorder.rmi_ = _append_metric_value(recorder.rmi_, calculate_rmi(recorder))
            recorder.sii_ = _append_metric_value(recorder.sii_, calculate_sii(recorder))

    if has_robot_position:
        if recorder.past_robot_position_:
            dx = (
                recorder.past_robot_position_.pose.position.x
                - recorder.robot_position_.pose.position.x
            )
            dy = (
                recorder.past_robot_position_.pose.position.y
                - recorder.robot_position_.pose.position.y
            )
            distance_change = math.hypot(dx, dy)

            recorder.path_length_ += distance_change
            update_path_irregularity(recorder, distance_change)

            if has_robot_velocity and recorder.past_robot_velocities_:
                acc_per_segment = calculate_acc_per_segment(recorder, distance_change)
                if acc_per_segment >= 0:
                    recorder.acceleration_per_segment_ = _append_metric_value(
                        recorder.acceleration_per_segment_, acc_per_segment
                    )

        recorder.past_robot_position_ = recorder.robot_position_

        if has_robot_velocity:
            recorder.past_robot_velocities_ = recorder.robot_velocities_
            recorder.past_robot_time_ = recorder.current_time_
        else:
            recorder.past_robot_velocities_ = None
            recorder.past_robot_time_ = None

    elif has_robot_velocity:
        recorder.past_robot_velocities_ = recorder.robot_velocities_
        recorder.past_robot_time_ = recorder.current_time_

    recorder.last_time_ = recorder.current_time_
