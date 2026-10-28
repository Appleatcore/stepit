#!/usr/bin/python3
"""Print the Go2W command/action chain and actor observation slices."""

import argparse
import math
from typing import Iterable, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray


WHEEL_JOINTS = [
    "FR_foot_joint",
    "FL_foot_joint",
    "RR_foot_joint",
    "RL_foot_joint",
]

THIGH_JOINTS = [
    "FR_thigh_joint",
    "FL_thigh_joint",
    "RR_thigh_joint",
    "RL_thigh_joint",
]

THIGH_INDICES = [1, 4, 7, 10]

LEG_DEFAULT = [
    0.0, 0.8, -1.5,
    0.0, 0.8, -1.5,
    0.0, 0.8, -1.5,
    0.0, 0.8, -1.5,
]

OBS_FIELDS = [
    ("ang_vel", "ang_vel", 0, 3),
    ("projected_gravity", "gravity", 3, 6),
    ("cmd_vel", "cmd_vel", 6, 9),
    ("joint_pos", "joint_pos", 9, 25),
    ("joint_vel", "joint_vel", 25, 41),
    ("last_action", "last_action", 41, 57),
    ("height_scan", "heightmap", 57, 244),
]


def fmt(values: Optional[Iterable[float]]) -> str:
    if values is None:
        return "None"
    return "[" + ", ".join(f"{float(value): .3f}" for value in values) + "]"


def fmt_values(values: Iterable[float], limit: Optional[int] = None) -> str:
    values = list(values)
    if limit is None or len(values) <= limit:
        return fmt(values)
    head_count = max(1, limit // 2)
    tail_count = max(1, limit - head_count)
    head = ", ".join(f"{float(value): .3f}" for value in values[:head_count])
    tail = ", ".join(f"{float(value): .3f}" for value in values[-tail_count:])
    return f"[{head}, ... , {tail}]"


def stats(values: Optional[Iterable[float]]) -> str:
    if values is None:
        return "missing"

    values = list(values)
    finite = [float(value) for value in values if math.isfinite(float(value))]
    nan_count = len(values) - len(finite)
    if not finite:
        return f"n={len(values)} finite=0 nan={nan_count}"

    mean = sum(finite) / len(finite)
    var = sum((value - mean) ** 2 for value in finite) / len(finite)
    return (
        f"n={len(values)} mean={mean:.6f} std={math.sqrt(var):.6f} "
        f"min={min(finite):.6f} max={max(finite):.6f} nan={nan_count}"
    )


def extract(msg: JointState, names: Iterable[str], field: str) -> Optional[list[float]]:
    name_to_index = {name: index for index, name in enumerate(msg.name)}
    values = getattr(msg, field)
    output = []
    for name in names:
        index = name_to_index.get(name)
        if index is None or index >= len(values):
            return None
        output.append(values[index])
    return output


def subtract(lhs: Optional[Iterable[float]], rhs: Optional[Iterable[float]]) -> Optional[list[float]]:
    if lhs is None or rhs is None:
        return None
    return [left - right for left, right in zip(lhs, rhs)]


class Go2WDebugNode(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("go2w_stepit_io_debug")
        qos = QoSProfile(depth=10)
        qos.history = HistoryPolicy.KEEP_LAST
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.cmd_vel = None
        self.cmd_stall = None
        self.action = None
        self.ang_vel = None
        self.gravity = None
        self.joint_pos = None
        self.joint_vel = None
        self.last_action = None
        self.heightmap = None
        self.observation = None
        self.joint_state = None
        self.cmd_vel_topic = None
        self.cmd_stall_topic = None
        self.action_topic = None
        self.ang_vel_topic = None
        self.gravity_topic = None
        self.joint_pos_topic = None
        self.joint_vel_topic = None
        self.last_action_topic = None
        self.heightmap_topic = None
        self.observation_topic = None
        self.joint_topic = None
        self.obs_mode = args.obs
        self.obs_values_limit = args.obs_values_limit

        for topic in args.cmd_vel_topics:
            self.create_subscription(Float32MultiArray, topic, self.make_array_cb("cmd_vel", topic), qos)
        for topic in args.cmd_stall_topics:
            self.create_subscription(Float32MultiArray, topic, self.make_array_cb("cmd_stall", topic), qos)
        for topic in args.action_topics:
            self.create_subscription(Float32MultiArray, topic, self.make_array_cb("action", topic), qos)
        for field_name in ("ang_vel", "gravity", "joint_pos", "joint_vel", "last_action", "heightmap", "observation"):
            for topic in [f"/field/{field_name}", f"/stepit/field/{field_name}"]:
                self.create_subscription(Float32MultiArray, topic, self.make_array_cb(field_name, topic), qos)
        for topic in args.joint_topics:
            self.create_subscription(JointState, topic, self.joint_cb, qos)

        self.create_timer(args.period, self.print_snapshot)

    def make_array_cb(self, field_name: str, topic: str):
        def cb(msg: Float32MultiArray) -> None:
            setattr(self, field_name, list(msg.data))
            setattr(self, f"{field_name}_topic", topic)

        return cb

    def joint_cb(self, msg: JointState) -> None:
        self.joint_state = msg
        self.joint_topic = msg.header.frame_id or "joint_states"

    def print_snapshot(self) -> None:
        wheel_action = None
        max_leg_dev = None
        if self.action is not None and len(self.action) >= 16:
            wheel_action = self.action[12:16]
            max_leg_dev = max(abs(self.action[i] - LEG_DEFAULT[i]) for i in range(12))

        thigh_action = None
        if self.action is not None and len(self.action) >= 12:
            thigh_action = [self.action[index] for index in THIGH_INDICES]
        thigh_default_q = [LEG_DEFAULT[index] for index in THIGH_INDICES]
        thigh_state_q = self.get_thigh_state_q()
        thigh_dev = subtract(thigh_action, thigh_default_q)
        thigh_err = subtract(thigh_action, thigh_state_q)

        cmd_dq = state_dq = gain_tau = None
        if self.joint_state is not None:
            cmd_dq = extract(self.joint_state, [name + "_cmd" for name in WHEEL_JOINTS], "velocity")
            state_dq = extract(self.joint_state, [name + "_joint" for name in WHEEL_JOINTS], "velocity")
            gain_tau = extract(self.joint_state, [name + "_gain" for name in WHEEL_JOINTS], "effort")

        error = None
        if cmd_dq is not None and state_dq is not None:
            error = [cmd - state for cmd, state in zip(cmd_dq, state_dq)]

        print(
            "cmd_vel="
            f"{fmt(self.cmd_vel)} "
            f"stall={fmt(self.cmd_stall)} | "
            f"wheel_action[12:16]={fmt(wheel_action)} "
            f"leg_max_dev={max_leg_dev if max_leg_dev is not None else 'None'} | "
            f"wheel_cmd_dq={fmt(cmd_dq)} "
            f"wheel_state_dq={fmt(state_dq)} "
            f"err={fmt(error)} "
            f"tau_des={fmt(gain_tau)} | "
            f"thigh_action={fmt(thigh_action)} "
            f"thigh_state_q={fmt(thigh_state_q)} "
            f"thigh_default_q={fmt(thigh_default_q)} "
            f"thigh_dev={fmt(thigh_dev)} "
            f"thigh_err={fmt(thigh_err)}",
            flush=True,
        )
        if self.obs_mode != "off":
            self.print_observation()

    def get_thigh_state_q(self) -> Optional[list[float]]:
        observation, _ = self.build_observation()
        if observation is not None and len(observation) >= 25:
            joint_pos = observation[9:25]
            return [joint_pos[index] for index in THIGH_INDICES]

        if self.joint_pos is not None and len(self.joint_pos) >= 12:
            return [self.joint_pos[index] for index in THIGH_INDICES]

        if self.joint_state is not None:
            return extract(self.joint_state, [name + "_joint" for name in THIGH_JOINTS], "position")
        return None

    def get_obs_field(self, attr_name: str) -> Optional[list[float]]:
        value = getattr(self, attr_name)
        if value is None:
            return None
        return list(value)

    def build_observation(self) -> tuple[Optional[list[float]], str]:
        if self.observation is not None:
            return list(self.observation), "direct"

        values = []
        for _, attr_name, _, end in OBS_FIELDS:
            field_values = self.get_obs_field(attr_name)
            if field_values is None:
                return None, "fields"
            values.extend(field_values)
            if len(values) < end:
                return None, "fields"
        return values, "fields"

    def print_observation(self) -> None:
        observation, source = self.build_observation()
        if observation is None:
            missing = [name for name, attr_name, _, _ in OBS_FIELDS if self.get_obs_field(attr_name) is None]
            print(f"OBS unavailable missing={missing}", flush=True)
            return

        print(f"OBS source={source} len={len(observation)} {stats(observation)}", flush=True)
        for display_name, attr_name, start, end in OBS_FIELDS:
            values = observation[start:end] if source == "direct" else self.get_obs_field(attr_name)
            expected = end - start
            if values is None:
                print(f"  obs[{start:03d}:{end:03d}] {display_name}: missing", flush=True)
                continue
            if len(values) != expected:
                print(
                    f"  obs[{start:03d}:{end:03d}] {display_name}: len={len(values)} expected={expected} "
                    f"{stats(values)} values={fmt_values(values, self.obs_values_limit)}",
                    flush=True,
                )
                continue

            limit = None if self.obs_mode == "full" else self.obs_values_limit
            print(
                f"  obs[{start:03d}:{end:03d}] {display_name}: "
                f"{stats(values)} values={fmt_values(values, limit)}",
                flush=True,
            )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--period", type=float, default=0.2)
    parser.add_argument("--cmd-vel-topics", nargs="+", default=["/field/cmd_vel", "/stepit/field/cmd_vel"])
    parser.add_argument("--cmd-stall-topics", nargs="+", default=["/field/cmd_stall", "/stepit/field/cmd_stall"])
    parser.add_argument("--action-topics", nargs="+", default=["/field/action", "/stepit/field/action"])
    parser.add_argument("--joint-topics", nargs="+", default=["/joint_states", "/stepit/joint_states"])
    parser.add_argument("--obs", choices=["off", "summary", "full"], default="summary")
    parser.add_argument("--obs-values-limit", type=int, default=32)
    args = parser.parse_args()

    rclpy.init()
    node = Go2WDebugNode(args)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
