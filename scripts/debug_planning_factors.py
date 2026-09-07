#!/usr/bin/env python3

import argparse
from datetime import datetime

import rclpy
from rclpy.node import Node

from crane_msgs.msg import RobotCommands

DEFAULT_FACTOR_KEYS = [
    "Attacker",
    "GoalKick",
    "Kick",
    "KickDecision",
    "KickAlignDiffDeg",
    "KickRobotBallDistance",
    "KickApproachDistance",
    "AttackerSafetyStop",
    "AttackerOverDribbleDetected",
    "AttackerOverDribbleDistance",
    "RVO2ZeroVelocityReason",
    "RVO2PrefSpeed",
    "RVO2OutputSpeed",
    "RVO2DistanceToTarget",
    "RVO2PositionTolerance",
    "RVO2CollisionAvoidance",
    "RVO2AdjustFieldBoundary",
    "RVO2AdjustPenaltyArea",
    "RVO2AdjustBallAvoidance",
    "RVO2AdjustPlacementAvoidance",
    "RVO2TargetAdjustedDistance",
    "RVO2TargetFallback",
]


class PlanningFactorDebugger(Node):
    def __init__(self, robot_id: int, only_kick: bool, factor_keys: list[str]) -> None:
        super().__init__("planning_factor_debugger")
        self.robot_id = robot_id
        self.only_kick = only_kick
        self.factor_keys = factor_keys
        self.subscription = self.create_subscription(
            RobotCommands, "/robot_commands", self._callback, 10
        )
        self.get_logger().info(
            f"Monitoring /robot_commands for robot_id={robot_id} (only_kick={only_kick})"
        )

    def _callback(self, msg: RobotCommands) -> None:
        command = next(
            (c for c in msg.robot_commands if c.robot_id == self.robot_id), None
        )
        if command is None:
            return

        factors = {factor.name: factor.value for factor in command.planning_factors}
        if self.only_kick and factors.get("Kick") != "AROUND_BALL_AND_KICK":
            return

        target_x = None
        target_y = None
        if command.position_target_mode:
            target_x = command.position_target_mode[0].target_x
            target_y = command.position_target_mode[0].target_y
        vel_r = 0.0
        vel_theta = 0.0
        if command.polar_velocity_target_mode:
            vel_r = command.polar_velocity_target_mode[0].target_velocity_r
            vel_theta = command.polar_velocity_target_mode[0].target_velocity_theta
        now = datetime.now().strftime("%H:%M:%S.%f")[:-3]

        key_values = [f"{key}={factors.get(key, '-')}" for key in self.factor_keys]
        print(
            f"[{now}] robot={command.robot_id} "
            f"vel_r={vel_r:.3f} "
            f"vel_theta={vel_theta:.3f} "
            f"target=({target_x},{target_y}) "
            f"stop_flag={command.stop_flag} " + " ".join(key_values),
            flush=True,
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Debug tool for /robot_commands planning_factors"
    )
    parser.add_argument("--robot-id", type=int, default=2, help="Target robot_id")
    parser.add_argument(
        "--only-kick",
        action="store_true",
        help="Filter to Kick=AROUND_BALL_AND_KICK only",
    )
    parser.add_argument(
        "--keys",
        type=str,
        default=",".join(DEFAULT_FACTOR_KEYS),
        help="Comma-separated planning factor keys to display",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    keys = [key.strip() for key in args.keys.split(",") if key.strip()]
    rclpy.init()
    node = PlanningFactorDebugger(args.robot_id, args.only_kick, keys)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
