#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import uvicorn
from fastapi import FastAPI, HTTPException
from .models import *  # Pydanticモデルをインポート
import asyncio
from concurrent.futures import ThreadPoolExecutor
import threading


# ROS 2メッセージの型をインポート
from geometry_msgs.msg import Pose2D
from crane_msgs.msg import (
    RobotCommand,
    RobotCommands,
    LocalPlannerConfig,
    PositionTargetMode,
    VelocityTargetMode,
    SimpleVelocityTargetMode,
)


class ROS2Bridge(Node):
    def __init__(self):
        super().__init__("ros2_bridge")

        # QoSプロファイルの設定
        qos_profile = rclpy.node.QoSProfile(
            depth=10, reliability=rclpy.qos.ReliabilityPolicy.RELIABLE
        )

        # パブリッシャーの初期化（ロボットの数に応じて動的に作成）
        self.publisher = self.create_publisher(RobotCommands, "/control_targets", qos_profile)
        self.robot_commands = {}

        # ステータス更新用のタイマー
        self.create_timer(0.1, self.timer_callback)  # 10Hz

    def convert_to_ros_message(self, command_model: RobotCommandModel) -> RobotCommand:
        """PydanticモデルからROS 2メッセージに変換"""
        msg = RobotCommand()

        # 基本フィールドの設定
        msg.robot_id = command_model.robot_id
        msg.local_goalie_enable = command_model.local_goalie_enable
        msg.enable_ball_centering_control = command_model.enable_ball_centering_control
        msg.chip_enable = command_model.chip_enable
        msg.stop_flag = command_model.stop_flag
        msg.lift_up_dribbler_flag = command_model.lift_up_dribbler_flag
        msg.kick_power = command_model.kick_power
        msg.dribble_power = command_model.dribble_power
        msg.enable_local_feedback = command_model.enable_local_feedback
        msg.target_theta = command_model.target_theta
        msg.omega_limit = command_model.omega_limit
        msg.theta_tolerance = command_model.theta_tolerance
        msg.latency_ms = command_model.latency_ms
        msg.elapsed_time_ms_since_last_vision = command_model.elapsed_time_ms_since_last_vision
        msg.skill_name = command_model.skill_name

        # LocalPlannerConfigの設定
        planner_config = command_model.local_planner_config
        msg.local_planner_config.disable_collision_avoidance = (
            planner_config.disable_collision_avoidance
        )
        msg.local_planner_config.disable_goal_area_avoidance = (
            planner_config.disable_goal_area_avoidance
        )
        msg.local_planner_config.disable_placement_avoidance = (
            planner_config.disable_placement_avoidance
        )
        msg.local_planner_config.disable_ball_avoidance = planner_config.disable_ball_avoidance
        msg.local_planner_config.disable_rule_area_avoidance = (
            planner_config.disable_rule_area_avoidance
        )
        msg.local_planner_config.max_acceleration = planner_config.max_acceleration
        msg.local_planner_config.max_velocity = planner_config.max_velocity
        msg.local_planner_config.terminal_velocity = planner_config.terminal_velocity
        msg.local_planner_config.priority = planner_config.priority

        # Pose2Dの設定
        # msg.current_velocity.x = command_model.current_velocity.x
        # msg.current_velocity.y = command_model.current_velocity.y
        # msg.current_velocity.theta = command_model.current_velocity.theta
        #
        # msg.current_pose.x = command_model.current_pose.x
        # msg.current_pose.y = command_model.current_pose.y
        # msg.current_pose.theta = command_model.current_pose.theta

        # コントロールモードに応じたターゲットモードの設定
        if command_model.control_mode == "POSITION_TARGET_MODE":
            msg.control_mode = RobotCommand.POSITION_TARGET_MODE
            if command_model.position_target_mode:
                target = PositionTargetMode()
                target.target_x = command_model.position_target_mode.target_x
                target.target_y = command_model.position_target_mode.target_y
                target.position_tolerance = command_model.position_target_mode.position_tolerance
                target.speed_limit_at_target = (
                    command_model.position_target_mode.speed_limit_at_target
                )
                msg.position_target_mode = [target]

        elif command_model.control_mode == "VELOCITY_TARGET_MODE":
            msg.control_mode = RobotCommand.VELOCITY_TARGET_MODE
            if command_model.velocity_target_mode:
                target = VelocityTargetMode()
                target.target_vx = command_model.velocity_target_mode.target_vx
                target.target_vy = command_model.velocity_target_mode.target_vy
                target.traj_origin_x = command_model.velocity_target_mode.traj_origin_x
                target.traj_origin_y = command_model.velocity_target_mode.traj_origin_y
                target.traj_origin_angle = command_model.velocity_target_mode.traj_origin_angle
                target.traj_curvature = command_model.velocity_target_mode.traj_curvature
                msg.velocity_target_mode = [target]

        elif command_model.control_mode == "SIMPLE_VELOCITY_TARGET_MODE":
            msg.control_mode = RobotCommand.SIMPLE_VELOCITY_TARGET_MODE
            if command_model.simple_velocity_target_mode:
                target = SimpleVelocityTargetMode()
                target.target_vx = command_model.simple_velocity_target_mode.target_vx
                target.target_vy = command_model.simple_velocity_target_mode.target_vy
                target.speed_limit_at_target = (
                    command_model.simple_velocity_target_mode.speed_limit_at_target
                )
                msg.simple_velocity_target_mode = [target]

        return msg

    def publish_command(self, robot_id: int, command: RobotCommandModel):
        """コマンドをパブリッシュ"""
        ros_msg = self.convert_to_ros_message(command)

        commands_msg = RobotCommands()
        commands_msg.robot_commands = [ros_msg]
        commands_msg.header.stamp = self.get_clock().now().to_msg()
        commands_msg.header.frame_id = "world"
        commands_msg.on_positive_half = False
        commands_msg.is_yellow = False

        self.publisher.publish(commands_msg)
        self.robot_commands[robot_id] = command

    def timer_callback(self):
        """定期的なステータス更新"""
        # ここでロボットの状態を監視・更新
        pass


def main(args=None):
    rclpy.init(args=args)

    # FastAPIアプリケーションの作成
    app = FastAPI()
    bridge = ROS2Bridge()

    # ROSノードのスピンを別スレッドで実行
    ros_thread = threading.Thread(target=lambda: rclpy.spin(bridge))
    ros_thread.daemon = True
    ros_thread.start()

    # FastAPIエンドポイントの設定
    @app.post("/api/robots/{robot_id}/command")
    async def set_robot_command(robot_id: int, command: RobotCommandModel):
        try:
            if command.robot_id != robot_id:
                raise HTTPException(status_code=400, detail="Robot ID mismatch")
            bridge.publish_command(robot_id, command)
            return {"status": "success"}
        except Exception as e:
            raise HTTPException(status_code=500, detail=str(e))

    @app.get("/api/robots/{robot_id}/command")
    async def get_robot_command(robot_id: int):
        command = bridge.robot_commands.get(robot_id)
        if command is None:
            raise HTTPException(status_code=404, detail="Robot command not found")
        return command

    @app.post("/api/robots/{robot_id}/emergency-stop")
    async def emergency_stop(robot_id: int):
        try:
            command = bridge.robot_commands.get(robot_id)
            if command:
                command.stop_flag = True
                bridge.publish_command(robot_id, command)
            return {"status": "success"}
        except Exception as e:
            raise HTTPException(status_code=500, detail=str(e))

    bridge.declare_parameter("web_server_host", "localhost")
    bridge.declare_parameter("web_server_port", 8000)

    # Web serverの起動
    uvicorn.run(
        app,
        host=bridge.get_parameter("web_server_host").value,
        port=bridge.get_parameter("web_server_port").value,
    )


if __name__ == "__main__":
    main()
