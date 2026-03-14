"""ロボット/ボール位置追跡."""

from dataclasses import dataclass

from .metrics import distance_2d, speed_2d
from .models import BagData


@dataclass
class RobotState:
    """ロボットの状態スナップショット."""

    t: float
    robot_id: int
    x: float
    y: float
    theta: float
    vx: float
    vy: float
    speed: float
    detected: bool
    dist_to_ball: float


@dataclass
class BallState:
    """ボールの状態スナップショット."""

    t: float
    x: float
    y: float
    vx: float
    vy: float
    speed: float


def track_robot(
    bag_data: BagData,
    robot_id: int,
    is_ours: bool = True,
    interval: float = 0.1,
    time_range: tuple[float, float] | None = None,
) -> list[RobotState]:
    """ロボットの位置を時系列で追跡する.

    Args:
        bag_data: BagData
        robot_id: 追跡するロボットID
        is_ours: 味方チームの場合True、敵チームの場合False
        interval: サンプリング間隔（秒）
        time_range: (start_sec, end_sec) bag開始からの相対秒数

    Returns:
        RobotStateのリスト
    """
    data = bag_data if time_range is None else bag_data.time_slice(*time_range)
    result: list[RobotState] = []

    for tm in data.sample("/world_model", interval):
        msg = tm.msg
        ball = msg.ball_info
        bx, by = ball.position.x, ball.position.y

        robots = msg.robot_info_ours if is_ours else msg.robot_info_theirs
        for r in robots:
            if r.id == robot_id:
                vx = getattr(r.velocity, "x", 0.0)
                vy = getattr(r.velocity, "y", 0.0)
                result.append(
                    RobotState(
                        t=tm.t,
                        robot_id=robot_id,
                        x=r.pose.x,
                        y=r.pose.y,
                        theta=r.pose.theta,
                        vx=vx,
                        vy=vy,
                        speed=speed_2d(vx, vy),
                        detected=r.available_vision,
                        dist_to_ball=distance_2d(r.pose.x, r.pose.y, bx, by),
                    )
                )
                break

    return result


def track_ball(
    bag_data: BagData,
    interval: float = 0.1,
    time_range: tuple[float, float] | None = None,
) -> list[BallState]:
    """ボールの位置を時系列で追跡する.

    Args:
        bag_data: BagData
        interval: サンプリング間隔（秒）
        time_range: (start_sec, end_sec) bag開始からの相対秒数

    Returns:
        BallStateのリスト
    """
    data = bag_data if time_range is None else bag_data.time_slice(*time_range)
    result: list[BallState] = []

    for tm in data.sample("/world_model", interval):
        ball = tm.msg.ball_info
        vx = ball.velocity.x
        vy = ball.velocity.y
        result.append(
            BallState(
                t=tm.t,
                x=ball.position.x,
                y=ball.position.y,
                vx=vx,
                vy=vy,
                speed=speed_2d(vx, vy),
            )
        )

    return result
