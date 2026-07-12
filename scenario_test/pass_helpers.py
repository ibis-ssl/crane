"""パスシナリオテスト用の観測ヘルパーと共通配置。

vision 情報のみから yellow(crane) のキック→受領を分類する。
bag 解析（crane_bag pass）と異なり意図（pass_target_id）は観測できないため、
「キック後に最初にボールへ触れたのが別の yellow ロボットなら成功」という
機能的成功の定義を用いる。

制約:
- rcst の Ball は速度を持たない（常に0）ため、位置差分から速度を推定する
- ボール高さ z も観測できないため、チップキックが敵の頭上を越える場合に
  INTERCEPTED と誤判定し得る（本シナリオはストレートパスが成立する配置を使う）

座標系（vision）: crane(yellow) は +x 側を守り、-x 方向へ攻める
（rcst の referee は blue_team_on_positive_half を送らない = false のため、
 crane_world_model_publisher は yellow を positive half と解釈する）。
"""

import dataclasses
import math
import time
from collections import deque

# ─── 判定パラメータ ──────────────────────────────────────────────────────────
KICK_DETECT_SPEED = 1.5  # キック開始とみなすボール速度 [m/s]
KICK_PROXIMITY = 0.4  # キッカー帰属のボール近傍距離 [m]
CONTACT_DIST = 0.13  # 接触とみなすロボット中心-ボール距離 [m]
KICKER_RELEASE_DIST = 0.5  # キッカー再接触を有効化するボール離脱距離 [m]
STOP_SPEED = 0.3  # こぼれ球とみなすボール速度 [m/s]
SETTLE_TIME = 0.2  # キック直後の判定無効時間 [s]
FIELD_HALF_X = 6.05  # 場外判定 [m]（フィールド 12x9 + マージン）
FIELD_HALF_Y = 4.55
SPEED_WINDOW_SEC = 0.08  # 速度推定の差分窓 [s]


@dataclasses.dataclass
class PassTrialResult:
    """1試行の結果。outcome は以下のいずれか:

    SUCCESS      キッカー以外の yellow が最初に接触
    SELF_TOUCH   キッカー自身が再接触（パス不成立）
    INTERCEPTED  blue が最初に接触
    OVERRUN      誰も触れずにボールが停止
    OUT_OF_PLAY  ボールが場外に出た
    NO_KICK      タイムアウトまでキックが発生しなかった
    TIMEOUT      キック後、解決しないままタイムアウト
    """

    outcome: str = "NO_KICK"
    kicker_id: int = -1
    receiver_id: int = -1
    kick_pos: tuple = (0.0, 0.0)
    end_pos: tuple = (0.0, 0.0)
    kick_speed: float = 0.0
    pass_distance: float = 0.0
    duration: float = 0.0

    def to_dict(self) -> dict:
        return dataclasses.asdict(self)


class _BallSpeedEstimator:
    """vision 位置差分によるボール速度推定（タイムスタンプ重複は無視）"""

    def __init__(self):
        self.samples = deque(maxlen=32)  # (t, x, y)

    def update(self, t: float, x: float, y: float) -> float:
        if self.samples and t <= self.samples[-1][0]:
            return self.speed()
        self.samples.append((t, x, y))
        return self.speed()

    def speed(self) -> float:
        if len(self.samples) < 2:
            return 0.0
        t1, x1, y1 = self.samples[-1]
        # 窓の外側で最も新しいサンプルとの差分を取る
        for t0, x0, y0 in reversed(self.samples):
            if t1 - t0 >= SPEED_WINDOW_SEC:
                return math.hypot(x1 - x0, y1 - y0) / (t1 - t0)
        t0, x0, y0 = self.samples[0]
        if t1 - t0 <= 0:
            return 0.0
        return math.hypot(x1 - x0, y1 - y0) / (t1 - t0)


def _nearest_robot(robots, x: float, y: float):
    """(id, distance) を返す。空なら (None, inf)"""
    best_id, best_d = None, float("inf")
    for robot in robots.values():
        d = math.hypot(robot.x - x, robot.y - y)
        if d < best_d:
            best_id, best_d = robot.id, d
    return best_id, best_d


def watch_pass_outcome(get_world, timeout_sec: float = 25.0) -> PassTrialResult:
    """次の yellow キック1本を追跡して結果を分類する。

    get_world: () -> VisionWorld（rcst_comm.observer.get_world を渡す）
    """
    result = PassTrialResult()
    estimator = _BallSpeedEstimator()
    start_wall = time.time()
    tracking = False
    kick_wall_time = 0.0
    ball_left_kicker = False
    prev_ball = None  # キック直前のボール位置（キック点の推定用）

    while time.time() - start_wall < timeout_sec:
        world = get_world()
        ball = world.get_ball()
        t = world.get_timestamp()
        speed = estimator.update(t, ball.x, ball.y)
        yellows = world.get_yellow_robots()
        blues = world.get_blue_robots()

        if not tracking:
            if speed >= KICK_DETECT_SPEED and prev_ball is not None:
                kick_x, kick_y = prev_ball
                y_id, y_d = _nearest_robot(yellows, kick_x, kick_y)
                b_id, b_d = _nearest_robot(blues, kick_x, kick_y)
                if y_id is not None and y_d <= KICK_PROXIMITY and y_d <= b_d:
                    tracking = True
                    kick_wall_time = time.time()
                    ball_left_kicker = False
                    result.kicker_id = y_id
                    result.kick_pos = (kick_x, kick_y)
                    result.kick_speed = speed
                # blue 起因の速度立ち上がり（跳ね返り等）は無視して次のキックを待つ
            else:
                prev_ball = (ball.x, ball.y)
            time.sleep(0.01)
            continue

        # ─ 追跡中 ─
        dt = time.time() - kick_wall_time
        result.duration = dt
        result.end_pos = (ball.x, ball.y)
        result.kick_speed = max(result.kick_speed, speed if dt <= 0.3 else 0.0)
        if (
            not ball_left_kicker
            and math.hypot(ball.x - result.kick_pos[0], ball.y - result.kick_pos[1])
            > KICKER_RELEASE_DIST
        ):
            ball_left_kicker = True

        if dt >= SETTLE_TIME:
            # 場外
            if abs(ball.x) > FIELD_HALF_X or abs(ball.y) > FIELD_HALF_Y:
                result.outcome = "OUT_OF_PLAY"
                break

            # 接触（最も近いロボットを優先）
            hit_id, hit_ours, hit_d = None, False, float("inf")
            for robot in yellows.values():
                if robot.id == result.kicker_id and not ball_left_kicker:
                    continue
                d = math.hypot(robot.x - ball.x, robot.y - ball.y)
                if d <= CONTACT_DIST and d < hit_d:
                    hit_id, hit_ours, hit_d = robot.id, True, d
            for robot in blues.values():
                d = math.hypot(robot.x - ball.x, robot.y - ball.y)
                if d <= CONTACT_DIST and d < hit_d:
                    hit_id, hit_ours, hit_d = robot.id, False, d
            if hit_id is not None:
                result.receiver_id = hit_id
                if not hit_ours:
                    result.outcome = "INTERCEPTED"
                elif hit_id == result.kicker_id:
                    result.outcome = "SELF_TOUCH"
                else:
                    result.outcome = "SUCCESS"
                break

            # こぼれ球
            if speed < STOP_SPEED:
                result.outcome = "OVERRUN"
                break

        time.sleep(0.01)
    else:
        if tracking:
            result.outcome = "TIMEOUT"

    result.pass_distance = math.hypot(
        result.end_pos[0] - result.kick_pos[0], result.end_pos[1] - result.kick_pos[1]
    )
    return result


# ─── 共通配置 ────────────────────────────────────────────────────────────────


def setup_buildup_static(comm) -> None:
    """ビルドアップ配置: シュートラインを blue の壁で塞ぎ、ウィングの受け手は空ける。

    ボール(2.0, 0)から見てゴールマウス(-6, ±0.9)は blue 壁で完全に遮蔽され
    （ゴール可視角 ≈ 0）、attacker はパスを選択せざるを得ない。
    受け手 2/3 は攻撃ハーフ（vision x < 0）にいるため pass_target 候補になる。
    """
    comm.send_empty_world()
    # yellow (crane): +x 側を守り -x 方向へ攻める
    comm.send_yellow_robot(0, 5.7, 0.0, math.radians(180))  # GK
    comm.send_yellow_robot(
        1, 2.4, 0.1, math.radians(180)
    )  # ボール至近（attacker 候補）
    comm.send_yellow_robot(2, -0.4, 2.2, math.radians(180))  # 受け手候補（左ウィング）
    comm.send_yellow_robot(3, -0.4, -2.2, math.radians(180))  # 受け手候補（右ウィング）
    comm.send_yellow_robot(4, 3.5, -1.5, math.radians(180))  # 後方サポート
    # blue: 全機静止（制御なし）。シュートコースを塞ぐ壁 + GK + 後方2機
    comm.send_blue_robot(0, -5.7, 0.0, 0.0)
    comm.send_blue_robot(1, 0.9, 0.0, 0.0)
    comm.send_blue_robot(2, 0.7, 0.25, 0.0)
    comm.send_blue_robot(3, 0.7, -0.25, 0.0)
    comm.send_blue_robot(4, -2.0, 0.6, 0.0)
    comm.send_blue_robot(5, -2.0, -0.6, 0.0)
    comm.send_ball(2.0, 0.0)


def setup_under_mark(comm) -> None:
    """受け手がゴール側からマークされた配置。

    マーカーはパスラインを塞がない位置（受け手のゴール側 0.7m）に置き、
    「密着マーク下でのレシーブ」を試す。直接のパスコース自体は通っている。
    """
    setup_buildup_static(comm)
    # 受け手(-0.4, ±2.2) のゴール(-6,0)側 0.7m にマーカーを追加
    comm.send_blue_robot(6, -1.05, 1.94, 0.0)
    comm.send_blue_robot(7, -1.05, -1.94, 0.0)


def run_pass_trial(comm, setup_fn, timeout_sec: float = 25.0) -> PassTrialResult:
    """1試行: STOP→配置→FORCE_START→パス1本の結果を観測して STOP で終了"""
    comm.change_referee_command("STOP", 1.0)
    setup_fn(comm)
    time.sleep(2.0)  # 配置反映と役割割当の安定待ち
    comm.observer.reset()
    comm.change_referee_command("FORCE_START", 0.5)
    result = watch_pass_outcome(comm.observer.get_world, timeout_sec)
    comm.change_referee_command("STOP", 1.0)
    return result
