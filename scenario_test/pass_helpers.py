"""パスシナリオテスト用の観測ヘルパーと共通配置。

vision 情報のみから yellow(crane) のキック→受領を分類する。
bag 解析（crane_bag pass）と異なり意図（pass_target_id）は観測できないため、
「キック後に最初にボールへ触れたのが別の yellow ロボットなら成功」という
機能的成功の定義を用いる。

制約:
- rcst の Ball は速度を持たない（常に0）ため、位置差分から速度を推定する
- ボール高さ z も観測できないため、チップキックが敵の頭上を越える場合に
  INTERCEPTED と誤判定し得る（本シナリオはストレートパスが成立する配置を使う）
- **静止しているのは blue だけ。** crane は yellow 全機を制御下に置いていて、
  配置した直後から自分の陣形へ動かし始める。保存ログの実測では、テレポートの
  0.09 秒後には動き出し、0.8 秒後に受け手が 0.7 m、3.8 秒後には 2.8 m 離れた。
  したがって「配置してしばらく待ってから開始する」と、テストが作ったパスコースは
  キック時点では存在しない。run_pass_trial は反映を確認し次第 FORCE_START する。

座標系（vision）: crane(yellow) が守るのは field_helpers.DEFENDED_SIDE 側で、
攻めるのは ATTACKING_SIDE 側。rcst 環境では -x を守り +x へ攻める（rcst が
blue_team_on_positive_half を送らず on_positive_half が初期値 false のままになるため。
根拠の連鎖は field_helpers.DEFENDED_SIDE のコメントに書いた）。

以前このファイルは逆（+x を守り -x へ攻める）を前提に配置しており、
attacker は自陣ゴールではなく相手ゴールの方向、つまり配置上の「後ろ」へ蹴って
いた。その結果ボールは 3 試行とも +4.6 付近（ゴールライン 4.5 の外）へ抜けていた。
場外判定が Division A 固定の 6.05 だったためにそれが場外と数えられず、壁で跳ね
返ったあとの接触を SUCCESS と分類していて、配置の向きが逆であることが見えなかった。

配置座標は Division を決め打ちせず vision の geometry から導出する（field_helpers）。
ロボット間隔のようなロボットスケールの距離は絶対値のまま持つ。
"""

import dataclasses
import math
import time
from collections import deque

from field_helpers import ATTACKING_SIDE, DEFENDED_SIDE, Field

# ─── 判定パラメータ ──────────────────────────────────────────────────────────
KICK_DETECT_SPEED = 1.5  # キック開始とみなすボール速度 [m/s]
KICK_PROXIMITY = 0.4  # キッカー帰属のボール近傍距離 [m]
CONTACT_DIST = 0.13  # 接触とみなすロボット中心-ボール距離 [m]
KICKER_RELEASE_DIST = 0.5  # キッカー再接触を有効化するボール離脱距離 [m]
STOP_SPEED = 0.3  # こぼれ球とみなすボール速度 [m/s]
SETTLE_TIME = 0.2  # キック直後の判定無効時間 [s]
OUT_OF_PLAY_MARGIN = 0.05  # 場外判定をフィールド境界から何 m 外に置くか
SPEED_WINDOW_SEC = 0.08  # 速度推定の差分窓 [s]
MARKER_DISTANCE = 0.7  # 受け手からマーカーまでの距離 [m]
PLACEMENT_TOLERANCE = 0.05  # 配置が vision に反映されたとみなす許容誤差 [m]
PLACEMENT_WAIT_TIMEOUT = 2.0  # 配置の反映を待つ上限 [s]
KICKOFF_DELAY = 0.1  # FORCE_START 送信後、観測を始めるまでの待ち [s]


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
    # 配置が vision に反映されるまでに要した秒数（NaN なら確認できなかった）
    placement_wait: float = float("nan")
    # キック時点で、各 yellow が配置座標からどれだけ離れていたか {id: m}
    # 最大値ではなく機体ごとに持つ。attacker がボールへ寄るのは当然の移動なので、
    # 「受け手が持ち場を離れたか」は個別に見ないと分からない。
    drift_at_kick: dict = dataclasses.field(default_factory=dict)

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


def _drifts(yellows, expected: dict | None) -> dict:
    """配置座標からのずれ [m] を機体ごとに返す。expected が無ければ空。"""
    if not expected:
        return {}
    return {
        rid: round(math.hypot(yellows[rid].x - x, yellows[rid].y - y), 3)
        for rid, (x, y) in expected.items()
        if rid in yellows
    }


def _wait_for_placement(field: Field, expected: dict) -> float:
    """配置した座標が vision に現れるまで待ち、要した秒数を返す。

    確認できないまま PLACEMENT_WAIT_TIMEOUT に達したら NaN を返す。

    以前はここが固定 2 秒の sleep だった（「配置反映と役割割当の安定待ち」）。
    しかし crane は yellow 全機を制御下に置いていて、テレポート直後から自分の
    陣形へ動かし始める。保存ログの実測では、配置 0.09 秒後には既に動き出し、
    0.8 秒後には受け手が 0.7 m、3.8 秒後には 2.8 m 離れていた。2 秒待つと、
    テストが作ったパスコースはキック時点では存在しない。
    反映を確認できた時点で抜けることで、意図した配置のまま試行を始める。
    """
    deadline = time.time() + PLACEMENT_WAIT_TIMEOUT
    start = time.time()
    while time.time() < deadline:
        robots = field.comm.observer.get_world().get_yellow_robots()
        if all(
            rid in robots
            and math.hypot(robots[rid].x - x, robots[rid].y - y) <= PLACEMENT_TOLERANCE
            for rid, (x, y) in expected.items()
        ):
            return time.time() - start
        time.sleep(0.01)
    return float("nan")


def watch_pass_outcome(
    field: Field, timeout_sec: float = 25.0, expected: dict | None = None
) -> PassTrialResult:
    """次の yellow キック1本を追跡して結果を分類する。

    expected を渡すと、キック時点で配置座標からどれだけずれていたかを
    result.max_drift_at_kick に記録する。crane は yellow 全機を制御下に置いて
    いるので、テストが作った配置は放っておくと崩れる。判定そのものには
    使わないが、「意図した配置で試行できたのか」がログから分かるようにする。
    """
    get_world = field.comm.observer.get_world
    field_half_x = field.half_length + OUT_OF_PLAY_MARGIN
    field_half_y = field.half_width + OUT_OF_PLAY_MARGIN
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
                _b_id, b_d = _nearest_robot(blues, kick_x, kick_y)
                if y_id is not None and y_d <= KICK_PROXIMITY and y_d <= b_d:
                    tracking = True
                    kick_wall_time = time.time()
                    ball_left_kicker = False
                    result.kicker_id = y_id
                    result.kick_pos = (kick_x, kick_y)
                    result.kick_speed = speed
                    result.drift_at_kick = _drifts(yellows, expected)
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
            if abs(ball.x) > field_half_x or abs(ball.y) > field_half_y:
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


def receiver_positions(field: Field) -> list:
    """受け手候補（左右ウィング）の座標。マーカー配置でも参照する。

    ハーフウェイラインをわずかに攻撃側へ越えた位置に置く。
    """
    x = field.x(0.07) * ATTACKING_SIDE
    return [(x, field.y(0.49)), (x, field.y(-0.49))]


def setup_buildup_static(field: Field) -> dict:
    """ビルドアップ配置: シュートラインを blue の壁で塞ぎ、ウィングの受け手は空ける。

    ボールから見て相手ゴールマウスは blue 壁で完全に遮蔽され（ゴール可視角 ≈ 0）、
    attacker はパスを選択せざるを得ない。
    受け手 2/3 は攻撃ハーフにいるため pass_target 候補になる。

    配置した yellow の座標を {id: (x, y)} で返す。run_pass_trial がこれを使って
    「配置が vision に反映されたか」を確かめ、反映され次第 FORCE_START する。
    """
    field.send_empty_world()
    # ボールは自陣側。そこから攻撃側のウィングへ繋ぐのがこのシナリオ。
    ball_x = field.x(0.33) * DEFENDED_SIDE
    left_receiver, right_receiver = receiver_positions(field)
    facing = math.atan2(0.0, ATTACKING_SIDE)  # 攻撃方向を向かせる

    # yellow (crane)
    yellows = {
        0: (field.from_goal_line(DEFENDED_SIDE, 0.3), 0.0),  # GK
        1: (ball_x + 0.4 * DEFENDED_SIDE, 0.1),  # ボール至近（attacker 候補）
        2: left_receiver,
        3: right_receiver,
        4: (field.x(0.58) * DEFENDED_SIDE, field.y(-0.33)),  # 後方サポート
    }
    for robot_id, (x, y) in yellows.items():
        field.send_yellow_robot(robot_id, x, y, facing)
    # blue: 全機静止（制御なし）。シュートコースを塞ぐ壁 + GK + 後方2機
    field.send_blue_robot(0, field.from_goal_line(ATTACKING_SIDE, 0.3), 0.0, 0.0)
    field.send_blue_robot(1, field.x(0.15) * ATTACKING_SIDE, 0.0, 0.0)
    field.send_blue_robot(2, field.x(0.117) * ATTACKING_SIDE, 0.25, 0.0)
    field.send_blue_robot(3, field.x(0.117) * ATTACKING_SIDE, -0.25, 0.0)
    field.send_blue_robot(4, field.x(0.33) * ATTACKING_SIDE, 0.6, 0.0)
    field.send_blue_robot(5, field.x(0.33) * ATTACKING_SIDE, -0.6, 0.0)
    field.send_ball(ball_x, 0.0)
    return yellows


def setup_under_mark(field: Field) -> dict:
    """受け手がゴール側からマークされた配置。

    マーカーはパスラインを塞がない位置（受け手のゴール側 0.7m）に置き、
    「密着マーク下でのレシーブ」を試す。直接のパスコース自体は通っている。
    """
    yellows = setup_buildup_static(field)
    # 受け手から、crane が攻めるゴールへ 0.7m 寄った位置にマーカーを置く
    goal = field.own_goal_center(ATTACKING_SIDE)
    for robot_id, receiver in zip((6, 7), receiver_positions(field)):
        marker_x, marker_y = field.toward(receiver, goal, MARKER_DISTANCE)
        field.send_blue_robot(robot_id, marker_x, marker_y, 0.0)
    return yellows


def run_pass_trial(
    field: Field, setup_fn, timeout_sec: float = 25.0
) -> PassTrialResult:
    """1試行: STOP→配置→（反映を確認したら即）FORCE_START→パス1本を観測

    配置してから FORCE_START までの時間を最小にするのが肝。crane は yellow 全機を
    動かすので、待てば待つほどテストが作った配置は崩れる。固定 sleep をやめ、
    vision に反映されたことを確認し次第そのまま開始する。
    """
    comm = field.comm
    comm.change_referee_command("STOP", 1.0)
    expected = setup_fn(field)
    placement_wait = _wait_for_placement(field, expected)
    comm.observer.reset()
    comm.change_referee_command("FORCE_START", KICKOFF_DELAY)
    result = watch_pass_outcome(field, timeout_sec, expected)
    result.placement_wait = placement_wait
    comm.change_referee_command("STOP", 1.0)
    return result
