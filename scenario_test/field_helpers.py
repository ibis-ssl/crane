"""フィールド寸法を vision の geometry パケットから導出する配置ヘルパー。

シナリオテストの座標をリーグの区分（Division A / B）に対して固定で書いてはいけない。
シミュレータはフィールド外へのテレポートを**拒否せずクランプする**ため、区分の違う
座標で走らせてもエラーにはならず、「要求した世界とは別の世界」でテストが進む。

実際に起きた例: Division B（9000x6000, boundary 300）で走る scenario シミュレータに対し、
テストが Division A の座標で `send_ball(5.5, 3.0)` を送っていた。壁は 4.5 + 0.3 = 4.8 に
あり、ボール半径 0.0215 を引いた 4.7785 にクランプされる（ログの実測値 4.779）。目標は
(4.0, 3.0) なので距離は 0.7785 で固定され、しきい値 0.20 に対して必ず失敗する。しかも
ボールの外側に回り込むにはロボット中心が x ≈ 4.89 > 4.8（壁の外）に必要で、
**構造的に成功しえない**配置だった。

このモジュールは 2 つを提供する:

- ランドマーク基準の座標計算（ゴールライン / タッチライン / 壁 / ペナルティエリアからの
  相対位置）。「壁から 0.45 m 内側」という意図がそのまま残るので、区分が変わっても
  意味が保たれる。
- 配置前の検査。フィールド外に出る座標は送信せずに AssertionError にする。静かに
  クランプされるより、その場で落ちたほうがよい。
"""

import math

from rcst.communication import Communication
from rcst.field_geometry import FieldGeometry

# 公式規則の値。配置がフィールドに収まるかの判定に使う。
BALL_RADIUS = 0.0215
ROBOT_RADIUS = 0.09

# crane(yellow) が守るゴールの側。
#
# rcst 環境では -x で確定する。referee パケットの経路を追うとそうなる:
#
# - rcst/sim_referee.py の to_referee_packet_string() は blue_team_on_positive_half を
#   一切セットしない。
# - crane_world_model_publisher/src/world_model_data_provider.cpp:204 は
#   `msg.has_field & BLUE_TEAM_ON_POSITIVE_HALF_FIELD_SET` のときだけ on_positive_half を
#   更新する。上記のとおりこのフィールドは来ないので、更新は起きない。
# - したがって on_positive_half は初期値 false のまま
#   （world_model_data_provider.hpp:287）で、:351 の
#   `our_goal_x = on_positive_half ? +field_w/2 : -field_w/2` は -x を返す。
#
# 実測とも一致する。フィールド中央に1機だけ置いて FORCE_START すると -x 側のゴール前
# (-4.04, 0.00) へ帰り、キックは +4.6 付近（攻撃側のゴールライン裏）へ飛ぶ。
#
# 配置を決めるテストはこの定数を使い、直に符号を書かないこと。実機のようにレフェリーが
# blue_team_on_positive_half を送る環境では向きが反転しうるので、rcst 以外へ流用する
# 場合は上の連鎖を再確認すること。
DEFENDED_SIDE = -1
# 攻める側（相手ゴールのある側）
ATTACKING_SIDE = -DEFENDED_SIDE


class Field:
    """geometry から導出した座標系と、検査付きの配置 API。

    `rcst_comm` をそのまま使わず、このクラス経由で配置すること。配置前に
    フィールドに収まるかを検査するのが存在理由なので、迂回すると意味がなくなる。
    """

    def __init__(self, comm: Communication, geometry: FieldGeometry):
        self._comm = comm
        self.geometry = geometry

    # ─── 寸法 ────────────────────────────────────────────────────────────────

    @property
    def comm(self) -> Communication:
        """referee 操作や observer 参照のための生の Communication。

        配置には使わないこと（検査を迂回する）。
        """
        return self._comm

    @property
    def clearance_to_get_behind_ball(self) -> float:
        """ボールの外側に回り込むためにロボット中心が必要とする余裕 [m]。

        壁からボールまでの距離がこれを下回ると、ロボットは壁の外に出ない限り
        ボールの裏を取れない。壁際シナリオが成立するかの下限。
        """
        return ROBOT_RADIUS + BALL_RADIUS

    @property
    def half_length(self) -> float:
        """中心からゴールラインまで [m]"""
        return self.geometry.half_length

    @property
    def half_width(self) -> float:
        """中心からタッチラインまで [m]"""
        return self.geometry.half_width

    @property
    def half_goal_width(self) -> float:
        """ゴール中心からゴールポストまで [m]。"""
        return self.geometry.half_goal_width

    @property
    def penalty_depth(self) -> float:
        """ゴールラインからペナルティエリア前縁まで [m]。"""
        return self.geometry.penalty_depth

    @property
    def wall_x(self) -> float:
        """中心からゴールライン裏の壁まで [m]。クランプが起きる境界。"""
        return self.geometry.wall_x

    @property
    def wall_y(self) -> float:
        """中心からタッチライン裏の壁まで [m]。クランプが起きる境界。"""
        return self.geometry.wall_y

    @property
    def penalty_half_width(self) -> float:
        return self.geometry.penalty_half_width_or_raise()

    # ─── ランドマーク基準の座標 ──────────────────────────────────────────────

    def from_goal_line(self, sign: int, inward: float = 0.0) -> float:
        """ゴールラインから `inward` [m] 内側の x。`sign` は +1 / -1。"""
        return sign * (self.half_length - inward)

    def from_touch_line(self, sign: int, inward: float = 0.0) -> float:
        """タッチラインから `inward` [m] 内側の y。`sign` は +1 / -1。"""
        return sign * (self.half_width - inward)

    def from_wall_x(self, sign: int, inward: float) -> float:
        """ゴールライン裏の壁から `inward` [m] 内側の x。壁際シナリオ用。"""
        return sign * (self.wall_x - inward)

    def from_wall_y(self, sign: int, inward: float) -> float:
        """タッチライン裏の壁から `inward` [m] 内側の y。壁際シナリオ用。"""
        return sign * (self.wall_y - inward)

    def penalty_front_x(self, sign: int) -> float:
        """ペナルティエリア前縁の x。`sign` は +1 / -1。"""
        return sign * self.geometry.penalty_area_x()

    def is_in_penalty_area(self, x: float, y: float) -> bool:
        return self.geometry.is_in_penalty_area(x, y)

    def x(self, ratio: float) -> float:
        """ハーフ長さに対する割合で表した x。ランドマークが無い位置に使う。"""
        return ratio * self.half_length

    def y(self, ratio: float) -> float:
        """ハーフ幅に対する割合で表した y。ランドマークが無い位置に使う。"""
        return ratio * self.half_width

    def column_y(self, count: int, span_ratio: float) -> list[float]:
        """`count` 機を y 方向に等間隔で並べたときの y 座標を上から順に返す。

        両端は `y(span_ratio)`。区分が変われば間隔も詰まるので、ロボット直径
        (2 * ROBOT_RADIUS) を下回る間隔になったら落とす。
        """
        if count <= 1:
            return [0.0] * count

        top = self.y(span_ratio)
        step = 2.0 * top / (count - 1)
        if abs(step) < 2.0 * ROBOT_RADIUS:
            raise AssertionError(
                f"{count} 機を span_ratio={span_ratio} で並べると間隔が {abs(step):.3f} m になり、"
                f"ロボット直径 {2.0 * ROBOT_RADIUS:.3f} m を下回って重なる（ハーフ幅 {self.half_width:.3f} m）。"
            )
        return [top - i * step for i in range(count)]

    def toward(self, from_xy, to_xy, distance: float) -> tuple[float, float]:
        """`from_xy` から `to_xy` の方向へ `distance` [m] 進んだ点。"""
        dx, dy = to_xy[0] - from_xy[0], to_xy[1] - from_xy[1]
        norm = math.hypot(dx, dy)
        if norm == 0.0:
            raise ValueError("方向が定まらない（2点が同一）")
        return (from_xy[0] + dx / norm * distance, from_xy[1] + dy / norm * distance)

    def own_goal_center(self, sign: int) -> tuple[float, float]:
        """`sign` 側のゴール中央（ゴールライン上）。"""
        return (self.from_goal_line(sign), 0.0)

    # ─── 検査付きの配置 ──────────────────────────────────────────────────────

    def send_empty_world(self) -> None:
        self._comm.send_empty_world()

    def send_ball(self, x: float, y: float, v_x: float = 0.0, v_y: float = 0.0) -> None:
        self._check("ボール", BALL_RADIUS, x, y)
        self._comm.send_ball(x, y, v_x, v_y)

    def send_yellow_robot(
        self, robot_id: int, x: float, y: float, orientation: float
    ) -> None:
        self._check(f"Yellow {robot_id}", ROBOT_RADIUS, x, y)
        self._comm.send_yellow_robot(robot_id, x, y, orientation)

    def send_blue_robot(
        self, robot_id: int, x: float, y: float, orientation: float
    ) -> None:
        self._check(f"Blue {robot_id}", ROBOT_RADIUS, x, y)
        self._comm.send_blue_robot(robot_id, x, y, orientation)

    def _check(self, what: str, radius: float, x: float, y: float) -> None:
        max_x = self.wall_x - radius
        max_y = self.wall_y - radius
        if abs(x) <= max_x and abs(y) <= max_y:
            return

        raise AssertionError(
            f"{what} を ({x:.3f}, {y:.3f}) に置こうとしたが、このフィールドでは"
            f"壁にめり込む。配置可能なのは |x| <= {max_x:.3f}, |y| <= {max_y:.3f}"
            f"（フィールド {self.geometry.field_length:.1f} x {self.geometry.field_width:.1f} m, boundary {self.geometry.boundary_width:.2f} m, 半径 {radius:.4f} m）。"
            "シミュレータは範囲外のテレポートを拒否せずクランプするので、"
            "このまま送ると別の位置で静かにテストが進む。"
        )


def make_field(comm: Communication, timeout: float = 10.0) -> Field:
    """vision から geometry が届くのを待って `Field` を作る。

    届かなければ既定値に落とさず例外にする。推測したフィールドで走るのが、
    このモジュールが防ごうとしている失敗そのものだから。
    """
    return Field(comm, comm.wait_for_geometry(timeout))
