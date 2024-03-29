# coding: UTF-8

# Copyright 2021 Roots
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math
from enum import Enum

from python_qt_binding.QtCore import QPointF, QRectF, QSizeF, Qt
from python_qt_binding.QtGui import QColor, QPainter, QPen
from python_qt_binding.QtWidgets import QWidget
from robocup_ssl_msgs.msg import (
    BallReplacement,
    GeometryFieldSize,
    Replacement,
    RobotId,
    RobotReplacement,
    TrackedFrame,
)


class ClickedObject(Enum):
    IS_NONE = 0
    IS_BALL_POS = 1
    IS_BALL_VEL = 2


class FieldWidget(QWidget):
    def __init__(self, parent=None):
        super(FieldWidget, self).__init__(parent)

        # 定数
        # Ref: named color  https://www.w3.org/TR/SVG11/types.html#ColorKeywords
        self._COLOR_FIELD_CARPET = Qt.green
        self._COLOR_FIELD_LINE = Qt.white
        self._COLOR_BALL = QColor("orange")
        self._COLOR_BLUE_ROBOT = Qt.cyan
        self._COLOR_YELLOW_ROBOT = Qt.yellow
        self._COLOR_REPLACEMENT_POS = QColor("magenta")
        self._COLOR_REPLACEMENT_VEL_ANGLE = QColor("darkviolet")
        self._COLOR_DESIGNATED_POSITION = QColor("red")
        self._THICKNESS_FIELD_LINE = 2
        self._MOUSE_WHEEL_ZOOM_RATE = 0.2  # マウスホイール操作による拡大縮小操作量
        self._LIMIT_SCALE = 0.2  # 縮小率の限界値
        # Ref: https://robocup-ssl.github.io/ssl-rules/sslrules.html#_ball
        self._RADIUS_BALL = 21.5  # diameter is 43 mm.
        # Ref: https://robocup-ssl.github.io/ssl-rules/sslrules.html#_shape
        self._RADIUS_ROBOT = 90  # diameter is 180 mm.
        self._RADIUS_REPLACEMENT_BALL_POS = self._RADIUS_BALL + 300
        self._RADIUS_REPLACEMENT_BALL_VEL = self._RADIUS_BALL + 500
        self._RADIUS_REPLACEMENT_ROBOT_POS = self._RADIUS_ROBOT + 100
        self._RADIUS_REPLACEMENT_ROBOT_ANGLE = self._RADIUS_ROBOT + 200
        self._GAIN_REPLACE_BALL_VEL = 0.001 * 3.0
        self._MAX_VELOCITY_OF_REPLACE_BALL = 8.0
        self._ID_POS = QPointF(150.0, 150.0)  # IDの表示位置 mm.
        self._RADIUS_DESIGNATED_POSITION = 150  # ボールプレースメント成功範囲

        # 外部からセットするパラメータ
        self._logger = None
        self._pub_replacement = None
        self._can_draw_geometry = False
        self._can_draw_detection = False
        self._can_draw_detection_tracked = False
        self._can_draw_replacement = False
        self._field = GeometryFieldSize()
        self._field.field_length = 12000  # resize_draw_area()で0 divisionを防ぐための初期値
        self._field.field_width = 9000  # resize_draw_area()で0 divisionを防ぐための初期値
        self._detections = {}
        self._detection_tracked = TrackedFrame()
        self._blue_robot_num = 0
        self._yellow_robot_num = 0
        # self._goal_poses = {}
        self._robot_commands = {}
        self._final_goal_poses = {}
        self._designated_position = {}  # ball placementの目標位置
        self._named_targets = {}
        self._robot_replacements = []

        # 内部で変更するパラメータ
        self._draw_area_scale = 1.0  # 描画領域の拡大縮小率
        self._draw_area_offset = QPointF(0.0, 0.0)  # 描画領域のオフセット
        self._draw_area_size = QSizeF(self.rect().size())  # 描画領域サイズ
        self._scale_field_to_draw = 1.0  # フィールド領域から描画領域に縮小するスケール
        self._do_rotate_draw_area = False  # 描画領域を90度回転するフラグ
        self._mouse_clicked_point = QPointF(0.0, 0.0)  # マウスでクリックした描画領域の座標
        self._mouse_current_point = QPointF(0.0, 0.0)  # マウスカーソルの現在座標
        self._mouse_drag_offset = QPointF(0.0, 0.0)  # マウスでドラッグした距離
        self._clicked_replacement_object = (
            ClickedObject.IS_NONE
        )  # マウスでクリックした、grSimのReplacementの対象

    def set_logger(self, logger):
        self._logger = logger

    def set_pub_replacement(self, pub_replacement):
        self._pub_replacement = pub_replacement

    def set_can_draw_geometry(self, enable=True):
        self._can_draw_geometry = enable

    def set_can_draw_detection(self, enable=True):
        self._can_draw_detection = enable

    def set_can_draw_detection_tracked(self, enable=True):
        self._can_draw_detection_tracked = enable

    def set_can_draw_replacement(self, enable=True):
        self._can_draw_replacement = enable

    def set_field(self, msg):
        self._field = msg.field
        self._resize_draw_area()

    def set_detection(self, msg):
        self._detections[msg.camera_id] = msg

    def set_detection_tracked(self, msg):
        self._detection_tracked = msg
        self._update_robot_num()

    def set_robot_command(self, msg, robot_id, is_yellow: bool):
        self._robot_commands[robot_id] = msg
        self.team_is_yellow = is_yellow

    def set_designated_position(self, msg):
        self._designated_position[0] = msg

    def set_named_targets(self, msg):
        # トピックを受け取るたびに初期化する
        self._named_targets = {}
        for i in range(len(msg.name)):
            self._named_targets[msg.name[i]] = msg.pose[i]

    def get_blue_robot_num(self):
        return self._blue_robot_num

    def get_yellow_robot_num(self):
        return self._yellow_robot_num

    def append_robot_replacement(self, is_yellow, robot_id, turnon):
        # turnon時はフィールド内に、turnoff時はフィールド外に、ID順でロボットを並べる
        ID_OFFSET_X = 0.2
        team_offset_x = -3.0 if is_yellow else 0.0
        turnon_offset_y = -4.6 if turnon else -5.6

        replacement = RobotReplacement()
        replacement.x = team_offset_x + ID_OFFSET_X * robot_id
        replacement.y = turnon_offset_y
        replacement.dir = 90.0
        replacement.id = robot_id
        replacement.yellowteam = is_yellow
        replacement.turnon.append(turnon)
        self._robot_replacements.append(replacement)

    def reset_topics(self):
        # 取得したトピックをリセットする
        self._detections = {}
        self._robot_commands = {}
        self._designated_position = {}

    def mousePressEvent(self, event):
        # マウスクリック時のイベント

        if event.buttons() == Qt.LeftButton:
            self._mouse_clicked_point = event.localPos()
            self._mouse_current_point = event.localPos()

            # ロボット、ボールをクリックしているか
            if self._can_draw_replacement:
                self._clicked_replacement_object = self._get_clicked_replacement_object(
                    self._mouse_clicked_point
                )

        elif event.buttons() == Qt.RightButton:
            self._reset_draw_area_offset_and_scale()

        self.update()

    def mouseMoveEvent(self, event):
        # マウス移動時のイベント
        self._mouse_current_point = event.localPos()

        if self._clicked_replacement_object != ClickedObject.IS_NONE:
            # Replacement時は何もしない
            pass
        elif event.buttons() == Qt.LeftButton:
            # 描画領域を移動するためのオフセットを計算
            self._mouse_drag_offset = (
                self._mouse_current_point - self._mouse_clicked_point
            ) / self._draw_area_scale

        self.update()

    def mouseReleaseEvent(self, event):
        # マウスクリック解除時のイベント
        # マウスのドラッグ操作で描画領域を移動する

        if self._can_draw_replacement:
            # grSim用のReplacementをpublish
            if self._clicked_replacement_object == ClickedObject.IS_BALL_POS:
                self._publish_ball_pos_replacement()
            elif self._clicked_replacement_object == ClickedObject.IS_BALL_VEL:
                self._publish_ball_vel_replacement()

        self._draw_area_offset += self._mouse_drag_offset
        self._mouse_drag_offset = QPointF(0.0, 0.0)  # マウスドラッグ距離の初期化
        self._clicked_replacement_object = ClickedObject.IS_NONE  # Replacementの初期化

        self.update()

    def wheelEvent(self, event):
        # マウスホイール回転時のイベント
        if event.angleDelta().y() > 0:
            self._draw_area_scale += self._MOUSE_WHEEL_ZOOM_RATE
        else:
            if self._draw_area_scale > self._LIMIT_SCALE:
                self._draw_area_scale -= self._MOUSE_WHEEL_ZOOM_RATE

        self.update()

    def resizeEvent(self, event):
        self._resize_draw_area()

    def paintEvent(self, event):
        painter = QPainter(self)

        # 描画領域の中心をWidgetの中心に持ってくる
        cx = float(self.width()) * 0.5
        cy = float(self.height()) * 0.5
        painter.translate(cx, cy)
        # 描画領域の拡大・縮小
        painter.scale(self._draw_area_scale, self._draw_area_scale)
        # 描画領域の移動
        painter.translate(self._draw_area_offset + self._mouse_drag_offset)
        # 描画領域の回転
        if self._do_rotate_draw_area is True:
            painter.rotate(-90)

        if self._can_draw_geometry:
            self._draw_geometry(painter)

        # 名前付きターゲットを描画
        self._draw_named_targets(painter)

        # ボールプレースメントでの目標位置と進入禁止エリアを描画
        self._draw_designated_position(painter)

        if self._can_draw_replacement:
            self._draw_replacement(painter)

        if self._can_draw_detection:
            self._draw_detection(painter)

        if self._can_draw_detection_tracked:
            self._draw_detection_tracked(painter)

        # ロボットのreplacementをpublishする
        if self._robot_replacements:
            replacement = Replacement()
            replacement.robots = self._robot_replacements
            self._pub_replacement.publish(replacement)
            self._robot_replacements = []

    def _get_clicked_replacement_object(self, clicked_point):
        # マウスでクリックした位置がボールやロボットに近いか判定する
        # 近ければreplacementと判定する
        field_clicked_pos = self._convert_draw_to_field_pos(clicked_point)

        clicked_object = ClickedObject.IS_NONE
        for detection in self._detections.values():
            for ball in detection.balls:
                clicked_object = self._get_clicked_ball_object(field_clicked_pos, ball)

        return clicked_object

    def _get_clicked_ball_object(self, clicked_pos, ball):
        ball_pos = QPointF(ball.x, ball.y)

        if self._is_clicked(ball_pos, clicked_pos, self._RADIUS_REPLACEMENT_BALL_POS):
            return ClickedObject.IS_BALL_POS
        elif self._is_clicked(ball_pos, clicked_pos, self._RADIUS_REPLACEMENT_BALL_VEL):
            return ClickedObject.IS_BALL_VEL
        else:
            return ClickedObject.IS_NONE

    def _is_clicked(self, pos1, pos2, threshold):
        # フィールド上のオブジェクトをクリックしたか判定する
        diff_pos = pos1 - pos2
        diff_norm = math.hypot(diff_pos.x(), diff_pos.y())

        if diff_norm < threshold:
            return True
        else:
            return False

    def _publish_ball_pos_replacement(self):
        # grSimのBall Replacementの位置をpublishする
        ball_replacement = BallReplacement()
        pos = self._convert_draw_to_field_pos(self._mouse_current_point)
        ball_replacement.x.append(pos.x() * 0.001)  # mm に変換
        ball_replacement.y.append(pos.y() * 0.001)  # mm に変換
        ball_replacement.vx.append(0.0)
        ball_replacement.vy.append(0.0)
        replacement = Replacement()
        replacement.ball.append(ball_replacement)
        self._pub_replacement.publish(replacement)

    def _publish_ball_vel_replacement(self):
        # grSimのBall Replacementの速度をpublishする
        ball_replacement = BallReplacement()

        start_pos = self._convert_draw_to_field_pos(self._mouse_clicked_point)
        end_pos = self._convert_draw_to_field_pos(self._mouse_current_point)

        # ball_replacement.x.append(start_pos.x() * 0.001)  # mm に変換
        # ball_replacement.y.append(start_pos.y() * 0.001)  # mm に変換

        diff_pos = end_pos - start_pos
        diff_norm = math.hypot(diff_pos.x(), diff_pos.y())

        velocity_norm = diff_norm * self._GAIN_REPLACE_BALL_VEL
        if velocity_norm > self._MAX_VELOCITY_OF_REPLACE_BALL:
            velocity_norm = self._MAX_VELOCITY_OF_REPLACE_BALL

        angle = math.atan2(diff_pos.y(), diff_pos.x())
        ball_replacement.vx.append(velocity_norm * math.cos(angle))
        ball_replacement.vy.append(velocity_norm * math.sin(angle))

        replacement = Replacement()
        replacement.ball.append(ball_replacement)
        self._pub_replacement.publish(replacement)

    def _update_robot_num(self):
        # 存在するロボットの台数を計上する
        blue_robot_num = 0
        yellow_robot_num = 0
        for robot in self._detection_tracked.robots:
            if len(robot.visibility) <= 0:
                continue

            if robot.visibility[0] < 0.01:
                continue

            if robot.robot_id.team_color == RobotId.TEAM_COLOR_YELLOW:
                yellow_robot_num += 1
            else:
                blue_robot_num += 1
        self._blue_robot_num = blue_robot_num
        self._yellow_robot_num = yellow_robot_num

    def _reset_draw_area_offset_and_scale(self):
        # 描画領域の移動と拡大・縮小を初期化する
        self._draw_area_offset = QPointF(0.0, 0.0)
        self._draw_area_scale = 1.0
        self._mouse_drag_offset = QPointF(0.0, 0.0)

    def _resize_draw_area(self):
        # Widgetのサイズに合わせて、描画用のフィールドサイズを変更する

        # Widgetの縦横比を算出
        widget_width = float(self.width())
        widget_height = float(self.height())
        widget_w_per_h = widget_width / widget_height

        # フィールドの縦横比を算出
        field_full_width = self._field.field_length + self._field.boundary_width * 2
        field_full_height = self._field.field_width + self._field.boundary_width * 2
        field_w_per_h = field_full_width / field_full_height
        field_h_per_w = 1.0 / field_w_per_h

        # 描画領域のサイズを決める
        # Widgetのサイズによって描画領域を回転するか判定する
        if widget_w_per_h >= field_w_per_h:
            # Widgetが横長のとき
            self._draw_area_size = QSizeF(widget_height * field_w_per_h, widget_height)
            self._do_rotate_draw_area = False

        elif widget_w_per_h <= field_h_per_w:
            # Widgetが縦長のとき
            self._draw_area_size = QSizeF(widget_width * field_w_per_h, widget_width)
            self._do_rotate_draw_area = True

        else:
            # 描画回転にヒステリシスをもたせる
            if self._do_rotate_draw_area is True:
                self._draw_area_size = QSizeF(widget_height, widget_height * field_h_per_w)
            else:
                self._draw_area_size = QSizeF(widget_width, widget_width * field_h_per_w)

        self._scale_field_to_draw = self._draw_area_size.width() / field_full_width

    def _draw_geometry(self, painter):
        # フィールド形状の描画

        # グリーンカーペットを描画
        painter.setBrush(self._COLOR_FIELD_CARPET)
        rect = QRectF(
            QPointF(
                -self._draw_area_size.width() * 0.5,
                -self._draw_area_size.height() * 0.5,
            ),
            self._draw_area_size,
        )
        painter.drawRect(rect)

        # 白線を描画
        painter.setPen(QPen(self._COLOR_FIELD_LINE, self._THICKNESS_FIELD_LINE))
        for line in self._field.field_lines:
            p1 = self._convert_field_to_draw_point(line.p1.x, line.p1.y)
            p2 = self._convert_field_to_draw_point(line.p2.x, line.p2.y)
            painter.drawLine(p1, p2)

        for arc in self._field.field_arcs:
            top_left = self._convert_field_to_draw_point(
                arc.center.x - arc.radius, arc.center.y + arc.radius
            )
            size = arc.radius * 2 * self._scale_field_to_draw
            rect = QRectF(top_left, QSizeF(size, size))

            # angle must be 1/16 degrees order
            start_angle = int(math.degrees(arc.a1) * 16)
            end_angle = int(math.degrees(arc.a2) * 16)
            span_angle = end_angle - start_angle
            painter.drawArc(rect, start_angle, span_angle)

        # ペナルティポイントを描画
        painter.setBrush(self._COLOR_FIELD_LINE)
        PENALTY_X = self._field.field_length * 0.5 - 8000  # ルールの2.1.3 Field Markingsを参照
        PENALTY_DRAW_SIZE = 50 * self._scale_field_to_draw
        point = self._convert_field_to_draw_point(PENALTY_X, 0.0)
        painter.drawEllipse(point, PENALTY_DRAW_SIZE, PENALTY_DRAW_SIZE)
        point = self._convert_field_to_draw_point(-PENALTY_X, 0.0)
        painter.drawEllipse(point, PENALTY_DRAW_SIZE, PENALTY_DRAW_SIZE)

        # ゴールを描画
        painter.setPen(QPen(Qt.black, self._THICKNESS_FIELD_LINE))
        painter.setBrush(self._COLOR_FIELD_CARPET)
        rect = QRectF(
            self._convert_field_to_draw_point(
                self._field.field_length * 0.5, self._field.goal_width * 0.5
            ),
            QSizeF(
                self._field.goal_depth * self._scale_field_to_draw,
                self._field.goal_width * self._scale_field_to_draw,
            ),
        )
        painter.drawRect(rect)

        painter.setPen(QPen(Qt.black, self._THICKNESS_FIELD_LINE))
        rect = QRectF(
            self._convert_field_to_draw_point(
                -self._field.field_length * 0.5 - self._field.goal_depth,
                self._field.goal_width * 0.5,
            ),
            QSizeF(
                self._field.goal_depth * self._scale_field_to_draw,
                self._field.goal_width * self._scale_field_to_draw,
            ),
        )
        painter.drawRect(rect)

    def _draw_detection(self, painter):
        # detectionトピックのロボット・ボールの描画

        for detection in self._detections.values():
            for robot in detection.robots_yellow:
                self._draw_detection_yellow_robot(painter, robot, detection.camera_id)

            for robot in detection.robots_blue:
                self._draw_detection_blue_robot(painter, robot, detection.camera_id)

            for ball in detection.balls:
                self._draw_detection_ball(painter, ball, detection.camera_id)

    def _draw_detection_tracked(self, painter):
        # detection_trackedトピックのロボット・ボールの描画
        for robot in self._detection_tracked.robots:
            self._draw_tracked_robot(painter, robot)

        for ball in self._detection_tracked.balls:
            self._draw_tracked_ball(painter, ball)

    def _draw_replacement(self, painter):
        # grSim Replacementの描画
        for detection in self._detections.values():
            for ball in detection.balls:
                self._draw_replacement_ball(painter, ball)

    def _draw_detection_ball(self, painter, ball, camera_id=-1):
        # detectionトピックのボールを描画する
        point = self._convert_field_to_draw_point(ball.x, ball.y)
        size = self._RADIUS_BALL * self._scale_field_to_draw

        painter.setPen(self._COLOR_BALL)
        painter.setBrush(self._COLOR_BALL)
        painter.drawEllipse(point, size, size)

    def _draw_tracked_ball(self, painter, ball):
        VELOCITY_THRESH = 0.3  # m/s
        PAINT_LENGTH = 10.0  # meters

        # detection_trackedトピックのボールを描画する

        # visibilityが下がるほど、色を透明にする
        color_pen = QColor(Qt.black)
        color_brush = QColor(self._COLOR_BALL)
        if len(ball.visibility) > 0:
            color_pen.setAlpha(int(255 * ball.visibility[0]))
            color_brush.setAlpha(int(255 * ball.visibility[0]))
        else:
            color_pen.setAlpha(0)
            color_brush.setAlpha(0)

        painter.setPen(color_pen)
        painter.setBrush(color_brush)
        point_pos = self._convert_field_to_draw_point(
            ball.pos.x * 1000, ball.pos.y * 1000
        )  # meters to mm
        size = self._RADIUS_BALL * self._scale_field_to_draw
        painter.drawEllipse(point_pos, size, size)

        # ボール速度を描画する
        if len(ball.vel) == 0:
            return
        velocity = ball.vel[0]

        # 速度が小さければ描画しない
        if math.hypot(velocity.x, velocity.y) < VELOCITY_THRESH:
            return
        direction = math.atan2(velocity.y, velocity.x)
        vel_end_x = PAINT_LENGTH * math.cos(direction) + ball.pos.x
        vel_end_y = PAINT_LENGTH * math.sin(direction) + ball.pos.y
        point_vel = self._convert_field_to_draw_point(
            vel_end_x * 1000, vel_end_y * 1000
        )  # meters to mm

        painter.setPen(QPen(QColor(102, 0, 255), 2))
        painter.drawLine(point_pos, point_vel)

    def _draw_detection_yellow_robot(self, painter, robot, camera_id=-1):
        # detectionトピックの黄色ロボットを描画する
        self._draw_detection_robot(painter, robot, self._COLOR_YELLOW_ROBOT, camera_id)

    def _draw_detection_blue_robot(self, painter, robot, camera_id=-1):
        # detectionトピックの青色ロボットを描画する
        self._draw_detection_robot(painter, robot, self._COLOR_BLUE_ROBOT, camera_id)

    def _draw_detection_robot(self, painter, robot, color, camera_id=-1):
        # ロボットを描画する
        point = self._convert_field_to_draw_point(robot.x, robot.y)
        size = self._RADIUS_ROBOT * self._scale_field_to_draw

        painter.setPen(color)
        painter.setBrush(color)
        painter.drawEllipse(point, size, size)

        painter.setPen(Qt.black)
        # ロボット角度
        if len(robot.orientation) > 0:
            line_x = self._RADIUS_ROBOT * math.cos(robot.orientation[0])
            line_y = self._RADIUS_ROBOT * math.sin(robot.orientation[0])
            line_point = point + self._convert_field_to_draw_point(line_x, line_y)
            painter.drawLine(point, line_point)

        # ロボットID
        if len(robot.robot_id) > 0:
            text_point = point + self._convert_field_to_draw_point(
                self._ID_POS.x(), self._ID_POS.y()
            )
            painter.drawText(text_point, str(robot.robot_id[0]))

    def _draw_tracked_robot(self, painter, robot):
        # detection_trackedトピックのロボットを描画する
        color_pen = QColor(Qt.black)
        color_brush = QColor(self._COLOR_BLUE_ROBOT)
        if robot.robot_id.team_color == RobotId.TEAM_COLOR_YELLOW:
            color_brush = QColor(self._COLOR_YELLOW_ROBOT)

        # visibilityが下がるほど、色を透明にする
        if len(robot.visibility) > 0:
            color_brush.setAlpha(int(255 * robot.visibility[0]))
            # ペンの色はvisibilityが0になるまで透明度を下げない
            if robot.visibility[0] > 0.01:
                color_pen.setAlpha(255)
            else:
                color_pen.setAlpha(0)
        else:
            color_pen.setAlpha(0)
            color_brush.setAlpha(0)

        self._draw_goal_pose(painter, robot)

        painter.setPen(color_pen)
        painter.setBrush(color_brush)
        point = self._convert_field_to_draw_point(
            robot.pos.x * 1000, robot.pos.y * 1000
        )  # meters to mm
        size = self._RADIUS_ROBOT * self._scale_field_to_draw
        painter.drawEllipse(point, size, size)

        # ロボット角度
        line_x = self._RADIUS_ROBOT * math.cos(robot.orientation)
        line_y = self._RADIUS_ROBOT * math.sin(robot.orientation)
        line_point = point + self._convert_field_to_draw_point(line_x, line_y)
        painter.drawLine(point, line_point)

        # ロボットID
        text_point = point + self._convert_field_to_draw_point(self._ID_POS.x(), self._ID_POS.y())
        painter.drawText(text_point, str(robot.robot_id.id))

    def _draw_goal_pose(self, painter, robot):
        # goal_poseトピックを描画する
        # ロボットの情報も参照するため、draw_tracked_robot()から呼び出すこと

        robot_id = robot.robot_id.id

        # robot_commandが存在しなければ終了
        robot_command = self._robot_commands.get(robot_id)
        if robot_command is None or robot_command.motion_mode_enable:
            return

        # team_colorが一致しなければ終了
        robot_is_yellow = robot.robot_id.team_color == RobotId.TEAM_COLOR_YELLOW
        if robot_is_yellow is not self.team_is_yellow:
            return

        brush_color = QColor("silver")
        line_color = QColor("red")
        self._draw_goal_pose_and_line_to_parent(
            painter,
            robot_command,
            robot.pos.x,
            robot.pos.y,
            robot_id,
            self._RADIUS_ROBOT * self._scale_field_to_draw,
            brush_color,
            line_color,
        )

        brush_color.setAlphaF(0.5)
        line_color.setAlphaF(0.5)
        if len(robot_command.target_x) > 1 and len(robot_command.target_y) > 1:
            self._draw_goal_pose_and_line_to_parent(
                painter,
                robot_command,
                robot_command.target_x[0],
                robot_command.target_y[0],
                robot_id,
                self._RADIUS_ROBOT * 0.7 * self._scale_field_to_draw,
                brush_color,
                line_color,
            )

    def _draw_goal_pose_and_line_to_parent(
        self,
        painter,
        robot_command,
        parent_x,
        parent_y,
        robot_id,
        size,
        color_brush,
        color_line_to_parent,
    ):
        if len(robot_command.target_x) == 0 or len(robot_command.target_y) == 0:
            return

        # 目標位置を描画する
        # parentはロボットだったり、回避位置だったりする
        painter.setPen(Qt.black)
        painter.setBrush(color_brush)
        # x,y座標
        point = self._convert_field_to_draw_point(
            robot_command.target_x[0] * 1000, robot_command.target_y[0] * 1000
        )  # meters to mm
        size = self._RADIUS_ROBOT * self._scale_field_to_draw
        painter.drawEllipse(point, size, size)

        # 角度
        theta = 0.0
        if len(robot_command.target_theta) > 0:
            theta = robot_command.target_theta[0]
        line_x = self._RADIUS_ROBOT * math.cos(theta)
        line_y = self._RADIUS_ROBOT * math.sin(theta)
        line_point = point + self._convert_field_to_draw_point(line_x, line_y)
        painter.drawLine(point, line_point)

        # ロボットID
        text_point = point + self._convert_field_to_draw_point(self._ID_POS.x(), self._ID_POS.y())
        painter.drawText(text_point, str(robot_id))

        # goal_poseとロボットを結ぶ直線を引く
        painter.setPen(QPen(color_line_to_parent, 2))
        parent_point = self._convert_field_to_draw_point(
            parent_x * 1000, parent_y * 1000
        )  # meters to mm
        painter.drawLine(point, parent_point)

    def _draw_velocity(self, painter, robot):
        # ロボットの情報も参照するため、draw_tracked_robot()から呼び出すこと

        robot_id = robot.robot_id.id

        # robot_commandが存在しなければ終了
        robot_command = self._robot_commands.get(robot_id)
        if robot_command is None:
            return

        # team_colorが一致しなければ終了
        robot_is_yellow = robot.robot_id.team_color == RobotId.TEAM_COLOR_YELLOW
        if robot_is_yellow is not self.team_is_yellow:
            return

        painter.setPen(Qt.black)
        painter.setBrush(self._COLOR_GOAL_POSE)
        # x,y座標
        point = self._convert_field_to_draw_point(
            robot_command.target_x[0] * 1000, robot_command.target_y[0] * 1000
        )  # meters to mm
        size = self._RADIUS_ROBOT * self._scale_field_to_draw
        painter.drawEllipse(point, size, size)

        # ロボットID
        text_point = point + self._convert_field_to_draw_point(self._ID_POS.x(), self._ID_POS.y())
        painter.drawText(text_point, str(robot_id))

        # goal_poseとロボットを結ぶ直線を引く
        painter.setPen(QPen(Qt.red, 2))
        robot_point = self._convert_field_to_draw_point(
            robot.pos.x * 1000, robot.pos.y * 1000
        )  # meters to mm

        vel_x = self._RADIUS_ROBOT + robot_command.target_x[0] * 1000
        vel_y = self._RADIUS_ROBOT + robot_command.target_y[0] * 1000
        if not robot_command.motion_mode_enable:
            vel_x = 0
            vel_y = 0
        vel_point = robot_point + self._convert_field_to_draw_point(vel_x, vel_y)

        painter.drawLine(vel_point, robot_point)

    def _draw_replacement_ball(self, painter, ball):
        # ボールのreplacementを描画する

        # ボール速度replacement判定エリアの描画
        point = self._convert_field_to_draw_point(ball.x, ball.y)
        size = self._RADIUS_REPLACEMENT_BALL_VEL * self._scale_field_to_draw
        painter.setPen(self._COLOR_REPLACEMENT_VEL_ANGLE)
        painter.setBrush(self._COLOR_REPLACEMENT_VEL_ANGLE)
        painter.drawEllipse(point, size, size)

        # ボール位置replacement判定エリアの描画
        point = self._convert_field_to_draw_point(ball.x, ball.y)
        size = self._RADIUS_REPLACEMENT_BALL_POS * self._scale_field_to_draw
        painter.setPen(self._COLOR_REPLACEMENT_POS)
        painter.setBrush(self._COLOR_REPLACEMENT_POS)
        painter.drawEllipse(point, size, size)

        # ボールのreplacement位置の描画
        if self._clicked_replacement_object == ClickedObject.IS_BALL_POS:
            end_point = self._apply_transpose_to_draw_point(self._mouse_current_point)
            painter.setPen(self._COLOR_REPLACEMENT_POS)
            painter.drawLine(point, end_point)

        # ボールのreplacement速度の描画
        if self._clicked_replacement_object == ClickedObject.IS_BALL_VEL:
            end_point = self._apply_transpose_to_draw_point(self._mouse_current_point)
            painter.setPen(self._COLOR_REPLACEMENT_VEL_ANGLE)
            painter.drawLine(point, end_point)

    def _draw_designated_position(self, painter):
        # ボールプレースメントの目標位置と進入禁止エリアを描画する関数
        if not self._designated_position:
            return
        if not self._detection_tracked.balls:
            return

        # 目標位置とボール位置を取得
        designated_point = self._convert_field_to_draw_point(
            self._designated_position[0].x, self._designated_position[0].y
        )

        ball = self._detection_tracked.balls[0]
        ball_point = self._convert_field_to_draw_point(
            ball.pos.x * 1000, ball.pos.y * 1000
        )  # meters to mm

        # ボール配置しないロボットは、ボールと目標位置を結ぶラインから0.5 m以上離れなければならない
        PROHIBITED_DISTANCE = 500
        color_prohibited = QColor(Qt.black)
        color_prohibited.setAlpha(100)
        painter.setPen(color_prohibited)
        painter.setBrush(color_prohibited)
        size = PROHIBITED_DISTANCE * self._scale_field_to_draw

        # 目標位置周りの進入禁止エリアを描画
        painter.drawEllipse(designated_point, size, size)

        # ボール周りの進入禁止エリアを描画
        painter.drawEllipse(ball_point, size, size)

        # ボールと目標位置を結ぶラインを描画
        pen_line = QPen()
        pen_line.setColor(color_prohibited)
        pen_line.setWidthF(size * 2.0)  # 幅を2倍にする
        painter.setPen(pen_line)
        painter.drawLine(designated_point, ball_point)

        # 目標位置を描画
        size = self._RADIUS_DESIGNATED_POSITION * self._scale_field_to_draw
        painter.setPen(self._COLOR_DESIGNATED_POSITION)
        painter.setBrush(self._COLOR_DESIGNATED_POSITION)
        painter.drawEllipse(designated_point, size, size)

    def _draw_named_targets(self, painter):
        # 名前付きターゲットを描画する
        TARGET_RADIUS = 70  # ターゲット位置の描画直径 mm
        NAME_POS = QPointF(100.0, 100.0)  # ターゲット名の描画座標 mm

        painter.setPen(Qt.black)
        painter.setBrush(Qt.white)

        for name, pose in self._named_targets.items():
            # x,y座標
            point = self._convert_field_to_draw_point(pose.x * 1000, pose.y * 1000)  # meters to mm
            size = TARGET_RADIUS * self._scale_field_to_draw
            painter.drawEllipse(point, size, size)

            # 角度
            line_x = TARGET_RADIUS * math.cos(pose.theta)
            line_y = TARGET_RADIUS * math.sin(pose.theta)
            line_point = point + self._convert_field_to_draw_point(line_x, line_y)
            painter.drawLine(point, line_point)

            # 名前
            text_point = point + self._convert_field_to_draw_point(NAME_POS.x(), NAME_POS.y())
            painter.drawText(text_point, str(name))

    def _convert_field_to_draw_point(self, x, y):
        # フィールド座標系を描画座標系に変換する
        draw_x = x * self._scale_field_to_draw
        draw_y = -y * self._scale_field_to_draw
        point = QPointF(draw_x, draw_y)
        return point

    def _convert_draw_to_field_pos(self, point):
        # 描画座標系をフィールド座標系に変換する

        # スケールを戻す
        field_x = point.x() / self._draw_area_scale
        field_y = point.y() / self._draw_area_scale

        # 移動を戻す
        field_x -= self._draw_area_offset.x() + self._mouse_drag_offset.x()
        field_y -= self._draw_area_offset.y() + self._mouse_drag_offset.y()

        # センタリング
        field_x -= self.width() * 0.5 / self._draw_area_scale
        field_y -= self.height() * 0.5 / self._draw_area_scale

        # 描画エリアの回転を反映
        if self._do_rotate_draw_area:
            field_x, field_y = -field_y, field_x

        field_x /= self._scale_field_to_draw
        field_y /= -self._scale_field_to_draw

        return QPointF(field_x, field_y)

    def _apply_transpose_to_draw_point(self, point):
        # Widget上のポイント（左上が0, 0となる座標系）を、
        # translate、scale、rotate適用後のポイントに変換する

        draw_point = QPointF()

        # 描画中心座標変更の適用
        draw_point.setX(point.x() - self.width() * 0.5)
        draw_point.setY(point.y() - self.height() * 0.5)

        # 描画領域の拡大・縮小の適用
        draw_point /= self._draw_area_scale

        # 描画領域の移動の適用
        draw_point -= self._draw_area_offset

        # 描画領域の回転の適用
        if self._do_rotate_draw_area is True:
            draw_point = QPointF(-draw_point.y(), draw_point.x())

        return draw_point
