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
import os
import time

# from frootspi_msgs.msg import BatteryVoltage
from functools import partial

import rclpy
from ament_index_python.resources import get_resource
from consai_visualizer_msgs.msg import ObjectsArray
from consai_visualizer.field_widget import FieldWidget
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QPointF, Qt, QTimer
from python_qt_binding.QtWidgets import QTreeWidgetItem, QWidget
from qt_gui.plugin import Plugin
from robocup_ssl_msgs.msg import BallReplacement, Replacement, RobotReplacement
from rqt_py_common.ini_helper import pack, unpack
from crane_msgs.msg import RobotFeedbackArray


class Visualizer(Plugin):
    def __init__(self, context):
        super(Visualizer, self).__init__(context)

        self.setObjectName("Visualizer")

        self._node = context.node
        self._logger = self._node.get_logger()

        self._widget = QWidget()

        # widgetを読み込む
        # FieldWidgetはカスタムウィジェットとしてuiファイルに設定済み
        pkg_name = "consai_visualizer"
        _, package_path = get_resource("packages", pkg_name)
        ui_file = os.path.join(package_path, "share", pkg_name, "resource", "visualizer.ui")
        loadUi(ui_file, self._widget, {"FieldWidget": FieldWidget})

        # rqtのUIにwidgetを追加する
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (" (%d)" % context.serial_number())
            )
        context.add_widget(self._widget)

        # loggerをセット
        self._widget.field_widget.set_logger(self._logger)
        self._add_visualizer_layer("caption", "caption")

        self._sub_visualize_objects_array = self._node.create_subscription(
            ObjectsArray,
            "visualizer_objects",
            self._callback_visualizer_objects,
            rclpy.qos.qos_profile_sensor_data,
        )

        self.sub_feedback = self._node.create_subscription(
            RobotFeedbackArray,
            "robot_feedback",
            self._callback_feedback,
            rclpy.qos.qos_profile_sensor_data,
        )

        self._pub_replacement = self._node.create_publisher(Replacement, "replacement", 10)

        # Parameterを設定する
        self._widget.field_widget.set_invert(self._node.declare_parameter("invert", False).value)

        for team in ["blue", "yellow"]:
            for turnon in ["on", "off"]:
                method = "self._widget.btn_all_" + turnon + "_" + team + ".clicked.connect"
                eval(method)(
                    partial(
                        self._publish_all_robot_turnon_replacement,
                        team == "yellow",
                        turnon == "on",
                    )
                )

        # レイヤーツリーの初期設定
        self._widget.layer_widget.itemChanged.connect(self._layer_state_changed)

        # 16 msec周期で描画を更新する
        self._timer = QTimer()
        self._timer.timeout.connect(self._widget.field_widget.update)
        self._timer.timeout.connect(self._publish_replacement)
        self._timer.start(16)

        # ロボットの死活監視
        # 1秒以上バッテリーの電圧が来ていないロボットは死んだとみなす
        self.latest_update_time = [0] * 16
        self._reset_timer = QTimer()
        self._reset_timer.timeout.connect(self._update_robot_synthetics)
        self._reset_timer.start(1000)

        self.latest_battery_voltage = [0] * 16

    def _callback_feedback(self, msg):
        for info in msg.feedback:
            try:
                self.latest_battery_voltage[info.robot_id] = info.voltage[0]
            except AttributeError:
                # 初期化より先にコールバックが呼ばれてしまうことがあるため、エラーを回避する
                pass
        # for synthetics

    def save_settings(self, plugin_settings, instance_settings):
        # UIを終了するときに実行される関数

        # layerとsub layerをカンマで結合して保存
        active_layers = self._extract_active_layers()
        combined_layers = list(map(lambda x: x[0] + "," + x[1], active_layers))
        instance_settings.set_value("active_layers", pack(combined_layers))

    def restore_settings(self, plugin_settings, instance_settings):
        # UIが起動したときに実行される関数

        # カンマ結合されたlayerを復元してセット
        combined_layers = unpack(instance_settings.value("active_layers", []))
        active_layers = list(map(lambda x: x.split(","), combined_layers))
        for layer, sub_layer in active_layers:
            self._add_visualizer_layer(layer, sub_layer, Qt.Checked)

    def _layer_state_changed(self):
        # レイヤーのチェックボックスが変更されたときに呼ばれる
        # 一括でON/OFFすると項目の数だけ実行される
        active_layers = self._extract_active_layers()
        self._widget.field_widget.set_active_layers(active_layers)

    def _callback_visualizer_objects(self, msg):
        # ここでレイヤーを更新する
        for objects in msg.objects:
            self._add_visualizer_layer(objects.layer, objects.sub_layer)
            self._widget.field_widget.set_visualizer_objects(objects)

    def _add_visualizer_layer(self, layer: str, sub_layer: str, state=Qt.Unchecked):
        # レイヤーに重複しないように項目を追加する
        if layer == "" or sub_layer == "":
            self._logger.warning("layer={} or sub_layer={} is empty".format(layer, sub_layer))
            return

        parents = self._widget.layer_widget.findItems(layer, Qt.MatchExactly, 0)

        if len(parents) == 0:
            new_parent = QTreeWidgetItem(self._widget.layer_widget)
            new_parent.setText(0, layer)
            new_parent.setFlags(new_parent.flags() | Qt.ItemIsTristate | Qt.ItemIsUserCheckable)

            new_child = QTreeWidgetItem(new_parent)
        else:
            for i in range(parents[0].childCount()):
                if sub_layer == parents[0].child(i).text(0):
                    return
            new_child = QTreeWidgetItem(parents[0])

        new_child.setText(0, sub_layer)
        new_child.setFlags(new_child.flags() | Qt.ItemIsUserCheckable)
        new_child.setCheckState(0, state)

    def _extract_active_layers(self) -> list[tuple[str, str]]:
        # チェックが入ったレイヤーを抽出する
        active_layers = []
        for index in range(self._widget.layer_widget.topLevelItemCount()):
            parent = self._widget.layer_widget.topLevelItem(index)
            if parent.checkState(0) == Qt.Unchecked:
                continue

            for child_index in range(parent.childCount()):
                child = parent.child(child_index)
                if child.checkState(0) != Qt.Checked:
                    continue
                active_layers.append((parent.text(0), child.text(0)))

        return active_layers

    def _publish_replacement(self) -> None:
        # 描画領域のダブルクリック操作が完了したら、grSimのReplacement情報をpublishする
        if not self._widget.field_widget.get_mouse_double_click_updated():
            return
        start, end = self._widget.field_widget.get_mouse_double_click_points()
        self._widget.field_widget.reset_mouse_double_click_updated()

        # チェックが入ったボタン情報を解析する
        button = self._widget.radio_buttons.checkedButton()
        if button.text() == "NONE":
            return
        elif button.text() == "Ball":
            self._publish_ball_replacement(start, end)
            return
        else:
            is_yellow = False
            if button.text()[0] == "Y":
                is_yellow = True
            robot_id = int(button.text()[1:])
            self._publish_robot_replacement(start, end, is_yellow, robot_id)
            return

    def _publish_ball_replacement(self, start: QPointF, end: QPointF) -> None:
        velocity = end - start
        ball_replacement = BallReplacement()
        ball_replacement.x.append(start.x())
        ball_replacement.y.append(start.y())
        ball_replacement.vx.append(velocity.x())
        ball_replacement.vy.append(velocity.y())
        replacement = Replacement()
        replacement.ball.append(ball_replacement)
        self._pub_replacement.publish(replacement)

    def _publish_robot_replacement(
        self, start: QPointF, end: QPointF, is_yellow: bool, robot_id: int
    ) -> None:
        theta_deg = math.degrees(math.atan2(end.y() - start.y(), end.x() - start.x()))

        robot_replacement = RobotReplacement()
        robot_replacement.x = start.x()
        robot_replacement.y = start.y()
        robot_replacement.dir = theta_deg
        robot_replacement.id = robot_id
        robot_replacement.yellowteam = is_yellow
        robot_replacement.turnon.append(True)
        replacement = Replacement()
        replacement.robots.append(robot_replacement)
        self._pub_replacement.publish(replacement)

    def _publish_all_robot_turnon_replacement(self, is_yellow, turnon):
        # turnon時はフィールド内に、turnoff時はフィールド外に、ID順でロボットを並べる
        OFFSET_X = 0.2
        team_offset_x = -3.0 if is_yellow else 0.0
        turnon_offset_y = -4.6 if turnon else -5.6

        replacement = Replacement()
        for i in range(11):
            robot_replacement = RobotReplacement()
            robot_replacement.x = team_offset_x + OFFSET_X * i
            robot_replacement.y = turnon_offset_y
            robot_replacement.dir = 90.0
            robot_replacement.id = i
            robot_replacement.yellowteam = is_yellow
            robot_replacement.turnon.append(turnon)
            replacement.robots.append(robot_replacement)
        self._pub_replacement.publish(replacement)

    def _update_robot_synthetics(self):
        # n秒以上バッテリーの電圧が来ていないロボットは死んだとみなす
        now = time.time()

        for i in range(16):
            diff_time = now - self.latest_update_time[i]

            try:
                getattr(self._widget, f"robot{i}_voltage").setText(
                    str(self.latest_battery_voltage[i])
                )
            except AttributeError:
                try:
                    getattr(self._widget, f"robot{i}_voltage").setText(str(0.0))
                except AttributeError:
                    pass
                pass
            if diff_time > 3.0:  # 死んだ判定
                # DEATH
                try:
                    getattr(self._widget, f"robot{i}_connection_status").setText("❌")
                except AttributeError:
                    # ロボット状態表示UIは12列しか用意されておらず、ID=12以降が来るとエラーになるため回避
                    pass
            else:
                # ALIVE
                try:
                    getattr(self._widget, f"robot{i}_connection_status").setText("👍")
                except AttributeError:
                    # ロボット状態表示UIは12列しか用意されておらず、ID=12以降が来るとエラーになるため回避
                    pass
