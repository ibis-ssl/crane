#!/usr/bin/env python2
# coding: UTF-8

#  Copyright (c) 2022 ibis-ssl
#
#  Permission is hereby granted, free of charge, to any person obtaining a copy
#  of this software and associated documentation files (the "Software"), to deal
#  in the Software without restriction, including without limitation the rights
#  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#  copies of the Software, and to permit persons to whom the Software is
#  furnished to do so, subject to the following conditions:
#
#  The above copyright notice and this permission notice shall be included in
#  all copies or substantial portions of the Software.
#
#  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
#  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
#  THE SOFTWARE.

import rospy
import math
import copy
from consai2_msgs.msg import RobotCommand, RobotCommands, ControlTarget
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Pose2D


def angle_normalize(angle):
    # 角度をpi  ~ -piの範囲に変換する
    while angle > math.pi:
        angle -= 2*math.pi

    while angle < -math.pi:
        angle += 2*math.pi

    return angle


class JoyWrapper(object):
    def __init__(self):

        self._MAX_ID = rospy.get_param('consai2_description/max_id')
        # 直接操縦かcontrol経由の操縦かを決める
        self._DIRECT = rospy.get_param('~direct')
        # /consai2_examples/launch/joystick_example.launch でキー割り当てを変更する
        self._BUTTON_SHUTDOWN_1 = rospy.get_param('~button_shutdown_1')
        self._BUTTON_SHUTDOWN_2 = rospy.get_param('~button_shutdown_2')

        self._BUTTON_MOVE_ENABLE = rospy.get_param('~button_move_enable')
        self._AXIS_VEL_SURGE = rospy.get_param('~axis_vel_surge')
        self._AXIS_VEL_SWAY = rospy.get_param('~axis_vel_sway')
        self._AXIS_VEL_ANGULAR = rospy.get_param('~axis_vel_angular')

        self._BUTTON_KICK_ENABLE = rospy.get_param('~button_kick_enable')
        self._BUTTON_KICK_STRAIGHT = rospy.get_param('~button_kick_straight')
        self._BUTTON_KICK_CHIP = rospy.get_param('~button_kick_chip')
        self._AXIS_KICK_POWER = rospy.get_param('~axis_kick_power')

        self._BUTTON_DRIBBLE_ENABLE = rospy.get_param('~button_dribble_enable')
        self._AXIS_DRIBBLE_POWER = rospy.get_param('~axis_dribble_power')

        self._BUTTON_ID_ENABLE = rospy.get_param('~button_id_enable')
        self._AXIS_ID_CHANGE = rospy.get_param('~axis_id_change')

        self._BUTTON_COLOR_ENABLE = rospy.get_param('~button_color_enable')
        self._AXIS_COLOR_CHANGE = rospy.get_param('~axis_color_change')

        self._BUTTON_ALL_ID_1 = rospy.get_param('~button_all_id_1')
        self._BUTTON_ALL_ID_2 = rospy.get_param('~button_all_id_2')
        self._BUTTON_ALL_ID_3 = rospy.get_param('~button_all_id_3')
        self._BUTTON_ALL_ID_4 = rospy.get_param('~button_all_id_4')

        # indirect control用の定数
        self._BUTTON_PATH_ENABLE = rospy.get_param('~button_path_enable')
        self._BUTTON_ADD_POSE = rospy.get_param('~button_add_pose')
        self._BUTTON_DELETE_PATH = rospy.get_param('~button_delete_path')
        self._BUTTON_SEND_TARGET = rospy.get_param('~button_send_target')

        self._MAX_VEL_SURGE = 1.0
        self._MAX_VEL_SWAY = 1.0
        self._MAX_VEL_ANGULAR = math.pi

        self._MAX_KICK_POWER = 1.0  # DO NOT EDIT
        self._KICK_POWER_CONTROL = 0.1

        self._MAX_DRIBBLE_POWER = 1.0  # DO NOT EDIT
        self._DRIBBLE_POWER_CONTROL = 0.1

        self._indirect_control_enable = True

        # direct control用のパラメータ
        self._kick_power = 0.5
        self._dribble_power = 0.5
        self._robot_id = 0
        self._is_yellow = False
        self._all_member = False

        self._pub_commands = rospy.Publisher(
                'consai2_control/robot_commands', RobotCommands, queue_size=1)

        self._joy_msg = None
        self._sub_joy = rospy.Subscriber(
                'joy', Joy, self._callback_joy, queue_size=1)
        self._pub_joy_target = rospy.Publisher(
                'consai2_examples/joy_target', ControlTarget, queue_size=1)
        self._joy_target = ControlTarget()
        self._joy_target.path.append(Pose2D())  # スタート位置をセット

    def _callback_joy(self, msg):
        self._joy_msg = msg
        # rospy.loginfo("en=%d =%f",
        #     self._joy_msg.buttons[self._BUTTON_ID_ENABLE],
        #     self._joy_msg.axes[self._AXIS_KICK_POWER])

    def _direct_control_update(self):
        # 直接ロボットを操縦する
        robot_commands = RobotCommands()
        robot_commands.header.stamp = rospy.Time.now()

        command = RobotCommand()

        # メッセージを取得してない場合は抜ける
        if self._joy_msg is None:
            return

        # シャットダウン
        if self._joy_msg.buttons[self._BUTTON_SHUTDOWN_1] and\
                self._joy_msg.buttons[self._BUTTON_SHUTDOWN_2]:
            rospy.signal_shutdown('finish')
            return

        # チームカラーの変更
        if self._joy_msg.buttons[self._BUTTON_COLOR_ENABLE]:
            if math.fabs(self._joy_msg.axes[self._AXIS_COLOR_CHANGE]) > 0:
                self._is_yellow = not self._is_yellow
                print('is_yellow: {}'.format(self._is_yellow))
                # キーが離れるまでループ
                while self._joy_msg.axes[self._AXIS_COLOR_CHANGE] != 0:
                    pass

        # IDの変更
        if self._joy_msg.buttons[self._BUTTON_ID_ENABLE]:
            if math.fabs(self._joy_msg.axes[self._AXIS_ID_CHANGE]) > 0:
                # 十字キーの入力に合わせて、IDを増減させる
                self._robot_id += int(self._joy_msg.axes[self._AXIS_ID_CHANGE])

                if self._robot_id > self._MAX_ID:
                    self._robot_id = self._MAX_ID
                if self._robot_id < 0:
                    self._robot_id = 0
                print('robot_id: {}'.format(self._robot_id))
                # キーが離れるまでループ
                while self._joy_msg.axes[self._AXIS_ID_CHANGE] != 0:
                    pass

        # 全ID操作の変更
        if self._joy_msg.buttons[self._BUTTON_ALL_ID_1] and \
                self._joy_msg.buttons[self._BUTTON_ALL_ID_2] and \
                self._joy_msg.buttons[self._BUTTON_ALL_ID_3] and \
                self._joy_msg.buttons[self._BUTTON_ALL_ID_4]:
            self._all_member = not self._all_member
            print('all_member: {}'.format(self._all_member))
            # キーが離れるまでループ
            while self._joy_msg.buttons[self._BUTTON_ALL_ID_1] != 0 or \
                    self._joy_msg.buttons[self._BUTTON_ALL_ID_2] != 0 or\
                    self._joy_msg.buttons[self._BUTTON_ALL_ID_3] != 0 or\
                    self._joy_msg.buttons[self._BUTTON_ALL_ID_4] != 0:
                pass

        # 走行
        if self._joy_msg.buttons[self._BUTTON_MOVE_ENABLE]:
            command.vel_surge = self._joy_msg.axes[self._AXIS_VEL_SURGE] \
                * self._MAX_VEL_SURGE
            command.vel_sway = self._joy_msg.axes[self._AXIS_VEL_SWAY] \
                * self._MAX_VEL_SWAY
            command.vel_angular = self._joy_msg.axes[self._AXIS_VEL_ANGULAR] \
                * self._MAX_VEL_ANGULAR

        # キック
        if self._joy_msg.buttons[self._BUTTON_KICK_ENABLE]:
            if self._joy_msg.buttons[self._BUTTON_KICK_STRAIGHT]:
                command.kick_power = self._kick_power
            elif self._joy_msg.buttons[self._BUTTON_KICK_CHIP]:
                command.kick_power = self._kick_power
                command.chip_enable = True

            # パワーの変更
            axis_value = self._joy_msg.axes[self._AXIS_KICK_POWER]
            if math.fabs(axis_value) > 0:
                self._kick_power += math.copysign(
                        self._KICK_POWER_CONTROL, axis_value)
                if self._kick_power > self._MAX_KICK_POWER:
                    self._kick_power = self._MAX_KICK_POWER
                if self._kick_power < 0.001:
                    self._kick_power = 0.0
                print('kick_power: {}'.format(self._kick_power))
                # キーが離れるまでループ
                while self._joy_msg.axes[self._AXIS_KICK_POWER] != 0:
                    pass

        # ドリブラー
        if self._joy_msg.buttons[self._BUTTON_DRIBBLE_ENABLE]:
            command.dribble_power = self._dribble_power

            # パワーの変更
            axis_value = self._joy_msg.axes[self._AXIS_DRIBBLE_POWER]
            if math.fabs(axis_value) > 0:
                self._dribble_power += math.copysign(
                        self._DRIBBLE_POWER_CONTROL, axis_value)
                if self._dribble_power > self._MAX_DRIBBLE_POWER:
                    self._dribble_power = self._MAX_DRIBBLE_POWER
                if self._dribble_power < 0.001:
                    self._dribble_power = 0.0
                print('dribble_power: {}'.format(self._dribble_power))
                # キーが離れるまでループ
                while self._joy_msg.axes[self._AXIS_DRIBBLE_POWER] != 0:
                    pass

        # チームカラーをセット
        robot_commands.is_yellow = self._is_yellow

        # IDとコマンドをセット
        if self._all_member:
            # 全IDのロボットを動かす
            for robot_id in range(self._MAX_ID + 1):
                command.robot_id = robot_id
                robot_commands.commands.append(copy.deepcopy(command))
        else:
            command.robot_id = self._robot_id
            robot_commands.commands.append(command)

        rospy.loginfo("surge=%f sway=%f omega=%f kick=%f chip=%f dribble=%f ",
                      command.vel_surge, command.vel_sway, command.vel_angular,
                      command.kick_power, command.chip_enable,
                      command.dribble_power)

        self._pub_commands.publish(robot_commands)

    def _indirect_control_update(self):
        # Controller経由でロボットを操縦する

        MOVE_GAIN = 0.04  # meters
        MOVE_GAIN_ANGLE = 0.04 * math.pi  # radians

        # メッセージを取得してない場合は抜ける
        if self._joy_msg is None:
            return

        # ID・Color変更用の変数
        prev_id = copy.deepcopy(self._robot_id)
        prev_is_yellow = copy.deepcopy(self._is_yellow)

        current_joy_pose = self._joy_target.path[-1]

        # シャットダウン
        if self._joy_msg.buttons[self._BUTTON_SHUTDOWN_1] and\
                self._joy_msg.buttons[self._BUTTON_SHUTDOWN_2]:
            rospy.signal_shutdown('finish')
            return

        # チームカラーの変更
        if self._joy_msg.buttons[self._BUTTON_COLOR_ENABLE]:
            # カラー変更直後は操縦を停止する
            self._indirect_control_enable = False

            if math.fabs(self._joy_msg.axes[self._AXIS_COLOR_CHANGE]) > 0:
                self._is_yellow = not self._is_yellow
                print('is_yellow: {}'.format(self._is_yellow))
                # キーが離れるまでループ
                while self._joy_msg.axes[self._AXIS_COLOR_CHANGE] != 0:
                    pass

        # IDの変更
        if self._joy_msg.buttons[self._BUTTON_ID_ENABLE]:
            # ID変更直後は操縦を停止する
            self._indirect_control_enable = False

            if math.fabs(self._joy_msg.axes[self._AXIS_ID_CHANGE]) > 0:
                # 十字キーの入力に合わせて、IDを増減させる
                self._robot_id += int(self._joy_msg.axes[self._AXIS_ID_CHANGE])

                if self._robot_id > self._MAX_ID:
                    self._robot_id = self._MAX_ID
                if self._robot_id < 0:
                    self._robot_id = 0
                print('robot_id: {}'.format(self._robot_id))
                # キーが離れるまでループ
                while self._joy_msg.axes[self._AXIS_ID_CHANGE] != 0:
                    pass

        # poseの更新
        if self._joy_msg.buttons[self._BUTTON_MOVE_ENABLE]:
            # 操縦を許可する
            self._indirect_control_enable = True

            if math.fabs(self._joy_msg.axes[self._AXIS_VEL_SWAY]):
                # joystickの仕様により符号を反転している
                gain = MOVE_GAIN*self._joy_msg.axes[self._AXIS_VEL_SWAY]
                current_joy_pose.x += math.copysign(
                        gain, -self._joy_msg.axes[self._AXIS_VEL_SWAY])

            if math.fabs(self._joy_msg.axes[self._AXIS_VEL_SURGE]):
                gain = MOVE_GAIN*self._joy_msg.axes[self._AXIS_VEL_SURGE]
                current_joy_pose.y += math.copysign(
                        gain, self._joy_msg.axes[self._AXIS_VEL_SURGE])

            if math.fabs(self._joy_msg.axes[self._AXIS_VEL_ANGULAR]):
                gain = MOVE_GAIN_ANGLE * self._joy_msg.axes[
                    self._AXIS_VEL_ANGULAR]
                current_joy_pose.theta += math.copysign(
                        gain, self._joy_msg.axes[self._AXIS_VEL_ANGULAR])

                current_joy_pose.theta = angle_normalize(
                        current_joy_pose.theta)

        # パスの末尾にセット
        self._joy_target.path[-1] = current_joy_pose

        # Pathの操作
        if self._joy_msg.buttons[self._BUTTON_PATH_ENABLE]:
            # 操縦を許可する
            self._indirect_control_enable = True

            # Poseの追加
            if self._joy_msg.buttons[self._BUTTON_ADD_POSE]:
                self._joy_target.path.append(copy.deepcopy(current_joy_pose))
                print('add pose: {}'.format(len(self._joy_target.path)))
                # キーが離れるまでループ
                while self._joy_msg.buttons[self._BUTTON_ADD_POSE] != 0:
                    pass

            # Pathの初期化
            if self._joy_msg.buttons[self._BUTTON_DELETE_PATH]:
                self._joy_target.path = []
                self._joy_target.path.append(Pose2D())
                print('delete path')
                # キーが離れるまでループ
                while self._joy_msg.buttons[self._BUTTON_DELETE_PATH] != 0:
                    pass

            # ControlTargetの送信
            if self._joy_msg.buttons[self._BUTTON_SEND_TARGET]:
                self._joy_target.robot_id = self._robot_id
                self._joy_target.control_enable = True

                color = 'blue'
                if self._is_yellow:
                    color = 'yellow'

                # 末尾に16進数の文字列をつける
                topic_id = hex(self._robot_id)[2:]
                topic_name = 'consai2_game/control_target_' \
                    + color + '_' + topic_id
                pub_control_target = rospy.Publisher(
                        topic_name, ControlTarget, queue_size=1)

                pub_control_target.publish(self._joy_target)
                # self._pub_control_target.publish(self._joy_target)
                print('send target')

        # Color, ID変更ボタンを押すことで操縦を停止できる
        if self._indirect_control_enable is False:
            stop_target = ControlTarget()
            stop_target.control_enable = True
            stop_target.goal_velocity = Pose2D(0, 0, 0)

            stop_target.robot_id = prev_id
            color = 'blue'
            if prev_is_yellow:
                color = 'yellow'

            # 末尾に16進数の文字列をつける
            topic_id = hex(prev_id)[2:]
            topic_name = 'consai2_game/control_target_' \
                + color + '_' + topic_id
            pub_control_target = rospy.Publisher(
                    topic_name, ControlTarget, queue_size=1)

            pub_control_target.publish(stop_target)

        self._pub_joy_target.publish(self._joy_target)

    def update(self):
        if self._DIRECT:
            self._direct_control_update()
        else:
            self._indirect_control_update()


def main():
    rospy.init_node('joystick_example')

    joy_wrapper = JoyWrapper()

    r = rospy.Rate(60)
    while not rospy.is_shutdown():
        joy_wrapper.update()
        r.sleep()


if __name__ == '__main__':
    main()
