# 座標系に関して

## フィールド座標系

Visionから来る座標系
長辺方向X，短辺方向Y

## ロボット座標系

前X，左Y，上Z

## RobotCommand

### target_velocity

ロボットが最終的に参照している速度

本当はlocal_plannerでロボット座標系に変換したいが，今はSimSenderでローカル座標系に変換している

### target_pose

current_pose と合わせてロボットが参考にするかも知れない

## crane_teleop
