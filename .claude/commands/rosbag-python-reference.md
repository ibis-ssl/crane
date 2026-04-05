---
name: rosbag-python-reference
description: "crane_bag CLIで不足する低レベル項目のPython解析リファレンス"
allowed-tools: ["Read", "Grep"]
---

# Crane Rosbag Python フィールドリファレンス

crane_bag CLIでは取得できない低レベルな詳細が必要な場合に、直接MCAPを読んでPythonで解析する際のフィールドリファレンス。

```python
# ===== Referee =====
msg.command.value    # int32: 0=HALT, 1=STOP, 2=NORMAL_START, 3=FORCE_START,
                     #        8=DIRECT_FREE_YELLOW, 9=DIRECT_FREE_BLUE,
                     #        16=BALL_PLACEMENT_YELLOW, 17=BALL_PLACEMENT_BLUE
msg.stage.value      # int32: 0=NORMAL_FIRST_HALF_PRE, 1=NORMAL_FIRST_HALF, etc.
msg.command_counter  # uint32: コマンド変化のたびにインクリメント
msg.yellow.name, msg.yellow.score, msg.yellow.yellow_cards, msg.yellow.red_cards
msg.yellow.foul_counter, msg.yellow.goalkeeper
msg.blue.name, msg.blue.score, ...  # 同上
# optional フィールドは has_field ビットマスクで判定
msg.has_field & 256  # DESIGNATED_POSITION_FIELD_SET
msg.designated_position.x, msg.designated_position.y  # mm単位
msg.has_field & 1024 # NEXT_COMMAND_FIELD_SET
msg.next_command.value
msg.current_action_time_remaining  # マイクロ秒 (int32)
msg.game_events  # list[GameEvent] - 現在のコマンド期間中のイベント累積

# ===== GameEvent =====
ge.type.value  # int32: GameEventType定数
               # 6=BALL_LEFT_FIELD_TOUCH_LINE, 7=BALL_LEFT_FIELD_GOAL_LINE
               # 8=GOAL, 13=KEEPER_HELD_BALL, 17=BOT_DRIBBLED_BALL_TOO_FAR
               # 18=BOT_KICKED_BALL_TOO_FAST, 21=BOT_CRASH_DRAWN, 22=BOT_CRASH_UNIQUE
               # 24=BOT_PUSHED_BOT, 26=BOT_HELD_BALL_DELIBERATELY, 27=BOT_TIPPED_OVER
               # 28=BOT_TOO_FAST_IN_STOP, 31=DEFENDER_IN_DEFENSE_AREA
ge.event.event_which  # int8: active フィールドを示す（0=未設定）
ge.event.bot_crash_unique.by_team.value  # 1=YELLOW, 2=BLUE
ge.event.bot_crash_unique.violator, ge.event.bot_crash_unique.victim  # ロボットID
ge.event.bot_crash_unique.location.x, .y  # 位置（メートル）
ge.event.bot_crash_unique.crash_speed  # m/s
ge.event.bot_too_fast_in_stop.by_bot, .speed, .location.x, .y
ge.event.keeper_held_ball.by_bot, .location.x, .y
ge.origin  # list[str]: イベントの発生元（GC etc.）

# ===== WorldModel =====
ball = msg.ball_info
ball.position.x, ball.position.y   # ボール位置（NOT ball.pose）
ball.velocity.x, ball.velocity.y   # ボール速度
ball.velocity_norm                  # 速度の大きさ
ball.detected, ball.tracker_detected
ball.state                          # 0=STOPPED, 1=ROLLING, 2=FLYING
for r in msg.robot_info_ours:
    r.id, r.pose.x, r.pose.y, r.pose.theta
    r.velocity.x, r.velocity.y
    r.available_vision, r.has_error  # 注意: r.detected は存在しない

# ===== RobotCommand（control_targets / robot_commands の要素）=====
rc.robot_id, rc.stop_flag
rc.control_mode   # 1=SIMPLE_VELOCITY, 2=POSITION_TARGET, 3=POLAR_VELOCITY
rc.kick_power, rc.dribble_power, rc.chip_enable
rc.target_theta
rc.planner_name
rc.polar_velocity_target_mode   # list型（len==0 or 1）
  pv = rc.polar_velocity_target_mode
  pv[0].target_velocity_r, pv[0].target_velocity_theta
rc.position_target_mode         # list型（len==0 or 1）
  pt = rc.position_target_mode
  pt[0].target_x, pt[0].target_y, pt[0].position_tolerance
rc.planning_factors             # list[NamedString]型 — スキル状態の追跡に最重要
  # .name: スキル名や属性（例: "Kick", "Attacker", "CommandAction"）
  # .value: 状態値（例: "AROUND_BALL_AND_KICK", "KICK::GOAL_KICK", "STOP_HERE"）

# ===== PlaySituation =====
msg.command.name   # 例: "INPLAY", "OUR_KICKOFF_START", "HALT"
msg.command.value  # 数値
msg.reason_text    # 遷移理由の説明文

# ===== GameAnalysis =====
msg.recommended_attacker_id      # 推奨アタッカーID（-1=未選択）
msg.attacker_suitability_score   # アタッカー適性スコア
msg.pass_target_id               # パス先ID（-1=未選択）
msg.recommended_pass_receiver_id
msg.ongoing_kick                 # list型（キック中かどうか）
msg.ball_threat, msg.our_slack, msg.their_slack
msg.has_sub_attacker_position

# ===== RobotSelectResults =====
for r in msg.results:
    r.name              # ロール名（例: "attacker_skill", "goalie_skill", "defender"）
    r.selected_robots   # 選択されたロボットIDリスト
    r.selectable_robots # 選択可能なロボットIDリスト
    r.min_robots_num, r.max_robots_num
```
