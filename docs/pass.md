# パス連携システム

- 本ドキュメントは Crane におけるパスの上位決定・受け手予約・受取準備・スイッチ抑制の仕組みを説明します。
- 目的: キック中に受け手が別スキルへ移行して取りこぼす事象を低減し、安定したパス連携を実現すること。

## 全体像

- 上位層でパス先ロボットを決定し、WorldModel の GameAnalysis に載せて配信。
- Attacker は配信されたパス先に基づいてキックを実施。
- 受け手側は PassReceiverPlanner が予約（指名）し、Receive スキルを割り当てて受け準備。
- 頻繁なパス先フリップを抑制するため、上位層の選定にヒステリシスを導入。

## 関係コンポーネント

- `crane_world_model_publisher`:
  - `PassTargetSelector` が `GameAnalysis.pass_scores` を算出し、`pass_target_id` を選定・配信。
  - 連続切替え抑制（ヒステリシス）と可視化出力を実装。
- `crane_tactic_coordinator`:
  - `GameAnalysis.pass_target_id` を参照してパス連携を調整。
  - `pass_receive` セッションで受け手ロボットに Receive スキルを割当（予約）。
- `crane_robot_skills::Attacker`:
  - `pass_target_id` を優先してパス先・キックターゲットを決定。
- `PassReceiverPlanner`:
  - 受け手ロボットに `Receive` を割当。キック中は能動受取、キック前はその場停止しつつボールを注視。

## メッセージ拡張

- `crane_msgs/msg/analysis/GameAnalysis.msg` に以下を追加:
  - `int32 pass_target_id` (-1 の場合は未選択)
- 既存の `FloatWithID[] pass_scores` は維持（降順、先頭が最良）。

## 選定とスイッチ抑制（上位レイヤ）

- 実装: `crane_world_model_publisher` の `PassTargetSelector::update()` が `pass_scores` を計算し、ヒステリシスを考慮して `pass_target_id` を決定。
- 抑制パラメータ（ROS 2 パラメータ）:
  - `pass_target.min_hold_duration_sec` 既定 0.5
  - `pass_target.min_improvement_margin` 既定 0.2
- 振る舞い:
  - 最後に選ばれたターゲットは最低0.5秒保持される（値はパラメータで可変）。
  - 改善値がしきい値を超えた場合のみ切り替え（わずかな差でのフリップを防止）。

## パス評価の基準点（動くボール対応）

- パス評価時に使用するボール基準点は次の通り:
  - ボール停止時: 現在のボール位置。
  - ボール移動時: 予測停止位置（`getPredictedPosition(getStopTime())`）。
- これにより、移動中のボールでも一貫したパス線（基準点→受け手）評価が可能になり、誤評価を低減します。

## Attacker の動作

- `AttackerState::KICK` と `FORCED_PASS` で `game_analysis.pass_target_id >= 0` を優先。
- パス先が自分自身またはキーパーの場合はフォールバック（従来ロジック）。

## 受け手予約と受取準備

- `TacticCoordinator` が `GameAnalysis.pass_target_id` を参照。
- `PassReceiverPlanner` が該当 ID を予約し、`Receive` スキルを割り当てて実行。
  - ボールが十分動いている、または `ongoing_kick.is_kicker_friend` が真のときは能動受け取り（`Receive::update()` 実行）。
  - キック前は整列動作を行わず、その場で停止しボールを注視。
  - 2025シーズンでは、PassTargetSelectorの確定まで待機する「遅延切替」ロジックが標準となり、受け手のスキルフリップが減少。

## セッション設定（例）

統一設定ファイル `session/crane_tactic_coordinator/config/unified_session_config.yaml` で設定：

```yaml
situations:
  INPLAY:
    sessions:
      - name: attacker_skill
        capacity: 1
      - name: pass_receive
        capacity: 1
      # その他のセッション
```

推奨順序: `attacker_skill` の直後に `pass_receive` を配置。

## Receive スキル主要パラメータ

- `policy`: `closest`（既定）, `min_slack`, `max_slack`
- `enable_active_receive`: true（既定）
- `enable_redirect`: false（既定）、`redirect_target`, `redirect_kick_power`
- `robot_acc_for_prediction`: 2.5（既定）
- `robot_max_vel_for_prediction`: 5.0（既定）
- `enable_software_bumper`: true（既定）、`software_bumper_start_time`

## WorldModelPublisher パラメータ

- `pass_target.min_hold_duration_sec`: ターゲットの最低保持時間（秒）
- `pass_target.min_improvement_margin`: 早期切替のための最小改善幅

## 動作確認手順（抜粋）

- ビルド（ワークスペースルートで実行）
  - `colcon build --symlink-install --packages-select crane_msgs crane_world_model_publisher crane_robot_skills crane_tactic_coordinator`
- 起動（例）
  - `ros2 launch crane_bringup crane.launch.xml sim:=true`
- 可視化・デバッグ
  - 受け手整列点やスコアは可視化トピックに反映（Session/WorldModel の可視化レイヤ参照）。
  - `ongoing_kick`, `pass_scores`, `pass_target_id` は `/world_model` を購読して確認。

## チューニング指針

- スイッチ抑制が強すぎる場合: `pass_target.min_hold_duration_sec` を0.3程度に短縮、または `min_improvement_margin` を小さく。
- 受け取り遅延がある場合: `Receive` の `robot_acc_for_prediction`, `robot_max_vel_for_prediction` を上げる。

## 既知の注意点

- 受け手が物理的に不達な位置にいる場合は Receive 側が `closest` へフォールバック。
- `pass_target_id` が未選択（-1）の場合、Attacker はパス分岐に入らず、PassReceiverPlanner はロボットを取得しません（受け手予約なし）。

## 変更ファイル一覧（実装反映）

- `crane_msgs/msg/analysis/GameAnalysis.msg`: `pass_target_id` の追加
- `crane_world_model_publisher`: `pass_target_id` の選定とスイッチ抑制
- `crane_robot_skills/src/attacker.cpp`: 上位決定の優先適用
- `session/.../pass_receiver_planner.hpp`: 受取準備ロジックの最適化
- `session/.../play_situation/*.yaml`: `pass_receive` の追加
