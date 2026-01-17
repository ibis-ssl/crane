"""Prompts for Gemini API analysis of RoboCup SSL annotations.

このモジュールは、Gemini APIで使用するプロンプトテンプレートを定義します。
"""

SYSTEM_INSTRUCTION = """あなたはRoboCup SSLの専門家です。試合中に記録された人間のアノテーションと、
その前後のロボット・ボールの状態データ（WorldModel）を分析し、以下の観点で評価してください:

1. **問題の根本原因**: 何が起きたのか、なぜ起きたのかを技術的に分析
2. **戦術的評価**: ロボットの動きや判断が戦術的に適切だったか
3. **改善提案**: 具体的で実装可能な改善策を優先度付きで提案

**コンテキスト情報**:
- アノテーション時刻の前3秒、後2秒のWorldModelデータが提供されます
- WorldModelには、ボールの位置・速度、全ロボットの位置・速度が含まれます
- RoboCup SSLは6vs6の小型ロボットサッカーです

**利用可能なツール**:
あなたは以下のツールを使用して、MCAPファイル内のデータを自律的に探索できます。
与えられた情報だけでは不十分な場合、必要に応じてこれらのツールを呼び出してください:

- `get_world_model_at_time`: 特定時刻のWorldModelスナップショットを取得
- `get_robot_trajectory`: ロボットの軌跡（位置・速度の時系列）を取得
- `get_ball_trajectory`: ボールの軌跡を取得
- `calculate_distance`: 2点間の距離を計算
- `get_robot_speed_history`: ロボットの速度履歴を取得
- `find_closest_robot_to_ball`: ボールに最も近いロボットを検索
- `get_ball_speed_history`: ボールの速度履歴を取得
- `check_robot_collision`: ロボット間の接近・衝突をチェック

**分析プロセス（必須）**:
1. まず提供された情報を確認する
2. **必ず以下のツールを使って詳細を調査してください**:
   - 関連ロボットがある場合: `get_robot_trajectory`と`get_robot_speed_history`で詳細な動きを確認
   - ボール関連の場合: `get_ball_trajectory`と`find_closest_robot_to_ball`で状況を把握
   - 衝突・接近の疑いがある場合: `check_robot_collision`で確認
3. 複数の角度から原因を調査する（例: ロボットの速度履歴、ボールとの距離、他のロボットとの関係）
4. 収集した情報を基に総合的な分析を行う

**重要**: 提供されたサマリーだけでなく、必ずツールを使って実際のデータを確認してください。

**回答フォーマット**:
必ず以下のJSON形式で回答してください:

```json
{
  "root_cause": "問題の根本原因（200字以内）",
  "tactical_analysis": "戦術的評価（200字以内）",
  "improvements": [
    {
      "priority": "HIGH|MEDIUM|LOW",
      "description": "改善提案の説明",
      "implementation": "実装方法の具体的なヒント"
    }
  ],
  "confidence": "HIGH|MEDIUM|LOW"
}
```

**重要**: JSON以外のテキストは含めないでください。"""


def create_annotation_analysis_prompt(
    label: str,
    description: str,
    category: str,
    priority: str,
    event_timestamp_ns: int,
    world_model_summary: str,
    position_info: str = "",
    robot_context: str = "",
) -> str:
    """
    アノテーション解析用のプロンプトを生成.

    Args:
        label: アノテーションのラベル
        description: アノテーションの詳細説明
        category: カテゴリ名（ISSUE, OBSERVATION, etc.）
        priority: 重要度（HIGH, MEDIUM, LOW, CRITICAL）
        event_timestamp_ns: イベント発生時刻（ナノ秒）
        world_model_summary: WorldModelデータのサマリー
        position_info: 位置情報（オプション）
        robot_context: ロボットコンテキスト（オプション）

    Returns:
        Gemini APIに送信するプロンプト文字列
    """
    event_time_sec = event_timestamp_ns / 1e9

    prompt = f"""# アノテーション情報

- **ラベル**: {label}
- **カテゴリ**: {category}
- **重要度**: {priority}
- **イベント時刻**: {event_time_sec:.3f}秒
- **詳細説明**: {description}
"""

    if position_info:
        prompt += f"\n{position_info}\n"

    if robot_context:
        prompt += f"\n{robot_context}\n"

    # WorldModelサマリーは提供するが、詳細はツールで取得するよう促す
    prompt += """
# 基本情報

WorldModelコンテキストが利用可能です（前3秒、後2秒）。

# 分析手順

**必ずツールを使用して以下を調査してください**：

1. 関連ロボットがある場合:
   - `get_robot_speed_history`でロボットの速度変化を確認
   - `get_robot_trajectory`で詳細な軌跡を取得

2. ボール関連の問題の場合:
   - `get_ball_speed_history`でボールの動きを確認
   - `find_closest_robot_to_ball`でボールとロボットの関係を調査

3. 衝突・干渉の疑いがある場合:
   - `check_robot_collision`で他のロボットとの接近をチェック

4. 特定時刻の詳細が必要な場合:
   - `get_world_model_at_time`で正確なスナップショットを取得

**重要**: サマリーだけで判断せず、必ずツールを使って実データを確認してください。

---

それでは、ツールを使用して詳細を調査し、分析結果をJSON形式で回答してください。"""

    return prompt


def summarize_world_model_context(world_model_snapshots: list) -> str:
    """
    WorldModelスナップショットをテキストサマリーに変換.

    Args:
        world_model_snapshots: WorldModelSnapshotのリスト

    Returns:
        人間が読めるテキストサマリー
    """
    if not world_model_snapshots:
        return "（WorldModelデータなし）"

    summary_lines = []

    # 時系列データの要約
    summary_lines.append(f"**サンプル数**: {len(world_model_snapshots)}")

    # 時間範囲
    first_time = world_model_snapshots[0].timestamp_ns / 1e9
    last_time = world_model_snapshots[-1].timestamp_ns / 1e9
    duration = last_time - first_time
    summary_lines.append(
        f"**時間範囲**: {first_time:.3f}秒 〜 {last_time:.3f}秒 ({duration:.2f}秒)"
    )

    # ボールの動き
    ball_positions = [snap.ball_position for snap in world_model_snapshots]
    ball_velocities = [snap.ball_velocity for snap in world_model_snapshots]

    # ボール移動距離
    total_ball_distance = sum(
        ((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2) ** 0.5
        for p1, p2 in zip(ball_positions[:-1], ball_positions[1:])
    )
    summary_lines.append(f"**ボール移動距離**: {total_ball_distance:.2f}m")

    # ボール速度範囲
    ball_speeds = [(vx**2 + vy**2 + vz**2) ** 0.5 for vx, vy, vz in ball_velocities]
    if ball_speeds:
        max_speed = max(ball_speeds)
        avg_speed = sum(ball_speeds) / len(ball_speeds)
        summary_lines.append(
            f"**ボール速度**: 最大 {max_speed:.2f}m/s, 平均 {avg_speed:.2f}m/s"
        )

    # ロボット台数
    if world_model_snapshots:
        first_snap = world_model_snapshots[0]
        summary_lines.append(
            f"**ロボット数**: 自チーム {len(first_snap.our_robots)}台, 相手チーム {len(first_snap.their_robots)}台"
        )

    # エラーを持つロボット
    error_robots = set()
    for snap in world_model_snapshots:
        for robot in snap.our_robots:
            if robot.get("has_error", False):
                error_robots.add(robot["id"])

    if error_robots:
        summary_lines.append(f"**エラー発生ロボット**: {sorted(error_robots)}")

    # 詳細な時系列データ（最初、中間、最後の3サンプルのみ）
    sample_indices = [
        0,
        len(world_model_snapshots) // 2,
        len(world_model_snapshots) - 1,
    ]
    summary_lines.append("\n## 時系列サンプル（抜粋）")

    for idx in sample_indices:
        if idx >= len(world_model_snapshots):
            continue

        snap = world_model_snapshots[idx]
        time_sec = snap.timestamp_ns / 1e9
        bx, by, bz = snap.ball_position
        bvx, bvy, bvz = snap.ball_velocity

        summary_lines.append(f"\n### t={time_sec:.3f}秒")
        summary_lines.append(f"- ボール位置: ({bx:.2f}, {by:.2f}, {bz:.2f})m")
        summary_lines.append(f"- ボール速度: ({bvx:.2f}, {bvy:.2f}, {bvz:.2f})m/s")

        # 自チームロボット（最大3台まで表示）
        summary_lines.append(f"- 自チームロボット ({len(snap.our_robots)}台):")
        for robot in snap.our_robots[:3]:
            rx, ry = robot["position"]
            rvx, rvy = robot["velocity"]
            error_marker = " ⚠️" if robot.get("has_error") else ""
            summary_lines.append(
                f"  - ID {robot['id']}: ({rx:.2f}, {ry:.2f})m, "
                f"速度 ({rvx:.2f}, {rvy:.2f})m/s{error_marker}"
            )

        # 相手チームロボット（最大3台まで表示）
        summary_lines.append(f"- 相手チームロボット ({len(snap.their_robots)}台):")
        for robot in snap.their_robots[:3]:
            rx, ry = robot["position"]
            summary_lines.append(f"  - ID {robot['id']}: ({rx:.2f}, {ry:.2f})m")

    return "\n".join(summary_lines)


def format_position_info(position: tuple[float, float, float]) -> str:
    """位置情報をフォーマット."""
    x, y, z = position
    return f"- **フィールド位置**: ({x:.2f}, {y:.2f}, {z:.2f})m"


def format_robot_context(
    related_robot_ids: list[int], robot_is_ours: list[bool]
) -> str:
    """ロボットコンテキストをフォーマット."""
    if not related_robot_ids:
        return ""

    robot_info = []
    for robot_id, is_ours in zip(related_robot_ids, robot_is_ours):
        team = "自チーム" if is_ours else "相手チーム"
        robot_info.append(f"{team} ID {robot_id}")

    return f"- **関連ロボット**: {', '.join(robot_info)}"
