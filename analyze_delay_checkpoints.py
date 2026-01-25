#!/usr/bin/env python3
"""
MCAPファイルから/robot_commandsトピックのdelay_checkpointsを解析するスクリプト

使用方法:
    ./analyze_delay_checkpoints.py <mcap_file_path>

ROS 2環境が必要です。実行前にワークスペースをソースしてください:
    source /opt/ros/jazzy/setup.bash
    source /home/hans/workspace/ibis_ws/install/setup.bash
"""

import sys
from pathlib import Path
from collections import defaultdict
import statistics

# ROS 2
import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# MCAP
from mcap.reader import make_reader


def extract_delay_checkpoints(msg):
    """
    VelocityCommandsメッセージからdelay_checkpoints情報を抽出

    Args:
        msg: crane_msgs.msg.VelocityCommands メッセージ

    Returns:
        dict: delay_checkpoints情報
    """
    # VelocityCommands全体レベルのdelay_checkpoints
    delay_cp = msg.delay_checkpoints

    checkpoints = []
    for cp in delay_cp.checkpoints:
        checkpoints.append(
            {
                "name": cp.name,
                "relative_time_us": cp.relative_time_us,
                "value": cp.value,
            }
        )

    # 各ロボットのVelocityCommand内のdelay_checkpoints
    robot_checkpoints = {}
    for robot_cmd in msg.robot_commands:
        robot_id = robot_cmd.robot_id
        robot_cp = robot_cmd.delay_checkpoints

        robot_cps = []
        for cp in robot_cp.checkpoints:
            robot_cps.append(
                {
                    "name": cp.name,
                    "relative_time_us": cp.relative_time_us,
                    "value": cp.value,
                }
            )

        if robot_cps:  # チェックポイントがある場合のみ追加
            robot_checkpoints[robot_id] = {
                "reference_timestamp_ns": robot_cp.reference_timestamp_ns,
                "checkpoints": robot_cps,
            }

    return {
        "timestamp_sec": msg.header.stamp.sec,
        "timestamp_nanosec": msg.header.stamp.nanosec,
        "reference_timestamp_ns": delay_cp.reference_timestamp_ns,
        "checkpoints": checkpoints,
        "robot_checkpoints": robot_checkpoints,
    }


def analyze_mcap(mcap_path: str):
    """MCAPファイルからdelay_checkpointsを解析"""

    print(f"MCAPファイルを解析中: {mcap_path}\n")

    # ROS 2メッセージ型を取得
    msg_type = get_message("crane_msgs/msg/VelocityCommands")

    # チェックポイント統計用
    checkpoint_times = defaultdict(list)
    checkpoint_intervals = defaultdict(list)  # チェックポイント間の間隔
    all_delay_data = []

    # ロボットごとのチェックポイント統計用
    robot_checkpoint_times = defaultdict(lambda: defaultdict(list))
    robot_checkpoint_values = defaultdict(
        lambda: defaultdict(list)
    )  # valueフィールドの値

    message_count = 0
    error_count = 0

    with open(mcap_path, "rb") as f:
        reader = make_reader(f)

        target_topic = "/robot_commands"

        for schema, channel, message in reader.iter_messages(topics=[target_topic]):
            message_count += 1

            try:
                # メッセージをデシリアライズ
                msg = deserialize_message(message.data, msg_type)

                # delay_checkpointsを抽出
                result = extract_delay_checkpoints(msg)
                all_delay_data.append(result)

                # チェックポイント時間を集計
                for cp in result["checkpoints"]:
                    checkpoint_times[cp["name"]].append(cp["relative_time_us"])

                # チェックポイント間の間隔を計算
                if len(result["checkpoints"]) > 1:
                    sorted_cps = sorted(
                        result["checkpoints"], key=lambda x: x["relative_time_us"]
                    )
                    for i in range(1, len(sorted_cps)):
                        prev_cp = sorted_cps[i - 1]
                        curr_cp = sorted_cps[i]
                        interval = (
                            curr_cp["relative_time_us"] - prev_cp["relative_time_us"]
                        )
                        interval_key = f"{prev_cp['name']} → {curr_cp['name']}"
                        checkpoint_intervals[interval_key].append(interval)

                # ロボットごとのチェックポイントを集計
                for robot_id, robot_data in result["robot_checkpoints"].items():
                    for cp in robot_data["checkpoints"]:
                        robot_checkpoint_times[cp["name"]][robot_id].append(
                            cp["relative_time_us"]
                        )
                        if cp["value"]:
                            robot_checkpoint_values[cp["name"]][robot_id].append(
                                cp["value"]
                            )

                # 最初の数メッセージを表示
                if message_count <= 3:
                    print(f"=== Message #{message_count} ===")
                    print(
                        f"  Timestamp: {result['timestamp_sec']}.{result['timestamp_nanosec']:09d}"
                    )
                    print(
                        f"  Reference timestamp: {result['reference_timestamp_ns']} ns"
                    )
                    print(
                        f"  VelocityCommands Checkpoints ({len(result['checkpoints'])}):"
                    )

                    sorted_cps = sorted(
                        result["checkpoints"], key=lambda x: x["relative_time_us"]
                    )
                    for i, cp in enumerate(sorted_cps):
                        if i == 0:
                            print(
                                f"    [{i + 1}] {cp['name']}: {cp['relative_time_us']} us"
                            )
                        else:
                            prev = sorted_cps[i - 1]
                            delta = cp["relative_time_us"] - prev["relative_time_us"]
                            print(
                                f"    [{i + 1}] {cp['name']}: {cp['relative_time_us']} us (+{delta} us)"
                            )

                        if cp["value"]:
                            print(f"        value: {cp['value']}")

                    # ロボットごとのチェックポイントを表示
                    if result["robot_checkpoints"]:
                        print(
                            f"\n  Robot Checkpoints ({len(result['robot_checkpoints'])} robots):"
                        )
                        for robot_id, robot_data in sorted(
                            result["robot_checkpoints"].items()
                        ):
                            print(f"    Robot {robot_id}:")
                            for cp in robot_data["checkpoints"]:
                                print(
                                    f"      - {cp['name']}: {cp['relative_time_us']} us",
                                    end="",
                                )
                                if cp["value"]:
                                    print(f" (value: {cp['value']})", end="")
                                print()
                    print()

            except Exception as e:
                error_count += 1
                if error_count <= 5:
                    print(f"エラー (Message #{message_count}): {e}")
                    import traceback

                    traceback.print_exc()

    print(f"\n解析完了: {message_count} メッセージ処理 ({error_count} エラー)\n")

    if not all_delay_data:
        print("delay_checkpointsデータが取得できませんでした。")
        return

    # === 統計情報を表示 ===
    print("=" * 80)
    print("チェックポイント統計 (各チェックポイントの相対時間)")
    print("=" * 80)

    for name in sorted(checkpoint_times.keys()):
        times = checkpoint_times[name]
        print(f"\n【{name}】")
        print(f"  出現回数: {len(times)}")
        print(
            f"  平均: {statistics.mean(times):.2f} us ({statistics.mean(times) / 1000:.3f} ms)"
        )
        print(
            f"  中央値: {statistics.median(times):.2f} us ({statistics.median(times) / 1000:.3f} ms)"
        )
        print(f"  最小: {min(times)} us ({min(times) / 1000:.3f} ms)")
        print(f"  最大: {max(times)} us ({max(times) / 1000:.3f} ms)")
        if len(times) > 1:
            stdev = statistics.stdev(times)
            print(f"  標準偏差: {stdev:.2f} us ({stdev / 1000:.3f} ms)")

    # === チェックポイント間の遅延統計 ===
    print("\n")
    print("=" * 80)
    print("チェックポイント間の遅延統計")
    print("=" * 80)

    for interval_key in sorted(checkpoint_intervals.keys()):
        intervals = checkpoint_intervals[interval_key]
        print(f"\n【{interval_key}】")
        print(f"  出現回数: {len(intervals)}")
        print(
            f"  平均: {statistics.mean(intervals):.2f} us ({statistics.mean(intervals) / 1000:.3f} ms)"
        )
        print(
            f"  中央値: {statistics.median(intervals):.2f} us ({statistics.median(intervals) / 1000:.3f} ms)"
        )
        print(f"  最小: {min(intervals)} us ({min(intervals) / 1000:.3f} ms)")
        print(f"  最大: {max(intervals)} us ({max(intervals) / 1000:.3f} ms)")
        if len(intervals) > 1:
            stdev = statistics.stdev(intervals)
            print(f"  標準偏差: {stdev:.2f} us ({stdev / 1000:.3f} ms)")

    # === 全体の処理遅延サマリー ===
    print("\n")
    print("=" * 80)
    print("全体の処理遅延サマリー")
    print("=" * 80)

    total_delays = []
    for data in all_delay_data:
        if data["checkpoints"]:
            sorted_cps = sorted(
                data["checkpoints"], key=lambda x: x["relative_time_us"]
            )
            # 最初から最後までの遅延
            total_delay = (
                sorted_cps[-1]["relative_time_us"] - sorted_cps[0]["relative_time_us"]
            )
            total_delays.append(total_delay)

    if total_delays:
        print("\n総処理時間 (最初のチェックポイント → 最後のチェックポイント):")
        print(
            f"  平均: {statistics.mean(total_delays):.2f} us ({statistics.mean(total_delays) / 1000:.3f} ms)"
        )
        print(
            f"  中央値: {statistics.median(total_delays):.2f} us ({statistics.median(total_delays) / 1000:.3f} ms)"
        )
        print(f"  最小: {min(total_delays)} us ({min(total_delays) / 1000:.3f} ms)")
        print(f"  最大: {max(total_delays)} us ({max(total_delays) / 1000:.3f} ms)")
        if len(total_delays) > 1:
            stdev = statistics.stdev(total_delays)
            print(f"  標準偏差: {stdev:.2f} us ({stdev / 1000:.3f} ms)")

    # === ロボットごとのチェックポイント統計 ===
    if robot_checkpoint_times:
        print("\n")
        print("=" * 80)
        print("ロボットごとのチェックポイント統計")
        print("=" * 80)

        for cp_name in sorted(robot_checkpoint_times.keys()):
            print(f"\n【{cp_name}】")

            for robot_id in sorted(robot_checkpoint_times[cp_name].keys()):
                times = robot_checkpoint_times[cp_name][robot_id]
                print(f"\n  Robot {robot_id}:")
                print(f"    出現回数: {len(times)}")
                print(
                    f"    平均: {statistics.mean(times):.2f} us ({statistics.mean(times) / 1000:.3f} ms)"
                )
                print(
                    f"    中央値: {statistics.median(times):.2f} us ({statistics.median(times) / 1000:.3f} ms)"
                )
                print(f"    最小: {min(times)} us ({min(times) / 1000:.3f} ms)")
                print(f"    最大: {max(times)} us ({max(times) / 1000:.3f} ms)")
                if len(times) > 1:
                    stdev = statistics.stdev(times)
                    print(f"    標準偏差: {stdev:.2f} us ({stdev / 1000:.3f} ms)")

                # valueフィールドの解析（特にvision_timestamps用）
                if (
                    cp_name in robot_checkpoint_values
                    and robot_id in robot_checkpoint_values[cp_name]
                ):
                    values = robot_checkpoint_values[cp_name][robot_id]
                    if values:
                        print("    value フィールドのサンプル (最初の5個):")
                        for i, val in enumerate(values[:5]):
                            print(f"      [{i + 1}] {val}")

    # === vision_timestamps の詳細解析 ===
    if "vision_timestamps" in robot_checkpoint_values:
        print("\n")
        print("=" * 80)
        print("vision_timestamps の詳細解析")
        print("=" * 80)

        for robot_id in sorted(robot_checkpoint_values["vision_timestamps"].keys()):
            values = robot_checkpoint_values["vision_timestamps"][robot_id]
            print(f"\nRobot {robot_id}:")
            print(f"  総サンプル数: {len(values)}")
            print("  サンプル値 (最初の10個):")
            for i, val in enumerate(values[:10]):
                print(f"    [{i + 1}] {val}")


def main():
    if len(sys.argv) != 2:
        print("使用方法: python3 analyze_delay_checkpoints.py <mcap_file_path>")
        print("\nROS 2環境をソースしてから実行してください:")
        print("  source /opt/ros/jazzy/setup.bash")
        print("  source /home/hans/workspace/ibis_ws/install/setup.bash")
        sys.exit(1)

    mcap_path = sys.argv[1]

    if not Path(mcap_path).exists():
        print(f"エラー: ファイルが見つかりません: {mcap_path}")
        sys.exit(1)

    # rclpyの初期化（メッセージデシリアライズのため）
    rclpy.init()

    try:
        analyze_mcap(mcap_path)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
