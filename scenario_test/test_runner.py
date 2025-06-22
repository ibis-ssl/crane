#!/usr/bin/env python3
"""
シナリオテスト実行スクリプト
テスト結果の統計とレポート生成機能付き
"""

import argparse
import json
import os
import sys
import time
import traceback
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Tuple

import pytest


class ScenarioTestRunner:
    """シナリオテスト実行とレポート生成クラス"""

    def __init__(self, vision_port: int = 10020, log_recorder: str = None):
        self.vision_port = vision_port
        self.log_recorder = log_recorder
        self.results = []
        self.start_time = None
        self.end_time = None

    def run_test(self, test_file: str) -> Tuple[bool, Dict]:
        """単一テストの実行"""
        test_name = Path(test_file).stem
        print(f"=== Running {test_name} ===")

        start_time = time.time()

        # pytest実行用の引数
        pytest_args = [
            test_file,
            "--vision_port",
            str(self.vision_port),
            "--logging",
            "-v",
            "--tb=short",
        ]

        if self.log_recorder:
            pytest_args.extend(["--log_recorder", self.log_recorder])

        try:
            # pytestを実行
            exit_code = pytest.main(pytest_args)
            success = exit_code == 0

            end_time = time.time()
            duration = end_time - start_time

            result = {
                "name": test_name,
                "file": test_file,
                "success": success,
                "duration": duration,
                "exit_code": exit_code,
                "timestamp": datetime.now().isoformat(),
                "error": None,
            }

            if success:
                print(f"✅ {test_name} passed in {duration:.2f}s")
            else:
                print(
                    f"❌ {test_name} failed in {duration:.2f}s (exit code: {exit_code})"
                )

        except Exception as e:
            end_time = time.time()
            duration = end_time - start_time

            result = {
                "name": test_name,
                "file": test_file,
                "success": False,
                "duration": duration,
                "exit_code": -1,
                "timestamp": datetime.now().isoformat(),
                "error": str(e),
                "traceback": traceback.format_exc(),
            }

            print(f"💥 {test_name} crashed in {duration:.2f}s: {e}")

        self.results.append(result)
        return result["success"], result

    def run_test_group(self, test_files: List[str]) -> Dict:
        """テストグループの実行"""
        self.start_time = time.time()

        success_count = 0
        total_count = len(test_files)

        print(f"Starting {total_count} scenario tests...")

        for test_file in test_files:
            success, result = self.run_test(test_file)
            if success:
                success_count += 1

            # テスト間の待機時間
            time.sleep(2)

        self.end_time = time.time()

        # 統計情報の生成
        total_duration = self.end_time - self.start_time
        success_rate = (success_count / total_count) * 100 if total_count > 0 else 0

        summary = {
            "total_tests": total_count,
            "passed_tests": success_count,
            "failed_tests": total_count - success_count,
            "success_rate": success_rate,
            "total_duration": total_duration,
            "start_time": datetime.fromtimestamp(self.start_time).isoformat(),
            "end_time": datetime.fromtimestamp(self.end_time).isoformat(),
            "results": self.results,
        }

        return summary

    def generate_report(self, summary: Dict, output_file: str = None):
        """テストレポートの生成"""
        if output_file is None:
            output_file = (
                f"scenario_test_report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
            )

        # JSONレポートの保存
        with open(output_file, "w", encoding="utf-8") as f:
            json.dump(summary, f, indent=2, ensure_ascii=False)

        # コンソール出力
        self.print_summary(summary)

        # GitHub Actions用の出力
        if os.getenv("GITHUB_ACTIONS"):
            self.generate_github_summary(summary)

        return output_file

    def print_summary(self, summary: Dict):
        """サマリーのコンソール出力"""
        print("\n" + "=" * 60)
        print("📊 シナリオテスト結果サマリー")
        print("=" * 60)

        print(f"総テスト数: {summary['total_tests']}")
        print(f"成功: {summary['passed_tests']} ✅")
        print(f"失敗: {summary['failed_tests']} ❌")
        print(f"成功率: {summary['success_rate']:.1f}%")
        print(f"実行時間: {summary['total_duration']:.1f}秒")

        if summary["failed_tests"] > 0:
            print("\n🔍 失敗したテスト:")
            for result in summary["results"]:
                if not result["success"]:
                    print(f"  - {result['name']} ({result['duration']:.1f}s)")
                    if result["error"]:
                        print(f"    エラー: {result['error']}")

        print("\n📋 全テスト詳細:")
        for result in summary["results"]:
            status = "✅" if result["success"] else "❌"
            print(f"  {status} {result['name']} ({result['duration']:.1f}s)")

    def generate_github_summary(self, summary: Dict):
        """GitHub Actions Summary の生成"""
        summary_file = os.getenv("GITHUB_STEP_SUMMARY")
        if not summary_file:
            return

        with open(summary_file, "a", encoding="utf-8") as f:
            f.write("## 🤖 シナリオテスト結果\n\n")

            # 統計表
            f.write("### 📊 統計\n\n")
            f.write("| 項目 | 値 |\n")
            f.write("|------|----|\n")
            f.write(f"| 総テスト数 | {summary['total_tests']} |\n")
            f.write(f"| 成功 | {summary['passed_tests']} ✅ |\n")
            f.write(f"| 失敗 | {summary['failed_tests']} ❌ |\n")
            f.write(f"| 成功率 | {summary['success_rate']:.1f}% |\n")
            f.write(f"| 実行時間 | {summary['total_duration']:.1f}秒 |\n\n")

            # 詳細結果表
            f.write("### 📋 詳細結果\n\n")
            f.write("| テスト名 | 結果 | 時間 |\n")
            f.write("|----------|------|------|\n")

            for result in summary["results"]:
                status = "✅ 成功" if result["success"] else "❌ 失敗"
                f.write(
                    f"| {result['name']} | {status} | {result['duration']:.1f}s |\n"
                )

            # 失敗詳細
            if summary["failed_tests"] > 0:
                f.write("\n### 🔍 失敗詳細\n\n")
                for result in summary["results"]:
                    if not result["success"]:
                        f.write(f"#### {result['name']}\n")
                        f.write(f"- **実行時間**: {result['duration']:.1f}秒\n")
                        f.write(f"- **終了コード**: {result['exit_code']}\n")
                        if result["error"]:
                            f.write(f"- **エラー**: `{result['error']}`\n")
                        f.write("\n")


def main():
    parser = argparse.ArgumentParser(description="シナリオテスト実行スクリプト")
    parser.add_argument("tests", nargs="+", help="実行するテストファイル")
    parser.add_argument("--vision_port", type=int, default=10020, help="Vision port")
    parser.add_argument("--log_recorder", help="ログレコーダーのパス")
    parser.add_argument("--output", help="レポート出力ファイル名")
    parser.add_argument(
        "--group",
        choices=["rule_compliance", "basic_actions", "quick_smoke", "all"],
        help="テストグループの選択",
    )

    args = parser.parse_args()

    # グループ指定の場合はテストファイルを自動選択
    if args.group:
        test_groups = {
            "rule_compliance": [
                "STOP_KEEPER_AREA.py",
                "STOP_MULTIPLE_ROBOTS_BALL.py",
                "FREE_KICK_DISTANCE.py",
                "BALL_PLACEMENT_ACCURACY.py",
                "ROBOT_COUNT_LIMIT.py",
            ],
            "basic_actions": [
                "BASIC_MOVEMENT_ACCURACY.py",
                "BALL_HANDLING_CONTROL.py",
                "FORMATION_MAINTENANCE.py",
                "COLLISION_AVOIDANCE.py",
                "REFEREE_RESPONSE_TIME.py",
            ],
            "quick_smoke": [
                "STOP_ROBOT_SPEED.py",
                "STOP_AVOID_BALL.py",
                "BASIC_MOVEMENT_ACCURACY.py",
            ],
        }

        if args.group == "all":
            test_files = test_groups["rule_compliance"] + test_groups["basic_actions"]
        else:
            test_files = test_groups[args.group]

        # scenario_test ディレクトリのパスを追加
        script_dir = Path(__file__).parent
        test_files = [str(script_dir / test_file) for test_file in test_files]
    else:
        test_files = args.tests

    # テストランナーの実行
    runner = ScenarioTestRunner(
        vision_port=args.vision_port, log_recorder=args.log_recorder
    )

    try:
        summary = runner.run_test_group(test_files)
        report_file = runner.generate_report(summary, args.output)

        print(f"\n📄 レポートが生成されました: {report_file}")

        # 失敗したテストがある場合は終了コード1で終了
        if summary["failed_tests"] > 0:
            sys.exit(1)
        else:
            sys.exit(0)

    except KeyboardInterrupt:
        print("\n⚠️ テスト実行が中断されました")
        sys.exit(130)
    except Exception as e:
        print(f"\n💥 テスト実行中にエラーが発生しました: {e}")
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
