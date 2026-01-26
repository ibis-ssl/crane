#!/usr/bin/env python3
"""
TIGERs対戦試合制御スクリプト

ssl-game-controllerのAPIを使用して試合を開始し、
結果を監視してテキストサマリーを出力します。
"""

import time
import sys
from typing import List, Tuple, Optional

try:
    import requests
except ImportError:
    print("requestsモジュールが必要です: pip install requests", file=sys.stderr)
    sys.exit(1)


class MatchController:
    def __init__(self):
        self.gc_host = "match_gc"
        self.gc_port = 8081
        self.gc_api_base = f"http://{self.gc_host}:{self.gc_port}/api"

        self.yellow_name = "ibis"
        self.blue_name = "TIGERs Mannheim"

        self.yellow_score = 0
        self.blue_score = 0
        self.events: List[Tuple[float, str]] = []
        self.start_time: Optional[float] = None
        self.match_duration = 0.0

    def wait_for_services(self, timeout: int = 120) -> bool:
        """サービスが起動するまで待機"""
        print("サービスの起動を待機中...")

        start = time.time()
        while time.time() - start < timeout:
            try:
                response = requests.get(f"{self.gc_api_base}/state", timeout=2)
                if response.status_code == 200:
                    print("✓ ssl-game-controller 起動確認")
                    return True
            except requests.exceptions.RequestException:
                time.sleep(2)
                continue

        print("✗ サービスの起動がタイムアウトしました", file=sys.stderr)
        return False

    def wait_for_teams(self, timeout: int = 60) -> bool:
        """チームの接続を待機"""
        print("チームの接続を待機中...")

        start = time.time()
        while time.time() - start < timeout:
            try:
                response = requests.get(f"{self.gc_api_base}/state", timeout=2)
                if response.status_code == 200:
                    # チーム接続状況を確認（簡易版）
                    print(".", end="", flush=True)
                    time.sleep(2)

                    # 一定時間経過したら接続完了とみなす
                    if time.time() - start > 20:
                        print("\n✓ チーム接続待機完了")
                        return True

            except requests.exceptions.RequestException:
                time.sleep(2)
                continue

        print("\n✗ チーム接続待機がタイムアウトしました", file=sys.stderr)
        return False

    def start_match_sequence(self) -> bool:
        """試合開始シーケンスを実行"""
        try:
            # 1. NORMALフェーズに移行
            print("試合フェーズをNORMALに設定中...")
            response = requests.post(
                f"{self.gc_api_base}/control/command",
                json={"command": "normalStart"},
                timeout=5,
            )

            if response.status_code != 200:
                print(f"✗ フェーズ設定失敗: {response.status_code}", file=sys.stderr)
                return False

            print("✓ NORMALフェーズに移行")
            time.sleep(2)

            # 2. キックオフコマンド（Yellowから）
            print("キックオフコマンドを送信...")
            response = requests.post(
                f"{self.gc_api_base}/control/command",
                json={"command": "kickoff", "forTeam": "YELLOW"},
                timeout=5,
            )

            if response.status_code == 200:
                print("✓ キックオフコマンド送信成功")
                self.start_time = time.time()
                return True
            else:
                print(f"✗ キックオフ失敗: {response.status_code}", file=sys.stderr)
                return False

        except Exception as e:
            print(f"✗ 試合開始エラー: {e}", file=sys.stderr)
            return False

    def monitor_match_via_api(self, max_duration: int = 120):
        """
        HTTP APIを使用して試合状態を監視

        Args:
            max_duration: 最大試合時間（秒）
        """
        print("試合状態を監視中（HTTP API使用）...")

        last_yellow_score = 0
        last_blue_score = 0
        last_stage = None

        start_monitor = time.time()

        while time.time() - start_monitor < max_duration:
            try:
                response = requests.get(f"{self.gc_api_base}/state", timeout=2)

                if response.status_code == 200:
                    state = response.json()

                    # スコア取得
                    if "teamState" in state:
                        yellow_state = state.get("teamState", {}).get("YELLOW", {})
                        blue_state = state.get("teamState", {}).get("BLUE", {})

                        self.yellow_score = yellow_state.get("goals", 0)
                        self.blue_score = blue_state.get("goals", 0)

                        # スコア変化を記録
                        elapsed = (
                            time.time() - self.start_time if self.start_time else 0
                        )

                        if self.yellow_score > last_yellow_score:
                            self.events.append((elapsed, "GOAL by Yellow"))
                            print(f"  [{elapsed:.1f}s] GOAL by Yellow!")
                            last_yellow_score = self.yellow_score

                        if self.blue_score > last_blue_score:
                            self.events.append((elapsed, "GOAL by Blue"))
                            print(f"  [{elapsed:.1f}s] GOAL by Blue!")
                            last_blue_score = self.blue_score

                    # ステージ確認
                    current_stage = state.get("stage", "")
                    if current_stage != last_stage:
                        elapsed = (
                            time.time() - self.start_time if self.start_time else 0
                        )
                        self.events.append((elapsed, f"Stage: {current_stage}"))
                        print(f"  [{elapsed:.1f}s] Stage: {current_stage}")
                        last_stage = current_stage

                        # POST_GAMEで終了
                        if current_stage == "POST_GAME":
                            print("試合が終了しました（POST_GAME）")
                            break

                time.sleep(1)

            except requests.exceptions.RequestException as e:
                print(f"API監視エラー: {e}", file=sys.stderr)
                time.sleep(2)
            except Exception as e:
                print(f"予期しないエラー: {e}", file=sys.stderr)
                time.sleep(2)

        self.match_duration = time.time() - self.start_time if self.start_time else 0
        print(f"\n試合終了（実行時間: {self.match_duration:.1f}秒）")

    def generate_summary(self) -> str:
        """試合サマリーを生成"""

        # 勝敗判定
        if self.yellow_score > self.blue_score:
            result = f"CRANE WIN ({self.yellow_name})"
        elif self.blue_score > self.yellow_score:
            result = f"TIGERs WIN ({self.blue_name})"
        else:
            result = "DRAW"

        summary = f"""=====================================
        TIGERs対戦結果サマリー
=====================================

【スコア】
  {self.yellow_name} (Yellow): {self.yellow_score}
  {self.blue_name} (Blue): {self.blue_score}

【勝敗】
  {result}

【試合時間】
  {self.match_duration:.1f}秒

【主要イベント】
"""

        if self.events:
            for event_time, event_desc in self.events:
                summary += f"  - [{event_time:.1f}s] {event_desc}\n"
        else:
            summary += "  - イベント記録なし\n"

        summary += "\n=====================================\n"

        return summary

    def save_results(self, filepath: str = "/app/results/match_result.txt"):
        """結果をファイルに保存"""
        summary = self.generate_summary()

        try:
            with open(filepath, "w", encoding="utf-8") as f:
                f.write(summary)
            print(f"✓ 結果を保存: {filepath}")
        except Exception as e:
            print(f"✗ 結果保存エラー: {e}", file=sys.stderr)

        # 標準出力にも表示
        print("\n" + summary)


def main():
    """メイン処理"""
    controller = MatchController()

    # サービス起動待機
    if not controller.wait_for_services(timeout=120):
        print("サービスが起動しませんでした", file=sys.stderr)
        sys.exit(1)

    # チーム接続待機
    if not controller.wait_for_teams(timeout=60):
        print("チームの接続がタイムアウトしました", file=sys.stderr)
        # 続行する（接続していなくても試合を開始）

    # 試合開始
    if not controller.start_match_sequence():
        print("試合を開始できませんでした", file=sys.stderr)
        sys.exit(1)

    # 試合監視（最大120秒）
    controller.monitor_match_via_api(max_duration=120)

    # 結果保存
    controller.save_results()

    print("\n✓ 試合制御完了")
    sys.exit(0)


if __name__ == "__main__":
    main()
