#!/usr/bin/env python3
"""
TIGERs対戦試合制御スクリプト

ssl-game-controllerのProtocol Buffersメッセージを使用して試合を監視し、
結果を監視してテキストサマリーを出力します。
"""

import time
import sys
import socket
import struct
from typing import List, Tuple, Optional
from importlib.util import find_spec

if find_spec("requests") is None:
    print("requestsモジュールが必要です: pip install requests", file=sys.stderr)
    sys.exit(1)


# Add proto directory to path
sys.path.insert(0, "/app/proto")
try:
    import ssl_gc_referee_message_pb2 as referee_pb2
except ImportError:
    print("protobufモジュールが必要です: pip install protobuf", file=sys.stderr)
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

        # Referee message parameters
        self.referee_address = "224.5.23.1"
        self.referee_port = 10003

        # Track processed game events to avoid duplicates
        self.processed_event_ids: set = set()

    @staticmethod
    def format_game_event(
        event, yellow_name: str = "ibis", blue_name: str = "TIGERs"
    ) -> str:
        """
        GameEventをシンプルな説明文に変換

        Args:
            event: GameEventオブジェクト
            yellow_name: Yellowチーム名
            blue_name: Blueチーム名

        Returns:
            イベントの説明文（元のイベント名 + チーム情報）
        """
        try:
            event_type = referee_pb2.GameEvent.Type.Name(event.type)
        except Exception:
            event_type = "UNKNOWN"

        # Team name helper
        def team_name(team_enum):
            if team_enum == 0:  # YELLOW
                return yellow_name
            elif team_enum == 1:  # BLUE
                return blue_name
            return "UNKNOWN"

        # Try to extract team information from common event fields
        team_info = ""
        try:
            # Most events have a 'by_team' field
            for field_name in ["by_team", "by_team_yellow", "by_team_blue"]:
                if event.HasField(field_name) or hasattr(event, field_name):
                    # Get the first subfield that exists
                    for sub in event.DESCRIPTOR.fields:
                        if hasattr(event, sub.name) and event.HasField(sub.name):
                            sub_event = getattr(event, sub.name)
                            if hasattr(sub_event, "by_team"):
                                team_info = f" by {team_name(sub_event.by_team)}"
                                break
                            elif hasattr(sub_event, "by_team_yellow"):
                                team_info = (
                                    f" (Yellow: {yellow_name}, Blue: {blue_name})"
                                )
                                break
                    break
        except Exception:
            pass

        return f"{event_type}{team_info}"

    def wait_for_services(self, timeout: int = 30) -> bool:
        """サービスが起動するまで待機（固定時間）"""
        print(f"サービスの起動を待機中（{timeout}秒）...")
        time.sleep(timeout)
        print("✓ サービス起動待機完了")
        return True

    def wait_for_teams(self, timeout: int = 20) -> bool:
        """チームの接続を待機（固定時間）"""
        print(f"チームの接続を待機中（{timeout}秒）...")
        time.sleep(timeout)
        print("✓ チーム接続待機完了")
        return True

    def create_referee_socket(self) -> Optional[socket.socket]:
        """レフェリーメッセージ受信用のマルチキャストソケットを作成"""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

            # Bind to the referee port
            sock.bind(("", self.referee_port))

            # Join multicast group
            mreq = struct.pack(
                "4sl", socket.inet_aton(self.referee_address), socket.INADDR_ANY
            )
            sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

            # Set non-blocking with timeout
            sock.settimeout(1.0)

            print(
                f"✓ レフェリーメッセージ受信開始: {self.referee_address}:{self.referee_port}"
            )
            return sock
        except Exception as e:
            print(f"✗ マルチキャストソケット作成エラー: {e}", file=sys.stderr)
            return None

    def start_match_sequence(self) -> bool:
        """試合開始シーケンスを実行（自動開始を前提）"""
        print("試合の自動開始を待機中...")
        print("（ssl-game-controllerの設定により自動的に試合が開始されます）")
        self.start_time = time.time()
        return True

    def monitor_match_protobuf(self, max_duration: int = 180):
        """
        Protocol Buffersを使用して試合状態を監視

        Args:
            max_duration: 最大試合時間（秒）
        """
        print(f"試合を監視中（Protocol Buffers使用、最大{max_duration}秒）...")

        sock = self.create_referee_socket()
        if not sock:
            print("⚠  ソケット作成失敗。簡易モードにフォールバック")
            return self.monitor_match_fallback(max_duration)

        last_yellow_score = 0
        last_blue_score = 0
        last_stage = None
        message_count = 0

        start_monitor = time.time()

        try:
            while time.time() - start_monitor < max_duration:
                try:
                    # Receive referee message
                    data, _ = sock.recvfrom(65536)
                    message_count += 1

                    # Parse protobuf message
                    referee_msg = referee_pb2.Referee()
                    referee_msg.ParseFromString(data)

                    # Get current scores
                    self.yellow_score = referee_msg.yellow.score
                    self.blue_score = referee_msg.blue.score

                    # Get current stage
                    current_stage = referee_pb2.Referee.Stage.Name(referee_msg.stage)

                    # Record score changes
                    elapsed = time.time() - self.start_time if self.start_time else 0

                    if self.yellow_score > last_yellow_score:
                        self.events.append((elapsed, "GOAL by Yellow (ibis)"))
                        print(
                            f"  [{elapsed:.1f}s] 🟡 GOAL by Yellow! (Score: {self.yellow_score}-{self.blue_score})"
                        )
                        last_yellow_score = self.yellow_score

                    if self.blue_score > last_blue_score:
                        self.events.append((elapsed, "GOAL by Blue (TIGERs)"))
                        print(
                            f"  [{elapsed:.1f}s] 🔵 GOAL by Blue! (Score: {self.yellow_score}-{self.blue_score})"
                        )
                        last_blue_score = self.blue_score

                    # Record stage changes
                    if current_stage != last_stage:
                        self.events.append((elapsed, f"Stage: {current_stage}"))
                        print(f"  [{elapsed:.1f}s] Stage: {current_stage}")
                        last_stage = current_stage

                        # End match at POST_GAME
                        if current_stage == "POST_GAME":
                            print("\n✓ 試合が終了しました（POST_GAME）")
                            break

                    # Process game events
                    for game_event in referee_msg.game_events:
                        # Create unique event ID to avoid duplicates
                        event_id = f"{game_event.type}_{elapsed:.1f}"
                        if event_id not in self.processed_event_ids:
                            self.processed_event_ids.add(event_id)
                            event_desc = self.format_game_event(
                                game_event, self.yellow_name, self.blue_name
                            )
                            self.events.append((elapsed, event_desc))
                            print(f"  [{elapsed:.1f}s] {event_desc}")

                    # Progress indicator
                    if message_count % 50 == 0:
                        print(
                            f"  経過時間: {elapsed:.0f}秒 (Score: {self.yellow_score}-{self.blue_score})"
                        )

                except socket.timeout:
                    # No message received, continue
                    continue
                except Exception as e:
                    print(f"⚠  メッセージ処理エラー: {e}", file=sys.stderr)
                    continue

        finally:
            sock.close()

        self.match_duration = time.time() - self.start_time if self.start_time else 0
        print(
            f"\n試合監視完了（実行時間: {self.match_duration:.1f}秒、受信メッセージ数: {message_count}）"
        )

    def monitor_match_fallback(self, max_duration: int = 120):
        """
        試合を固定時間待機（フォールバック版）

        Args:
            max_duration: 試合時間（秒）
        """
        print(f"試合を監視中（フォールバック: {max_duration}秒待機）...")
        print(
            "（注: レフェリーメッセージを受信できないため、スコア/イベントの監視はできません）"
        )

        for i in range(max_duration):
            if i % 10 == 0:
                elapsed = time.time() - self.start_time if self.start_time else i
                print(f"  経過時間: {elapsed:.0f}秒")
            time.sleep(1)

        self.match_duration = (
            time.time() - self.start_time if self.start_time else max_duration
        )
        print(f"\n試合終了（実行時間: {self.match_duration:.1f}秒）")

    def generate_summary(self) -> str:
        """試合サマリーを生成"""

        # 勝敗判定
        if self.yellow_score > self.blue_score:
            result = f"CRANE WIN ({self.yellow_name})"
        elif self.blue_score > self.yellow_score:
            result = f"TIGERs WIN ({self.blue_name})"
        elif self.yellow_score == 0 and self.blue_score == 0 and not self.events:
            result = "試合完了（スコア情報なし）"
        else:
            result = "DRAW (引き分け)"

        # スコア情報の有無を判定
        has_score_info = (
            self.yellow_score > 0 or self.blue_score > 0 or len(self.events) > 0
        )

        summary = f"""=====================================
        TIGERs対戦結果サマリー
=====================================

【試合実行状況】
  ✓ 試合が正常に実行されました

【スコア】
  {self.yellow_name} (Yellow): {self.yellow_score}
  {self.blue_name} (Blue): {self.blue_score}"""

        if not has_score_info:
            summary += "\n  ⚠  レフェリーメッセージを受信できませんでした"

        summary += f"""

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
            summary += "  - レフェリーメッセージを受信できなかったため、イベント記録はありません\n"

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

    # サービス起動待機（30秒）
    if not controller.wait_for_services(timeout=30):
        print("サービスが起動しませんでした", file=sys.stderr)
        sys.exit(1)

    # チーム接続待機（20秒）
    if not controller.wait_for_teams(timeout=20):
        print("チームの接続がタイムアウトしました", file=sys.stderr)
        # 続行する（接続していなくても試合を開始）

    # 試合開始（自動開始を前提）
    if not controller.start_match_sequence():
        print("試合を開始できませんでした", file=sys.stderr)
        sys.exit(1)

    # 試合監視（Protocol Buffers使用、最大180秒）
    controller.monitor_match_protobuf(max_duration=180)

    # 結果保存
    controller.save_results()

    print("\n✓ 試合制御完了")
    sys.exit(0)


if __name__ == "__main__":
    main()
