#!/usr/bin/env python3
"""
TIGERs対戦試合制御スクリプト (Protocol Buffers版)

ssl-game-controllerのProtocol Buffersメッセージを使用して試合を制御・監視します。
"""

import time
import sys
import socket
import struct
import asyncio
from typing import List, Tuple, Optional

try:
    import websockets
except ImportError as e:
    print(f"必要なモジュールが不足: {e}", file=sys.stderr)
    print("pip install websocketsを実行してください", file=sys.stderr)
    sys.exit(1)

# Add proto directory to path
sys.path.insert(0, "/app/proto")
try:
    # API protobuf imports
    from api import ssl_gc_api_pb2
    from state import ssl_gc_referee_message_pb2 as referee_pb2
except ImportError as e:
    print(f"protobufモジュールのインポートエラー: {e}", file=sys.stderr)
    print("protoファイルをコンパイルしてください", file=sys.stderr)
    sys.exit(1)


class MatchController:
    def __init__(self):
        self.gc_host = "localhost"
        self.gc_port = 8082
        self.gc_ws_url = f"ws://{self.gc_host}:{self.gc_port}/api/control"

        self.yellow_name = "ibis"
        self.blue_name = "TIGERs Mannheim"

        self.yellow_score = 0
        self.blue_score = 0
        self.events: List[Tuple[float, str]] = []
        self.start_time: Optional[float] = None
        self.match_duration = 0.0

        # Referee message parameters
        self.referee_address = "224.5.23.1"
        self.referee_port = 11003  # Updated port

    def wait_for_services(self, timeout: int = 30) -> bool:
        """サービスが起動するまで待機"""
        print(f"サービスの起動を待機中（{timeout}秒）...")
        time.sleep(timeout)
        print("✓ サービス起動待機完了")
        return True

    def wait_for_teams(self, timeout: int = 20) -> bool:
        """チームの接続を待機"""
        print(f"チームの接続を待機中（{timeout}秒）...")
        time.sleep(timeout)
        print("✓ チーム接続待機完了")
        return True

    def create_referee_socket(self) -> Optional[socket.socket]:
        """レフェリーメッセージ受信用のマルチキャストソケットを作成"""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(("", self.referee_port))
            mreq = struct.pack(
                "4sl", socket.inet_aton(self.referee_address), socket.INADDR_ANY
            )
            sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
            sock.settimeout(1.0)
            print(
                f"✓ レフェリーメッセージ受信開始: {self.referee_address}:{self.referee_port}"
            )
            return sock
        except Exception as e:
            print(f"✗ マルチキャストソケット作成エラー: {e}", file=sys.stderr)
            return None

    async def send_ws_command(
        self, input_msg: ssl_gc_api_pb2.Input, description: str = ""
    ) -> bool:
        """WebSocket経由でProtocol Buffersコマンドを送信"""
        try:
            print(f"  WebSocket接続試行: {self.gc_ws_url}")
            async with websockets.connect(
                self.gc_ws_url, open_timeout=10, close_timeout=5
            ) as ws:
                print(f"  接続成功: {ws.remote_address}")
                # Protocol Buffersメッセージをバイナリにシリアライズ
                binary_message = input_msg.SerializeToString()
                print(f"  メッセージサイズ: {len(binary_message)} bytes")

                # バイナリメッセージを送信
                await ws.send(binary_message)
                print(f"✓ コマンド送信: {description}")

                # 応答を受信
                try:
                    response = await asyncio.wait_for(ws.recv(), timeout=2.0)

                    # 応答がstr（テキストフレーム）の場合はJSON形式
                    if isinstance(response, str):
                        try:
                            import json

                            data = json.loads(response)
                            # match_typeやチーム名が設定されているか確認
                            if "matchState" in data and data["matchState"] is not None:
                                match_type = data["matchState"].get(
                                    "match_type", "UNKNOWN"
                                )
                                yellow_name = (
                                    data["matchState"]
                                    .get("team_state", {})
                                    .get("YELLOW", {})
                                    .get("name", "Unknown")
                                )
                                blue_name = (
                                    data["matchState"]
                                    .get("team_state", {})
                                    .get("BLUE", {})
                                    .get("name", "Unknown")
                                )
                                print(
                                    f"  応答: match_type={match_type}, YELLOW={yellow_name}, BLUE={blue_name}"
                                )
                            else:
                                print("  応答: matchState=null")
                        except Exception:
                            print(f"  応答受信（JSON）: {len(response)} bytes")
                    else:
                        # バイナリ応答の場合
                        output_msg = ssl_gc_api_pb2.Output()
                        output_msg.ParseFromString(response)
                        print(f"  応答受信（Binary）: {len(response)} bytes")
                except asyncio.TimeoutError:
                    print("  応答タイムアウト（正常）")

                return True
        except Exception as e:
            print(f"✗ WebSocket通信エラー: {e}", file=sys.stderr)
            import traceback

            traceback.print_exc()
            return False

    def check_current_stage(self) -> Optional[str]:
        """現在のステージを確認"""
        try:
            sock = self.create_referee_socket()
            if not sock:
                return None

            sock.settimeout(3.0)
            data, _ = sock.recvfrom(65536)
            referee_msg = referee_pb2.Referee()
            referee_msg.ParseFromString(data)
            stage = referee_pb2.Referee.Stage.Name(referee_msg.stage)
            sock.close()
            return stage
        except Exception as e:
            print(f"  ステージ確認失敗: {e}")
            return None

    def start_match_sequence(self) -> bool:
        """試合開始シーケンスを実行（Engine自動実行 + 必要に応じて強制開始）"""
        print("試合開始前の状態確認...")

        initial_stage = self.check_current_stage()
        if initial_stage:
            print(f"  初期ステージ: {initial_stage}")

        print("\nEngine自動実行を待機中...")
        print("（autoContinue: trueによりEngineが自動的に試合を進行させます）")

        # Engineが処理するまで十分な時間待機
        time.sleep(10.0)

        # 最終ステージと現在のコマンドを確認
        sock = self.create_referee_socket()
        force_start_needed = False

        if sock:
            try:
                sock.settimeout(3.0)
                data, _ = sock.recvfrom(65536)
                referee_msg = referee_pb2.Referee()
                referee_msg.ParseFromString(data)

                current_stage = referee_pb2.Referee.Stage.Name(referee_msg.stage)
                current_command = referee_pb2.Referee.Command.Name(referee_msg.command)

                print(f"  現在のステージ: {current_stage}")
                print(f"  現在のコマンド: {current_command}")

                # STOPまたはHALT状態の場合、強制開始が必要
                if current_command in ["STOP", "HALT"]:
                    force_start_needed = True
                    print(
                        "\n⚠  自動開始が動作していないため、WebSocket API経由で試合を強制開始します"
                    )

                sock.close()

            except Exception as e:
                print(f"⚠  状態確認エラー: {e}")
                sock.close()

        # 強制開始が必要な場合、WebSocket API経由でコマンドを送信
        if force_start_needed:
            try:
                print("  Step 1: KICKOFF for YELLOW")
                input_msg = ssl_gc_api_pb2.Input()
                input_msg.change.new_command.type = (
                    ssl_gc_api_pb2.Input.Change.NewCommand.Type.KICKOFF
                )
                input_msg.change.new_command.for_team = ssl_gc_api_pb2.Team.YELLOW
                asyncio.run(self.send_ws_command(input_msg, "KICKOFF for YELLOW"))
                time.sleep(2)

                print("  Step 2: NORMAL_START")
                input_msg = ssl_gc_api_pb2.Input()
                input_msg.change.new_command.type = (
                    ssl_gc_api_pb2.Input.Change.NewCommand.Type.NORMAL_START
                )
                input_msg.change.new_command.for_team = ssl_gc_api_pb2.Team.UNKNOWN
                asyncio.run(self.send_ws_command(input_msg, "NORMAL_START"))
                time.sleep(2)

                print("  Step 3: FORCE_START")
                input_msg = ssl_gc_api_pb2.Input()
                input_msg.change.new_command.type = (
                    ssl_gc_api_pb2.Input.Change.NewCommand.Type.FORCE_START
                )
                input_msg.change.new_command.for_team = ssl_gc_api_pb2.Team.UNKNOWN
                asyncio.run(self.send_ws_command(input_msg, "FORCE_START"))
                time.sleep(2)

                print("✓ 強制開始コマンド送信完了")

            except Exception as e:
                print(f"✗ 強制開始エラー: {e}", file=sys.stderr)

        print("✓ 試合開始シーケンス完了")
        return True

    def monitor_match(self, max_duration: int = 180) -> bool:
        """試合を監視（Protocol Buffers使用）"""
        print(f"試合を監視中（Protocol Buffers使用、最大{max_duration}秒）...")

        sock = self.create_referee_socket()
        if not sock:
            return False

        self.start_time = time.time()
        last_print_time = time.time()
        print_interval = 2.0

        try:
            while True:
                try:
                    data, _ = sock.recvfrom(65536)
                    referee_msg = referee_pb2.Referee()
                    referee_msg.ParseFromString(data)

                    # Score update
                    self.yellow_score = referee_msg.yellow.score
                    self.blue_score = referee_msg.blue.score

                    # Elapsed time
                    elapsed = time.time() - self.start_time
                    current_time = time.time()

                    # Stage and command
                    stage = referee_pb2.Referee.Stage.Name(referee_msg.stage)
                    command = referee_pb2.Referee.Command.Name(referee_msg.command)

                    # Event count
                    event_count = len(referee_msg.game_events)

                    # Print status every interval
                    if current_time - last_print_time >= print_interval:
                        print(
                            f"  経過時間: {int(elapsed)}秒 "
                            f"(Score: {self.yellow_score}-{self.blue_score}), "
                            f"Command: {command}, Events: {event_count}"
                        )
                        last_print_time = current_time

                    # Check if match ended
                    if stage == "POST_GAME" or elapsed > max_duration:
                        self.match_duration = elapsed
                        break

                except socket.timeout:
                    continue

        except KeyboardInterrupt:
            print("\n試合監視を中断しました")
        finally:
            sock.close()

        return True

    def save_results(self, filename: str = "match_result.txt") -> None:
        """試合結果をファイルに保存"""
        try:
            with open(f"/app/results/{filename}", "w", encoding="utf-8") as f:
                f.write("=== TIGERs対戦試合結果 ===\n\n")
                f.write(f"試合時間: {self.match_duration:.1f}秒\n")
                f.write(
                    f"最終スコア: {self.yellow_name} {self.yellow_score} - {self.blue_score} {self.blue_name}\n\n"
                )
                f.write(f"イベント数: {len(self.events)}\n")

                if self.events:
                    f.write("\n=== イベント履歴 ===\n")
                    for timestamp, event_desc in self.events:
                        f.write(f"[{timestamp:6.1f}s] {event_desc}\n")

            print(f"✓ 試合結果を保存: {filename}")
        except Exception as e:
            print(f"✗ 結果保存エラー: {e}", file=sys.stderr)

    def run(self) -> int:
        """メイン実行フロー"""
        print("=== TIGERs対戦試合制御スクリプト (Protocol Buffers版) ===\n")

        # サービス起動待機
        if not self.wait_for_services():
            return 1

        # チーム接続待機
        if not self.wait_for_teams():
            return 1

        # 試合開始
        if not self.start_match_sequence():
            return 1

        # 試合監視
        if not self.monitor_match():
            return 1

        # 結果保存
        self.save_results()

        print("\n✓ 試合制御完了")
        return 0


if __name__ == "__main__":
    controller = MatchController()
    sys.exit(controller.run())
