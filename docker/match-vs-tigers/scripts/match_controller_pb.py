#!/usr/bin/env python3
"""
TIGERs対戦試合制御スクリプト (Protocol Buffers版)

ssl-game-controllerのProtocol Buffersメッセージを使用して試合を制御・監視します。
"""

import asyncio
import json
import os
import socket
import struct
import sys
import time

try:
    import websockets
except ImportError as e:
    print(f"必要なモジュールが不足: {e}", file=sys.stderr)
    print("pip install websocketsを実行してください", file=sys.stderr)
    sys.exit(1)

# Add proto directory to path
sys.path.insert(0, "/app/proto")
try:
    from api import ssl_gc_api_pb2
    from engine import ssl_gc_engine_pb2 as engine_pb2
    from ssl_simulation import ssl_simulation_control_pb2
    from state import ssl_gc_common_pb2 as common_pb2
    from state import ssl_gc_game_event_pb2 as game_event_pb2
    from state import ssl_gc_referee_message_pb2 as referee_pb2
except ImportError as e:
    print(f"protobufモジュールのインポートエラー: {e}", file=sys.stderr)
    print("protoファイルをコンパイルしてください", file=sys.stderr)
    sys.exit(1)

try:
    from google.protobuf.json_format import MessageToJson
except ImportError as e:
    print(f"google.protobuf.json_formatのインポートエラー: {e}", file=sys.stderr)
    sys.exit(1)

# HALT状態でGCの継続アクションを代行するまでの秒数
HALT_RECOVERY_TIMEOUT = 1
# STOP状態でGCの継続アクションを代行するまでの秒数
STOP_CONTINUE_TIMEOUT = 3
# 継続アクションでも動かなかった場合にFORCE_STARTを送信するまでの秒数
STOP_RECOVERY_TIMEOUT = 15

# 代行する継続アクションの優先順位（この一覧に無いものは絶対に送らない）。
# ボール配置を最優先にするのは、場外のまま再開すると NO_PROGRESS_IN_GAME に
# なるため。ステージ送りを最後にするのは、ボールを戻す前に前後半を進めない
# ようにするため。
#
# 許可リスト方式にしているのは、GCが END_GAME のような「押したら試合が終わる」
# アクションも READY_MANUAL として並べてくるため。「BLOCKEDでない先頭を押す」
# という一般則にすると試合を即終了させてしまう。
CONTINUE_ACTION_PRIORITY = (
    "BALL_PLACEMENT_START",
    "NEXT_COMMAND",
    "RESUME_FROM_HALT",
    "NEXT_STAGE",
)
# 実行可能とみなす継続アクションの状態（BLOCKED / DISABLED は送っても無駄で、
# 送る次コマンドが無い状態のNEXT_COMMANDはGCをクラッシュさせることがある）
ACTIONABLE_STATES = ("READY_AUTO", "READY_MANUAL")

# 後半が終わったステージでは NEXT_STAGE の代わりに END_GAME を送る。
# GCのステージ連鎖は固定順で、NORMAL_SECOND_HALF の次は EXTRA_TIME_BREAK に
# なる。延長戦はスコアが引き分けのときに意味を持つもので、CIの対戦では不要
# （実際に 0-6 で決着しているのに延長前半まで進んでしまった）。
# END_GAME で POST_GAME へ送れば、監視ループが stage == "POST_GAME" を見て
# 正常に試合を終えられる。
STAGES_TO_END_INSTEAD_OF_ADVANCE = ("NORMAL_SECOND_HALF",)
# PREPARE_KICKOFF/PENALTY後にNORMAL_STARTを送信するまでの秒数
PREPARE_RECOVERY_TIMEOUT = 5

# フィールド設定（ssl-game-controller-match.yaml に合わせる）
FIELD_HALF_LENGTH = 6.0  # field-length: 12 の半分 [m]
PENALTY_KICK_DIST = 8.0  # penalty-kick-dist-to-goal [m]
# ペナルティスポットのセンターからの距離: PENALTY_KICK_DIST - FIELD_HALF_LENGTH = 2.0 [m]
PENALTY_SPOT_OFFSET = PENALTY_KICK_DIST - FIELD_HALF_LENGTH


class MatchController:
    def __init__(self):
        self.gc_host = "localhost"
        self.gc_port = 8082
        self.gc_ws_url = f"ws://{self.gc_host}:{self.gc_port}/api/control"

        self.yellow_name = "ibis"
        self.blue_name = "TIGERs Mannheim"

        self.yellow_score = 0
        self.blue_score = 0
        self.events: list[tuple[float, str]] = []
        self.start_time: float | None = None
        self.match_duration = 0.0

        # ファウル・カード追跡
        self.yellow_cards = {"YELLOW": 0, "BLUE": 0}
        self.foul_counters = {"YELLOW": 0, "BLUE": 0}
        self.max_allowed_bots = {"YELLOW": 0, "BLUE": 0}
        # (elapsed_sec, team_name, event_type) のリスト
        self.foul_event_log: list[tuple[float, str, str]] = []
        # event_type -> count per team
        self.foul_event_counts: dict = {"YELLOW": {}, "BLUE": {}}

        # Referee message parameters
        self.referee_address = "224.5.23.1"
        self.referee_port = 11003

        # Vision parameters (ER-Force simulator publishes on the simulated vision port)
        self.vision_address = "224.5.23.2"
        self.vision_port = 10020

        # ssl-simulation-protocol control endpoint (ER-Force simulator-cli)
        self.sim_host = "localhost"
        self.sim_control_port = 10300
        self._sim_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def _create_multicast_socket(
        self, address: str, port: int, timeout: float = 1.0
    ) -> socket.socket | None:
        """マルチキャストUDPソケットを作成"""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(("", port))
            mreq = struct.pack("4sl", socket.inet_aton(address), socket.INADDR_ANY)
            sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
            sock.settimeout(timeout)
            return sock
        except Exception as e:  # noqa: BLE001
            print(
                f"✗ マルチキャストソケット作成エラー ({address}:{port}): {e}",
                file=sys.stderr,
            )
            return None

    def create_referee_socket(self) -> socket.socket | None:
        """レフェリーメッセージ受信用のマルチキャストソケットを作成"""
        sock = self._create_multicast_socket(self.referee_address, self.referee_port)
        if sock:
            print(
                f"✓ レフェリーメッセージ受信開始: {self.referee_address}:{self.referee_port}"
            )
        return sock

    def teleport_ball_to_position(self, x: float, y: float) -> None:
        """ssl-simulation-protocolでボールを指定座標にテレポート"""
        try:
            cmd = ssl_simulation_control_pb2.SimulatorCommand()
            tp = cmd.control.teleport_ball
            tp.x = x
            tp.y = y
            tp.vx = 0.0
            tp.vy = 0.0
            self._sim_sock.sendto(
                cmd.SerializeToString(),
                (self.sim_host, self.sim_control_port),
            )
            print(f"✓ ボールをテレポート: ({x:.2f}, {y:.2f})")
        except Exception as e:  # noqa: BLE001
            print(f"  ボールテレポートエラー: {e}")

    def teleport_ball_to_center(self) -> None:
        """フィールドセンターへテレポート"""
        self.teleport_ball_to_position(0.0, 0.0)

    def handle_command_change(self, command: str, referee_msg) -> None:
        """コマンド変化時にボールを適切な位置に配置"""
        if command in ("PREPARE_KICKOFF_YELLOW", "PREPARE_KICKOFF_BLUE"):
            self.teleport_ball_to_center()
        elif command == "PREPARE_PENALTY_YELLOW":
            # Yellowにペナルティ → Blueがキック → Yellow側ゴール(+x方向)へ
            # ペナルティスポット: センターから -PENALTY_SPOT_OFFSET = -2.0 m
            self.teleport_ball_to_position(-PENALTY_SPOT_OFFSET, 0.0)
        elif command == "PREPARE_PENALTY_BLUE":
            # Blueにペナルティ → Yellowがキック → Blue側ゴール(-x方向)へ
            # ペナルティスポット: センターから +PENALTY_SPOT_OFFSET = +2.0 m
            self.teleport_ball_to_position(PENALTY_SPOT_OFFSET, 0.0)
        elif command in (
            "BALL_PLACEMENT_YELLOW",
            "BALL_PLACEMENT_BLUE",
        ) and referee_msg.HasField("designated_position"):
            # SSL referee protobufはミリメートル単位、ssl-simulation-protocolはメートル単位
            self.teleport_ball_to_position(
                referee_msg.designated_position.x / 1000.0,
                referee_msg.designated_position.y / 1000.0,
            )

    def _extract_event_team(self, ev) -> str | None:
        """ゲームイベントからチーム名を抽出する"""
        field_name = ev.WhichOneof("event")
        if not field_name:
            return None
        try:
            sub = getattr(ev, field_name)
            if sub.HasField("by_team"):
                return common_pb2.Team.Name(sub.by_team)
        except (AttributeError, ValueError):
            pass
        return None

    def wait_for_gc_ready(self, timeout: int = 60) -> bool:
        """GCのWebSocket接続が利用可能になるまでポーリング"""
        print(f"Game Controllerの起動を待機中（最大{timeout}秒）...")
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                result = asyncio.run(self._check_gc_connection())
                if result:
                    print("✓ Game Controller準備完了")
                    return True
            except Exception:  # noqa: BLE001, S110
                pass
            time.sleep(2)
        print("✗ Game Controllerの起動タイムアウト", file=sys.stderr)
        return False

    async def _check_gc_connection(self) -> bool:
        """GCへのWebSocket接続と初期Outputメッセージ受信を確認"""
        async with websockets.connect(self.gc_ws_url, open_timeout=3) as ws:
            await asyncio.wait_for(ws.recv(), timeout=3.0)
            return True

    def wait_for_referee(self, timeout: int = 60) -> bool:
        """レフェリーメッセージのUDPパケット受信をポーリング確認"""
        print(f"レフェリーメッセージの受信を待機中（最大{timeout}秒）...")
        sock = self._create_multicast_socket(
            self.referee_address, self.referee_port, timeout=2.0
        )
        if not sock:
            return False
        deadline = time.time() + timeout
        try:
            while time.time() < deadline:
                try:
                    data, _ = sock.recvfrom(65536)
                    referee_msg = referee_pb2.Referee()
                    referee_msg.ParseFromString(data)
                    command = referee_pb2.Referee.Command.Name(referee_msg.command)
                    print(f"✓ レフェリーメッセージ受信確認 (command={command})")
                    return True
                except TimeoutError:
                    continue
        finally:
            sock.close()
        print("✗ レフェリーメッセージ受信タイムアウト", file=sys.stderr)
        return False

    def wait_for_vision(self, timeout: int = 60) -> bool:
        """シミュレータからのVisionデータ到達をポーリング確認"""
        print(f"Visionデータの受信を待機中（最大{timeout}秒）...")
        sock = self._create_multicast_socket(
            self.vision_address, self.vision_port, timeout=2.0
        )
        if not sock:
            return False
        deadline = time.time() + timeout
        try:
            while time.time() < deadline:
                try:
                    data, _ = sock.recvfrom(65536)
                    if len(data) > 0:
                        print("✓ Visionデータ受信確認")
                        return True
                except TimeoutError:
                    continue
        finally:
            sock.close()
        print("✗ Visionデータ受信タイムアウト", file=sys.stderr)
        return False

    async def send_input(
        self, input_msg: ssl_gc_api_pb2.Input, description: str = ""
    ) -> bool:
        """WebSocket経由でInputメッセージをJSON形式で送信"""
        try:
            async with websockets.connect(self.gc_ws_url, open_timeout=10) as ws:
                # GC接続時の初期Outputメッセージを受信
                await asyncio.wait_for(ws.recv(), timeout=5.0)
                # JSON形式でシリアライズして送信
                json_str = MessageToJson(input_msg, preserving_proto_field_name=True)
                await ws.send(json_str)
                print(f"✓ コマンド送信: {description}")
                # 応答受信
                try:
                    response = await asyncio.wait_for(ws.recv(), timeout=5.0)
                    if isinstance(response, str):
                        data = json.loads(response)
                        if data.get("matchState"):
                            print("  GC応答: matchState受信")
                except asyncio.TimeoutError:
                    print("  応答タイムアウト（正常）")
                return True
        except Exception as e:  # noqa: BLE001
            print(f"✗ WebSocket通信エラー: {e}", file=sys.stderr)
            return False

    async def send_continue_action(self, action_type, for_team=None) -> bool:
        """ContinueActionを送信（GC UIの「Continue」ボタンと同等）"""
        input_msg = ssl_gc_api_pb2.Input()
        input_msg.continue_action.type = action_type
        input_msg.continue_action.for_team = (
            common_pb2.UNKNOWN if for_team is None else for_team
        )
        type_name = engine_pb2.ContinueAction.Type.Name(action_type)
        return await self.send_input(input_msg, f"ContinueAction: {type_name}")

    async def _fetch_continue_actions(self) -> list | None:
        """GCが現在提示している継続アクションを読む。

        Outputメッセージは {matchState, gcState, protocol, config} の形で、
        continueActions は gcState の下にある。しかも完全な状態が入るのは
        接続直後の1通目だけで、以降は差分なので取りこぼさないよう数通読む。
        見つからなければ None を返す（空リストと区別する）。
        """
        async with websockets.connect(self.gc_ws_url, open_timeout=3) as ws:
            for _ in range(5):
                msg = json.loads(await asyncio.wait_for(ws.recv(), timeout=3.0))
                gc_state = msg.get("gcState")
                if isinstance(gc_state, dict) and "continueActions" in gc_state:
                    return gc_state["continueActions"]
        return None

    def get_continue_actions(self) -> list:
        """継続アクション一覧を取得する（取得できなければ空リスト）"""
        try:
            actions = asyncio.run(self._fetch_continue_actions())
        except Exception as e:  # noqa: BLE001
            print(f"  継続アクション取得エラー: {type(e).__name__}: {e}")
            return []
        if actions is None:
            # 黙って空を返すと「代行すべきアクションが無い」と区別できず、
            # 不具合が無言で握り潰される
            print("  ⚠ 継続アクションを読めなかった（gcStateにcontinueActionsが無い）")
            return []
        return actions

    def send_available_continue_action(self, stage: str | None = None) -> bool:
        """GCが提示している継続アクションを代行実行する。

        GCエンジンが autoContinue で自動実行するのは ContinueActions[0] が
        READY_AUTO のときだけ（process_continue.go の defaultAction は
        先頭要素を無条件に返す）。前半の時間切れで NEXT_STAGE(READY_MANUAL)
        が先頭に入ると、その後ろに BALL_PLACEMENT_START(READY_AUTO) が
        並んでいても自動実行は止まる。オペレータが居ないCIではここで代わりに
        押してやる必要がある。
        """
        priority = CONTINUE_ACTION_PRIORITY
        if stage in STAGES_TO_END_INSTEAD_OF_ADVANCE:
            priority = tuple(
                "END_GAME" if name == "NEXT_STAGE" else name for name in priority
            )
        actions = self.get_continue_actions()
        usable = {
            a.get("type"): a
            for a in actions
            if a.get("state") in ACTIONABLE_STATES and a.get("type")
        }
        for type_name in priority:
            action = usable.get(type_name)
            if action is None:
                continue
            team_name = action.get("forTeam", "UNKNOWN")
            print(f"  継続アクションを代行: {type_name} ({team_name})")
            try:
                asyncio.run(
                    self.send_continue_action(
                        engine_pb2.ContinueAction.Type.Value(type_name),
                        common_pb2.Team.Value(team_name),
                    )
                )
            except Exception as e:  # noqa: BLE001
                print(f"  {type_name}エラー: {e}")
                return False
            return True
        if actions:
            offered = ", ".join(f"{a.get('type')}({a.get('state')})" for a in actions)
            print(f"  実行可能な継続アクションなし: {offered}")
        return False

    def get_current_command(self) -> str | None:
        """現在のコマンドをレフェリーメッセージから取得"""
        try:
            sock = self._create_multicast_socket(
                self.referee_address, self.referee_port, timeout=3.0
            )
            if not sock:
                return None
            try:
                data, _ = sock.recvfrom(65536)
                referee_msg = referee_pb2.Referee()
                referee_msg.ParseFromString(data)
                return referee_pb2.Referee.Command.Name(referee_msg.command)
            finally:
                sock.close()
        except Exception as e:  # noqa: BLE001
            print(f"  コマンド確認失敗: {e}")
            return None

    def start_match_sequence(self) -> bool:
        """ContinueActionを使って試合開始シーケンスを実行"""
        print("試合開始シーケンスを実行中...")

        current_command = self.get_current_command()
        print(f"  現在のコマンド: {current_command}")

        # HALT/STOP以外の状態はすべてスキップ
        # GCは continue-from-halt: true により自動遷移するため、
        # PREPARE_KICKOFF等にある場合もNEXT_COMMANDを送らない
        # （NORMAL_START状態でNEXT_COMMANDを送るとGCがnilポインタpanicする）
        if current_command not in ("HALT", "STOP", None):
            print(
                f"  試合はすでに開始済みまたは開始中 ({current_command})、シーケンスをスキップ"
            )
            print("✓ 試合開始シーケンス完了")
            return True

        # NEXT_COMMAND でキックオフ準備を開始
        print("  NEXT_COMMAND を送信...")
        asyncio.run(self.send_continue_action(engine_pb2.ContinueAction.NEXT_COMMAND))
        time.sleep(3)

        current_command = self.get_current_command()
        print(f"  コマンド確認: {current_command}")

        if current_command in ("STOP", "HALT", None):
            # STOPのままなら FORCE_START で強制開始
            print("  FORCE_START を送信...")
            asyncio.run(
                self.send_continue_action(engine_pb2.ContinueAction.FORCE_START)
            )
            time.sleep(2)
        else:
            # PREPARED → NORMAL_START でキックオフ
            print("  NORMAL_START を送信...")
            asyncio.run(
                self.send_continue_action(engine_pb2.ContinueAction.NORMAL_START)
            )
            time.sleep(2)

        print("✓ 試合開始シーケンス完了")
        return True

    def monitor_match(self, max_duration: int = 180) -> bool:
        """試合を監視（Protocol Buffers使用、STOP自動回復付き）"""
        print(f"試合を監視中（Protocol Buffers使用、最大{max_duration}秒）...")

        sock = self.create_referee_socket()
        if not sock:
            return False

        self.start_time = time.time()
        last_print_time = time.time()
        print_interval = 10.0
        stop_since: float | None = None
        last_continue_try = 0.0
        prev_yellow_score = 0
        prev_blue_score = 0
        last_event_count = 0
        last_command = ""

        try:
            while True:
                try:
                    data, _ = sock.recvfrom(65536)
                    referee_msg = referee_pb2.Referee()
                    referee_msg.ParseFromString(data)

                    # スコア更新
                    self.yellow_score = referee_msg.yellow.score
                    self.blue_score = referee_msg.blue.score

                    # イエローカード・ファウルカウント更新
                    self.yellow_cards["YELLOW"] = referee_msg.yellow.yellow_cards
                    self.yellow_cards["BLUE"] = referee_msg.blue.yellow_cards
                    self.foul_counters["YELLOW"] = referee_msg.yellow.foul_counter
                    self.foul_counters["BLUE"] = referee_msg.blue.foul_counter
                    self.max_allowed_bots["YELLOW"] = (
                        referee_msg.yellow.max_allowed_bots
                    )
                    self.max_allowed_bots["BLUE"] = referee_msg.blue.max_allowed_bots

                    # ゴール検知 → ボールをセンターへ移動
                    if (
                        self.yellow_score != prev_yellow_score
                        or self.blue_score != prev_blue_score
                    ):
                        print(
                            f"  ⚽ ゴール! スコア: {self.yellow_score}-{self.blue_score}"
                        )
                        self.teleport_ball_to_center()
                        stop_since = None  # STOP回復タイマーをリセット
                        prev_yellow_score = self.yellow_score
                        prev_blue_score = self.blue_score

                    current_time = time.time()
                    elapsed = current_time - self.start_time

                    stage = referee_pb2.Referee.Stage.Name(referee_msg.stage)
                    command = referee_pb2.Referee.Command.Name(referee_msg.command)

                    # 新規イベントのみ処理（game_eventsは累積リスト）
                    event_count = len(referee_msg.game_events)
                    has_possible_goal = False
                    if event_count > last_event_count:
                        new_event_types = []
                        for ev in referee_msg.game_events[last_event_count:]:
                            try:
                                ev_type = game_event_pb2.GameEvent.Type.Name(ev.type)
                            except Exception:  # noqa: BLE001
                                ev_type = str(ev.type)
                            new_event_types.append(ev_type)

                            # ファウルイベントのチームと種別を記録
                            team_str = self._extract_event_team(ev)
                            if team_str and self.start_time:
                                t = time.time() - self.start_time
                                self.foul_event_log.append((t, team_str, ev_type))
                                counts = self.foul_event_counts.setdefault(team_str, {})
                                counts[ev_type] = counts.get(ev_type, 0) + 1

                        print(
                            f"  [イベント] {', '.join(new_event_types)} (Score: {self.yellow_score}-{self.blue_score})"
                        )
                        last_event_count = event_count
                        has_possible_goal = "POSSIBLE_GOAL" in new_event_types
                    elif event_count < last_event_count:
                        last_event_count = event_count

                    # commandが変化したらログ出力 + ボール配置 + タイマーリセット
                    if command != last_command:
                        print(
                            f"  [コマンド変化] {last_command} → {command} (Stage: {stage})"
                        )
                        self.handle_command_change(command, referee_msg)
                        stop_since = None
                        last_command = command

                    # STOP/HALT 状態の処理
                    if command == "HALT":
                        # ゴール後のHALTは NEXT_COMMAND でキックオフへ進むが、
                        # ステージの時間が尽きているとNEXT_COMMANDはDISABLEDになり、
                        # 必要なのはNEXT_STAGEになる。どちらが要るかはGCに聞く。
                        # 無条件にNEXT_COMMANDを送り続けると、送る次コマンドが無い
                        # 場合にGCがnilポインタ参照でクラッシュする
                        # （statemachine/change_command.go:103）。
                        if stop_since is None:
                            stop_since = current_time
                        elif current_time - stop_since > HALT_RECOVERY_TIMEOUT:
                            self.send_available_continue_action(stage)
                            stop_since = None
                    elif command == "STOP":
                        if stop_since is None:
                            stop_since = current_time
                            last_continue_try = current_time
                        elif has_possible_goal:
                            if current_time - stop_since > STOP_RECOVERY_TIMEOUT:
                                print("  POSSIBLE_GOAL検出 → ACCEPT_GOALを送信")
                                try:
                                    asyncio.run(
                                        self.send_continue_action(
                                            engine_pb2.ContinueAction.ACCEPT_GOAL
                                        )
                                    )
                                except Exception as e:  # noqa: BLE001
                                    print(f"  ACCEPT_GOALエラー: {e}")
                                stop_since = None
                        elif current_time - stop_since > STOP_RECOVERY_TIMEOUT:
                            # 継続アクションでも動かないときだけの最終手段。
                            # FORCE_STARTはボール配置を飛ばして再開するため、
                            # ボールが場外のままだとNO_PROGRESS_IN_GAMEを繰り返す。
                            print(
                                f"  ⚠ 継続アクション後もSTOPが{STOP_RECOVERY_TIMEOUT}秒継続 → FORCE_STARTを送信"
                            )
                            try:
                                asyncio.run(
                                    self.send_continue_action(
                                        engine_pb2.ContinueAction.FORCE_START
                                    )
                                )
                            except Exception as e:  # noqa: BLE001
                                print(f"  FORCE_STARTエラー: {e}")
                            stop_since = None
                        elif current_time - last_continue_try > STOP_CONTINUE_TIMEOUT:
                            last_continue_try = current_time
                            self.send_available_continue_action(stage)
                    elif command in (
                        "PREPARE_KICKOFF_YELLOW",
                        "PREPARE_KICKOFF_BLUE",
                        "PREPARE_PENALTY_YELLOW",
                        "PREPARE_PENALTY_BLUE",
                    ):
                        if stop_since is None:
                            stop_since = current_time
                        elif current_time - stop_since > PREPARE_RECOVERY_TIMEOUT:
                            print(f"  {command} → NORMAL_STARTを送信")
                            try:
                                asyncio.run(
                                    self.send_continue_action(
                                        engine_pb2.ContinueAction.NORMAL_START
                                    )
                                )
                            except Exception as e:  # noqa: BLE001
                                print(f"  NORMAL_STARTエラー: {e}")
                            stop_since = None
                    else:
                        stop_since = None

                    # 定期的な状態出力
                    if current_time - last_print_time >= print_interval:
                        print(
                            f"  経過時間: {int(elapsed)}秒 "
                            f"(Score: {self.yellow_score}-{self.blue_score}), "
                            f"Stage: {stage}, Command: {command}, Events: {event_count}"
                        )
                        last_print_time = current_time

                    # 試合終了判定
                    if stage == "POST_GAME" or elapsed > max_duration:
                        self.match_duration = elapsed
                        print(f"  試合終了: stage={stage}, elapsed={int(elapsed)}秒")
                        break

                except TimeoutError:
                    # パケットなしの場合もタイムアウト確認
                    if self.start_time and time.time() - self.start_time > max_duration:
                        self.match_duration = time.time() - self.start_time
                        break
                    continue

        except KeyboardInterrupt:
            print("\n試合監視を中断しました")
        finally:
            sock.close()

        return True

    def save_results(self, filename: str = "match_result.txt") -> None:
        """試合結果をファイルに保存（CIワークフローが認識できる勝敗文字列を含む）"""
        if self.yellow_score > self.blue_score:
            result_str = "CRANE WIN"
        elif self.blue_score > self.yellow_score:
            result_str = "TIGERs WIN"
        else:
            result_str = "DRAW"

        try:
            with open(f"/app/results/{filename}", "w", encoding="utf-8") as f:
                f.write("=== TIGERs対戦試合結果 ===\n\n")
                f.write(f"試合時間: {self.match_duration:.1f}秒\n")
                f.write(
                    f"最終スコア: {self.yellow_name} {self.yellow_score} - {self.blue_score} {self.blue_name}\n\n"
                )
                f.write(f"結果: {result_str}\n")
                f.write(f"イベント数: {len(self.events)}\n")

                if self.events:
                    f.write("\n=== イベント履歴 ===\n")
                    f.writelines(
                        f"[{timestamp:6.1f}s] {event_desc}\n"
                        for timestamp, event_desc in self.events
                    )

                # ファウル・カード詳細
                f.write("\n=== ファウル・カード詳細 ===\n")
                for team_key, team_display in [
                    ("YELLOW", self.yellow_name),
                    ("BLUE", self.blue_name),
                ]:
                    yc = self.yellow_cards.get(team_key, 0)
                    fc = self.foul_counters.get(team_key, 0)
                    mb = self.max_allowed_bots.get(team_key, 0)
                    f.write(f"\n{team_key} ({team_display}):\n")
                    f.write(f"  イエローカード: {yc}枚\n")
                    f.write(f"  ファウルカウント: {fc}\n")
                    f.write(f"  最終出場可能台数: {mb}\n")
                    counts = self.foul_event_counts.get(team_key, {})
                    if counts:
                        f.write("  ファウル内訳:\n")
                        f.writelines(
                            f"    {ev_type}: {cnt}回\n"
                            for ev_type, cnt in sorted(
                                counts.items(), key=lambda x: -x[1]
                            )
                        )

                if self.foul_event_log:
                    f.write("\n=== ファウル時系列 ===\n")
                    f.writelines(
                        f"[{t:6.1f}s] {team}: {ev_type}\n"
                        for t, team, ev_type in self.foul_event_log
                    )

            print(f"✓ 試合結果を保存: {filename}")
            print(f"  結果: {result_str}")
        except Exception as e:  # noqa: BLE001
            print(f"✗ 結果保存エラー: {e}", file=sys.stderr)

    def run(self) -> int:
        """メイン実行フロー"""
        print("=== TIGERs対戦試合制御スクリプト (Protocol Buffers版) ===\n")

        # Game Controller WebSocket準備待機
        if not self.wait_for_gc_ready():
            return 1

        # レフェリーメッセージ受信確認（GCとUDP疎通確認）
        if not self.wait_for_referee():
            return 1

        # Visionデータ受信確認（シミュレータ起動確認）
        if not self.wait_for_vision():
            print("⚠ Visionデータ未受信（シミュレータが遅れている可能性あり、続行）")

        # 試合開始シーケンス
        if not self.start_match_sequence():
            return 1

        # 試合監視
        max_duration = int(os.environ.get("MATCH_MAX_DURATION", "840"))
        if not self.monitor_match(max_duration=max_duration):
            return 1

        # 結果保存
        self.save_results()

        self._sim_sock.close()
        print("\n✓ 試合制御完了")
        return 0


if __name__ == "__main__":
    controller = MatchController()
    sys.exit(controller.run())
