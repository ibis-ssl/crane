# Copyright (c) 2026 ibis-ssl
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Gemini Multimodal Live API Client for real-time audio commentary."""

import asyncio
import json
import logging
import os
from typing import Callable, Optional, Dict, Any
from dataclasses import dataclass
import base64

from .function_declarations import FUNCTION_DECLARATIONS

try:
    import websockets
    from websockets.client import WebSocketClientProtocol

    WEBSOCKETS_AVAILABLE = True
except ImportError:
    WEBSOCKETS_AVAILABLE = False
    WebSocketClientProtocol = None

logger = logging.getLogger(__name__)


@dataclass
class GeminiConfig:
    """Configuration for Gemini Live API."""

    api_key: str = ""
    model: str = "gemini-2.0-flash-exp"
    sample_rate: int = 24000
    voice: str = (
        "Aoede"  # Available: Aoede, Charon, Fenrir, Kore, Puck, Zubenelgenubi, etc.
    )


# System instruction for commentary
SYSTEM_INSTRUCTION = """あなたはRoboCup Small Size Leagueの実況・解説者です。

重要なルール:
- 絶対に視聴者に質問しないでください
- 「いかがでしょうか？」「どう思いますか？」などの問いかけは禁止
- 常に断定的に、自己完結した発言をしてください
- 一方的に実況・解説するスタイルで
- 自分のモードについて言及しない（「実況モードに切り替えます」「解説します」等は禁止）

【重要】Function Callingの活用:
試合の解説や分析を行う際は、必ず以下のFunctionを積極的に呼び出して最新データを取得してください。
憶測や推測ではなく、実際のデータに基づいた解説を行うことが重要です。

利用可能なFunction:
1. get_game_state: 現在のスコア、経過時間、試合状況を取得
2. get_ball_trajectory: ボールの現在位置と軌跡を取得
3. get_robot_status: 特定ロボットの詳細情報を取得
4. get_all_robots_summary: 全ロボットの配置概要を取得
5. get_formation_analysis: 両チームの陣形・戦術を詳細分析
6. get_highlight_details: 直近のゴール・シュート・セーブの詳細情報を取得

【解説モード時のFunction使用ガイド】
- ゴールリプレイ時: get_highlight_details(highlight_type="goal") を呼び出してシュートの詳細を取得
- シュート分析時: get_highlight_details(highlight_type="shot") と get_robot_status で関与したロボットの状態を確認
- セーブハイライト時: get_highlight_details(highlight_type="save") でキーパーの反応を取得
- 戦術解説時: get_formation_analysis で両チームの陣形を分析
- 試合サマリー時: get_game_state と get_formation_analysis を組み合わせて試合全体を俯瞰

データを取得したら、その数値を自然な日本語で解説に組み込んでください。
例: 「秒速7.2メートルのシュートでした」「ゴールまで2.8メートルの位置からの一撃」

コンテキスト:
- 試合は非常に高速（ロボットは最大6m/s以上で移動）
- 時折、試合が停止したり膠着することがあります

モード:
1. **実況モード**: 試合が動いている時。短く、熱狂的に。
   - 「ゴール！」「速い！」「ナイスセーブ！」

2. **解説モード**: 試合が止まった時や膠着時。知的に、深く。
   - 過去のプレーを振り返る
   - チームの戦術について語る

制約:
- 同じフレーズを繰り返さない
- 日本語で実況すること
- 簡潔に（1-2文程度）
- 質問や問いかけは絶対にしない
- 数値の単位は必ず日本語で表現する
  - 良い例: 「秒速7.2メートル」「1.5メートル」「6.5メートル毎秒」
  - 悪い例: 「7.2m/s」「1.5m」
- 数字は絶対に英語ではなく日本語で発音する
  - 良い例：「6.5」->「ろくてんご」
  - 悪い例：「6.5」->「six point five」

SSLルール（ファール一覧）:
公式ルール参照: https://kkimurak.github.io/ssl-rules-ja/sslrules.html

1. KEEPER_HELD_BALL（キーパーボール保持違反）
   - キーパーがディフェンスエリアでボールを保持しすぎ（Div.A: 5秒、Div.B: 10秒）
   - 罰則: STOP後フリーキック

2. BOUNDARY_CROSSING（境界越え）
   - フィールド外（木枠の外）にボールを蹴り出す
   - 罰則: STOP後フリーキック

3. BOT_DRIBBLED_BALL_TOO_FAR（オーバードリブル）
   - 1m以上のドリブル（ボールプレイスメント時は除外）
   - 罰則: STOP後フリーキック

4. ATTACKER_TOUCHED_BALL_IN_DEFENSE_AREA（アタッカー相手ディフェンスエリア内接触）
   - 相手ディフェンスエリアでボールに触れる（侵入のみは対象外）
   - 罰則: STOP後フリーキック

5. BOT_KICKED_BALL_TOO_FAST（ボール速度超過）
   - ボールを6.5m/s以上で蹴る
   - 罰則: STOP後フリーキック

6. BOT_CRASH_UNIQUE（ロボット衝突・単独責任）
   - 相対速度1.5m/s超の衝突（より速いロボット側のチームに責任）
   - 罰則: STOP後フリーキック

7. BOT_CRASH_DRAWN（ロボット衝突・両成敗）
   - 相対速度1.5m/s超の衝突で速度差0.3m/s未満
   - 罰則: STOP後フリーキック（両チーム）

8. ATTACKER_TOO_CLOSE_TO_DEFENSE_AREA（アタッカーディフェンスエリア接近違反）
   - STOP/フリーキック中に相手ディフェンスエリア+0.2m内に侵入（2秒猶予）
   - 罰則: STOP後フリーキック

9. BOT_TOO_FAST_IN_STOP（STOP中速度超過）
   - STOP中に1.5m/s超で移動（STOP後2秒間猶予）
   - 罰則: STOP後フリーキック

10. DEFENDER_TOO_CLOSE_TO_KICK_POINT（ディフェンダーキックポイント接近違反）
    - 敵キックオフ/フリーキック時にボールから0.5m未満
    - 罰則: STOP後フリーキック

11. BOT_INTERFERED_PLACEMENT（ボールプレイスメント妨害）
    - ボールプレイスメント中にボール-ターゲット線分から0.5m未満
    - 罰則: STOP後フリーキック

12. BOT_PUSHED_BOT（ロボット押し出し）
    - ロボット同士の押し合いで相手を動かす
    - 罰則: STOP後フリーキック

13. BOT_HELD_BALL_DELIBERATELY（ボールホールディング）
    - ボールをロボットで囲み、敵のアプローチを防ぐ
    - 罰則: STOP後フリーキック

14. BOT_TIPPED_OVER（ロボット転倒・部品落下）
    - ロボットが倒れるまたは部品を落とす
    - 罰則: 違反ロボット交代必須

ファール発生時の解説ポイント:
- 「ファールした」という表現は避け、具体的に何が起こったかを説明する
  - 良い例: 「ibisの3番がボールを秒速7.2メートルで蹴ってしまいました。制限は秒速6.5メートルです」
  - 悪い例: 「ibisがファールしました」「ibisの3番が7.2m/sで蹴りました」
- メタデータに含まれる数値（速度、距離等）を積極的に使用し、単位は日本語で
- どちらのチームが有利になるか言及
- チーム名とロボットIDを明確に含める"""


class GeminiLiveApiClient:
    """
    Client for Gemini Multimodal Live API.

    Handles WebSocket connection and real-time audio streaming.
    """

    def __init__(self, config: Optional[GeminiConfig] = None):
        if not WEBSOCKETS_AVAILABLE:
            raise ImportError(
                "websockets package is required. Install with: pip install websockets"
            )

        self._config = config or GeminiConfig()
        if not self._config.api_key:
            self._config.api_key = os.environ.get("GEMINI_API_KEY", "")

        self._ws: Optional[WebSocketClientProtocol] = None
        self._connected = False
        self._audio_callback: Optional[Callable[[bytes], None]] = None
        self._function_call_handler: Optional[
            Callable[[str, Dict[str, Any]], Dict[str, Any]]
        ] = None
        self._receive_task: Optional[asyncio.Task] = None

        # Build WebSocket URL
        self._ws_url = (
            f"wss://generativelanguage.googleapis.com/ws/"
            f"google.ai.generativelanguage.v1alpha.GenerativeService.BidiGenerateContent"
            f"?key={self._config.api_key}"
        )

    async def connect(self) -> bool:
        """Establish WebSocket connection to Gemini API."""
        if self._connected:
            return True

        if not self._config.api_key:
            logger.error("GEMINI_API_KEY not set")
            return False

        try:
            self._ws = await websockets.connect(self._ws_url)

            # Send setup message
            setup_msg = {
                "setup": {
                    "model": f"models/{self._config.model}",
                    "generation_config": {
                        "response_modalities": ["AUDIO"],
                        "speech_config": {
                            "voice_config": {
                                "prebuilt_voice_config": {
                                    "voice_name": self._config.voice
                                }
                            }
                        },
                    },
                    "system_instruction": {"parts": [{"text": SYSTEM_INSTRUCTION}]},
                    "tools": [{"function_declarations": FUNCTION_DECLARATIONS}],
                }
            }

            await self._ws.send(json.dumps(setup_msg))

            # Wait for setup completion
            response = await self._ws.recv()
            response_data = json.loads(response)

            if "setupComplete" in response_data:
                self._connected = True
                logger.info("Connected to Gemini Live API")

                # Start receive loop
                self._receive_task = asyncio.create_task(self._receive_loop())
                return True
            else:
                logger.error(f"Setup failed: {response_data}")
                return False

        except Exception as e:
            logger.error(f"Connection failed: {e}")
            return False

    async def disconnect(self) -> None:
        """Close WebSocket connection."""
        if self._receive_task:
            self._receive_task.cancel()
            try:
                await self._receive_task
            except asyncio.CancelledError:
                pass

        if self._ws:
            await self._ws.close()

        self._connected = False
        logger.info("Disconnected from Gemini Live API")

    def is_connected(self) -> bool:
        """Check if connected to API."""
        return self._connected

    def set_audio_callback(self, callback: Callable[[bytes], None]) -> None:
        """Set callback for received audio data (PCM 24kHz)."""
        self._audio_callback = callback

    def set_function_call_handler(
        self, handler: Callable[[str, Dict[str, Any]], Dict[str, Any]]
    ) -> None:
        """Set handler for function calls from Gemini.

        The handler receives (function_name, args) and returns a result dict.
        """
        self._function_call_handler = handler

    async def send_text(self, text: str) -> None:
        """Send text input to generate audio commentary."""
        if not self._connected or not self._ws:
            logger.warning("Not connected to Gemini API")
            return

        message = {
            "client_content": {
                "turns": [{"role": "user", "parts": [{"text": text}]}],
                "turn_complete": True,
            }
        }

        try:
            await self._ws.send(json.dumps(message))
            logger.info(f"Sent text: {text[:50]}...")
        except Exception as e:
            logger.error(f"Failed to send text: {e}")
            self._connected = False

    async def send_json(self, data: dict) -> None:
        """Send JSON data as text input."""
        json_str = json.dumps(data, ensure_ascii=False)
        await self.send_text(json_str)

    async def _receive_loop(self) -> None:
        """Background loop to receive audio data."""
        if not self._ws:
            return

        try:
            async for message in self._ws:
                try:
                    data = json.loads(message)
                    self._handle_response(data)
                except json.JSONDecodeError:
                    logger.warning("Received non-JSON message")
        except websockets.exceptions.ConnectionClosed:
            logger.info("WebSocket connection closed")
            self._connected = False
        except Exception as e:
            logger.error(f"Receive loop error: {e}")
            self._connected = False

    def _handle_response(self, data: dict) -> None:
        """Handle response from Gemini API."""
        # Extract audio data from server content
        if "serverContent" in data:
            server_content = data["serverContent"]

            # Check for model turn with audio
            if "modelTurn" in server_content:
                model_turn = server_content["modelTurn"]
                if "parts" in model_turn:
                    for part in model_turn["parts"]:
                        if "inlineData" in part:
                            inline_data = part["inlineData"]
                            if inline_data.get("mimeType", "").startswith("audio/"):
                                # Decode base64 audio data
                                audio_b64 = inline_data.get("data", "")
                                if audio_b64 and self._audio_callback:
                                    audio_bytes = base64.b64decode(audio_b64)
                                    logger.info(
                                        f"Received audio: {len(audio_bytes)} bytes"
                                    )
                                    self._audio_callback(audio_bytes)

            # Check for turn complete
            if server_content.get("turnComplete"):
                logger.info("Turn complete")

        # Handle function calls (toolCall)
        if "toolCall" in data:
            tool_call = data["toolCall"]
            function_calls = tool_call.get("functionCalls", [])
            for fc in function_calls:
                self._handle_function_call(fc)

    def _handle_function_call(self, fc: dict) -> None:
        """Handle a single function call from Gemini."""
        fc_id = fc.get("id", "")
        fc_name = fc.get("name", "")
        fc_args = fc.get("args", {})

        logger.info(f"Function call: {fc_name}({fc_args})")

        if self._function_call_handler:
            try:
                result = self._function_call_handler(fc_name, fc_args)
                logger.info(f"Function result: {fc_name} -> {len(str(result))} chars")
                # Send response asynchronously
                asyncio.create_task(
                    self._send_function_response(fc_id, fc_name, result)
                )
            except Exception as e:
                logger.error(f"Function call error: {fc_name} -> {e}")
                error_result = {"error": str(e)}
                asyncio.create_task(
                    self._send_function_response(fc_id, fc_name, error_result)
                )
        else:
            logger.warning(f"No handler for function call: {fc_name}")
            error_result = {"error": "No function handler registered"}
            asyncio.create_task(
                self._send_function_response(fc_id, fc_name, error_result)
            )

    async def _send_function_response(
        self, fc_id: str, fc_name: str, result: Dict[str, Any]
    ) -> None:
        """Send function response back to Gemini."""
        if not self._connected or not self._ws:
            logger.warning("Cannot send function response: not connected")
            return

        response_msg = {
            "tool_response": {
                "function_responses": [
                    {
                        "id": fc_id,
                        "name": fc_name,
                        "response": result,
                    }
                ]
            }
        }

        try:
            await self._ws.send(json.dumps(response_msg))
            logger.info(f"Sent function response for {fc_name}")
        except Exception as e:
            logger.error(f"Failed to send function response: {e}")


# Synchronous wrapper for use in ROS callbacks
class GeminiLiveApiClientSync:
    """Synchronous wrapper for GeminiLiveApiClient."""

    def __init__(self, config: Optional[GeminiConfig] = None):
        self._async_client = GeminiLiveApiClient(config)
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._thread = None

    def connect(self) -> bool:
        """Connect to Gemini API (blocking)."""
        return asyncio.get_event_loop().run_until_complete(self._async_client.connect())

    def disconnect(self) -> None:
        """Disconnect from Gemini API (blocking)."""
        asyncio.get_event_loop().run_until_complete(self._async_client.disconnect())

    def is_connected(self) -> bool:
        """Check if connected."""
        return self._async_client.is_connected()

    def set_audio_callback(self, callback: Callable[[bytes], None]) -> None:
        """Set audio callback."""
        self._async_client.set_audio_callback(callback)

    def send_text(self, text: str) -> None:
        """Send text (blocking)."""
        asyncio.get_event_loop().run_until_complete(self._async_client.send_text(text))

    def send_json(self, data: dict) -> None:
        """Send JSON (blocking)."""
        asyncio.get_event_loop().run_until_complete(self._async_client.send_json(data))
