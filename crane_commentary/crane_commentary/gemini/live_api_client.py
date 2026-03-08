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
from typing import Callable, Optional, Dict, Any, List
from dataclasses import dataclass, field
import base64

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
    system_instruction: str = ""
    tools_config: List[Dict[str, Any]] = field(default_factory=list)


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
        self._on_disconnect_callback: Optional[Callable[[], None]] = None

        # Build WebSocket URL
        self._ws_url = (
            f"wss://generativelanguage.googleapis.com/ws/"
            f"google.ai.generativelanguage.v1alpha.GenerativeService.BidiGenerateContent"
            f"?key={self._config.api_key}"
        )

    def set_disconnect_callback(self, callback: Callable[[], None]) -> None:
        """Set callback invoked when WebSocket connection is lost."""
        self._on_disconnect_callback = callback

    async def connect(self) -> bool:
        """Establish WebSocket connection to Gemini API."""
        if self._connected:
            return True

        # Clean up any existing connection before reconnecting
        if self._receive_task and not self._receive_task.done():
            self._receive_task.cancel()
            try:
                await self._receive_task
            except asyncio.CancelledError:
                pass
            self._receive_task = None

        if self._ws:
            try:
                await self._ws.close()
            except Exception:
                pass
            self._ws = None

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
                    "system_instruction": {
                        "parts": [{"text": self._config.system_instruction}]
                    },
                }
            }

            if self._config.tools_config:
                setup_msg["setup"]["tools"] = [
                    {"function_declarations": self._config.tools_config}
                ]

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
            if self._on_disconnect_callback:
                self._on_disconnect_callback()
        except Exception as e:
            logger.error(f"Receive loop error: {e}")
            self._connected = False
            if self._on_disconnect_callback:
                self._on_disconnect_callback()

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
