#!/usr/bin/env python3
# Copyright (c) 2026 ibis-ssl
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""SSL Commentary System ROS2 Node."""

import asyncio
import logging
import os
import threading
import time
import json
import yaml  # Requires PyYAML
from typing import Optional

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from crane_msgs.msg import PlaySituation, RonarEvent, WorldModel, RobotSelectResults, PositionCommands

from crane_commentary.data import (
    generate_initial_context,
    get_team_profile_from_data,
    get_team_reading_from_data,
)
from crane_commentary.statler import WorldModelWriter, WorldModelReader
from crane_commentary.statler.world_model_reader import CommentaryMode
from crane_commentary.gemini import GeminiLiveApiClient, FunctionHandler
from crane_commentary.gemini.live_api_client import GeminiConfig
from crane_commentary.audio import PcmAudioOutput
from crane_commentary.self_commentary import IntentTracker, CommentaryGenerator

logger = logging.getLogger(__name__)


class CommentaryNode(Node):
    """
    ROS2 Node for SSL Commentary System.

    Subscribes to RONAR events and generates real-time audio commentary
    using Gemini Multimodal Live API.
    """

    def __init__(self):
        super().__init__("commentary_node")

        # Declare parameters
        self.declare_parameter("gemini_api_key", "")
        self.declare_parameter("gemini_model", "gemini-2.0-flash-exp")
        self.declare_parameter("audio_sample_rate", 24000)
        self.declare_parameter("audio_device", "")
        self.declare_parameter("writer_update_rate", 1.0)
        self.declare_parameter("analyst_silence_threshold", 5.0)
        self.declare_parameter("mode", "reflex_analyst")

        # Get parameters
        api_key = self.get_parameter("gemini_api_key").value or os.environ.get(
            "GEMINI_API_KEY", ""
        )
        model = self.get_parameter("gemini_model").value
        sample_rate = self.get_parameter("audio_sample_rate").value
        audio_device = self.get_parameter("audio_device").value or None
        writer_rate = self.get_parameter("writer_update_rate").value
        self._analyst_threshold = self.get_parameter("analyst_silence_threshold").value
        self._mode = self.get_parameter("mode").value

        # Initialize Statler components
        self._writer = WorldModelWriter()
        self._reader = WorldModelReader(self._writer)

        # Initialize Function Handler for Gemini Function Calling
        self._function_handler = FunctionHandler(self._writer)

        # Load configuration files
        self._ssl_rules = {}
        self._team_profiles = {}
        self._load_config_files()

        # Load system instruction
        system_instruction = ""
        try:
            pkg_share = get_package_share_directory("crane_commentary")
            # Choose system instruction based on mode
            if self._mode == "self_commentary":
                instruction_filename = "system_instruction_self.md"
            else:
                instruction_filename = "system_instruction.md"

            instruction_path = os.path.join(
                pkg_share, "config", instruction_filename
            )
            if os.path.exists(instruction_path):
                with open(instruction_path, "r", encoding="utf-8") as f:
                    system_instruction = f.read()
                self.get_logger().info(
                    f"Loaded system instruction from {instruction_path} (mode: {self._mode})"
                )
            else:
                self.get_logger().warning(
                    f"System instruction file not found: {instruction_path}"
                )
        except Exception as e:
            self.get_logger().error(f"Failed to load system instruction: {e}")

        # Load tools configuration
        tools_config = []
        try:
            pkg_share = get_package_share_directory("crane_commentary")
            tools_path = os.path.join(pkg_share, "config", "function_declarations.json")
            if os.path.exists(tools_path):
                with open(tools_path, "r", encoding="utf-8") as f:
                    tools_config = json.load(f)
                self.get_logger().info(f"Loaded tools config from {tools_path}")
            else:
                self.get_logger().warning(f"Tools config file not found: {tools_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load tools config: {e}")

        # Initialize Gemini client
        gemini_config = GeminiConfig(
            api_key=api_key,
            model=model,
            sample_rate=sample_rate,
            system_instruction=system_instruction,
            tools_config=tools_config,
        )
        self._gemini_client = GeminiLiveApiClient(gemini_config)

        # Initialize audio output (required)
        self._audio_output = PcmAudioOutput(
            sample_rate=sample_rate,
            device=audio_device,
        )
        self.get_logger().info("Audio output initialized (sounddevice)")

        # Set up audio callback
        self._gemini_client.set_audio_callback(self._on_audio_received)

        # Set up function call handler for Gemini Function Calling
        self._gemini_client.set_function_call_handler(self._function_handler.handle)

        # Subscribers
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self._ronar_sub = self.create_subscription(
            RonarEvent,
            "/ronar_events",
            self._on_ronar_event,
            qos,
        )

        self._play_situation_sub = self.create_subscription(
            PlaySituation,
            "/play_situation",
            self._on_play_situation,
            qos,
        )

        # WorldModel subscription for Function Calling data
        self._world_model_sub = self.create_subscription(
            WorldModel,
            "/world_model",
            self._on_world_model,
            qos,
        )

        # Self-commentary mode initialization
        if self._mode == "self_commentary":
            self._intent_tracker = IntentTracker()
            self._commentary_generator = CommentaryGenerator(self._intent_tracker)

            # Subscribe to robot selection and control targets
            self._robot_select_sub = self.create_subscription(
                RobotSelectResults,
                "/robot_select_results",
                self._on_robot_select_results,
                qos,
            )
            self._control_targets_sub = self.create_subscription(
                PositionCommands,
                "/control_targets",
                self._on_control_targets,
                qos,
            )

            # Self-commentary timer (2 seconds interval)
            self._self_commentary_timer = self.create_timer(
                2.0,
                self._self_commentary_callback,
            )

            self.get_logger().info("Self-commentary mode enabled")

        # Timers
        self._writer_timer = self.create_timer(
            1.0 / writer_rate,
            self._writer_update_callback,
        )

        self._analyst_timer = self.create_timer(
            1.0,
            self._analyst_check_callback,
        )

        # State
        self._last_event_time = self.get_clock().now()
        self._connected = False
        self._event_loop: Optional[asyncio.AbstractEventLoop] = None
        self._loop_thread: Optional[threading.Thread] = None

        # Team information
        self._our_team_name: str = "ibis"
        self._their_team_name: Optional[str] = None
        self._initial_context_sent: bool = False

        # Event cooldown (prevent spam)
        self._last_commentary_time: dict[str, float] = {}
        self._event_cooldowns = {
            "POSSESSION_CHANGE": 3.0,  # 3秒間隔
            "SHOT": 2.0,
            "FAST_SHOT": 2.0,
            "GOAL": 5.0,
            "BALL_OUT": 3.0,
            "SET_PLAY": 5.0,
            "PASS": 2.0,
            # Play Switcher イベント
            "HALT": 3.0,
            "STOP": 3.0,
            "INPLAY_START": 2.0,
            "TIMEOUT": 10.0,
            "HALF_TIME": 10.0,
            "GAME_END": 10.0,
            # Autoref イベント
            "FOUL": 5.0,
        }

        self.get_logger().info("Commentary node initialized")

    def _load_config_files(self) -> None:
        """Load SSL rules and team profiles from YAML configuration files."""
        try:
            pkg_share = get_package_share_directory("crane_commentary")

            # Load SSL rules
            rules_path = os.path.join(pkg_share, "config", "ssl_rules.yaml")
            if os.path.exists(rules_path):
                with open(rules_path, "r", encoding="utf-8") as f:
                    self._ssl_rules = yaml.safe_load(f)
                self.get_logger().info(f"Loaded SSL rules from {rules_path}")
            else:
                self.get_logger().warning(f"SSL rules file not found: {rules_path}")

            # Load team profiles
            profiles_path = os.path.join(pkg_share, "config", "team_profiles.yaml")
            if os.path.exists(profiles_path):
                with open(profiles_path, "r", encoding="utf-8") as f:
                    self._team_profiles = yaml.safe_load(f)
                self.get_logger().info(f"Loaded team profiles from {profiles_path}")
            else:
                self.get_logger().warning(
                    f"Team profiles file not found: {profiles_path}"
                )

        except Exception as e:
            self.get_logger().error(f"Failed to load config files: {e}")

    def _run_event_loop(self, loop: asyncio.AbstractEventLoop) -> None:
        """Run asyncio event loop in background thread."""
        asyncio.set_event_loop(loop)
        loop.run_forever()

    def connect(self) -> bool:
        """Connect to Gemini API."""
        # Create new event loop for asyncio
        self._event_loop = asyncio.new_event_loop()

        # Start event loop in background thread
        self._loop_thread = threading.Thread(
            target=self._run_event_loop,
            args=(self._event_loop,),
            daemon=True,
        )
        self._loop_thread.start()

        # Connect to Gemini API
        future = asyncio.run_coroutine_threadsafe(
            self._gemini_client.connect(),
            self._event_loop,
        )

        try:
            success = future.result(timeout=10.0)
            if success:
                self._connected = True
                self._audio_output.start()
                self.get_logger().info("Connected to Gemini API")
                self.get_logger().info("Audio output started")
                self.get_logger().info(
                    "Waiting for team information from PlaySituation..."
                )
                # Initial context will be sent after receiving PlaySituation
            else:
                self.get_logger().error("Failed to connect to Gemini API")
            return success
        except Exception as e:
            self.get_logger().error(f"Connection error: {e}")
            return False

    def disconnect(self) -> None:
        """Disconnect from Gemini API."""
        if self._event_loop and self._connected:
            future = asyncio.run_coroutine_threadsafe(
                self._gemini_client.disconnect(),
                self._event_loop,
            )
            try:
                future.result(timeout=5.0)
            except Exception:
                pass

        self._audio_output.stop()
        self._connected = False

        # Stop event loop
        if self._event_loop:
            self._event_loop.call_soon_threadsafe(self._event_loop.stop)

        if self._loop_thread:
            self._loop_thread.join(timeout=2.0)

        self.get_logger().info("Disconnected from Gemini API")

    def _on_play_situation(self, msg: PlaySituation) -> None:
        """Handle incoming PlaySituation messages to get team names."""
        team_changed = False

        # Update team names
        if msg.our_team_info.name and msg.our_team_info.name != self._our_team_name:
            old_name = self._our_team_name
            self._our_team_name = msg.our_team_info.name
            self.get_logger().info(
                f"Our team name updated: {old_name} -> {self._our_team_name}"
            )
            team_changed = True

        if (
            msg.their_team_info.name
            and msg.their_team_info.name != self._their_team_name
        ):
            old_name = self._their_team_name
            self._their_team_name = msg.their_team_info.name
            self.get_logger().info(
                f"Their team name updated: {old_name} -> {self._their_team_name}"
            )
            team_changed = True

        # Send initial context when both team names are available
        if not self._initial_context_sent and self._their_team_name and self._connected:
            self.get_logger().info(
                f"Both team names received: {self._our_team_name} vs {self._their_team_name}"
            )
            self._send_initial_context()
            # Send greeting after initial context
            self._send_to_gemini("実況システム起動。RoboCup SSL の実況を開始します。")
        # If initial context already sent and team changed, send update
        elif self._initial_context_sent and team_changed and self._connected:
            self.get_logger().info("Sending team information update to Gemini")
            self._send_team_update()

    def _on_world_model(self, msg: WorldModel) -> None:
        """Handle incoming WorldModel messages for Function Calling data."""
        # Update WorldModelWriter with full world model data
        self._writer.update_from_world_model(msg)

    def _on_ronar_event(self, msg: RonarEvent) -> None:
        """Handle incoming RONAR events."""
        self._last_event_time = self.get_clock().now()

        # Map event type to string
        event_types = {
            RonarEvent.EVENT_POSSESSION_CHANGE: "POSSESSION_CHANGE",
            RonarEvent.EVENT_PASS: "PASS",
            RonarEvent.EVENT_SHOT: "SHOT",
            RonarEvent.EVENT_FAST_SHOT: "FAST_SHOT",
            RonarEvent.EVENT_GOAL: "GOAL",
            RonarEvent.EVENT_SAVE: "SAVE",
            RonarEvent.EVENT_INTERCEPTION: "INTERCEPTION",
            RonarEvent.EVENT_BALL_OUT: "BALL_OUT",
            RonarEvent.EVENT_SET_PLAY: "SET_PLAY",
            RonarEvent.EVENT_COLLISION: "COLLISION",
            # Play Switcher イベント
            RonarEvent.EVENT_HALT: "HALT",
            RonarEvent.EVENT_STOP: "STOP",
            RonarEvent.EVENT_INPLAY_START: "INPLAY_START",
            RonarEvent.EVENT_TIMEOUT: "TIMEOUT",
            RonarEvent.EVENT_HALF_TIME: "HALF_TIME",
            RonarEvent.EVENT_GAME_END: "GAME_END",
            # Autoref イベント
            RonarEvent.EVENT_FOUL: "FOUL",
        }
        event_type = event_types.get(msg.event_type, "UNKNOWN")

        # Build event data
        event_data = {
            "position": {"x": msg.position.x, "y": msg.position.y},
            "ball_speed": msg.ball_speed,
            "confidence": msg.confidence,
        }

        if msg.has_primary_robot:
            event_data["primary_robot"] = {
                "id": msg.primary_robot_id,
                "is_ours": msg.primary_robot_is_ours,
            }

        if msg.has_secondary_robot:
            event_data["secondary_robot"] = {
                "id": msg.secondary_robot_id,
                "is_ours": msg.secondary_robot_is_ours,
            }

        # Parse metadata_json and convert team names to readings
        if msg.metadata_json:
            try:
                metadata = json.loads(msg.metadata_json)

                # Convert team name to reading if exists
                if "team" in metadata:
                    team_key = metadata["team"]
                    metadata["team"] = get_team_reading_from_data(
                        team_key, self._team_profiles
                    )

                event_data["metadata"] = metadata
            except json.JSONDecodeError:
                self.get_logger().warning(
                    f"Failed to parse metadata_json: {msg.metadata_json}"
                )

        self.get_logger().info(f"Received event: {event_type}")

        # Add event to writer
        self._writer.add_event(event_type, event_data)

        # Check cooldown
        current_time = time.time()
        cooldown = self._event_cooldowns.get(event_type, 1.0)
        last_time = self._last_commentary_time.get(event_type, 0.0)

        if current_time - last_time < cooldown:
            self.get_logger().info(f"Skipping {event_type} (cooldown)")
            return

        # Generate reflex commentary
        if self._connected:
            self._reader.set_mode(CommentaryMode.REFLEX)
            request = self._reader.generate_reflex(event_type, event_data)

            # Send to Gemini API (high priority events only)
            if request.priority >= 1:
                json_payload = self._reader.to_gemini_json(request)
                self.get_logger().info(f"Sending reflex commentary for {event_type}")
                self._send_to_gemini(json_payload)
                self._last_commentary_time[event_type] = current_time

    def _writer_update_callback(self) -> None:
        """Periodic callback to update WorldModelWriter."""
        # TODO: Get actual game state from WorldModel subscription
        # For now, use placeholder values
        self._writer.update(
            play_situation=50,  # INPLAY
            our_score=0,
            their_score=0,
            elapsed_seconds=0.0,
        )

    def _analyst_check_callback(self) -> None:
        """Check if analyst mode should be activated."""
        if not self._connected:
            return

        # Calculate time since last event
        now = self.get_clock().now()
        silence_duration = (now - self._last_event_time).nanoseconds / 1e9

        # If silence exceeds threshold, switch to analyst mode
        if silence_duration > self._analyst_threshold:
            if self._reader.get_mode() != CommentaryMode.ANALYST:
                self._reader.set_mode(CommentaryMode.ANALYST)
                self.get_logger().info("Switching to analyst mode")

                # Generate analysis
                request = self._reader.generate_analysis()
                if request:
                    json_payload = self._reader.to_gemini_json(request)
                    self._send_to_gemini(json_payload)

    def _send_initial_context(self) -> None:
        """Send SSL rules and team info as initial context."""
        if self._initial_context_sent:
            return

        context = generate_initial_context(
            ssl_rules=self._ssl_rules,
            team_profiles=self._team_profiles,
            our_team_name=self._our_team_name,
            their_team_name=self._their_team_name,
        )

        self.get_logger().info("Sending initial context to Gemini")
        self._send_to_gemini(f"[SYSTEM CONTEXT]\n{context}")
        self._initial_context_sent = True

    def _send_team_update(self) -> None:
        """Send team information update to Gemini."""
        update = {
            "type": "team_update",
            "our_team": {
                "name": get_team_reading_from_data(
                    self._our_team_name, self._team_profiles
                ),
                "key": self._our_team_name,
                **get_team_profile_from_data(self._our_team_name, self._team_profiles),
            },
        }

        if self._their_team_name:
            update["their_team"] = {
                "name": get_team_reading_from_data(
                    self._their_team_name, self._team_profiles
                ),
                "key": self._their_team_name,
                **get_team_profile_from_data(
                    self._their_team_name, self._team_profiles
                ),
            }

        update_json = json.dumps(update, ensure_ascii=False, indent=2)
        self._send_to_gemini(f"[TEAM UPDATE]\n{update_json}")

    def _send_to_gemini(self, json_payload: str) -> None:
        """Send payload to Gemini API."""
        if not self._connected or not self._event_loop:
            return

        try:
            asyncio.run_coroutine_threadsafe(
                self._gemini_client.send_text(json_payload),
                self._event_loop,
            )
        except Exception as e:
            self.get_logger().error(f"Failed to send to Gemini: {e}")

    def _on_audio_received(self, pcm_data: bytes) -> None:
        """Handle received audio from Gemini API."""
        self._audio_output.play(pcm_data)

    # ========== Self-commentary mode callbacks ==========

    def _on_robot_select_results(self, msg: RobotSelectResults) -> None:
        """Handle incoming RobotSelectResults messages (self-commentary mode)."""
        if self._mode != "self_commentary":
            return

        self._intent_tracker.update_from_robot_select_results(msg)

    def _on_control_targets(self, msg: PositionCommands) -> None:
        """Handle incoming PositionCommands messages (self-commentary mode)."""
        if self._mode != "self_commentary":
            return

        self._intent_tracker.update_from_control_targets(msg)

    def _self_commentary_callback(self) -> None:
        """Periodic callback for self-commentary mode."""
        if not self._connected:
            return

        # Detect intent changes
        changes = self._intent_tracker.detect_changes()

        if not changes:
            return

        # Generate commentary JSON
        json_payload = self._commentary_generator.generate_commentary(changes)

        if json_payload:
            self.get_logger().info(f"Self-commentary: {len(changes)} changes detected")
            self._send_to_gemini(json_payload)


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    node = CommentaryNode()

    # Connect to Gemini API
    if not node.connect():
        node.get_logger().warning("Running without Gemini API connection")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.disconnect()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
