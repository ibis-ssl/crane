# Copyright (c) 2026 ibis-ssl
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""World Model Reader - Generates commentary from game state."""

import json
from enum import Enum
from typing import Dict, Any, Optional
from dataclasses import dataclass

from .world_model_writer import WorldModelWriter, GameContext


class CommentaryMode(Enum):
    """Commentary generation mode."""

    REFLEX = "reflex"  # Immediate reaction to events
    ANALYST = "analyst"  # In-depth analysis during pauses


@dataclass
class CommentaryRequest:
    """Request for commentary generation."""

    mode: CommentaryMode
    event_type: Optional[str] = None
    event_data: Optional[Dict[str, Any]] = None
    context: Optional[GameContext] = None
    priority: int = 1  # 0=low, 1=medium, 2=high


class WorldModelReader:
    """
    Statler Architecture - Reader Component.

    Generates commentary content based on events and game state.
    Supports two modes:
    - Reflex: Quick reactions to events
    - Analyst: Deep analysis during game pauses
    """

    def __init__(self, writer: WorldModelWriter):
        self._writer = writer
        self._current_mode = CommentaryMode.REFLEX
        self._last_commentary_time = 0.0
        self._silence_threshold = 5.0  # Seconds of silence before analyst mode

        # Event type to Japanese commentary hints
        self._reflex_templates = {
            "GOAL": "ゴール！",
            "FAST_SHOT": "強烈なシュート！",
            "SHOT": "シュート！",
            "SAVE": "ナイスセーブ！",
            "INTERCEPTION": "インターセプト！",
            "POSSESSION_CHANGE": "ボール奪取",
            "BALL_OUT": "ボールアウト",
            "SET_PLAY": "セットプレー",
        }

    def set_mode(self, mode: CommentaryMode) -> None:
        """Set the current commentary mode."""
        self._current_mode = mode

    def get_mode(self) -> CommentaryMode:
        """Get the current commentary mode."""
        return self._current_mode

    def generate_reflex(
        self, event_type: str, event_data: Dict[str, Any]
    ) -> CommentaryRequest:
        """
        Generate reflex-mode commentary for an event.

        Returns a CommentaryRequest with JSON payload for Gemini API.
        """
        context = self._writer.get_context()

        # Determine priority
        priority = 1
        if event_type in ["GOAL"]:
            priority = 2
        elif event_type in ["FAST_SHOT", "SAVE"]:
            priority = 2
        elif event_type in ["POSSESSION_CHANGE", "BALL_OUT"]:
            priority = 0

        return CommentaryRequest(
            mode=CommentaryMode.REFLEX,
            event_type=event_type,
            event_data=event_data,
            context=context,
            priority=priority,
        )

    def generate_analysis(self) -> Optional[CommentaryRequest]:
        """
        Generate analyst-mode commentary.

        Called during game pauses or dead air.
        Returns None if no analysis is needed.
        """
        context = self._writer.get_context()
        highlights = self._writer.get_pending_highlights()

        # Check if we have something to talk about
        if not highlights and not context.recent_events:
            return None

        # Build analysis payload
        payload = {
            "mode": "analyst",
            "analysis_type": self._determine_analysis_type(context, highlights),
            "context": {
                "score": {"ours": context.our_score, "theirs": context.their_score},
                "elapsed_minutes": context.elapsed_seconds / 60.0,
                "momentum": context.momentum,
            },
        }

        # Add highlight data if available
        if highlights:
            top_highlight = max(highlights, key=lambda h: h.score)
            payload["highlight"] = {
                "type": top_highlight.event_type,
                "data": top_highlight.data,
                "importance": top_highlight.score,
            }

        return CommentaryRequest(
            mode=CommentaryMode.ANALYST,
            event_data=payload,
            context=context,
            priority=1,
        )

    def to_gemini_json(self, request: CommentaryRequest) -> str:
        """Convert CommentaryRequest to JSON for Gemini API."""
        if request.mode == CommentaryMode.REFLEX:
            payload = {
                "mode": "reflex",
                "event": {
                    "type": request.event_type,
                    "hint": self._reflex_templates.get(request.event_type, ""),
                    "data": request.event_data or {},
                },
                "context": self._context_to_dict(request.context),
            }
        else:
            payload = {
                "mode": "analyst",
                "data": request.event_data or {},
                "context": self._context_to_dict(request.context),
            }

        return json.dumps(payload, ensure_ascii=False)

    def _context_to_dict(self, context: Optional[GameContext]) -> Dict[str, Any]:
        """Convert GameContext to dictionary."""
        if not context:
            return {}
        return {
            "score": {"ours": context.our_score, "theirs": context.their_score},
            "elapsed_minutes": context.elapsed_seconds / 60.0,
            "momentum": context.momentum,
            "recent_events": context.recent_events[-3:],
        }

    def _determine_analysis_type(self, context: GameContext, highlights: list) -> str:
        """Determine what type of analysis to provide."""
        if highlights:
            top = max(highlights, key=lambda h: h.score)
            if top.event_type == "GOAL":
                return "goal_replay"
            elif top.event_type in ["FAST_SHOT", "SHOT"]:
                return "shot_analysis"
            elif top.event_type == "SAVE":
                return "save_highlight"

        # Default analysis types
        if context.elapsed_seconds > 60 * 5:  # After 5 minutes
            return "game_summary"
        else:
            return "team_introduction"
