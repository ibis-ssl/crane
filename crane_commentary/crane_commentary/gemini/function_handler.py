# Copyright (c) 2026 ibis-ssl
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Function Call Handler for Gemini Live API."""

import logging
from typing import Any, Dict

from crane_commentary.statler import WorldModelWriter

logger = logging.getLogger(__name__)


class FunctionHandler:
    """
    Handles function calls from Gemini Live API.

    Routes function calls to the appropriate data provider methods
    in WorldModelWriter.
    """

    def __init__(self, writer: WorldModelWriter):
        """Initialize handler with WorldModelWriter reference."""
        self._writer = writer

    def handle(self, name: str, args: Dict[str, Any]) -> Dict[str, Any]:
        """
        Handle a function call from Gemini.

        Args:
            name: Function name (e.g., "get_game_state")
            args: Function arguments as a dictionary

        Returns:
            Result dictionary to be sent back to Gemini
        """
        handlers = {
            "get_game_state": self._handle_get_game_state,
            "get_ball_trajectory": self._handle_get_ball_trajectory,
            "get_robot_status": self._handle_get_robot_status,
            "get_all_robots_summary": self._handle_get_all_robots_summary,
            "get_formation_analysis": self._handle_get_formation_analysis,
            "get_highlight_details": self._handle_get_highlight_details,
        }

        handler = handlers.get(name)
        if handler:
            try:
                return handler(args)
            except Exception as e:
                logger.error(f"Error handling {name}: {e}")
                return {"error": str(e)}
        else:
            logger.warning(f"Unknown function: {name}")
            return {"error": f"Unknown function: {name}"}

    def _handle_get_game_state(self, args: Dict[str, Any]) -> Dict[str, Any]:
        """Handle get_game_state function call."""
        return self._writer.get_game_state_data()

    def _handle_get_ball_trajectory(self, args: Dict[str, Any]) -> Dict[str, Any]:
        """Handle get_ball_trajectory function call."""
        seconds = args.get("seconds", 3.0)
        return self._writer.get_ball_trajectory_data(seconds)

    def _handle_get_robot_status(self, args: Dict[str, Any]) -> Dict[str, Any]:
        """Handle get_robot_status function call."""
        robot_id = args.get("robot_id")
        is_ours = args.get("is_ours")

        if robot_id is None or is_ours is None:
            return {"error": "robot_id and is_ours are required"}

        return self._writer.get_robot_status_data(int(robot_id), bool(is_ours))

    def _handle_get_all_robots_summary(self, args: Dict[str, Any]) -> Dict[str, Any]:
        """Handle get_all_robots_summary function call."""
        team = args.get("team", "all")
        return self._writer.get_all_robots_summary_data(team)

    def _handle_get_formation_analysis(self, args: Dict[str, Any]) -> Dict[str, Any]:
        """Handle get_formation_analysis function call."""
        focus = args.get("focus", "both")
        return self._writer.get_formation_analysis_data(focus)

    def _handle_get_highlight_details(self, args: Dict[str, Any]) -> Dict[str, Any]:
        """Handle get_highlight_details function call."""
        highlight_type = args.get("highlight_type", "any")
        count = args.get("count", 1)
        return self._writer.get_highlight_details_data(highlight_type, count)
