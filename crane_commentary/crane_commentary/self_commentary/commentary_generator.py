# Copyright (c) 2026 ibis-ssl
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Commentary generation for self-commentary mode."""

import json
from typing import List, Dict, Any

from .intent_tracker import IntentChange, ChangeType, IntentTracker


class CommentaryGenerator:
    """Generates self-commentary JSON payloads from intent changes."""

    def __init__(self, intent_tracker: IntentTracker):
        self._tracker = intent_tracker

    def generate_commentary(self, changes: List[IntentChange], max_changes: int = 3) -> str:
        """Generate self-commentary JSON from intent changes.

        Args:
            changes: List of IntentChange objects
            max_changes: Maximum number of changes to include (default: 3)

        Returns:
            JSON string to send to Gemini
        """
        if not changes:
            return ""

        # Limit to top N changes (already sorted by priority)
        main_changes = changes[:max_changes]

        # Build payload
        payload = {
            "mode": "self_commentary",
            "perspective": "first_person",
            "changes": [self._change_to_dict(c) for c in main_changes],
            "team_summary": self._build_team_summary(),
        }

        return json.dumps(payload, ensure_ascii=False, indent=2)

    def _change_to_dict(self, change: IntentChange) -> Dict[str, Any]:
        """Convert IntentChange to dictionary.

        Args:
            change: IntentChange object

        Returns:
            Dictionary representation
        """
        return {
            "type": change.change_type.value,
            "robot_id": change.robot_id,
            "description": self._generate_description(change),
            "old_value": change.old_value,
            "new_value": change.new_value,
            "context": change.context,
        }

    def _generate_description(self, change: IntentChange) -> str:
        """Generate human-readable description for a change.

        Args:
            change: IntentChange object

        Returns:
            Japanese description string
        """
        robot_id = change.robot_id

        if change.change_type == ChangeType.TACTIC_CHANGED:
            return f"ロボット{robot_id}のタクティクスを{change.old_value}から{change.new_value}に切り替えました"

        elif change.change_type == ChangeType.SKILL_CHANGED:
            return f"ロボット{robot_id}のスキルを{change.old_value}から{change.new_value}に移行しました"

        elif change.change_type == ChangeType.STATE_CHANGED:
            skill = change.context.get("skill", "スキル")
            return f"ロボット{robot_id}の{skill}が{change.old_value}から{change.new_value}に遷移しました"

        elif change.change_type == ChangeType.SKILL_COMPLETED:
            return f"ロボット{robot_id}の{change.old_value}が成功しました"

        elif change.change_type == ChangeType.SKILL_FAILED:
            return f"ロボット{robot_id}の{change.old_value}が失敗しました"

        elif change.change_type == ChangeType.ROBOT_ASSIGNED:
            return f"ロボット{robot_id}を{change.new_value}に割り当てました"

        return f"ロボット{robot_id}の変化を検出しました"

    def _build_team_summary(self) -> Dict[str, Any]:
        """Build a summary of the team's current intent.

        Returns:
            Dictionary with team summary
        """
        intents = self._tracker.get_current_intents()

        # Group robots by tactic
        tactics_map: Dict[str, List[int]] = {}
        for robot_id, intent in intents.items():
            tactic = intent.tactic_name
            if tactic and tactic != "Unknown":
                if tactic not in tactics_map:
                    tactics_map[tactic] = []
                tactics_map[tactic].append(robot_id)

        # Build robot details
        robots_detail = {}
        for robot_id, intent in intents.items():
            robots_detail[str(robot_id)] = {
                "tactic": intent.tactic_name,
                "skill": intent.skill_name,
                "state": intent.skill_state,
            }

        return {
            "active_tactics": list(tactics_map.keys()),
            "robot_count": len(intents),
            "tactics_assignment": tactics_map,
            "robots": robots_detail,
        }
