# Copyright (c) 2026 ibis-ssl
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Intent tracking for self-commentary mode."""

from dataclasses import dataclass, field
from typing import Dict, List, Optional
from enum import Enum


class ChangeType(Enum):
    """Type of intent change."""

    TACTIC_CHANGED = "tactic_changed"
    SKILL_CHANGED = "skill_changed"
    STATE_CHANGED = "state_changed"
    ROBOT_ASSIGNED = "robot_assigned"
    SKILL_COMPLETED = "skill_completed"
    SKILL_FAILED = "skill_failed"


@dataclass
class RobotIntent:
    """Represents a robot's current intent."""

    robot_id: int
    tactic_name: str = ""
    skill_name: str = ""  # state_factors[0].name
    skill_state: str = ""  # state_factors[0].value
    nested_skills: List[Dict[str, str]] = field(
        default_factory=list
    )  # state_factors[1:]
    planner_name: str = ""

    def __eq__(self, other: object) -> bool:
        """Compare two RobotIntent objects."""
        if not isinstance(other, RobotIntent):
            return NotImplemented
        return (
            self.robot_id == other.robot_id
            and self.tactic_name == other.tactic_name
            and self.skill_name == other.skill_name
            and self.skill_state == other.skill_state
        )


@dataclass
class IntentChange:
    """Represents a change in robot intent."""

    change_type: ChangeType
    robot_id: int
    old_value: str
    new_value: str
    context: Dict[str, any] = field(default_factory=dict)

    def priority(self) -> int:
        """Get priority of this change (higher = more important)."""
        priority_map = {
            ChangeType.SKILL_COMPLETED: 3,
            ChangeType.SKILL_FAILED: 3,
            ChangeType.TACTIC_CHANGED: 2,
            ChangeType.SKILL_CHANGED: 2,
            ChangeType.ROBOT_ASSIGNED: 2,
            ChangeType.STATE_CHANGED: 1,
        }
        return priority_map.get(self.change_type, 0)


# 重要スキルリスト（状態遷移も追跡する）
IMPORTANT_SKILLS = {
    "Attacker",
    "Goalie",
    "Receiver",
    "Marker",
    "Penalty",
    "SimpleAttacker",
    "DefenseWall",
}


class IntentTracker:
    """Tracks robot intents and detects changes."""

    def __init__(self):
        # Current intent state (robot_id -> RobotIntent)
        self._current_intents: Dict[int, RobotIntent] = {}
        # Previous intent state
        self._previous_intents: Dict[int, RobotIntent] = {}
        # Tactic mapping (robot_id -> tactic_name)
        self._robot_to_tactic: Dict[int, str] = {}

    def update_from_robot_select_results(self, msg: any) -> None:
        """Update tactic information from RobotSelectResults message.

        Args:
            msg: RobotSelectResults message
        """
        # Clear old mapping
        self._robot_to_tactic.clear()

        # Build new mapping
        for result in msg.results:
            tactic_name = result.name
            for robot_id in result.selected_robots:
                self._robot_to_tactic[robot_id] = tactic_name

    def update_from_control_targets(self, msg: any) -> None:
        """Update intent information from PositionCommands message.

        Args:
            msg: PositionCommands message
        """
        # Store previous state
        self._previous_intents = self._current_intents.copy()

        # Build new current state
        new_intents = {}
        for cmd in msg.robot_commands:
            robot_id = cmd.robot_id
            tactic_name = self._robot_to_tactic.get(robot_id, "Unknown")

            # Extract skill and state from state_factors
            skill_name = ""
            skill_state = ""
            nested_skills = []

            if hasattr(cmd, "state_factors") and cmd.state_factors:
                # First element is the top-level skill
                if len(cmd.state_factors) > 0:
                    skill_name = cmd.state_factors[0].name
                    skill_state = cmd.state_factors[0].value

                # Remaining elements are nested skills
                for i in range(1, len(cmd.state_factors)):
                    nested_skills.append(
                        {
                            "name": cmd.state_factors[i].name,
                            "value": cmd.state_factors[i].value,
                        }
                    )

            planner_name = getattr(cmd, "planner_name", "")

            new_intents[robot_id] = RobotIntent(
                robot_id=robot_id,
                tactic_name=tactic_name,
                skill_name=skill_name,
                skill_state=skill_state,
                nested_skills=nested_skills,
                planner_name=planner_name,
            )

        self._current_intents = new_intents

    def detect_changes(self) -> List[IntentChange]:
        """Detect changes between previous and current intents.

        Returns:
            List of IntentChange objects, sorted by priority
        """
        changes = []

        # No previous state -> initial assignment
        if not self._previous_intents:
            for robot_id, intent in self._current_intents.items():
                if intent.tactic_name and intent.tactic_name != "Unknown":
                    changes.append(
                        IntentChange(
                            change_type=ChangeType.ROBOT_ASSIGNED,
                            robot_id=robot_id,
                            old_value="",
                            new_value=intent.tactic_name,
                            context={
                                "skill": intent.skill_name,
                                "state": intent.skill_state,
                            },
                        )
                    )
            return sorted(changes, key=lambda c: c.priority(), reverse=True)

        # Compare with previous state
        for robot_id, current in self._current_intents.items():
            previous = self._previous_intents.get(robot_id)

            # New robot assignment
            if not previous:
                if current.tactic_name and current.tactic_name != "Unknown":
                    changes.append(
                        IntentChange(
                            change_type=ChangeType.ROBOT_ASSIGNED,
                            robot_id=robot_id,
                            old_value="",
                            new_value=current.tactic_name,
                            context={
                                "skill": current.skill_name,
                                "state": current.skill_state,
                            },
                        )
                    )
                continue

            # Tactic change
            if previous.tactic_name != current.tactic_name:
                changes.append(
                    IntentChange(
                        change_type=ChangeType.TACTIC_CHANGED,
                        robot_id=robot_id,
                        old_value=previous.tactic_name,
                        new_value=current.tactic_name,
                        context={
                            "skill": current.skill_name,
                            "state": current.skill_state,
                        },
                    )
                )

            # Skill completion/failure detection
            if previous.skill_state == "RUNNING" and current.skill_state == "SUCCESS":
                changes.append(
                    IntentChange(
                        change_type=ChangeType.SKILL_COMPLETED,
                        robot_id=robot_id,
                        old_value=previous.skill_name,
                        new_value=current.skill_name,
                        context={
                            "skill": current.skill_name,
                            "state": current.skill_state,
                        },
                    )
                )
            elif previous.skill_state == "RUNNING" and current.skill_state == "FAILURE":
                changes.append(
                    IntentChange(
                        change_type=ChangeType.SKILL_FAILED,
                        robot_id=robot_id,
                        old_value=previous.skill_name,
                        new_value=current.skill_name,
                        context={
                            "skill": current.skill_name,
                            "state": current.skill_state,
                        },
                    )
                )

            # Skill change
            if previous.skill_name != current.skill_name:
                changes.append(
                    IntentChange(
                        change_type=ChangeType.SKILL_CHANGED,
                        robot_id=robot_id,
                        old_value=previous.skill_name,
                        new_value=current.skill_name,
                        context={
                            "old_state": previous.skill_state,
                            "new_state": current.skill_state,
                        },
                    )
                )

            # State change (only for important skills)
            elif (
                previous.skill_state != current.skill_state
                and current.skill_name in IMPORTANT_SKILLS
            ):
                changes.append(
                    IntentChange(
                        change_type=ChangeType.STATE_CHANGED,
                        robot_id=robot_id,
                        old_value=previous.skill_state,
                        new_value=current.skill_state,
                        context={
                            "skill": current.skill_name,
                        },
                    )
                )

        # Sort by priority (highest first)
        return sorted(changes, key=lambda c: c.priority(), reverse=True)

    def get_current_intents(self) -> Dict[int, RobotIntent]:
        """Get current robot intents.

        Returns:
            Dictionary of robot_id -> RobotIntent
        """
        return self._current_intents.copy()

    def get_intent(self, robot_id: int) -> Optional[RobotIntent]:
        """Get intent for a specific robot.

        Args:
            robot_id: Robot ID

        Returns:
            RobotIntent or None if not found
        """
        return self._current_intents.get(robot_id)
