# Copyright (c) 2026 ibis-ssl
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""World Model Writer - Maintains game narrative in background."""

import json
import math
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Optional, List, Dict, Any, Tuple
from datetime import datetime


@dataclass
class BallTrajectoryPoint:
    """Single point in ball trajectory history."""

    timestamp: float
    position: Tuple[float, float, float]  # x, y, z
    velocity: Tuple[float, float, float]  # vx, vy, vz


@dataclass
class RobotSnapshot:
    """Snapshot of a robot's state."""

    robot_id: int
    is_ours: bool
    position: Tuple[float, float, float]  # x, y, theta
    velocity: Tuple[float, float]  # linear_speed, angular_speed
    is_available: bool
    has_ball_contact: bool


@dataclass
class GameContext:
    """Current game context for commentary generation."""

    play_situation: int = 0
    our_score: int = 0
    their_score: int = 0
    elapsed_seconds: float = 0.0
    momentum: str = "NEUTRAL"  # OURS, THEIRS, NEUTRAL
    last_possession_team: Optional[str] = None
    recent_events: List[str] = field(default_factory=list)


@dataclass
class HighlightEvent:
    """Important event for replay/analysis."""

    event_type: str
    timestamp: datetime
    score: float  # Importance score (0-100)
    data: Dict[str, Any] = field(default_factory=dict)


class WorldModelWriter:
    """
    Statler Architecture - Writer Component.

    Maintains the game narrative and context in background.
    Updates at ~1Hz with game flow information.
    """

    def __init__(self):
        self._context = GameContext()
        self._highlights: List[HighlightEvent] = []
        self._narrative_cache: str = ""
        self._lock = threading.Lock()

        # Configuration
        self.max_highlights = 100
        self.max_recent_events = 10

        # Ball trajectory history (10 seconds at ~30Hz)
        self._ball_trajectory: deque[BallTrajectoryPoint] = deque(maxlen=300)

        # Robot snapshots
        self._robot_snapshots_ours: Dict[int, RobotSnapshot] = {}
        self._robot_snapshots_theirs: Dict[int, RobotSnapshot] = {}

        # Additional game state
        self._play_situation_name: str = "UNKNOWN"
        self._our_goalie_id: int = 0
        self._their_goalie_id: int = 0
        self._ball_possession_team: Optional[str] = None

        # Current ball state
        self._current_ball_pos: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._current_ball_vel: Tuple[float, float, float] = (0.0, 0.0, 0.0)

    def update(
        self,
        play_situation: int,
        our_score: int,
        their_score: int,
        elapsed_seconds: float,
    ) -> None:
        """Update game context (called periodically, ~1Hz)."""
        with self._lock:
            self._context.play_situation = play_situation
            self._context.our_score = our_score
            self._context.their_score = their_score
            self._context.elapsed_seconds = elapsed_seconds

            # Update momentum based on recent events
            self._update_momentum()

            # Rebuild narrative cache
            self._rebuild_narrative()

    def add_event(self, event_type: str, data: Dict[str, Any]) -> None:
        """Add a new event to recent history."""
        with self._lock:
            self._context.recent_events.append(event_type)
            if len(self._context.recent_events) > self.max_recent_events:
                self._context.recent_events.pop(0)

            # Check if this is a highlight-worthy event
            score = self._calculate_highlight_score(event_type, data)
            if score >= 50:
                highlight = HighlightEvent(
                    event_type=event_type,
                    timestamp=datetime.now(),
                    score=score,
                    data=data,
                )
                self._highlights.append(highlight)
                if len(self._highlights) > self.max_highlights:
                    # Remove lowest score highlight
                    self._highlights.sort(key=lambda h: h.score, reverse=True)
                    self._highlights.pop()

    def get_narrative(self) -> str:
        """Get current game narrative as JSON string."""
        with self._lock:
            return self._narrative_cache

    def get_context(self) -> GameContext:
        """Get current game context."""
        with self._lock:
            return GameContext(
                play_situation=self._context.play_situation,
                our_score=self._context.our_score,
                their_score=self._context.their_score,
                elapsed_seconds=self._context.elapsed_seconds,
                momentum=self._context.momentum,
                last_possession_team=self._context.last_possession_team,
                recent_events=self._context.recent_events.copy(),
            )

    def get_pending_highlights(self) -> List[HighlightEvent]:
        """Get highlights that haven't been commented on yet."""
        with self._lock:
            # Return highlights from the last 30 seconds
            now = datetime.now()
            recent = [
                h for h in self._highlights if (now - h.timestamp).total_seconds() < 30
            ]
            return recent

    def _update_momentum(self) -> None:
        """Update momentum based on recent events."""
        if not self._context.recent_events:
            self._context.momentum = "NEUTRAL"
            return

        # Count possession changes and shots in recent events
        our_actions = sum(
            1
            for e in self._context.recent_events[-5:]
            if e in ["SHOT", "FAST_SHOT", "GOAL"]
        )
        their_actions = sum(
            1 for e in self._context.recent_events[-5:] if e in ["SAVE", "INTERCEPTION"]
        )

        if our_actions > their_actions + 1:
            self._context.momentum = "OURS"
        elif their_actions > our_actions + 1:
            self._context.momentum = "THEIRS"
        else:
            self._context.momentum = "NEUTRAL"

    def _rebuild_narrative(self) -> None:
        """Rebuild the narrative cache."""
        narrative = {
            "game_state": {
                "situation": self._context.play_situation,
                "score": {
                    "ours": self._context.our_score,
                    "theirs": self._context.their_score,
                },
                "elapsed_minutes": self._context.elapsed_seconds / 60.0,
                "momentum": self._context.momentum,
            },
            "recent_flow": self._context.recent_events[-5:],
            "highlights_available": len([h for h in self._highlights if h.score >= 70]),
        }
        self._narrative_cache = json.dumps(narrative, ensure_ascii=False)

    def _calculate_highlight_score(
        self, event_type: str, data: Dict[str, Any]
    ) -> float:
        """Calculate importance score for an event."""
        scores = {
            "GOAL": 100,
            "FAST_SHOT": 80,
            "SAVE": 75,
            "SHOT": 60,
            "INTERCEPTION": 55,
            "PASS": 30,
            "POSSESSION_CHANGE": 20,
        }
        base_score = scores.get(event_type, 10)

        # Bonus for context
        if event_type == "GOAL":
            # Equalizer or go-ahead goal is more important
            if data.get("score_diff_after", 0) == 0:
                base_score += 10  # Equalizer
            elif data.get("score_diff_after", 0) == 1:
                base_score += 5  # Go-ahead

        if event_type in ["SHOT", "FAST_SHOT"]:
            # High speed shots are more exciting
            speed = data.get("ball_speed", 0)
            if speed > 8.0:
                base_score += 15
            elif speed > 6.0:
                base_score += 5

        return min(base_score, 100)

    # ========== Function Calling Data Providers ==========

    def update_from_world_model(self, world_model_msg: Any) -> None:
        """Update all data from WorldModel ROS message.

        This method extracts robot positions, ball state, and game info
        from the WorldModel message to provide data for Function Calling.
        """
        with self._lock:
            current_time = time.time()

            # Update play situation
            if hasattr(world_model_msg, "play_situation"):
                ps = world_model_msg.play_situation
                # command is NamedInt type, need to access .value
                command_value = (
                    ps.command.value if hasattr(ps.command, "value") else ps.command
                )
                self._play_situation_name = self._situation_to_name(command_value)
                if hasattr(ps, "our_team_info"):
                    self._context.our_score = ps.our_team_info.score
                if hasattr(ps, "their_team_info"):
                    self._context.their_score = ps.their_team_info.score

            # Update goalie IDs
            if hasattr(world_model_msg, "our_goalie_id"):
                self._our_goalie_id = world_model_msg.our_goalie_id
            if hasattr(world_model_msg, "their_goalie_id"):
                self._their_goalie_id = world_model_msg.their_goalie_id

            # Update ball state
            if hasattr(world_model_msg, "ball_info"):
                ball = world_model_msg.ball_info
                pos = (
                    ball.position.x,
                    ball.position.y,
                    getattr(ball.position, "z", 0.0),
                )
                vel = (
                    ball.velocity.x,
                    ball.velocity.y,
                    getattr(ball.velocity, "z", 0.0),
                )
                self._current_ball_pos = pos
                self._current_ball_vel = vel

                # Add to trajectory
                self._ball_trajectory.append(
                    BallTrajectoryPoint(
                        timestamp=current_time,
                        position=pos,
                        velocity=vel,
                    )
                )

            # Update our robots
            if hasattr(world_model_msg, "robot_info_ours"):
                for robot in world_model_msg.robot_info_ours:
                    self._robot_snapshots_ours[robot.id] = RobotSnapshot(
                        robot_id=robot.id,
                        is_ours=True,
                        position=(robot.pose.x, robot.pose.y, robot.pose.theta),
                        velocity=(
                            math.hypot(robot.velocity.x, robot.velocity.y),
                            robot.velocity.theta,
                        ),
                        is_available=(
                            robot.available_vision
                            or robot.available_feedback
                            or robot.available_tracker
                        ),
                        has_ball_contact=getattr(robot, "ball_contact", False),
                    )

            # Update their robots
            if hasattr(world_model_msg, "robot_info_theirs"):
                for robot in world_model_msg.robot_info_theirs:
                    self._robot_snapshots_theirs[robot.id] = RobotSnapshot(
                        robot_id=robot.id,
                        is_ours=False,
                        position=(robot.pose.x, robot.pose.y, robot.pose.theta),
                        velocity=(
                            math.hypot(robot.velocity.x, robot.velocity.y),
                            robot.velocity.theta,
                        ),
                        is_available=(
                            robot.available_vision
                            or robot.available_feedback
                            or robot.available_tracker
                        ),
                        has_ball_contact=False,  # We don't track opponent ball contact
                    )

            # Determine ball possession
            self._ball_possession_team = self._determine_possession()

    def get_game_state_data(self) -> Dict[str, Any]:
        """Get game state data for get_game_state function."""
        with self._lock:
            return {
                "score": {
                    "ours": self._context.our_score,
                    "theirs": self._context.their_score,
                },
                "elapsed_minutes": round(self._context.elapsed_seconds / 60.0, 1),
                "play_situation": self._play_situation_name,
                "play_situation_detail": self._get_play_situation_detail(),
                "momentum": self._context.momentum,
                "recent_events": self._context.recent_events[-5:],
                "highlights_count": len([h for h in self._highlights if h.score >= 70]),
            }

    def get_ball_trajectory_data(self, seconds: float = 3.0) -> Dict[str, Any]:
        """Get ball trajectory data for get_ball_trajectory function."""
        seconds = min(seconds, 10.0)  # Max 10 seconds
        with self._lock:
            current_time = time.time()
            cutoff_time = current_time - seconds

            # Current ball state
            speed = math.hypot(
                self._current_ball_vel[0],
                self._current_ball_vel[1],
            )
            current = {
                "position": {
                    "x": round(self._current_ball_pos[0], 2),
                    "y": round(self._current_ball_pos[1], 2),
                    "z": round(self._current_ball_pos[2], 2),
                },
                "velocity": {
                    "x": round(self._current_ball_vel[0], 2),
                    "y": round(self._current_ball_vel[1], 2),
                },
                "speed_mps": round(speed, 2),
                "state": self._determine_ball_state(speed),
            }

            # Trajectory (max 30 points)
            trajectory = []
            for pt in self._ball_trajectory:
                if pt.timestamp >= cutoff_time:
                    trajectory.append(
                        {
                            "time_offset_sec": round(pt.timestamp - current_time, 2),
                            "position": {
                                "x": round(pt.position[0], 2),
                                "y": round(pt.position[1], 2),
                            },
                        }
                    )
            # Limit to 30 points
            if len(trajectory) > 30:
                step = len(trajectory) // 30
                trajectory = trajectory[::step][:30]

            return {
                "current": current,
                "trajectory": trajectory,
                "summary": self._generate_ball_summary(),
            }

    def get_robot_status_data(self, robot_id: int, is_ours: bool) -> Dict[str, Any]:
        """Get robot status data for get_robot_status function."""
        with self._lock:
            snapshots = (
                self._robot_snapshots_ours if is_ours else self._robot_snapshots_theirs
            )
            robot = snapshots.get(robot_id)

            if not robot:
                return {"error": f"Robot {robot_id} not found"}

            goalie_id = self._our_goalie_id if is_ours else self._their_goalie_id

            return {
                "robot_id": robot_id,
                "is_ours": is_ours,
                "position": {
                    "x": round(robot.position[0], 2),
                    "y": round(robot.position[1], 2),
                    "theta": round(robot.position[2], 2),
                },
                "velocity": {
                    "linear_mps": round(robot.velocity[0], 2),
                    "angular_rps": round(robot.velocity[1], 2),
                },
                "ball_contact": {"has_contact": robot.has_ball_contact},
                "is_goalkeeper": robot_id == goalie_id,
                "is_available": robot.is_available,
                "role_hint": self._infer_robot_role(robot_id, is_ours),
            }

    def get_all_robots_summary_data(self, team: str = "all") -> Dict[str, Any]:
        """Get all robots summary data for get_all_robots_summary function."""
        with self._lock:
            result: Dict[str, Any] = {}

            if team in ("ours", "all"):
                result["ours"] = self._build_team_summary(True)
            if team in ("theirs", "all"):
                result["theirs"] = self._build_team_summary(False)

            result["ball_possession"] = self._ball_possession_team
            return result

    # ========== Helper Methods ==========

    def _situation_to_name(self, situation: int) -> str:
        """Convert play situation code to name."""
        # Based on PlaySituation.msg constants
        situations = {
            0: "HALT",
            1: "STOP",
            11: "OUR_KICKOFF_PREP",
            12: "OUR_KICKOFF_START",
            21: "THEIR_KICKOFF_PREP",
            22: "THEIR_KICKOFF_START",
            50: "INPLAY",
            100: "HALF_TIME",
            101: "POST_GAME",
        }
        return situations.get(situation, f"UNKNOWN_{situation}")

    def _get_play_situation_detail(self) -> str:
        """Get human-readable play situation detail."""
        details = {
            "HALT": "試合停止中",
            "STOP": "ボール停止待ち",
            "OUR_KICKOFF_PREP": "自チーム・キックオフ準備",
            "OUR_KICKOFF_START": "自チーム・キックオフ開始",
            "THEIR_KICKOFF_PREP": "相手チーム・キックオフ準備",
            "THEIR_KICKOFF_START": "相手チーム・キックオフ開始",
            "INPLAY": "試合進行中",
            "HALF_TIME": "ハーフタイム",
            "POST_GAME": "試合終了",
        }
        return details.get(self._play_situation_name, "不明")

    def _determine_ball_state(self, speed: float) -> str:
        """Determine ball state based on speed."""
        if speed < 0.1:
            return "STOPPED"
        elif speed < 3.0:
            return "ROLLING_SLOW"
        elif speed < 6.0:
            return "ROLLING_FAST"
        else:
            return "FAST_MOVING"

    def _generate_ball_summary(self) -> str:
        """Generate a summary of ball movement."""
        if not self._ball_trajectory:
            return "ボール情報なし"

        speed = math.hypot(self._current_ball_vel[0], self._current_ball_vel[1])
        x, y = self._current_ball_pos[0], self._current_ball_pos[1]

        # Determine field zone
        if x < -3.0:
            zone = "自陣深く"
        elif x < 0:
            zone = "自陣"
        elif x < 3.0:
            zone = "相手陣"
        else:
            zone = "相手陣深く"

        if y > 2.0:
            side = "左サイド"
        elif y < -2.0:
            side = "右サイド"
        else:
            side = "中央"

        if speed < 0.1:
            return f"ボールは{zone}{side}で静止"
        elif speed < 3.0:
            return f"ボールは{zone}{side}をゆっくり移動中"
        else:
            return f"ボールは{zone}{side}を高速で移動中（{speed:.1f}m/s）"

    def _determine_possession(self) -> Optional[str]:
        """Determine which team has ball possession."""
        # Check if any of our robots has ball contact
        for robot in self._robot_snapshots_ours.values():
            if robot.has_ball_contact:
                return "ours"

        # Simple proximity check
        ball_pos = self._current_ball_pos[:2]
        min_our_dist = float("inf")
        min_their_dist = float("inf")

        for robot in self._robot_snapshots_ours.values():
            if robot.is_available:
                dist = math.hypot(
                    robot.position[0] - ball_pos[0],
                    robot.position[1] - ball_pos[1],
                )
                min_our_dist = min(min_our_dist, dist)

        for robot in self._robot_snapshots_theirs.values():
            if robot.is_available:
                dist = math.hypot(
                    robot.position[0] - ball_pos[0],
                    robot.position[1] - ball_pos[1],
                )
                min_their_dist = min(min_their_dist, dist)

        if min_our_dist < 0.3:
            return "ours"
        elif min_their_dist < 0.3:
            return "theirs"
        elif min_our_dist < min_their_dist - 0.5:
            return "ours"
        elif min_their_dist < min_our_dist - 0.5:
            return "theirs"
        return None

    def _infer_robot_role(self, robot_id: int, is_ours: bool) -> str:
        """Infer robot role based on position."""
        snapshots = (
            self._robot_snapshots_ours if is_ours else self._robot_snapshots_theirs
        )
        robot = snapshots.get(robot_id)
        if not robot:
            return "不明"

        goalie_id = self._our_goalie_id if is_ours else self._their_goalie_id
        if robot_id == goalie_id:
            return "ゴールキーパー"

        x = robot.position[0]
        # Adjust based on which half we're on (assuming positive = opponent's goal)
        if x < -4.0:
            return "守備（ゴール前）"
        elif x < -2.0:
            return "守備"
        elif x < 2.0:
            return "中盤"
        elif x < 4.0:
            return "攻撃"
        else:
            return "攻撃（ゴール前）"

    def _build_team_summary(self, is_ours: bool) -> Dict[str, Any]:
        """Build summary for a team."""
        snapshots = (
            self._robot_snapshots_ours if is_ours else self._robot_snapshots_theirs
        )
        goalie_id = self._our_goalie_id if is_ours else self._their_goalie_id

        active_robots = [r for r in snapshots.values() if r.is_available]
        robots_info = []

        for robot in active_robots:
            role = self._infer_robot_role(robot.robot_id, is_ours)
            zone = self._get_position_zone(robot.position[0])
            robots_info.append(
                {
                    "id": robot.robot_id,
                    "role": role,
                    "position_zone": zone,
                }
            )

        # Determine formation
        formation = self._determine_formation(active_robots, goalie_id)

        return {
            "active_count": len(active_robots),
            "goalkeeper_id": goalie_id,
            "robots": robots_info,
            "formation_summary": formation,
        }

    def _get_position_zone(self, x: float) -> str:
        """Get position zone name based on x coordinate."""
        if x < -4.0:
            return "goal_area"
        elif x < -2.0:
            return "defense"
        elif x < 2.0:
            return "midfield"
        elif x < 4.0:
            return "attack"
        else:
            return "opponent_goal_area"

    def _determine_formation(self, robots: List[RobotSnapshot], goalie_id: int) -> str:
        """Determine formation based on robot positions."""
        # Count robots in each zone (excluding goalkeeper)
        zones = {"defense": 0, "midfield": 0, "attack": 0}

        for robot in robots:
            if robot.robot_id == goalie_id:
                continue
            x = robot.position[0]
            if x < -2.0:
                zones["defense"] += 1
            elif x < 2.0:
                zones["midfield"] += 1
            else:
                zones["attack"] += 1

        d, m, a = zones["defense"], zones["midfield"], zones["attack"]

        if d >= 3:
            return f"{d}-{m}-{a}（守備的布陣）"
        elif a >= 3:
            return f"{d}-{m}-{a}（攻撃的布陣）"
        else:
            return f"{d}-{m}-{a}（バランス型）"
