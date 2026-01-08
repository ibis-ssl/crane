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

    def get_formation_analysis_data(self, focus: str = "both") -> Dict[str, Any]:
        """Get formation analysis data for get_formation_analysis function."""
        with self._lock:
            result: Dict[str, Any] = {}

            # Analyze both teams
            if focus in ("offensive", "both"):
                result["ours"] = self._analyze_team_formation(True)
            if focus in ("defensive", "both"):
                result["theirs"] = self._analyze_team_formation(False)

            # Tactical situation
            result["tactical_situation"] = self._analyze_tactical_situation()

            return result

    def get_highlight_details_data(
        self, highlight_type: str = "any", count: int = 1
    ) -> Dict[str, Any]:
        """Get highlight details data for get_highlight_details function."""
        count = min(count, 5)  # Max 5 highlights

        with self._lock:
            # Filter highlights by type
            type_mapping = {
                "goal": ["GOAL"],
                "shot": ["SHOT", "FAST_SHOT"],
                "save": ["SAVE"],
                "any": ["GOAL", "SHOT", "FAST_SHOT", "SAVE"],
            }
            allowed_types = type_mapping.get(highlight_type, type_mapping["any"])

            filtered = [
                h for h in self._highlights if h.event_type in allowed_types
            ]

            # Sort by timestamp (most recent first) and take top N
            filtered.sort(key=lambda h: h.timestamp, reverse=True)
            selected = filtered[:count]

            now = datetime.now()
            highlights_data = []

            for h in selected:
                highlight_info = self._build_highlight_detail(h, now)
                highlights_data.append(highlight_info)

            return {"highlights": highlights_data, "total_available": len(filtered)}

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

    def _analyze_team_formation(self, is_ours: bool) -> Dict[str, Any]:
        """Analyze a single team's formation."""
        snapshots = (
            self._robot_snapshots_ours if is_ours else self._robot_snapshots_theirs
        )
        goalie_id = self._our_goalie_id if is_ours else self._their_goalie_id

        active_robots = [r for r in snapshots.values() if r.is_available]

        # Calculate formation string
        formation = self._determine_formation(active_robots, goalie_id)

        # Determine pattern (spread/compact/balanced)
        pattern = self._determine_pattern(active_robots)

        # Determine pressure zone
        pressure_zone = self._determine_pressure_zone(active_robots, is_ours)

        # Count robots near ball
        robots_near_ball = self._count_robots_near_ball(active_robots, threshold=1.5)

        # Goalkeeper position analysis
        goalkeeper = snapshots.get(goalie_id)
        gk_info = {
            "x": round(goalkeeper.position[0], 2) if goalkeeper else 0.0,
            "y": round(goalkeeper.position[1], 2) if goalkeeper else 0.0,
            "advanced": (
                goalkeeper.position[0] > -4.5
                if (is_ours and goalkeeper)
                else False
            ),
        }

        return {
            "formation": formation,
            "pattern": pattern,
            "pressure_zone": pressure_zone,
            "robots_near_ball": robots_near_ball,
            "goalkeeper_position": gk_info,
        }

    def _determine_pattern(self, robots: List[RobotSnapshot]) -> str:
        """Determine team's positioning pattern (spread/compact/balanced)."""
        if len(robots) < 2:
            return "unknown"

        # Calculate average distance between robots
        positions = [(r.position[0], r.position[1]) for r in robots]
        total_dist = 0.0
        count = 0
        for i, p1 in enumerate(positions):
            for p2 in positions[i + 1 :]:
                dist = math.hypot(p1[0] - p2[0], p1[1] - p2[1])
                total_dist += dist
                count += 1

        avg_dist = total_dist / count if count > 0 else 0

        if avg_dist < 2.0:
            return "compact"
        elif avg_dist > 4.0:
            return "spread"
        else:
            return "balanced"

    def _determine_pressure_zone(
        self, robots: List[RobotSnapshot], is_ours: bool
    ) -> str:
        """Determine where the team is applying pressure."""
        if not robots:
            return "unknown"

        avg_x = sum(r.position[0] for r in robots) / len(robots)

        # Adjust for team side (assuming positive x = opponent's goal for "ours")
        if is_ours:
            if avg_x > 2.0:
                return "opponent_half"
            elif avg_x < -2.0:
                return "own_half"
            else:
                return "midfield"
        else:
            # Opponent's perspective is reversed
            if avg_x < -2.0:
                return "opponent_half"
            elif avg_x > 2.0:
                return "own_half"
            else:
                return "midfield"

    def _count_robots_near_ball(
        self, robots: List[RobotSnapshot], threshold: float
    ) -> int:
        """Count robots within threshold distance of the ball."""
        ball_pos = self._current_ball_pos[:2]
        count = 0
        for robot in robots:
            dist = math.hypot(
                robot.position[0] - ball_pos[0], robot.position[1] - ball_pos[1]
            )
            if dist < threshold:
                count += 1
        return count

    def _analyze_tactical_situation(self) -> Dict[str, Any]:
        """Analyze overall tactical situation."""
        ball_x, ball_y = self._current_ball_pos[0], self._current_ball_pos[1]

        # Ball zone
        if ball_x < -3.0:
            x_zone = "own_deep"
        elif ball_x < 0:
            x_zone = "own_half"
        elif ball_x < 3.0:
            x_zone = "opponent_half"
        else:
            x_zone = "opponent_deep"

        if ball_y > 1.5:
            y_zone = "left"
        elif ball_y < -1.5:
            y_zone = "right"
        else:
            y_zone = "center"

        ball_zone = f"{x_zone}_{y_zone}"

        # Numerical advantage near ball
        our_near = self._count_robots_near_ball(
            list(self._robot_snapshots_ours.values()), 2.0
        )
        their_near = self._count_robots_near_ball(
            list(self._robot_snapshots_theirs.values()), 2.0
        )

        # Determine attack/defense style
        ball_speed = math.hypot(self._current_ball_vel[0], self._current_ball_vel[1])
        if ball_speed > 4.0:
            attack_style = "counter"
        elif self._ball_possession_team == "ours":
            attack_style = "possession"
        else:
            attack_style = "transition"

        return {
            "ball_zone": ball_zone,
            "numerical_advantage": {
                "zone": "ball_vicinity",
                "ours": our_near,
                "theirs": their_near,
            },
            "attack_style": attack_style,
            "defense_style": "zonal",  # Simplified for now
        }

    def _build_highlight_detail(
        self, highlight: HighlightEvent, now: datetime
    ) -> Dict[str, Any]:
        """Build detailed information for a single highlight."""
        time_offset = -(now - highlight.timestamp).total_seconds()

        result = {
            "type": highlight.event_type.lower(),
            "timestamp_offset_sec": round(time_offset, 1),
            "importance_score": highlight.score,
        }

        # Extract shooter info from event data
        data = highlight.data
        if "primary_robot" in data:
            robot_info = data["primary_robot"]
            result["shooter"] = {
                "robot_id": robot_info.get("id", -1),
                "is_ours": robot_info.get("is_ours", True),
                "position_at_shot": data.get("position", {"x": 0, "y": 0}),
                "distance_to_goal_m": self._calculate_distance_to_goal(
                    data.get("position", {"x": 0, "y": 0}),
                    robot_info.get("is_ours", True),
                ),
            }

        # Shot details
        ball_speed = data.get("ball_speed", 0)
        result["shot_details"] = {
            "ball_speed_mps": round(ball_speed, 1),
            "shot_angle_deg": self._estimate_shot_angle(data),
            "target_zone": self._determine_target_zone(data),
            "shot_type": "direct",  # Simplified
        }

        # Goalkeeper response (if save)
        if highlight.event_type == "SAVE" and "secondary_robot" in data:
            gk_info = data["secondary_robot"]
            result["goalkeeper_response"] = {
                "robot_id": gk_info.get("id", 0),
                "reaction_time_sec": 0.2,  # Estimated
                "dive_direction": self._estimate_dive_direction(data),
                "save_attempt": True,
            }
        elif highlight.event_type == "GOAL":
            result["goalkeeper_response"] = {
                "robot_id": self._their_goalie_id,
                "reaction_time_sec": 0.15,
                "dive_direction": "none",
                "save_attempt": False,
            }

        # Context
        result["context"] = {
            "score_before": data.get("score_before", {"ours": 0, "theirs": 0}),
            "score_after": data.get("score_after", {"ours": 0, "theirs": 0}),
            "game_minute": round(self._context.elapsed_seconds / 60.0, 1),
            "significance": self._determine_goal_significance(data),
        }

        return result

    def _calculate_distance_to_goal(
        self, position: Dict[str, float], is_ours: bool
    ) -> float:
        """Calculate distance from position to opponent's goal."""
        goal_x = 6.0 if is_ours else -6.0
        goal_y = 0.0
        return round(
            math.hypot(position.get("x", 0) - goal_x, position.get("y", 0) - goal_y),
            1,
        )

    def _estimate_shot_angle(self, data: Dict[str, Any]) -> float:
        """Estimate shot angle in degrees."""
        pos = data.get("position", {"x": 0, "y": 0})
        # Simplified: angle from shot position to goal center
        goal_x = 6.0
        dx = goal_x - pos.get("x", 0)
        dy = 0 - pos.get("y", 0)
        angle_rad = math.atan2(dy, dx)
        return round(math.degrees(angle_rad), 1)

    def _determine_target_zone(self, data: Dict[str, Any]) -> str:
        """Determine which part of goal was targeted."""
        # Simplified based on ball y position
        pos = data.get("position", {"x": 0, "y": 0})
        y = pos.get("y", 0)
        if y > 0.3:
            return "top_left"
        elif y < -0.3:
            return "top_right"
        else:
            return "center"

    def _estimate_dive_direction(self, data: Dict[str, Any]) -> str:
        """Estimate goalkeeper dive direction."""
        pos = data.get("position", {"x": 0, "y": 0})
        y = pos.get("y", 0)
        if y > 0.2:
            return "left"
        elif y < -0.2:
            return "right"
        else:
            return "center"

    def _determine_goal_significance(self, data: Dict[str, Any]) -> str:
        """Determine the significance of a goal."""
        before = data.get("score_before", {"ours": 0, "theirs": 0})
        after = data.get("score_after", {"ours": 0, "theirs": 0})

        our_diff = after.get("ours", 0) - before.get("ours", 0)

        if our_diff > 0:
            # We scored
            if before.get("ours", 0) < before.get("theirs", 0):
                if after.get("ours", 0) == after.get("theirs", 0):
                    return "equalizer"
                elif after.get("ours", 0) > after.get("theirs", 0):
                    return "comeback"
            elif before.get("ours", 0) == before.get("theirs", 0):
                return "go_ahead"
            else:
                return "insurance"

        return "regular"
