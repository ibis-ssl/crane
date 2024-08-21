# The origins of this game event.
# Empty, if it originates from game controller.
# Contains autoRef name(s), if it originates from one or more autoRefs.
# Ignored if sent by autoRef to game controller.
string[] origin

# the event that occurred
oneof event {
  # Ball out of field events (stopping)
  BallLeftField ball_left_field_touch_line
  BallLeftField ball_left_field_goal_line
  AimlessKick aimless_kick

  # Stopping Fouls
  AttackerTooCloseToDefenseArea attacker_too_close_to_defense_area
  DefenderInDefenseArea defender_in_defense_area
  BoundaryCrossing boundary_crossing
  KeeperHeldBall keeper_held_ball
  BotDribbledBallTooFar bot_dribbled_ball_too_far

  BotPushedBot bot_pushed_bot
  BotHeldBallDeliberately bot_held_ball_deliberately
  BotTippedOver bot_tipped_over

  # Non-Stopping Fouls
  AttackerTouchedBallInDefenseArea attacker_touched_ball_in_defense_area
  BotKickedBallTooFast bot_kicked_ball_too_fast
  BotCrashUnique bot_crash_unique
  BotCrashDrawn bot_crash_drawn

  # Fouls while ball out of play
  DefenderTooCloseToKickPoint defender_too_close_to_kick_point
  BotTooFastInStop bot_too_fast_in_stop
  BotInterferedPlacement bot_interfered_placement

  # Scoring goals
  Goal possible_goal
  Goal goal
  Goal invalid_goal

  # Other events
  AttackerDoubleTouchedBall attacker_double_touched_ball
  PlacementSucceeded placement_succeeded
  PenaltyKickFailed penalty_kick_failed

  NoProgressInGame no_progress_in_game
  PlacementFailed placement_failed
  MultipleCards multiple_cards
  MultipleFouls multiple_fouls
  BotSubstitution bot_substitution
  TooManyRobots too_many_robots
  ChallengeFlag challenge_flag
  EmergencyStop emergency_stop

  UnsportingBehaviorMinor unsporting_behavior_minor
  UnsportingBehaviorMajor unsporting_behavior_major

  # Deprecated events
  Prepared prepared
  IndirectGoal indirect_goal
  ChippedGoal chipped_goal
  KickTimeout kick_timeout
  AttackerTouchedOpponentInDefenseArea attacker_touched_opponent_in_defense_area
  AttackerTouchedOpponentInDefenseArea attacker_touched_opponent_in_defense_area_skipped
  BotCrashUnique bot_crash_unique_skipped
  BotPushedBot bot_pushed_bot_skipped
  DefenderInDefenseAreaPartially defender_in_defense_area_partially
  MultiplePlacementFailures multiple_placement_failures
}

# the ball left the field normally
message BallLeftField {
  # the team that last touched the ball
  uint8 by_team
  # the bot that last touched the ball
  uint32 by_bot
  # the location where the ball left the field [m]
  Vector2 location
}

# the ball left the field via goal line and a team committed an aimless kick
message AimlessKick {
  # the team that last touched the ball
  uint8 by_team
  # the bot that last touched the ball
  uint32 by_bot
  # the location where the ball left the field [m]
  Vector2 location
  # the location where the ball was last touched [m]
  Vector2 kick_location
}

# a team shot a goal
message Goal {
  # the team that scored the goal
  uint8 by_team
  # the team that shot the goal (different from by_team for own goals)
  uint8 kicking_team
  # the bot that shot the goal
  uint32 kicking_bot
  # the location where the ball entered the goal [m]
  Vector2 location
  # the location where the ball was kicked (for deciding if this was a valid goal) [m]
  Vector2 kick_location
  # the maximum height the ball reached during the goal kick (for deciding if this was a valid goal) [m]
  float32 max_ball_height
  # number of robots of scoring team when the ball entered the goal (for deciding if this was a valid goal)
  uint32 num_robots_by_team
  # The UNIX timestamp [μs] when the scoring team last touched the ball
  uint64 last_touch_by_team
  # An additional message with e.g. a reason for invalid goals
  string message
}

# the ball entered the goal directly during an indirect free kick
message IndirectGoal {
  # the team that tried to shoot the goal
  uint8 by_team
  # the bot that kicked the ball - at least the team must be set
  uint32 by_bot
  # the location where the ball entered the goal [m]
  Vector2 location
  # the location where the ball was kicked [m]
  Vector2 kick_location
}

# the ball entered the goal, but was initially chipped
message ChippedGoal {
  # the team that tried to shoot the goal
  uint8 by_team
  # the bot that kicked the ball
  uint32 by_bot
  # the location where the ball entered the goal [m]
  Vector2 location
  # the location where the ball was kicked [m]
  Vector2 kick_location
  # the maximum height [m] of the ball, before it entered the goal and since the last kick [m]
  float32 max_ball_height
}

# a bot moved too fast while the game was stopped
message BotTooFastInStop {
  # the team that found guilty
  uint8 by_team
  # the bot that was too fast
  uint32 by_bot
  # the location of the bot [m]
  Vector2 location
  # the bot speed [m/s]
  float32 speed
}

# a bot of the defending team got too close to the kick point during a free kick
message DefenderTooCloseToKickPoint {
  # the team that was found guilty
  uint8 by_team
  # the bot that violates the distance to the kick point
  uint32 by_bot
  # the location of the bot [m]
  Vector2 location
  # the distance [m] from bot to the kick point (including the minimum radius)
  float32 distance
}

# two robots crashed into each other with similar speeds
message BotCrashDrawn {
  # the bot of the yellow team
  uint32 bot_yellow
  # the bot of the blue team
  uint32 bot_blue
  # the location of the crash (center between both bots) [m]
  Vector2 location
  # the calculated crash speed [m/s] of the two bots
  float32 crash_speed
  # the difference [m/s] of the velocity of the two bots
  float32 speed_diff
  # the angle [rad] in the range [0, π] of the bot velocity vectors
  # an angle of 0 rad (  0°) means, the bots barely touched each other
  # an angle of π rad (180°) means, the bots crashed frontal into each other
  float32 crash_angle
}

# two robots crashed into each other and one team was found guilty to due significant speed difference
message BotCrashUnique {
  # the team that caused the crash
  uint8 by_team
  # the bot that caused the crash
  uint32 violator
  # the bot of the opposite team that was involved in the crash
  uint32 victim
  # the location of the crash (center between both bots) [m]
  Vector2 location
  # the calculated crash speed vector [m/s] of the two bots
  float32 crash_speed
  # the difference [m/s] of the velocity of the two bots
  float32 speed_diff
  # the angle [rad] in the range [0, π] of the bot velocity vectors
  # an angle of 0 rad (  0°) means, the bots barely touched each other
  # an angle of π rad (180°) means, the bots crashed frontal into each other
  float32 crash_angle
}

# a bot pushed another bot over a significant distance
message BotPushedBot {
  # the team that pushed the other team
  uint8 by_team
  # the bot that pushed the other bot
  uint32 violator
  # the bot of the opposite team that was pushed
  uint32 victim
  # the location of the push (center between both bots) [m]
  Vector2 location
  # the pushed distance [m]
  float32 pushed_distance
}

# a bot tipped over
message BotTippedOver {
  # the team that found guilty
  uint8 by_team
  # the bot that tipped over
  uint32 by_bot
  # the location of the bot [m]
  Vector2 location
  # the location of the ball at the moment when this foul occurred [m]
  Vector2 ball_location
}

# a defender other than the keeper was fully located inside its own defense and touched the ball
message DefenderInDefenseArea {
  # the team that found guilty
  uint8 by_team
  # the bot that is inside the penalty area
  uint32 by_bot
  # the location of the bot [m]
  Vector2 location
  # the distance [m] from bot case to the nearest point outside the defense area
  float32 distance
}

# a defender other than the keeper was partially located inside its own defense area and touched the ball
message DefenderInDefenseAreaPartially {
  # the team that found guilty
  uint8 by_team
  # the bot that is partially inside the penalty area
  uint32 by_bot
  # the location of the bot
  Vector2 location
  # the distance [m] that the bot is inside the penalty area
  float32 distance
  # the location of the ball at the moment when this foul occurred [m]
  Vector2 ball_location
}

# an attacker touched the ball inside the opponent defense area
message AttackerTouchedBallInDefenseArea {
  # the team that found guilty
  uint8 by_team
  # the bot that is inside the penalty area
  uint32 by_bot
  # the location of the bot [m]
  Vector2 location
  # the distance [m] that the bot is inside the penalty area
  float32 distance
}

# a bot kicked the ball too fast
message BotKickedBallTooFast {
  # the team that found guilty
  uint8 by_team
  # the bot that kicked too fast
  uint32 by_bot
  # the location of the ball at the time of the highest speed [m]
  Vector2 location
  # the absolute initial ball speed (kick speed) [m/s]
  float32 initial_ball_speed
  # was the ball chipped?
  bool chipped
}

# a bot dribbled to ball too far
message BotDribbledBallTooFar {
  # the team that found guilty
  uint8 by_team
  # the bot that dribbled too far
  uint32 by_bot
  # the location where the dribbling started [m]
  Vector2 start
  # the location where the maximum dribbling distance was reached [m]
  Vector2 end
}

# an attacker touched the opponent robot inside defense area
message AttackerTouchedOpponentInDefenseArea {
  # the team that found guilty
  uint8 by_team
  # the bot that touched the opponent robot
  uint32 by_bot
  # the bot of the opposite team that was touched
  uint32 victim
  # the location of the contact point between both bots [m]
  Vector2 location
}

# an attacker touched the ball multiple times when it was not allowed to
message AttackerDoubleTouchedBall {
  # the team that found guilty
  uint8 by_team
  # the bot that touched the ball twice
  uint32 by_bot
  # the location of the ball when it was first touched [m]
  Vector2 location
}

# an attacker was located too near to the opponent defense area during stop or free kick
message AttackerTooCloseToDefenseArea {
  # the team that found guilty
  uint8 by_team
  # the bot that is too close to the defense area
  uint32 by_bot
  # the location of the bot [m]
  Vector2 location
  # the distance [m] of the bot to the penalty area
  float32 distance
  # the location of the ball at the moment when this foul occurred [m]
  Vector2 ball_location
}

# a bot held the ball for too long
message BotHeldBallDeliberately {
  # the team that found guilty
  uint8 by_team
  # the bot that holds the ball
  uint32 by_bot
  # the location of the ball [m]
  Vector2 location
  # the duration [s] that the bot hold the ball
  float32 duration
}

# a bot interfered the ball placement of the other team
message BotInterferedPlacement {
  # the team that found guilty
  uint8 by_team
  # the bot that interfered the placement
  uint32 by_bot
  # the location of the bot [m]
  Vector2 location
}

# a team collected multiple cards (yellow and red), which results in a penalty kick
message MultipleCards {
  # the team that received multiple yellow cards
  uint8 by_team
}

# a team collected multiple fouls, which results in a yellow card
message MultipleFouls {
  # the team that collected multiple fouls
  uint8 by_team
  # the list of game events that caused the multiple fouls
  GameEvent[] caused_game_events
}

# a team failed to place the ball multiple times in a row
message MultiplePlacementFailures {
  # the team that failed multiple times
  uint8 by_team
}

# timeout waiting for the attacking team to perform the free kick
message KickTimeout {
  # the team that that should have kicked
  uint8 by_team
  # the location of the ball [m]
  Vector2 location
  # the time [s] that was waited
  float32 time
}

# game was stuck
message NoProgressInGame {
  # the location of the ball
  Vector2 location
  # the time [s] that was waited
  float32 time
}

# ball placement failed
message PlacementFailed {
  # the team that failed
  uint8 by_team
  # the remaining distance [m] from ball to placement position
  float32 remaining_distance
}

# a team was found guilty for minor unsporting behavior
message UnsportingBehaviorMinor {
  # the team that found guilty
  uint8 by_team
  # an explanation of the situation and decision
  string reason
}

# a team was found guilty for major unsporting behavior
message UnsportingBehaviorMajor {
  # the team that found guilty
  uint8 by_team
  # an explanation of the situation and decision
  string reason
}

# a keeper held the ball in its defense area for too long
message KeeperHeldBall {
  # the team that found guilty
  uint8 by_team
  # the location of the ball [m]
  Vector2 location
  # the duration [s] that the keeper hold the ball
  float32 duration
}

# a team successfully placed the ball
message PlacementSucceeded {
  # the team that did the placement
  uint8 by_team
  # the time [s] taken for placing the ball
  float32 time_taken
  # the distance [m] between placement location and actual ball position
  float32 precision
  # the distance [m] between the initial ball location and the placement position
  float32 distance
}

# both teams are prepared - all conditions are met to continue (with kickoff or penalty kick)
message Prepared {
  # the time [s] taken for preparing
  float32 time_taken
}

# bots are being substituted by a team
message BotSubstitution {
  # the team that substitutes robots
  uint8 by_team
}

# A challenge flag, requested by a team previously, is flagged
message ChallengeFlag {
  # the team that substitutes robots
  uint8 by_team
}

# An emergency stop, requested by team previously, occurred
message EmergencyStop {
  # the team that substitutes robots
  uint8 by_team
}

# a team has too many robots on the field
message TooManyRobots {
  # the team that has too many robots
  uint8 by_team
  # number of robots allowed at the moment
  int32 num_robots_allowed
  # number of robots currently on the field
  int32 num_robots_on_field
  # the location of the ball at the moment when this foul occurred [m]
  Vector2 ball_location
}

# a robot chipped the ball over the field boundary out of the playing surface
message BoundaryCrossing {
  # the team that has too many robots
  uint8 by_team
  # the location of the ball [m]
  Vector2 location
}

# the penalty kick failed (by time or by keeper)
message PenaltyKickFailed {
  # the team that last touched the ball
  uint8 by_team
  # the location of the ball at the moment of this event [m]
  Vector2 location
}

# Enum for game event types
enum Type {
  UNKNOWN_GAME_EVENT_TYPE = 0

  # Ball out of field events (stopping)
  BALL_LEFT_FIELD_TOUCH_LINE = 6
  BALL_LEFT_FIELD_GOAL_LINE = 7
  AIMLESS_KICK = 11

  # Stopping Fouls
  ATTACKER_TOO_CLOSE_TO_DEFENSE_AREA = 19
  DEFENDER_IN_DEFENSE_AREA = 31
  BOUNDARY_CROSSING = 41
  KEEPER_HELD_BALL = 13
  BOT_DRIBBLED_BALL_TOO_FAR = 17

  BOT_PUSHED_BOT = 24
  BOT_HELD_BALL_DELIBERATELY = 26
  BOT_TIPPED_OVER = 27

  # Non-Stopping Fouls
  ATTACKER_TOUCHED_BALL_IN_DEFENSE_AREA = 15
  BOT_KICKED_BALL_TOO_FAST = 18
  BOT_CRASH_UNIQUE = 22
  BOT_CRASH_DRAWN = 21

  # Fouls while ball out of play
  DEFENDER_TOO_CLOSE_TO_KICK_POINT = 29
  BOT_TOO_FAST_IN_STOP = 28
  BOT_INTERFERED_PLACEMENT = 20

  # Scoring goals
  POSSIBLE_GOAL = 39
  GOAL = 8
  INVALID_GOAL = 42

  # Other events
  ATTACKER_DOUBLE_TOUCHED_BALL = 14
  PLACEMENT_SUCCEEDED = 5
  PENALTY_KICK_FAILED = 43

  NO_PROGRESS_IN_GAME = 2
  PLACEMENT_FAILED = 3
  MULTIPLE_CARDS = 32
  MULTIPLE_FOULS = 34
  BOT_SUBSTITUTION = 37
  TOO_MANY_ROBOTS = 38
  CHALLENGE_FLAG = 44
  EMERGENCY_STOP = 45

  UNSPORTING_BEHAVIOR_MINOR = 35
  UNSPORTING_BEHAVIOR_MAJOR = 36

  # Deprecated events
  PREPARED = 1
  INDIRECT_GOAL = 9
  CHIPPED_GOAL = 10
  KICK_TIMEOUT = 12
  ATTACKER_TOUCHED_OPPONENT_IN_DEFENSE_AREA = 16
  ATTACKER_TOUCHED_OPPONENT_IN_DEFENSE_AREA_SKIPPED = 40
  BOT_CRASH_UNIQUE_SKIPPED = 23
  BOT_PUSHED_BOT_SKIPPED = 25
  DEFENDER_IN_DEFENSE_AREA_PARTIALLY = 30
  MULTIPLE_PLACEMENT_FAILURES = 33
}
