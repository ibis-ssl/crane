# The origins of this game event.
# Empty, if it originates from game controller.
# Contains autoRef name(s), if it originates from one or more autoRefs.
# Ignored if sent by autoRef to game controller.
string[] origin

# the event that occurred

######## START ONEOF ########

# Ball out of field events (stopping)
BallLeftField[<=1] ball_left_field_touch_line
BallLeftField[<=1] ball_left_field_goal_line
AimlessKick[<=1] aimless_kick

# Stopping Fouls
AttackerTooCloseToDefenseArea[<=1] attacker_too_close_to_defense_area
DefenderInDefenseArea[<=1] defender_in_defense_area
BoundaryCrossing[<=1] boundary_crossing
KeeperHeldBall[<=1] keeper_held_ball
BotDribbledBallTooFar[<=1] bot_dribbled_ball_too_far

BotPushedBot[<=1] bot_pushed_bot
BotHeldBallDeliberately[<=1] bot_held_ball_deliberately
BotTippedOver[<=1] bot_tipped_over

# Non-Stopping Fouls
AttackerTouchedBallInDefenseArea[<=1] attacker_touched_ball_in_defense_area
BotKickedBallTooFast[<=1] bot_kicked_ball_too_fast
BotCrashUnique[<=1] bot_crash_unique
BotCrashDrawn[<=1] bot_crash_drawn

# Fouls while ball out of play
DefenderTooCloseToKickPoint[<=1] defender_too_close_to_kick_point
BotTooFastInStop[<=1] bot_too_fast_in_stop
BotInterferedPlacement[<=1] bot_interfered_placement

# Scoring goals
Goal[<=1] possible_goal
Goal[<=1] goal
Goal[<=1] invalid_goal

# Other events
AttackerDoubleTouchedBall[<=1] attacker_double_touched_ball
PlacementSucceeded[<=1] placement_succeeded
PenaltyKickFailed[<=1] penalty_kick_failed

NoProgressInGame[<=1] no_progress_in_game
PlacementFailed[<=1] placement_failed
MultipleCards[<=1] multiple_cards
MultipleFouls[<=1] multiple_fouls
BotSubstitution[<=1] bot_substitution
TooManyRobots[<=1] too_many_robots
ChallengeFlag[<=1] challenge_flag
EmergencyStop[<=1] emergency_stop

UnsportingBehaviorMinor[<=1] unsporting_behavior_minor
UnsportingBehaviorMajor[<=1] unsporting_behavior_major

# Deprecated events
Prepared[<=1] prepared
IndirectGoal[<=1] indirect_goal
ChippedGoal[<=1] chipped_goal
KickTimeout[<=1] kick_timeout
AttackerTouchedOpponentInDefenseArea[<=1] attacker_touched_opponent_in_defense_area
AttackerTouchedOpponentInDefenseArea[<=1] attacker_touched_opponent_in_defense_area_skipped
BotCrashUnique[<=1] bot_crash_unique_skipped
BotPushedBot[<=1] bot_pushed_bot_skipped
DefenderInDefenseAreaPartially[<=1] defender_in_defense_area_partially
MultiplePlacementFailures[<=1] multiple_placement_failures

######## END ONEOF ########
