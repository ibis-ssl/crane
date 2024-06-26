# TIGERs Sumatra

## Attackerの状態機械

<https://github.com/TIGERs-Mannheim/Sumatra/blob/master/modules/moduli-ai/src/main/java/edu/tigers/sumatra/ai/pandora/roles/offense/attacker/AttackerRole.java>

```mermaid
stateDiagram-v2
    Protect --> ApproachBallLine: ballMoves
    Protect --> Kick : switchToKick

    Dribble --> DribbleKick: switchToDribbleKick
    Dribble --> Kick: switchToKick
    Dribble --> Protect: FAILURE

    ApproachBallLine --> Receive: SUCCESS
    ApproachBallLine --> approachAndStopBall: FAILURE
    ApproachBallLine --> approachAndStopBall: closeToBall

    approachAndStopBall --> Protect: SUCCESS
    approachAndStopBall --> Protect: FAILURE

    Kick --> Receive: SUCCESS
    Kick --> Protect: FAILURE / INVALID
    Kick --> FreeKick: waitForKick
    Kick --> FreeKick: useSingleTouch

    FreeKick --> ApproachBallLine: SUCCESS
    FreeKick --> Protect: FAILURE

    Receive --> Protect: SUCCESS
    Receive --> Protect: FAILURE
    Receive --> Redirect: SwitchToRedirect

    Redirect --> ApproachBallLine: SUCCESS
    Redirect --> Protect: FAILURE
    Redirect --> Receive: switchToReceive

    DribbleKick --> Protect: dribblingKickIsBlocked
    DribbleKick --> Protect: FAILUR
```

## Keeperの状態機械

```mermaid
stateDiagram-v2
    Stop --> Defend: !stoped

    PreparePenalty --> Defend: !isPreparePenalty

    MoveToPenaltyArea --> Defend: SUCCESS
    MoveToPenaltyArea --> Defend: isKeeperWellInsidePenaltyArea
    MoveToPenaltyArea --> Stop: isStopped
    MoveToPenaltyArea --> PreparePenalty: isPreparePenalty

    Defend --> Pass: ballCanBePassedOutOfPenaltyArea
    Defend --> Rambo: canGoOut
    Defend --> GetBallContact: isBallBetweenGoalyAndGoal
    Defend --> MoveToPenaltyArea: isOutsidePenaltyArea
    Defend --> Stop: isStopped
    Defend --> PreparePenalty: isPreparePenalty
    Defend --> Intercept: canInterceptSafely

    Pass --> Defend: isBallMoving
    Pass --> MoveInFrontOfBall: ballPlacementRequired
    Pass --> Stop: isStopped
    Pass --> PreparePenalty: isPreparePenalty

    Intercept --> Defend: hasInterceptionFailed
    Intercept --> Pass: ballPlacementRequired
    Intercept --> Stop: isStopped
    Intercept --> PreparePenalty: isPreparePenalty

    Rambo --> Defend: isBallInPenaltyArea(0) || isGoalKick()
    Rambo --> Stop: isStopped
    Rambo --> PreparePenalty: isPreparePenalty

    MoveInFrontOfBall --> Defend: isBallMoving
    MoveInFrontOfBall --> Defend: ballPlaced
    MoveInFrontOfBall --> GetBallContact: SUCCESS
    MoveInFrontOfBall --> Stop: isStopped
    MoveInFrontOfBall --> PreparePenalty: isPreparePenalty

    GetBallContact --> MoveWithBall: SUCCESS
    GetBallContact --> MoveInFrontOfBall: FAILURE
    GetBallContact --> Stop: isStopped
    GetBallContact --> PreparePenalty: isPreparePenalty

    MoveWithBall --> Defend: SUCCESS
    MoveWithBall --> MoveInFrontOfBall: FAILURE
    MoveWithBall --> Stop: isStopped
    MoveWithBall --> PreparePenalty: isPreparePenalty

```

## BallPlacementの状態機械

```mermaid
stateDiagram-v2
    Receive --> Prepare: SUCCESS
    Receive --> StopBall: FAILURE
    Prepare --> DropBall: ballIsPlaced
    Prepare --> GetBallContact: success
    Prepare --> Receive: ballMoving
    Prepare --> Pass: ballNeedsToBePassed
    Prepare --> GetBallContact: skipPrepare
    StopBall --> Prepare: SUCCESS
    StopBall --> Prepare: FAILURE
    GetBallContact --> MoveWithBall: SUCCESS
    GetBallContact --> DropBall: FAILURE
    MoveWithBall --> DropBall: SUCCESS
    MoveWithBall --> Receive: FAILURE
    Pass --> ClearBall: FAILURE
    Pass --> ClearBall: SUCCESS
    Pass --> ClearBall: pass mode is NONE
    DropBall --> ClearBall: FAILURE
    DropBall --> ClearBall: SUCCESS
```

## AOffensiveActionMoves

TDP2018に解説されている。

- ForcedPass
- DirectKick
  - 敵ゴールへの直接シュート
- ClearingKick
  - 味方の危ない状況からのクリア
- StandardPass
- LowChanceKick
  - 敵ゴールへの直接シュート
- GoToOtherHalf
  - 自陣でボールをキープしているが適切なパスターゲットがない場合
- KickInsBlue
  - 敵ロボットがいない場所にボールを蹴る
- RedirectGoalShot
- RedirectPass
- Receive
