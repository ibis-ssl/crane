# ディフェンス

## Planner

### TotalDefensePlanner

現在使用中

TotalDefensePlannerは、ゴールキーパーとディフェンダーの配置を決定するプランナーです。このプランナーでは、`calculateRobotCommand`関数を用いてディフェンダーの目標位置を計算し、`getSelectedRobots`関数を用いてゴールキーパーとディフェンダーを選択します。ディフェンダーの目標位置は、`getDefenseLinePoints`関数によって計算されます。

`getDefenseLinePoints`関数は、以下の手順でディフェンダーの配置を決定します。

1. `getDefenseLinePointParameter`関数を使用して、ボールライン上のパラメータを計算します。`DefenseLinePointParameter`は、自陣ペナルティエリアを囲む線をパラメータ化したもので、0から`threshold3`までの値を持ちます。このパラメータは、ディフェンダーが配置されるべきボールライン上の位置を示します。
2. パラメータが存在しない場合、ペナルティエリア内にボールが侵入したときにディフェンダーがいなくなるのを防ぐために、代替のボールラインを使用してパラメータを計算します。
3. パラメータが存在する場合、`getDefenseLinePoint`関数を使用して、パラメータに対応するディフェンダーの目標位置を計算します。
4. 複数のディフェンダーを配置する必要がある場合、`DEFENSE_INTERVAL`パラメータを使用して、ディフェンダー間の間隔を決定します。`DEFENSE_INTERVAL`パラメータのデフォルト値は0.2です。
5. `getDefenseLinePointParameterThresholds`関数を使用して、ディフェンダーが配置されるべき範囲を制限します。

各パラメータの説明は、以下のとおりです。

* `DEFENSE_INTERVAL`: ディフェンダー間の間隔を決定するパラメータです。
* `OFFSET_X`: ペナルティエリアのコーナーを計算する際に使用するX軸方向のオフセットです。
* `OFFSET_Y`: ペナルティエリアのコーナーを計算する際に使用するY軸方向のオフセットです。

`getDefenseLinePointParameter`関数で得られるパラメータは、以下の図に示すように、自陣ペナルティエリアを囲む線をパラメータ化したものです。

```mermaid
graph LR
    A[p1] -- parameter < threshold1 --> B[p2]
    B -- threshold1 + parameter < threshold2 --> C[p3]
    C -- threshold2 + parameter < threshold3 --> D[p4]
    style A fill:#f9f,stroke:#333,stroke-width:2px
    style B fill:#f9f,stroke:#333,stroke-width:2px
    style C fill:#f9f,stroke:#333,stroke-width:2px
    style D fill:#f9f,stroke:#333,stroke-width:2px
```

この図では、p1, p2, p3, p4はペナルティエリアのコーナーを表し、parameterはp1からp2、p2からp3、p3からp4への距離を示します。

### DefenderPlanner

現在非推奨

DefenderPlannerは、ディフェンダーの配置を決定するプランナーです。このプランナーは、`calculateRobotCommand`関数を使用して、ディフェンダーの目標位置を計算します。ディフェンダーの目標位置は、`getDefenseLinePoints`関数または`getDefenseArcPoints`関数を使用して計算されます。

### TigersGoaliePlanner

TigersGoaliePlannerは、Tigersのゴールキーパーの行動を決定するプランナーです。このプランナーは、`calculateRobotCommand`関数を使用して、ゴールキーパーの状態に基づいて、ゴールキーパーの行動を決定します。ゴールキーパーの状態は、以下の状態遷移図に基づいて更新されます。

```mermaid
stateDiagram
    [*] --> STOP
    STOP --> DEFEND : not isStopped()
    PREPARE_PENALTY --> DEFEND : not isPreparePenalty()
    MOVE_TO_PENALTY_AREA --> DEFEND : status == SUCCESS
    MOVE_TO_PENALTY_AREA --> DEFEND : isKeeperWellInsidePenaltyArea()
    MOVE_TO_PENALTY_AREA --> STOP : isStopped()
    MOVE_TO_PENALTY_AREA --> PREPARE_PENALTY : isPreparePenalty()
    DEFEND --> PASS : ballCanBePassedOutOfPenaltyArea()
    DEFEND --> RAMBO : canGoOut()
    DEFEND --> GET_BALL_CONTACT : isBallBetweenGoalieAndGoal()
    DEFEND --> MOVE_TO_PENALTY_AREA : isOutsidePenaltyArea()
    DEFEND --> STOP : isStopped()
    DEFEND --> PREPARE_PENALTY : isPreparePenalty()
    DEFEND --> INTERCEPT : canInterceptSafely()
    PASS --> DEFEND : isBallMoving()
    PASS --> MOVE_IN_FRONT_OF_BALL : isBallPlacementRequired()
    PASS --> STOP : isStopped()
    PASS --> PREPARE_PENALTY : isPreparePenalty()
    INTERCEPT --> DEFEND : hasInterceptionFailed()
    INTERCEPT --> PASS : ballCanBePassedOutOfPenaltyArea()
    INTERCEPT --> STOP : isStopped()
    INTERCEPT --> PREPARE_PENALTY : isPreparePenalty()
    RAMBO --> DEFEND : world_model->point_checker.isPenaltyArea(world_model.ball.pos) or isGoalKick()
    RAMBO --> STOP : isStopped()
    RAMBO --> PREPARE_PENALTY : isPreparePenalty()
    MOVE_IN_FRONT_OF_BALL --> DEFEND : isBallMoving()
    MOVE_IN_FRONT_OF_BALL --> DEFEND : isBallPlaced()
    MOVE_IN_FRONT_OF_BALL --> GET_BALL_CONTACT : status == SUCCESS
    MOVE_IN_FRONT_OF_BALL --> STOP : isStopped()
    MOVE_IN_FRONT_OF_BALL --> PREPARE_PENALTY : isPreparePenalty()
    GET_BALL_CONTACT --> MOVE_WITH_BALL : status == SUCCESS
    GET_BALL_CONTACT --> MOVE_IN_FRONT_OF_BALL : status == FAILURE
    GET_BALL_CONTACT --> STOP : isStopped()
    GET_BALL_CONTACT --> PREPARE_PENALTY : isPreparePenalty()
    MOVE_WITH_BALL --> DEFEND : status == SUCCESS
    MOVE_WITH_BALL --> MOVE_IN_FRONT_OF_BALL : status == FAILURE
    MOVE_WITH_BALL --> STOP : isStopped()
    MOVE_WITH_BALL --> PREPARE_PENALTY : isPreparePenalty()
```

## Skill

### Goalie

Goalieスキルは、ゴールキーパーの行動を制御するスキルです。このスキルは、ボールの位置、速度、および他のロボットの位置に基づいて、ゴールキーパーの行動を決定します。主なロジックは`inplay`関数にあり、以下の状況に応じて行動を決定します。

* **HALT**: その場に停止
* **THEIR_PENALTY_PREPARATION, THEIR_PENALTY_START**: ボールの排出を停止
* **COMMAND_STOP**: ボールの排出を停止
* **ボール排出**:
  * ボールが止まっていて、味方ペナルティエリア内にある場合、ボールをペナルティエリア外に出します。
  * パスできるロボットのリストを作成し、最も適切なロボットにパスします。
* **シュートブロック**:
  * ボールがゴールに向かっている場合、ボールライン上の最も近い点を計算し、その点に移動してシュートをブロックします。
* **ボールを待ち受ける**:
  * 上記以外の場合、デフォルト位置（自ゴールの中央の0.9倍の位置）に移動し、ボールを待ち受けます。
  * ボールが自コートにある場合、敵のパス先を予測し、その位置に移動してパスカットを試みます。

### Marker

Markerスキルは、指定された敵ロボットをマークするスキルです。このスキルは、`update`関数にあり、`mark_mode`パラメータに基づいて、以下のモードでマーキングを行います。

* **save_goal**:
  * 敵ロボットと自ゴールの中間の位置に移動します。
  * 敵ロボットがシュートするのを防ぐことを目的とします。
* **intercept_pass**:
  * 敵ロボットとボールの中間の位置に移動します。
  * 敵ロボットへのパスを阻止することを目的とします。
