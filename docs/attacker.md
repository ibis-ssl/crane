# Attackerスキル

- ENTRY_POINT
- FORCED_PASS
- REDIRECT_GOAL_KICK
- GOAL_KICK
- CLEARING_KICK
- STANDARD_PASS
- LOW_CHANCE_GOAL_KICK
- MOVE_BALL_TO_OPPONENT_HALF
- RECEIVE_BALL

```mermaid
graph TD
    ENTRY_POINT(ENTRY_POINT) -.-> FORCED_PASS_C{味方セットプレー？}
subgraph セットプレー
    FORCED_PASS_C -- YES --> FORCED_PASS
end

%%subgraph 敵ボール対処
%%    STEAL_BALL_C
%%end

subgraph シュートチャンスは逃さない

end

subgraph 敵ボール対処
    FORCED_PASS_C -- NO --> CUT_THEIR_PASS_C{動いている敵ボール？}
    CUT_THEIR_PASS_C -- NO --> STEAL_BALL_C{止まっている敵ボール & ボールに敵が近い？}
    CUT_THEIR_PASS_C -- YES --> CUT_THEIR_PASS
end

subgraph シュートチャンスは逃さない
    STEAL_BALL_C -- NO --> REDIRECT_GOAL_KICK_C{ボールが遠い＆ゴールが見えている}
    REDIRECT_GOAL_KICK_C -- NO --> GOAL_KICK_C{ボールが近い＆ゴールが見えている}
    REDIRECT_GOAL_KICK_C -- YES --> REDIRECT_GOAL_KICK
    GOAL_KICK_C -- YES --> GOAL_KICK
end

subgraph 敵ボール対処
    STEAL_BALL_C -- YES --> STEAL_BALL
end

subgraph 状況改善
    GOAL_KICK_C -- NO --> CLEARING_KICK_C{やばい状況？（未実装）}
    CLEARING_KICK_C -- NO --> STANDARD_PASS_C{パスできそうな相手がいる？}
    CLEARING_KICK_C -- YES --> CLEARING_KICK
    STANDARD_PASS_C -- YES --> STANDARD_PASS
    STANDARD_PASS_C -- NO --> LOW_CHANCE_GOAL_KICK_C{低確率でもシュートできそう（相手コート限定）？}
    LOW_CHANCE_GOAL_KICK_C -- NO --> MOVE_BALL_TO_OPPONENT_HALF_C{自コートでボールを持っている？}
    LOW_CHANCE_GOAL_KICK_C -- YES --> LOW_CHANCE_GOAL_KICK
    MOVE_BALL_TO_OPPONENT_HALF_C -- YES --> MOVE_BALL_TO_OPPONENT_HALF
    MOVE_BALL_TO_OPPONENT_HALF_C -- NO --> RECEIVE_BALL
end
```

- ENTRY_POINT
- FORCED_PASS
  - 二度と帰ってこなくてOK
- REDIRECT_GOAL_KICK
  - 入：ボールが遠く＆動いている＆自分に向かってきている＆リダイレクト角度
  - 出：ボールが止まった
- GOAL_KICK
  - 入：ボールが近い＆ゴールが見える＆ボールが止まっている
  - 出：ボールが動いた
- STANDARD_PASS
  - 入：ボールが近い＆ボールが停止＆良いパス相手がいる
  - 出：ボールが動いた
- LOW_CHANCE_GOAL_KICK
  - 相手コートで
- MOVE_BALL_TO_OPPONENT_HALF
- RECEIVE_BALL
- FINAL_GUARD

再編成

- ENTRY_POINT
- FORCED_PASS
- RECEIVE
  - RECEIVE
  - REDIRECT_GOAL_KICK
- KICK
  - GOAL_KICK
    - LOW_CHANCE_GOAL_KICK
  - STANDARD_PASS
  - MOVE_BALL_TO_OPPONENT_HALF
- FINAL_GUARD
