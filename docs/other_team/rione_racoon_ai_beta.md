# RACOON AI beta of Ri-one

https://github.com/Rione/ssl-RACOON-AI-beta

## Role分け

https://github.com/Rione/ssl-RACOON-AI-beta/tree/competition/racoon_ai/strategy/role


### Offense

#### 決定戦略

https://github.com/Rione/ssl-RACOON-AI-beta/blob/competition/racoon_ai/strategy/role/role.py#L138

1. 有効でキーパーでもDeffenseでもないロボットを抽出
2. 相手ゴールに近い順に規定台数を選択．それ以外は削除
3. なにかの角度に並べ替え？

#### SubRole

https://github.com/Rione/ssl-RACOON-AI-beta/blob/competition/racoon_ai/strategy/role/subrole.py#L53

- our_attacker
  - ボールに一番近いロボットが1台選ばれる
- receiver
  - our_attackerに近いロボットが1台選ばれる

※enemy_attackerは敵チームのボールに一番近いロボット

## Scheme

### defense
- デフォルトポジションがある
- ボールがゴールに向かっていたら阻止する？

### goal_keeper
### offense
- indirectならreceiver(一番近い味方)にパス
- そうでなければゴールにシュート

- 関数名
  - block_their_attacker
  - default_position
  - penalty_kick
  - stop_attacker
  - stop_offense
  - pass_to_receiver
  - shoot_to_goal
  - direct_their
    - ボールに一番近い味方がblock_their_attacker
### out_of_play

#### Defense
#### その他
- キーパー

## コマンド周り

- avoid_panalty_area
- avoid_ball
- avoid_enemy
- speed_limiter
- pid(目標位置, ロボット)
- to_front_ball
- ball_around
