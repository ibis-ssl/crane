# ディフェンス戦術システム

## 目的

自チームゴール防衛および失点リスク低減のため、ゴールキーパー（Goalie）と複数ディフェンダー（Defender / Marker）を協調配置・制御します。

## 戦術構成と配置規約

- **統合守備戦略（TotalDefenseSession）**:
  - キーパー1台と指定台数のディフェンダーを協調配置。
  - `getDefenseLinePoints` により自陣ペナルティエリア外周を囲む守備ライン上に等間隔で配置。
- **脅威マーク戦略（SecondThreatDefenderSession / Marker）**:
  - 最もシュート脅威度の高い敵ロボットへ個別にディフェンダーを割り当て、パスコース遮断（`intercept_pass`）またはシュートブロック（`save_goal`）を実行。
- **ゴールキーパー（Goalie）**:
  - ボール軌道予測に基づきゴールライン上の交点へ移動してシュートを阻止する。味方ペナルティエリア内の停止球は、敵の接近中は待機し、条件を満たせば敵ゴール方向へチップで排出する。

## 安全・ルール制約規約

- **ペナルティエリア侵入防止**: ディフェンダーが誤って自陣ペナルティエリアに侵入しないよう、`crane_local_planner` のエリア回避を適用する。詳細は [ルール制約](./rule.md) を参照。

## 実装リファレンス

- 統合守備セッション: [total_defense_session.hpp](https://github.com/ibis-ssl/crane/blob/develop/crane_sessions/include/crane_sessions/total_defense_session.hpp)
- キーパースキル: [goalie.hpp](https://github.com/ibis-ssl/crane/blob/develop/crane_robot_skills/include/crane_robot_skills/goalie.hpp)
- マークスキル: [marker.hpp](https://github.com/ibis-ssl/crane/blob/develop/crane_robot_skills/include/crane_robot_skills/marker.hpp)
- 局所回避: [局所経路計画（Local Planner）](./rvo2_local_planner.md)
