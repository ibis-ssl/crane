# Crane ガイド

Crane は Vision から世界モデルを作り、セッションで役割を割り当て、スキル・局所経路計画を経てロボットへ指令を送ります。[パッケージ一覧](packages/index.md)から各実装へ進めます。

## 動かす・調べる

- [環境構築・起動](setup.md) / [Docker](docker.md)
- [試合チェックリスト](match.md) / [Vision設定](vision.md)
- [ネットワークと実機通信](network.md) / [診断](diagnostics.md)
- [開発・テスト](tools.md) / [ER-Forceシミュレータ](erforce_sim.md) / [grSim](grSim.md)

## 設計・拡張

- [座標・単位](coordinates.md) / [競技ルールへの対応](rule.md)
- [スキルの追加](skill.md) / [WorldModelの利用](world_model_wrapper.md) / [可視化](visualizer.md)
- [局所経路計画](rvo2_local_planner.md)
- [ボールトラッキング](ball_tracking_system.md) / [キャリブレーション](ball_model_calibration_guide.md)
- [攻撃](offense.md) / [Attacker](attacker.md) / [パス](pass.md) / [守備](defense.md)

API・設定値・実装一覧はソースが正本です。このガイドには手順・設計理由・制約を残します。

この `docs/` は [同期設定](https://github.com/ibis-ssl/crane/blob/develop/.github/sync.yml) で外部ドキュメントへコピーされます。削除したページが公開先からも消えるかは同期アクションの設定を確認してから反映し、この変更では公開操作を行いません。
