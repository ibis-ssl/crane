# crane_msgs

## 役割

Craneシステム全体で使用されるカスタムROS 2メッセージ、サービス、アクション定義を提供するパッケージです。

## 固有の制約

- ボールの3D軌道表現には `geometry_msgs/Vector3 position` および `velocity` の `z` 成分を使用すること。
- 全メッセージ定義は `msg/` 直下に配置されていること。
- トピックガイドは存在しないため、各メッセージ型のフィールド定義は `.msg` ファイルを参照すること。

## リンク

- ソースディレクトリ: [crane_msgs](https://github.com/ibis-ssl/crane/tree/develop/crane_msgs)
- メッセージ定義: [msg](https://github.com/ibis-ssl/crane/tree/develop/crane_msgs/msg)
