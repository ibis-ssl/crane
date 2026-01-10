# crane_msg_wrappers

## 概要

ROS 2メッセージの**変換・ラッパーユーティリティ**を提供するパッケージです。crane_msgsと他のメッセージ型の相互変換、WorldModelWrapper・PlaySituationWrapper等の高レベルラッパークラスにより、メッセージ処理を簡素化します。

## 主要機能

- **メッセージ変換**: crane_msgs ⇔ 標準ROS 2メッセージの相互変換
- **高レベルラッパー**: WorldModelWrapper、PlaySituationWrapperによる統合インターフェース
- **型安全変換**: コンパイル時の型チェックによる安全なメッセージ処理
- **パフォーマンス最適化**: 効率的なメモリ使用とゼロコピー変換

## アーキテクチャ上の役割

Craneシステムの**メッセージ処理基盤**として、異なるメッセージ型間の変換を担い、上位システムからメッセージ形式の詳細を隠蔽します。

## 主要コンポーネント

- **WorldModelWrapper**: 統合世界モデルの高レベルインターフェース
- **PlaySituationWrapper**: 試合状況の抽象化
- **変換関数群**: 各種メッセージ型の相互変換

## 使用方法

```cpp
#include <crane_msg_wrappers/world_model_wrapper.hpp>

class MyNode : public rclcpp::Node
{
private:
  crane::WorldModelWrapper::SharedPtr world_model;

public:
  MyNode() : Node("my_node")
  {
    // ノードを渡して初期化（自動的にサブスクライバが設定されます）
    world_model = std::make_shared<crane::WorldModelWrapper>(*this);

    // 更新時のコールバック登録
    world_model->addCallback([this]() {
        // ボール情報へのアクセス
        auto ball_pos = world_model->ball().pos;

        // ロボット情報へのアクセス
        if (auto robot = world_model->getOurRobot(0)) {
            // ロボットが有効な場合
        }
    });
  }
};
```

## 最近の開発状況

🟢 **安定**: メッセージ変換ライブラリとして成熟しており、新メッセージ型追加時の変換関数追加や変換効率の向上が継続的に行われています。

---

**関連パッケージ**: [crane_msgs](./crane_msgs.md) | [crane_geometry](./crane_geometry.md) | [crane_physics](./crane_physics.md)
