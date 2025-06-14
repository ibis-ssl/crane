# Simple AI

## 立ち上げ

```bash
ros2 launch crane_bringup crane.launch.py simple_ai:=true sim:=false
```

## GUI使い方

- ロボットの設定
  - IDを指定
- コマンドの追加
  - プルダウンでコマンドの種類を選ぶ
  - 引数を設定する
  - 追加ボタンを押す
- コマンドキューの編集
  - コマンドキューの中身を選択する
  - 編集できる
  - 実行中は編集できなくなる
- コマンドキューの実行
  - 実行ボタンを押す
    - 実行中は停止ボタンになる
  - コマンドは上から順に実行される

## ノードダイアグラム

```mermaid
graph TD
    subgraph interface
        VisionComponent[Vision Component]
        CraneSender[crane_sender]
        CraneRobotReceiver[crane_robot_receiver]
    end

    VisionTracker[Vision Tracker]
    WorldModelPublisher[crane_world_model_publisher]
    SimpleAINode[crane_simple_ai]
    LocalPlanner[crane_local_planner]

    subgraph RealWorld
        ActualRobot[Actual Robot CM4]
        SSLVisionService[SSL Vision Service]
    end
    %% Node names have been updated to be more specific. Exact names for VisionComponent
    %% and VisionTracker might vary based on the specific vision pipeline components used.

    SSLVisionService -. UDP .->  VisionComponent
    VisionComponent -- /detection -->  VisionTracker
    VisionTracker -- /detection_tracked -->  WorldModelPublisher
    VisionComponent -- /geometry -->  WorldModelPublisher

    WorldModelPublisher -- /world_model -->  SimpleAINode
    SimpleAINode -- /control_targets --> LocalPlanner
    LocalPlanner -- /robot_commands -->  CraneSender

    CraneSender -. UDP .->  ActualRobot

    ActualRobot -. UDP .->  CraneRobotReceiver
    CraneRobotReceiver -- /feedback -->  WorldModelPublisher
```
