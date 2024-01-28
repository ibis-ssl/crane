# Simple AI

## 使い方

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
        VisionNode[Vision Component]
        Sender[Real Sender]
        Receiver[Robot Receiver]
    end

    VT[Vision Tracker]
    WP[World Model Publisher]
    Main[Simple AI]
    LP[Local Planner]

    subgraph RealWorld
        Robot[Actual Robot CM4]
        SSLVision[SSL Vision]
    end
    
    SSLVision -. UDP .->  VisionNode
    VisionNode -- /detection -->  VT
    VT -- /detection_tracked -->  WP
    VisionNode -- /geometry -->  WP
    
    WP -- /world_model -->  Main
    Main -- /control_targets --> LP
    LP -- /robot_commands -->  Sender
    
    Sender -. UDP .->  Robot
    
    Robot -. UDP .->  Receiver
    Receiver -- /feedback -->  WP
```
