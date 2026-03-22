# crane_bag

## 概要

`crane_bag`は、Crane rosbagデータの解析・調査を行うC++ CLIツールとPythonライブラリです。ROS 2のrosbag2 APIを使用して、試合記録から位置情報・審判コマンド・制御ターゲット・イベントを抽出・分析します。

## 主要機能

- **info**: トピック一覧と統計情報の表示
- **survey**: 試合全体の概要サーベイ
- **track**: ロボット・ボールの位置追跡
- **events**: ゴール・キック・ファウル等のイベント検出
- **control**: 制御ターゲットとplanning_factors分析
- **referee**: 審判コマンド遷移の抽出

## アーキテクチャ上の役割

**依存レイヤ**: 統合層（Layer 5）- オフライン解析ツール

C++実装（`crane_bag_main`実行ファイル）とPythonライブラリ（`crane_mcap_tools`パッケージで利用）の2形態で提供。

## コンポーネント

### 実行ファイル

#### crane_bag

C++ CLIツール。ROS 2環境なしでrosbagを解析可能。

### ライブラリモジュール (src/bag/)

- **BagReader**: rosbag2_cppによるデータ読み込み
- **bag_control**: control_target・planning_factors分析
- **bag_events**: ゴール・キック・ファウル等のイベント検出
- **bag_tracking**: ロボット・ボール位置追跡
- **bag_referee**: 審判コマンドの解析・遷移抽出
- **bag_survey**: 試合概要の集計

## 依存関係

- `crane_msg_wrappers`, `crane_msgs`, `crane_robot_skills`
- `robocup_ssl_msgs`, `rosbag2_cpp`, `rosidl_runtime_cpp`
- `nlohmann-json-dev`, `libgoogle-glog-dev`

## 使用方法

```bash
# Bag情報の表示
crane_bag info /path/to/rosbag_dir

# 概要サーベイ
crane_bag survey /path/to/rosbag_dir

# ボール追跡
crane_bag track /path/to/rosbag_dir --ball --interval 0.5

# ロボット追跡
crane_bag track /path/to/rosbag_dir --robot 1
crane_bag track /path/to/rosbag_dir --robot 2 --enemy --time 60.0:120.0

# イベント検出
crane_bag events /path/to/rosbag_dir
crane_bag events /path/to/rosbag_dir --type goal kick foul

# 制御ターゲット分析
crane_bag control /path/to/rosbag_dir --robot 0
crane_bag control /path/to/rosbag_dir --robot 0 --changes-only

# 審判コマンド
crane_bag referee /path/to/rosbag_dir --changes-only

# JSON出力
crane_bag info /path/to/rosbag_dir --format json
```

## 最近の開発状況

- **2026年2月（PR #1232）**: `crane_debug_tools`から分割・独立パッケージ化
- **2026年1月（PR #1196）**: C++ CLIツールとPythonライブラリの初期実装

## 関連パッケージ

- [crane_mcap_tools](./crane_mcap_tools.md) - Pythonベースの解析ツール・SVG動画生成
- [crane_msgs](./crane_msgs.md) - 解析対象のメッセージ定義
