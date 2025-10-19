# Crane - RoboCup Small Size League (SSL) AI Framework

[ibis-ssl](https://ibis-ssl.github.io/ibis_documentation/)による高性能な自律ロボットサッカーAIシステム

## 概要

CraneはROS 2 Jazzyベースの自律ロボティクスシステムで、RoboCup Small Size League (SSL)競技向けに設計されています。小型自律ロボットチームのサッカー試合を制御するAIフレームワークです。

## セットアップ

詳細な環境構築手順は[docs/setup.md](docs/setup.md)を参照してください。

**クイックスタート:**

```bash
# 依存関係のインストール
vcs import src < src/crane/dependency_jazzy.repos
rosdep install -riy --from-paths src

# ビルド
colcon build --symlink-install
source install/local_setup.bash
```

**必要要件:** Ubuntu 24.04 LTS + ROS 2 Jazzy

## 使用方法

詳細な起動方法とパラメータ設定は[docs/setup.md](docs/setup.md)を参照してください。

**クイックスタート:**

```bash
# シミュレーション環境での起動
ros2 launch crane_bringup crane.launch.xml sim:=true

# 実機環境での起動
ros2 launch crane_bringup crane.launch.xml sim:=false
```

## 開発

- **コード規約**: C++20標準準拠、ament_cmake_auto
- **テスト**: GTest（ユニット）+ Python RCST（統合）
- **詳細ドキュメント**: [docs/](docs/)ディレクトリを参照

## ドキュメント

- **[docs/setup.md](docs/setup.md)**: 環境構築とシステム起動
- **[docs/index.md](docs/index.md)**: アーキテクチャと設計概要
- **[docs/skill.md](docs/skill.md)**: スキルシステム実装ガイド
- **[docs/ball_tracking_system.md](docs/ball_tracking_system.md)**: EKFボール追跡システム
- **[docs/world_model_wrapper.md](docs/world_model_wrapper.md)**: WorldModelWrapper使用ガイド

## 関連リンク

- [ibis-ssl公式ドキュメント](https://ibis-ssl.github.io/ibis_documentation/)
- [RoboCup SSL](https://ssl.robocup.org/)
- [grSim シミュレータ](https://github.com/RoboCup-SSL/grSim)
