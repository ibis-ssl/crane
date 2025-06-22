# シナリオテスト

CraneプロジェクトのSSL (Small Size League) 対応ロボットサッカーシステムの動作テストを行うためのシナリオテストスイートです。

## 📋 テスト一覧

### ルール遵守テスト

SSL規則への準拠を確認するテストです。

| テスト名 | ファイル | 内容 |
|---------|---------|------|
| ペナルティエリア制限 | `STOP_KEEPER_AREA.py` | ゴールキーパー以外のロボットがペナルティエリアに侵入しないことを確認 |
| ボール距離制限 | `STOP_MULTIPLE_ROBOTS_BALL.py` | STOP時に複数ロボットがボールから500mm以上離れることを確認 |
| フリーキック距離 | `FREE_KICK_DISTANCE.py` | フリーキック時に相手ロボットが規定距離を保つことを確認 |
| ボール配置精度 | `BALL_PLACEMENT_ACCURACY.py` | ボール配置の精度テスト（指定位置から150mm以内） |
| ロボット数制限 | `ROBOT_COUNT_LIMIT.py` | フィールド上のロボット数制限（最大11台）遵守を確認 |

### 基本動作テスト

ロボットの基本的な動作能力を確認するテストです。

| テスト名 | ファイル | 内容 |
|---------|---------|------|
| 移動精度 | `BASIC_MOVEMENT_ACCURACY.py` | ロボットの基本移動精度テスト |
| ボール制御 | `BALL_HANDLING_CONTROL.py` | ボール保持・制御の基本動作テスト |
| フォーメーション | `FORMATION_MAINTENANCE.py` | フォーメーション維持テスト |
| 衝突回避 | `COLLISION_AVOIDANCE.py` | ロボット間衝突回避テスト |
| レフェリー応答 | `REFEREE_RESPONSE_TIME.py` | レフェリーコマンドへの応答時間テスト |

### 既存テスト

| テスト名 | ファイル | 内容 |
|---------|---------|------|
| ロボット速度制限 | `STOP_ROBOT_SPEED.py` | STOP時のロボット速度制限テスト |
| ボール回避 | `STOP_AVOID_BALL.py` | STOP時のボール回避テスト |
| ペナルティ退場 | `emit_from_penalty_01.py` | ペナルティエリアからのボール排出テスト |

## 🚀 実行方法

### ローカル実行

#### 前提条件

```bash
# ROS 2 Jazzy 環境をセットアップ
source /opt/ros/jazzy/setup.bash

# Craneワークスペースをビルド
colcon build --symlink-install
source install/local_setup.bash

# RCSTライブラリをインストール
pip install git+https://github.com/SSL-Roots/robocup_scenario_test
pip install pytest
```

#### シミュレーション環境の起動

```bash
# Docker環境でgrSimとCraneを起動
cd docker/scenario
docker compose up -d
```

#### 個別テストの実行

```bash
cd scenario_test

# 単一テストの実行
python3 STOP_ROBOT_SPEED.py

# pytestでの実行
pytest BASIC_MOVEMENT_ACCURACY.py --vision_port=10020 --logging
```

#### テストランナーを使用した実行

```bash
# グループ別実行
python3 test_runner.py --group rule_compliance
python3 test_runner.py --group basic_actions
python3 test_runner.py --group quick_smoke

# 複数テストの実行
python3 test_runner.py STOP_ROBOT_SPEED.py BASIC_MOVEMENT_ACCURACY.py

# 全テストの実行
python3 test_runner.py --group all
```

### GitHub Actions での実行

#### 自動実行

- **Pull Request**: クイック・スモークテスト（3件）が自動実行
- **Merge Group**: 全テストが自動実行

#### 手動実行

1. GitHubリポジトリの「Actions」タブを開く
2. 「scenario test (grouped)」ワークフローを選択
3. 「Run workflow」をクリック
4. テストグループを選択して実行

利用可能なテストグループ：

- `quick_smoke`: 高速実行用（3件）
- `rule_compliance`: ルール遵守テスト（5件）
- `basic_actions`: 基本動作テスト（5件）
- `all`: 全テスト（13件）

## 📊 テスト結果の確認

### コンソール出力

テスト実行後に統計情報と詳細結果が表示されます：

```
📊 シナリオテスト結果サマリー
========================================
総テスト数: 5
成功: 4 ✅
失敗: 1 ❌
成功率: 80.0%
実行時間: 45.2秒
```

### レポートファイル

`test_runner.py`を使用すると、JSON形式の詳細レポートが生成されます：

```json
{
  "total_tests": 5,
  "passed_tests": 4,
  "failed_tests": 1,
  "success_rate": 80.0,
  "total_duration": 45.2,
  "results": [...]
}
```

### GitHub Actions Summary

GitHub Actionsでの実行時は、ジョブサマリーに結果が表示されます。

## 🛠️ テストの追加・カスタマイズ

### 新しいテストの作成

1. `scenario_test/`ディレクトリに新しい`.py`ファイルを作成
2. RCSTフレームワークを使用してテストを実装
3. GitHub Actionsの設定にテストを追加

#### テンプレート

```python
import math
import time
from rcst.communication import Communication
from rcst import calc
from rcst.ball import Ball
from rcst.robot import RobotDict

def test_my_scenario(rcst_comm: Communication):
    \"\"\"新しいシナリオテスト\"\"\"
    rcst_comm.send_empty_world()

    # テストの設定
    rcst_comm.send_ball(0.0, 0.0)
    rcst_comm.send_yellow_robot(0, -1.0, 0.0, math.radians(0))

    # レフェリーコマンド
    rcst_comm.change_referee_command("FORCE_START", 1.0)

    # テスト実行
    time.sleep(3)

    # 結果の評価
    world = rcst_comm.observer.get_world()
    # ... アサーション

    assert True, "テスト条件"

if __name__ == "__main__":
    rcst_comm = Communication()
    test_my_scenario(rcst_comm)
    rcst_comm.close()
    print("MY_SCENARIO test passed")
```

### GitHub Actionsへの追加

1. `.github/workflows/scenario_test.yaml`のmatrixに追加
2. `.github/workflows/scenario_test_groups.yaml`の適切なグループに追加
3. `test_runner.py`のテストグループ定義に追加

## 🔍 トラブルシューティング

### よくある問題

#### Docker環境の問題

```bash
# Docker環境のリセット
docker compose -f docker/scenario/docker-compose.yaml down
docker compose -f docker/scenario/docker-compose.yaml up -d
```

#### RCSTライブラリの問題

```bash
# RCSTライブラリの再インストール
pip uninstall robocup-scenario-test
pip install git+https://github.com/SSL-Roots/robocup_scenario_test
```

#### ポート競合

```bash
# 使用中のポートを確認
netstat -tulpn | grep :10020
netstat -tulpn | grep :10003
```

### ログとデバッグ

#### テスト実行ログ

```bash
# 詳細ログ付きでテスト実行
pytest BASIC_MOVEMENT_ACCURACY.py -v -s --tb=long
```

#### Docker ログ

```bash
# Crane システムのログ
docker logs crane

# grSim シミュレーターのログ
docker logs grsim
```

### GitHub Actions のデバッグ

失敗したテストの動画ログが自動的にartifactとしてアップロードされます。

## 📚 関連ドキュメント

- [Crane アーキテクチャ](../docs/)
- [RCST フレームワーク](https://github.com/SSL-Roots/robocup_scenario_test)
- [SSL 規則](https://robocup-ssl.github.io/ssl-rules/)
- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/)

## 🤝 貢献

新しいテストシナリオの追加や既存テストの改善は歓迎します。Pull Requestを作成する前に：

1. テストが適切に動作することを確認
2. 必要に応じてドキュメントを更新
3. コミットメッセージは日本語で記述

## 📄 ライセンス

このテストスイートはCraneプロジェクトと同じライセンスの下で配布されます。
