# トラブルシューティングガイド: Crane Debug Tools

このガイドは、crane_debug_toolsの使用時によく発生する問題の診断と解決に役立ちます。

## 目次
1. [クイック診断](#クイック診断)
2. [よくある問題](#よくある問題)
3. [システム依存関係](#システム依存関係)
4. [ネットワークと通信](#ネットワークと通信)
5. [パフォーマンス問題](#パフォーマンス問題)
6. [統合問題](#統合問題)
7. [高度なデバッグ](#高度なデバッグ)

## クイック診断

### ヘルスチェックスクリプト

システムステータスを確認するための簡単なヘルスチェックを作成します：

```bash
#!/bin/bash
# crane_debug_health_check.sh

echo "=== Crane Debug Tools Health Check ==="

# Check ROS 2 environment
echo "1. Checking ROS 2 environment..."
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ ROS_DISTRO not set. Source your ROS 2 installation."
    exit 1
else
    echo "✅ ROS_DISTRO: $ROS_DISTRO"
fi

# Check workspace
echo "2. Checking workspace..."
if [ -z "$AMENT_PREFIX_PATH" ]; then
    echo "❌ Workspace not sourced. Run: source install/local_setup.bash"
    exit 1
else
    echo "✅ Workspace sourced"
fi

# Check crane_debug_tools package
echo "3. Checking crane_debug_tools package..."
if ! ros2 pkg list | grep -q crane_debug_tools; then
    echo "❌ crane_debug_tools package not found. Build the package first."
    exit 1
else
    echo "✅ crane_debug_tools package available"
fi

# Check crane_skill command
echo "4. Checking crane_skill command..."
if ! command -v crane_skill &> /dev/null; then
    echo "❌ crane_skill command not found. Check installation."
    exit 1
else
    echo "✅ crane_skill command available"
fi

# Check action server
echo "5. Checking action server..."
if ! timeout 5s ros2 action list | grep -q skill_execution; then
    echo "⚠️  Action server not available. Start crane_robot_skills."
else
    echo "✅ Action server available"
fi

# Check basic functionality
echo "6. Testing basic functionality..."
if timeout 10s crane_skill list > /dev/null 2>&1; then
    echo "✅ Basic functionality working"
else
    echo "❌ Basic functionality test failed"
fi

echo "=== Health check complete ==="
```

### クイックテストコマンド

```bash
# 基本的なCLI機能をテスト
crane_skill list

# ROS 2接続をテスト
ros2 node list
ros2 action list

# スキル実行をテスト（サーバーが利用可能な場合）
crane_skill run Sleep 0 duration:0.1
```

## よくある問題

### 問題1: `crane_skill`コマンドが見つからない

**症状:**
```bash
$ crane_skill list
bash: crane_skill: command not found
```

**原因と解決策:**

#### 原因1: ワークスペースがsourceされていない
```bash
# 解決策: ワークスペースをsourceする
cd /path/to/your/workspace
source install/local_setup.bash

# 永続的な設定のために~/.bashrcに追加
echo "source /path/to/your/workspace/install/local_setup.bash" >> ~/.bashrc
```

#### 原因2: パッケージがビルドされていない
```bash
# 解決策: パッケージをビルドする
colcon build --packages-select crane_debug_tools

# ビルドが失敗した場合、クリーンして再ビルド
colcon build --packages-select crane_debug_tools --cmake-clean-cache
```

#### 原因3: スクリプトが実行可能でない
```bash
# 解決策: スクリプトを実行可能にする
chmod +x install/crane_debug_tools/lib/crane_debug_tools/crane_skill

# スクリプトの場所を確認
find install/ -name crane_skill -type f
```

#### 原因4: Pythonパスの問題
```bash
# 解決策: Pythonパスと依存関係を確認
python3 -c "import rclpy; print('rclpy OK')"
python3 -c "import crane_msgs; print('crane_msgs OK')"

# インポートが失敗した場合、パッケージのインストールを確認
ros2 pkg list | grep crane_msgs
```

### 問題2: アクションサーバーが利用できない

**症状:**
```bash
$ crane_skill run Sleep 0 duration:1.0
Error: Action server not available after 10 seconds
```

**診断手順:**

#### 手順1: crane_robot_skillsが実行されているか確認
```bash
# craneノードを確認
ros2 node list | grep crane

# 期待される出力には以下のようなノードが含まれるはずです:
# /crane_robot_skills
# /crane_world_model_publisher
# /crane_session_controller
```

#### 手順2: アクションサーバーを具体的に確認
```bash
# すべてのアクションサーバーをリスト
ros2 action list

# スキル実行アクションを探す
ros2 action list | grep skill_execution

# アクションサーバー情報を確認
ros2 action info /simple_ai/skill_execution
```

#### 手順3: システム起動を確認
```bash
# アクションサーバーが見つからない場合、craneシステムを起動
ros2 launch crane_bringup crane.launch.py

# または、robot skillsを具体的に開始
ros2 run crane_robot_skills crane_robot_skills_node
```

**解決策:**

#### 解決策1: craneシステムを起動
```bash
# 標準起動
ros2 launch crane_bringup crane.launch.py

# シミュレーション付き
ros2 launch crane_bringup crane.launch.py sim:=true

# 起動出力でエラーを確認
```

#### 解決策2: 依存関係を確認
```bash
# すべてのcraneパッケージがビルドされていることを確認
colcon list --packages-up-to crane_robot_skills

# 不足しているパッケージをビルド
colcon build --packages-up-to crane_robot_skills
```

#### 解決策3: ネットワーク設定
```bash
# ROSドメインIDを確認（すべてのノードで一致する必要があります）
echo $ROS_DOMAIN_ID

# 複数のマシンを使用している場合、ネットワークを確認
ros2 doctor

# 必要に応じてネットワーク設定をリセット
unset ROS_DOMAIN_ID
ros2 daemon stop
ros2 daemon start
```

### 問題3: パラメータ型エラー

**症状:**
```bash
$ crane_skill run Kick 0 target_x:1.0 target_y:2.0 kick_power:abc
Error: Invalid parameter type for kick_power: expected number
```

**一般的なパラメータ問題:**

#### 問題: 文字列が数値として解釈される
```bash
# 間違い: 浮動小数点の小数点がない
crane_skill run Kick 0 kick_power:5

# 正しい: 明示的な小数点
crane_skill run Kick 0 kick_power:5.0
```

#### 問題: ブール形式
```bash
# 間違い: 数値ブール
crane_skill run EmplaceRobot 0 precise_positioning:1

# 正しい: 文字列ブール
crane_skill run EmplaceRobot 0 precise_positioning:true
```

#### 問題: 必須パラメータが不足
```bash
# 必須パラメータのスキルドキュメントを確認
ros2 interface show crane_msgs/action/SkillExecution

# 各スキルに適切なパラメータを使用
crane_skill run Kick 0 target_x:1.0 target_y:2.0  # kick_powerが必要な場合があります
```

**パラメータデバッグ:**

```bash
# パラメータ解析をテスト
crane_skill run Sleep 0 duration:1.0  # シンプルな浮動小数点パラメータ
crane_skill run Idle 0                # パラメータなし

# パラメータ形式を確認
echo "Testing parameter: key:value format required"
```

### 問題4: ロボットIDが範囲外

**症状:**
```bash
$ crane_skill run Kick 16 target_x:1.0
Error: Robot ID must be between 0 and 15
```

**解決策:**

```bash
# 有効なロボットID（0-15）を使用
crane_skill run Kick 0 target_x:1.0    # 有効
crane_skill run Kick 15 target_x:1.0   # 有効（最大）

# マルチロボットの場合、ID形式を確認
crane_skill multi Idle 0,1,2           # 有効な形式
crane_skill multi Idle "0, 1, 2"       # 間違い: スペースは許可されていません
```

### 問題5: スキル実行タイムアウト

**症状:**
```bash
$ crane_skill run Kick 0 target_x:1.0 target_y:2.0
Executing skill 'Kick' on robot 0...
Goal accepted by server, executing...
Skill execution timed out or failed
```

**診断手順:**

#### ロボットステータスを確認
```bash
# ワールドモデルを監視
ros2 topic echo /world_model --once

# ロボット位置を確認
ros2 topic echo /world_model | grep -A 20 robot_info_ours

# ロボットコマンドを監視
ros2 topic echo /robot_commands
```

#### システムステータスを確認
```bash
# システムヘルスを監視
ros2 topic list | grep crane
ros2 node list | grep crane

# エラーメッセージを確認
ros2 topic echo /rosout | grep ERROR
```

**解決策:**

#### 解決策1: ロボットの可視性を確認
```bash
# ワールドモデルでロボットが検出されているか確認
ros2 topic echo /world_model --once | grep robot_info_ours

# ロボットが検出されない場合、ビジョンシステムを確認
ros2 topic echo /vision_data
```

#### 解決策2: シミュレーション環境を確認
```bash
# シミュレーションを使用している場合、grSimが実行されていることを確認
# 必要に応じてシミュレーション環境を起動
ros2 launch crane_bringup crane.launch.py sim:=true

# シミュレーショントピックを確認
ros2 topic list | grep sim
```

#### 解決策3: まずシンプルなスキルを試す
```bash
# 複雑な実行を必要としないスキルでテスト
crane_skill run Idle 0
crane_skill run Sleep 0 duration:1.0

# その後、移動スキルに進む
crane_skill run EmplaceRobot 0 target_x:0.0 target_y:0.0
```

## システム依存関係

### ROS 2依存関係

#### ROS 2パッケージの不足
```bash
# 必要なパッケージを確認
rosdep check --from-paths src --ignore-src

# 不足している依存関係をインストール
rosdep install --from-paths src --ignore-src -y

# crane_msgsが利用可能であることを確認
ros2 interface list | grep crane_msgs
```

#### バージョン互換性
```bash
# ROS 2バージョンを確認
ros2 --version

# パッケージバージョンを確認
ros2 pkg xml crane_debug_tools | grep version

# バージョン競合を確認
colcon list --packages-up-to crane_debug_tools
```

### Python依存関係

#### Pythonパッケージの不足
```bash
# Pythonインポートをテスト
python3 -c "
import rclpy
import crane_msgs
from crane_msgs.action import SkillExecution
from crane_msgs.msg import NamedValueArray
print('All imports successful')
"

# インポートが失敗した場合、パッケージを再ビルド
colcon build --packages-select crane_msgs crane_debug_tools
```

#### Pythonパスの問題
```bash
# PythonパスがROSパッケージを含んでいるか確認
python3 -c "import sys; print('\n'.join(sys.path))"

# PYTHONPATHがinstallディレクトリを含んでいることを確認
echo $PYTHONPATH | grep install

# PYTHONPATHが間違っている場合、ワークスペースをsource
source install/local_setup.bash
```

### ビルド依存関係

#### ビルドツールの不足
```bash
# 必要なビルドツールをインストール
sudo apt update
sudo apt install build-essential cmake python3-colcon-common-extensions

# C++開発用
sudo apt install g++ gdb

# Python開発用
sudo apt install python3-dev python3-pip
```

#### CMakeの問題
```bash
# ビルド問題が継続する場合、CMakeキャッシュをクリーン
colcon build --packages-select crane_debug_tools --cmake-clean-cache

# 強制再ビルド
rm -rf build/ install/
colcon build --packages-select crane_debug_tools
```

## ネットワークと通信

### ROS 2通信の問題

#### ドメインIDの競合
```bash
# 現在のドメインIDを確認
echo $ROS_DOMAIN_ID

# 競合が存在する場合、一意のドメインIDを設定
export ROS_DOMAIN_ID=42

# 新しいドメインでROSデーモンをリセット
ros2 daemon stop
ros2 daemon start

# 通信をテスト
ros2 node list
```

#### マルチキャストの問題
```bash
# マルチキャスト接続をテスト
ros2 multicast send

# 別のターミナルで
ros2 multicast receive

# マルチキャストが失敗した場合、ネットワーク設定を確認
ip route show
```

#### Network Interface Problems
```bash
# ネットワークインターフェースを確認
ip addr show

# ROS 2ディスカバリをテスト
ros2 doctor

# 複数のインターフェースを使用している場合、一つを指定
export ROS_NETWORK_INTERFACE=eth0
```

### Action Communication

#### Action Server Connection
```bash
# アクションサーバーステータスを監視
watch "ros2 action list | grep skill_execution"

# アクションサーバーノードを確認
ros2 node info /crane_robot_skills

# アクションサーバーを直接テスト
ros2 action send_goal /simple_ai/skill_execution crane_msgs/action/SkillExecution "{robot_id: 0, name: 'Sleep', parameter: {float_values: [{name: 'duration', value: 1.0}]}}"
```

#### Action Client Debugging
```bash
# アクションクライアントのデバッグ出力を有効化
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"
export RCUTILS_LOGGING_USE_STDOUT=1
export RCUTILS_LOGGING_BUFFERED_STREAM=1

# デバッグ出力で実行
crane_skill run Sleep 0 duration:1.0
```

## パフォーマンス問題

### 実行が遅い

#### 症状
- コマンドの完了に時間がかかる
- アクションサーバーからの応答が遅い
- CPUまたはメモリ使用量が高い

#### 診断手順
```bash
# システムリソースを監視
htop
iotop

# ROS 2パフォーマンスを確認
ros2 topic hz /world_model
ros2 topic bw /robot_commands

# 実行時間をプロファイル
time crane_skill run Sleep 0 duration:1.0
```

#### 解決策

**ROS 2設定を最適化:**
```bash
# より高速DDS実装を使用
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp

# 必要に応じてQoS設定を調整
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

# ディスカバリトラフィックを減らす
export ROS_STATIC_PEERS=localhost
```

**システム最適化:**
```bash
# プロセス優先度を上げる
nice -n -10 crane_skill run Kick 0 target_x:1.0 target_y:2.0

# 競合プロセスを監視して終了
ps aux | grep -v grep | grep ros
```

### メモリの問題

#### メモリリーク
```bash
# 時間経過でメモリ使用量を監視
watch "ps aux | grep crane_skill | grep -v grep"

# メモリプロファイリングツールを使用
valgrind --tool=memcheck --leak-check=full crane_skill run Sleep 0 duration:1.0
```

#### 大きなシナリオファイル
```bash
# 大きなシナリオファイルの場合、小さなチャンクに分割
# シナリオ実行中のメモリを監視
watch "free -h && echo '---' && ps aux | grep crane_skill"
```

## 統合の問題

### CI/CD統合の問題

#### パイプラインの失敗
```bash
# CI/CDコマンドをローカルでテスト
export CI=true
crane_skill scenario scenarios/basic_test.json

# 終了コードを確認
crane_skill run Sleep 0 duration:1.0
echo "Exit code: $?"
```

#### ヘッドレス環境
```bash
# GUI依存関係がないことを確認
export DISPLAY=""
export HEADLESS=1

# 最小限の環境でテスト
docker run --rm -v $(pwd):/workspace ubuntu:22.04 bash -c "
  cd /workspace && 
  source /opt/ros/jazzy/setup.bash && 
  crane_skill list
"
```

### Docker統合

#### コンテナの問題
```bash
# デバッグツール付きコンテナをビルド
FROM ros:jazzy
COPY . /workspace
WORKDIR /workspace
RUN colcon build --packages-select crane_debug_tools
RUN echo 'source install/local_setup.bash' >> ~/.bashrc

# コンテナ内でテスト
docker run -it --rm my_crane_image crane_skill list
```

#### Docker内のネットワーク
```bash
# ROS 2用のホストネットワーキングを使用
docker run --network host my_crane_image

# またはコンテナネットワークを設定
docker run -e ROS_DOMAIN_ID=0 -p 11311:11311 my_crane_image
```

## 高度なデバッグ

### デバッグログ

#### 詳細なログを有効化
```bash
# デバッグ用のログレベルを設定
export RCUTILS_LOGGING_SEVERITY_THRESHOLD=DEBUG

# 特定のロガーを有効化
export RCUTILS_LOGGING_CONFIG_FILE=/path/to/logging.conf

# ログ設定を作成
cat > logging.conf << EOF
logger_names=crane_debug_tools
logger.crane_debug_tools.level=DEBUG
EOF
```

#### ログ分析
```bash
# ログをファイルにキャプチャ
crane_skill run Kick 0 target_x:1.0 2>&1 | tee debug.log

# ログパターンを分析
grep ERROR debug.log
grep WARNING debug.log
grep "skill" debug.log
```

### コアダンプとクラッシュ

#### コアダンプを有効化
```bash
# コアダンプを有効化
ulimit -c unlimited

# コアダンプパターンを設定
echo "/tmp/core.%e.%p" | sudo tee /proc/sys/kernel/core_pattern

# コアダンプ収集で実行
crane_skill run Kick 0 target_x:1.0

# コアダンプを分析（クラッシュが発生した場合）
gdb crane_skill_cli core.crane_skill_cli.1234
```

#### GDBでのデバッグ
```bash
# デバッガー下で実行
gdb --args crane_skill run Kick 0 target_x:1.0

# ブレークポイントを設定して分析
(gdb) break main
(gdb) run
(gdb) backtrace
```

### ネットワークデバッグ

#### パケット分析
```bash
# ROS 2トラフィックをキャプチャ
sudo tcpdump -i any -w ros2_traffic.pcap port 7400-7500

# Wiresharkで分析（利用可能な場合）
wireshark ros2_traffic.pcap

# またはテキストツールで分析
tcpdump -r ros2_traffic.pcap -A | grep skill_execution
```

#### DDSデバッグ
```bash
# DDSデバッグを有効化
export CYCLONEDX_ENABLE_LOGGING=1
export CYCLONEDX_LOG_LEVEL=finest

# DDS設定を確認
ros2 doctor --report
```

### カスタムデバッグツール

#### デバッグスクリプトを作成
```bash
#!/bin/bash
# debug_crane_skill.sh

echo "=== Debug Information ==="
echo "Date: $(date)"
echo "User: $(whoami)"
echo "Working Directory: $(pwd)"
echo "ROS_DISTRO: $ROS_DISTRO"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"

echo -e "\n=== Node List ==="
ros2 node list

echo -e "\n=== Action List ==="
ros2 action list

echo -e "\n=== Topic List ==="
ros2 topic list | grep crane

echo -e "\n=== Skill Execution Test ==="
crane_skill run Sleep 0 duration:0.5

echo -e "\n=== Debug Complete ==="
```

#### 監視スクリプト
```bash
#!/bin/bash
# monitor_crane_debug.sh

echo "Monitoring crane debug tools..."
while true; do
    echo "$(date): Checking system status"
    
    # アクションサーバーを確認
    if ros2 action list | grep -q skill_execution; then
        echo "✅ Action server available"
    else
        echo "❌ Action server not available"
    fi
    
    # メモリ使用量を確認
    echo "Memory: $(free -h | grep Mem | awk '{print $3 "/" $2}')"
    
    # CPU使用量を確認
    echo "CPU: $(top -bn1 | grep "Cpu(s)" | sed "s/.*, *\([0-9.]*\)%* id.*/\1/" | awk '{print 100 - $1"%"}')"
    
    sleep 10
done
```

このトラブルシューティングガイドは、よくある問題とその解決策を包括的にカバーし、ユーザーがcrane_debug_toolsの問題を効果的に診断し、解決できるように支援します。