# API リファレンス: Crane Debug Tools

このドキュメントは、crane_debug_toolsパッケージを使用または拡張する開発者向けの包括的なAPIリファレンスを提供します。

## 目次
1. [コアクラス](#core-classes)
2. [CLI インターフェース API](#cli-interface-api)
3. [シナリオシステム API](#scenario-system-api)
4. [パラメータシステム API](#parameter-system-api)
5. [拡張ポイント](#extension-points)
6. [ROS 2 統合](#ros-2-integration)
7. [エラーハンドリング](#error-handling)

## コアクラス

### SkillTesterCLI (C++)

インタラクティブCLIインターフェースのメインクラスです。

```cpp
class SkillTesterCLI : public rclcpp::Node
{
public:
    using SkillExecutionAction = crane_msgs::action::SkillExecution;
    using SkillExecutionClient = rclcpp_action::Client<SkillExecutionAction>;

    // コンストラクタ
    SkillTesterCLI();
    
    // メイン実行ループ
    void run();

private:
    // スキル実行
    void executeSkill(const std::string& skill_name, int robot_id, 
                     const std::map<std::string, std::string>& params);
    
    // コマンド処理
    void processCommand(const std::string& input);
    
    // UI ヘルパー
    void printWelcomeMessage();
    void printHelp();
    void listSkills();

    // メンバー
    SkillExecutionClient::SharedPtr client_;
    std::vector<std::string> available_skills_;
    int num_robots_;
};
```

**主要メソッド:**

#### `SkillTesterCLI()`
ROS 2ノードとアクションクライアントを初期化するコンストラクタ。

**パラメータ:** なし  
**戻り値:** なし  
**例:**
```cpp
auto node = std::make_shared<SkillTesterCLI>();
```

#### `void run()`
インタラクティブCLIループを開始します。

**パラメータ:** なし  
**戻り値:** void  
**動作:** ユーザーが終了するまでブロック  
**例:**
```cpp
node->run();
```

#### `void executeSkill(...)`
指定されたパラメータでスキルを実行します。

**パラメータ:**
- `skill_name` (const std::string&): 実行するスキルの名前
- `robot_id` (int): 対象ロボットID (0-15)
- `params` (const std::map<std::string, std::string>&): スキルパラメータ

**戻り値:** void  
**例外:** 無効なパラメータの場合に std::runtime_error  
**例:**
```cpp
std::map<std::string, std::string> params = {
    {"target_x", "1.0"},
    {"target_y", "2.0"},
    {"kick_power", "5.0"}
};
tester.executeSkill("Kick", 0, params);
```

### SkillCLI (Python)

コマンドラインスクリプトインターフェース用のPythonクラスです。

```python
class SkillCLI(Node):
    """クレーンロボットスキル用コマンドラインインターフェース"""
    
    AVAILABLE_SKILLS = [
        "Sleep", "Idle", "Kick", "Receive", "Goalie", 
        "Attacker", "SubAttacker", "StealBall", ...
    ]
    
    def __init__(self):
        """CLIノードを初期化"""
        
    def wait_for_server(self, timeout: float = 10.0) -> bool:
        """アクションサーバーが利用可能になるまで待機"""
        
    def execute_skill(self, skill_name: str, robot_id: int, 
                     parameters: Optional[Dict[str, Any]] = None) -> bool:
        """指定されたパラメータでスキルを実行"""
```

**主要メソッド:**

#### `__init__()`
ROS 2ノードとアクションクライアントを初期化します。

**パラメータ:** なし  
**例:**
```python
cli = SkillCLI()
```

#### `wait_for_server(timeout: float = 10.0) -> bool`
アクションサーバーが利用可能になるまで待機します。

**パラメータ:**
- `timeout` (float): 最大待機時間（秒）

**戻り値:** bool - サーバーが利用可能なTrue、そうでなければFalse  
**例:**
```python
if cli.wait_for_server():
    print("サーバー準備完了")
```

#### `execute_skill(...) -> bool`
スキルを実行し、完了を待ちます。

**パラメータ:**
- `skill_name` (str): スキルの名前
- `robot_id` (int): 対象ロボットID
- `parameters` (Optional[Dict[str, Any]]): スキルパラメータ

**戻り値:** bool - 実行成功時にTrue  
**例:**
```python
success = cli.execute_skill("Kick", 0, {
    "target_x": 1.0,
    "target_y": 2.0,
    "kick_power": 5.0
})
```

## CLI インターフェース API

### コマンドラインインターフェース

`crane_skill`スクリプトは統一されたコマンドラインインターフェースを提供します。

#### コマンド構造
```bash
crane_skill <command> [arguments] [options]
```

### 利用可能コマンド

#### `list`
利用可能なすべてのスキルを一覧表示します。

**構文:** `crane_skill list`  
**戻り値:** 成功時に0  
**出力:** スキル名、1行に1つ  
**例:**
```bash
$ crane_skill list
Sleep
Idle
Kick
Receive
...
```

#### `run`
単一のスキルを実行します。

**構文:** `crane_skill run <skill> <robot_id> [param:value ...]`

**パラメータ:**
- `skill` (str): スキル名（利用可能スキルリストに含まれている必要があります）
- `robot_id` (int): ロボットID (0-15)
- `param:value` (str): key:value形式のパラメータペア

**戻り値:** 成功時に0、失敗時に1  
**例:**
```bash
crane_skill run Kick 0 target_x:1.0 target_y:2.0 kick_power:5.0
```

#### `multi`
複数のロボットでスキルを実行します。

**構文:** `crane_skill multi <skill> <robot_ids> [param:value ...]`

**パラメータ:**
- `skill` (str): スキル名
- `robot_ids` (str): カンマ区切りのロボットID（例: "0,1,2"）
- `param:value` (str): パラメータペア

**戻り値:** すべてのロボットが成功した場合0、いずれかが失敗した場合1  
**例:**
```bash
crane_skill multi Attacker 0,1,2
crane_skill multi EmplaceRobot 1,2,3 target_x:1.0 target_y:0.0
```

#### `scenario`
JSONファイルからスキルシナリオを実行します。

**構文:** `crane_skill scenario <file.json>`

**パラメータ:**
- `file.json` (str): シナリオファイルへのパス

**戻り値:** すべてのスキルが成功した場合0、いずれかが失敗した場合1  
**例:**
```bash
crane_skill scenario scenarios/basic_test.json
```

## シナリオシステム API

### JSON スキーマ

シナリオファイルはこのJSONスキーマに準拠している必要があります:

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "type": "object",
  "properties": {
    "name": {
      "type": "string",
      "description": "人間が読みやすいシナリオ名"
    },
    "description": {
      "type": "string", 
      "description": "詳細なシナリオ説明"
    },
    "skills": {
      "type": "array",
      "minItems": 1,
      "items": {
        "type": "object",
        "properties": {
          "name": {
            "type": "string",
            "description": "スキル名"
          },
          "robot_id": {
            "type": "integer",
            "minimum": 0,
            "maximum": 15,
            "description": "対象ロボットID"
          },
          "parameters": {
            "type": "object",
            "description": "キーバリューペアとしてのスキルパラメータ"
          },
          "delay": {
            "type": "number",
            "minimum": 0,
            "description": "スキル完了後の遅延時間（秒）"
          },
          "description": {
            "type": "string",
            "description": "人間が読みやすいステップ説明"
          }
        },
        "required": ["name", "robot_id"]
      }
    }
  },
  "required": ["name", "skills"]
}
```

### シナリオ関数

#### `load_scenario(scenario_file: str) -> List[Dict[str, Any]]`
シナリオファイルをロードし、検証します。

**パラメータ:**
- `scenario_file` (str): JSONシナリオファイルへのパス

**戻り値:** スキル辞書のリスト  
**例外:** 無効な形式の場合にValueError  
**例:
```python
skills = load_scenario("test_scenario.json")
for skill in skills:
    print(f"スキル: {skill['name']}, ロボット: {skill['robot_id']}")
```

#### `execute_scenario(cli: SkillCLI, skills: List[Dict[str, Any]]) -> bool`
スキルのシーケンスを実行します。

**パラメータ:**
- `cli` (SkillCLI): 初期化済みCLIインスタンス
- `skills` (List[Dict]): スキル定義のリスト

**戻り値:** bool - すべてのスキルが成功した場合にTrue  
**例:**
```python
cli = SkillCLI()
skills = load_scenario("my_test.json")
success = execute_scenario(cli, skills)
```

### シナリオ検証

#### `validate_scenario_schema(scenario: Dict) -> List[str]`
JSONスキーマに対してシナリオを検証します。

**パラメータ:**
- `scenario` (Dict): パース済みシナリオ辞書

**戻り値:** 検証エラーメッセージのリスト（有効な場合は空）  
**例:
```python
with open("scenario.json") as f:
    scenario = json.load(f)

errors = validate_scenario_schema(scenario)
if errors:
    print("検証エラー:", errors)
```

## パラメータシステム API

### パラメータタイプ

システムは4つのパラメータタイプを自動検出でサポートします:

| タイプ | Pythonタイプ | ROSメッセージ | 検出パターン |
|------|-------------|-------------|-------------------|
| ブール値 | `bool` | `NamedBool` | `"true"`, `"false"` （大文字小文字区別なし） |
| 整数 | `int` | `NamedInt` | 小数点のない整数 |
| 浮動小数点 | `float` | `NamedFloat` | 小数点ありの数値 |
| 文字列 | `str` | `NamedString` | その他のすべての値 |

### パラメータ処理関数

#### `parse_parameters(param_strings: List[str]) -> Dict[str, Any]`
コマンドラインパラメータ文字列をパースします。

**パラメータ:**
- `param_strings` (List[str]): "key:value"文字列のリスト

**戻り値:** 型付き値の辞書  
**例:**
```python
params = parse_parameters([
    "target_x:1.5",      # -> {"target_x": 1.5}
    "kick_power:5.0",    # -> {"kick_power": 5.0}
    "enable_chip:true",  # -> {"enable_chip": True}
    "mode:attack"        # -> {"mode": "attack"}
])
```

#### パラメータタイプ変換

**C++実装:**
```cpp
crane_msgs::msg::NamedValueArray convertParameters(
    const std::map<std::string, std::string>& params) {
    
    crane_msgs::msg::NamedValueArray result;
    
    for (const auto& [key, value] : params) {
        if (isBooleanString(value)) {
            crane_msgs::msg::NamedBool bool_param;
            bool_param.name = key;
            bool_param.value = parseBool(value);
            result.bool_values.push_back(bool_param);
        }
        else if (isFloatString(value)) {
            crane_msgs::msg::NamedFloat float_param;
            float_param.name = key;
            float_param.value = std::stof(value);
            result.float_values.push_back(float_param);
        }
        // ... similar for int and string
    }
    
    return result;
}
```

**Python実装:
```python
def convert_parameters(parameters: Dict[str, Any]) -> NamedValueArray:
    """パラメータをROSメッセージ形式に変換"""
    
    result = NamedValueArray()
    
    for key, value in parameters.items():
        if isinstance(value, bool):
            param = NamedBool()
            param.name = key
            param.value = value
            result.bool_values.append(param)
        elif isinstance(value, int):
            param = NamedInt()
            param.name = key
            param.value = value
            result.int_values.append(param)
        # ... floatとstringも同様
    
    return result
```

### 型検出ユーティリティ

#### `isBooleanString(const std::string& value) -> bool`
ブール値を検出します。

**パラメータ:**
- `value` (const std::string&): テストする文字列

**戻り値:** bool - 文字列がブール値を表す場合にTrue  
**受け入れ可能値:** "true", "false", "TRUE", "FALSE", "True", "False"  
**例:
```cpp
assert(isBooleanString("true") == true);
assert(isBooleanString("1.5") == false);
```

#### `isFloatString(const std::string& value) -> bool`
浮動小数点値を検出します。

**パラメータ:**
- `value` (const std::string&): テストする文字列

**戻り値:** bool - 文字列が浮動小数点値を表す場合にTrue  
**例:
```cpp
assert(isFloatString("1.5") == true);
assert(isFloatString("42") == false);  // 整数、浮動小数点ではない
assert(isFloatString("abc") == false);
```

## 拡張ポイント

### カスタムスキルプラグイン

スキルプラグインインターフェースを実装してカスタムスキルハンドラーを作成します:

```cpp
class CustomSkillPlugin : public SkillPlugin {
public:
    std::string getName() const override {
        return "CustomSkill";
    }
    
    ParameterSchema getParameterSchema() const override {
        ParameterSchema schema;
        schema.addParameter("custom_param", ParameterType::FLOAT, true);
        schema.addParameter("mode", ParameterType::STRING, false);
        return schema;
    }
    
    ValidationResult validateParameters(const ParameterMap& params) const override {
        ValidationResult result;
        
        // カスタム検証ロジック
        if (params.find("custom_param") != params.end()) {
            float value = std::stof(params.at("custom_param"));
            if (value < 0.0 || value > 10.0) {
                result.addError("custom_paramは0.0から10.0の間である必要があります");
            }
        }
        
        return result;
    }
};
```

### カスタムコマンド拡張

カスタムコマンドでCLIを拡張します:

```python
class BenchmarkCommand(CommandExtension):
    """パフォーマンスベンチマーク用カスタムコマンド"""
    
    def get_command_name(self) -> str:
        return "benchmark"
    
    def get_description(self) -> str:
        return "スキルのパフォーマンスベンチマークを実行"
    
    def add_arguments(self, parser: argparse.ArgumentParser):
        parser.add_argument("skill", help="ベンチマークするスキル")
        parser.add_argument("--iterations", type=int, default=10,
                          help="反復回数")
        parser.add_argument("--robots", default="0",
                          help="カンマ区切りのロボットID")
    
    def execute(self, args: argparse.Namespace) -> int:
        # カスタムベンチマークロジック
        return self.run_benchmark(args.skill, args.iterations, args.robots)

# 拡張を登録
CLI_EXTENSIONS.register(BenchmarkCommand())
```

### シナリオプロセッサー

カスタムシナリオプロセッサーを作成します:

```python
class ConditionalScenarioProcessor:
    """条件ロジックでシナリオを処理"""
    
    def process_scenario(self, scenario: Dict) -> Dict:
        """条件実行ロジックを追加"""
        
        processed_skills = []
        for skill in scenario["skills"]:
            # 条件ロジックを追加
            if self.should_execute_skill(skill):
                processed_skills.append(skill)
        
        scenario["skills"] = processed_skills
        return scenario
    
    def should_execute_skill(self, skill: Dict) -> bool:
        """条件に基づいてスキルを実行するかどうかを判定"""
        # カスタムロジックをここに記述
        return True
```

## ROS 2 統合

### アクションインターフェース

システムは標準的なROS 2アクションパターンを使用します:

```cpp
// Action definition (from crane_msgs)
action SkillExecution {
    # ゴール
    uint8 robot_id
    string name
    NamedValueArray parameter
    ---
    # 結果  
    int16 result
    ---
    # フィードバック
    string message
}
```

### アクションクライアントの使用方法

```cpp
// アクションクライアントを作成
auto client = rclcpp_action::create_client<SkillExecutionAction>(
    node, "/simple_ai/skill_execution");

// ゴールを送信
auto goal_msg = SkillExecutionAction::Goal();
goal_msg.robot_id = robot_id;
goal_msg.name = skill_name;
goal_msg.parameter = convertParameters(parameters);

auto send_goal_options = rclcpp_action::Client<SkillExecutionAction>::SendGoalOptions();

send_goal_options.goal_response_callback = 
    [](const auto& goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(logger, "ゴールが拒否されました");
        }
    };

send_goal_options.feedback_callback = 
    [](const auto&, const auto& feedback) {
        RCLCPP_INFO(logger, "フィードバック: %s", feedback->message.c_str());
    };

send_goal_options.result_callback = 
    [](const auto& result) {
        RCLCPP_INFO(logger, "結果: %d", result.result->result);
    };

client->async_send_goal(goal_msg, send_goal_options);
```

### トピックサブスクリプション

トピックサブスクリプションでシステム状態を監視します:

```python
# ワールドモデルサブスクリプション
def world_model_callback(msg):
    """ワールドモデル更新を処理"""
    ball_position = (msg.ball_info.position.x, msg.ball_info.position.y)
    robot_positions = [(r.pose.position.x, r.pose.position.y) 
                      for r in msg.robot_info_ours]

world_model_sub = node.create_subscription(
    WorldModel, '/world_model', world_model_callback, 10)

# ロボットコマンドサブスクリプション  
def robot_commands_callback(msg):
    """ロボットコマンド出力を監視"""
    for cmd in msg.robot_commands:
        print(f"ロボット {cmd.robot_id}: ターゲット=({cmd.target_x}, {cmd.target_y})")

commands_sub = node.create_subscription(
    RobotCommands, '/robot_commands', robot_commands_callback, 10)
```

## エラーハンドリング

### 例外階層

```python
class CraneDebugError(Exception):
    """crane debug toolsの基本例外"""
    pass

class SkillExecutionError(CraneDebugError):
    """スキル実行中のエラー"""
    def __init__(self, skill_name: str, robot_id: int, message: str):
        self.skill_name = skill_name
        self.robot_id = robot_id
        super().__init__(f"スキル {skill_name} がロボット {robot_id} で失敗: {message}")

class ParameterValidationError(CraneDebugError):
    """無効なスキルパラメータ"""
    def __init__(self, parameter: str, message: str):
        self.parameter = parameter
        super().__init__(f"パラメータ {parameter}: {message}")

class ActionServerUnavailableError(CraneDebugError):
    """アクションサーバーが利用不可"""
    pass

class ScenarioValidationError(CraneDebugError):
    """無効なシナリオ形式"""
    pass
```

### エラーハンドリングパターン

```python
try:
    success = cli.execute_skill("Kick", 0, {"target_x": 1.0, "target_y": 2.0})
    if not success:
        raise SkillExecutionError("Kick", 0, "実行失敗")
        
except ActionServerUnavailableError:
    print("エラー: crane_robot_skillsアクションサーバーが利用不可")
    print("クレーンシステムが実行中であることを確認してください")
    return 1
    
except ParameterValidationError as e:
    print(f"パラメータエラー: {e}")
    return 1
    
except SkillExecutionError as e:
    print(f"実行エラー: {e}")
    return 1
```

### リターンコード

CLIツールは標準的なUnixリターンコードを使用します:

| コード | 意味 | 説明 |
|------|---------|-------------|
| 0 | 成功 | すべての操作が正常に完了 |
| 1 | 一般エラー | スキル実行失敗または無効なパラメータ |
| 2 | 使用エラー | 無効なコマンド構文または引数不足 |
| 3 | サーバーエラー | アクションサーバー利用不可または通信失敗 |
| 4 | 検証エラー | 無効なシナリオ形式またはパラメータ検証失敗 |

**使用例:**
```bash
crane_skill run Kick 0 target_x:1.0 target_y:2.0
echo $?  # リターンコードを確認 (0 = 成功、非ゼロ = エラー)
```

このAPIリファレンスは、crane_debug_toolsを使用する開発者向けの包括的なドキュメントを提供し、システムの効果的な使用と拡張を可能にします。