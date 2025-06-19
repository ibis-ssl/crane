# 設計概要: Crane Debug Tools

このドキュメントは、crane_debug_toolsパッケージの設計思想、アーキテクチャ、および技術的決定事項の包括的な概要を提供します。

## 設計思想

### 基本原則

1. **シンプル第一**: 複雑なモノリシックなインターフェースよりもシンプルで組み合わせ可能なツールを優先
2. **自動化対応**: すべての機能がスクリプト化および自動化可能であること
3. **開発者中心**: 開発者の生産性とワークフロー効率を最適化
4. **プラットフォーム非依存**: 異なる開発環境間で一貫して動作
5. **パフォーマンス指向**: リソース使用量と実行オーバーヘッドを最小化

### 設計目標

- **Qt依存関係の置き換え**: 重いGUIフレームワーク依存関係を排除
- **リモート開発の有効化**: SSHおよびリモート接続での開発をサポート
- **テストワークフローの改善**: 自動化および回帰テスト用のより良いツールを提供
- **マルチロボット重視**: 協調マルチロボットテストのネイティブサポート
- **CI/CD統合**: 継続的インテグレーションワークフローのファーストクラスサポート

## アーキテクチャ概要

### システムアーキテクチャ

```
┌─────────────────────────────────────────────────────────────┐
│                    crane_debug_tools                        │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌─────────────────┐    ┌─────────────────┐               │
│  │   CLI Interface │    │  Web Interface  │               │
│  │                 │    │   (Future)      │               │
│  │ • Interactive   │    │ • Browser-based │               │
│  │ • Batch Scripts │    │ • Real-time viz │               │
│  │ • Automation    │    │ • Remote access │               │
│  └─────────────────┘    └─────────────────┘               │
│           │                       │                        │
│           └───────────┬───────────┘                        │
│                       │                                    │
│  ┌─────────────────────────────────────────────────────────┤
│  │              Core Engine                                │
│  │                                                         │
│  │ • Skill Execution Management                            │
│  │ • Parameter Type Resolution                             │
│  │ • Multi-Robot Coordination                              │
│  │ • Scenario Processing                                   │
│  │ • Error Handling & Validation                           │
│  └─────────────────────────────────────────────────────────┤
│                       │                                    │
├───────────────────────┼────────────────────────────────────┤
│                  ROS 2 Layer                               │
│                                                             │
│ ┌─────────────────┐   │   ┌─────────────────┐               │
│ │ Action Clients  │   │   │   Subscribers   │               │
│ │                 │   │   │                 │               │
│ │ • SkillExecution│───┼───│ • WorldModel    │               │
│ │ • Goal/Feedback │   │   │ • RobotCommands │               │
│ │ • Result Handle │   │   │ • Status Topics │               │
│ └─────────────────┘   │   └─────────────────┘               │
└───────────────────────┼────────────────────────────────────┘
                        │
        ┌───────────────┼───────────────┐
        │           ROS 2 Middleware    │
        │                               │
        │ • Action Server Communication │
        │ • Topic Subscription          │
        │ • Parameter Management        │
        │ • Node Lifecycle              │
        └───────────────────────────────┘
                        │
        ┌───────────────┼───────────────┐
        │         crane_robot_skills    │
        │                               │
        │ • Skill Implementations       │
        │ • Robot Command Generation    │
        │ • World Model Integration     │
        │ • Behavior State Machines     │
        └───────────────────────────────┘
```

### コンポーネントアーキテクチャ

#### 1. CLIインターフェース層

**対話型CLI (`crane_skill_cli`)**
- フィードバック付きリアルタイムスキル実行
- タブ補完とコマンド履歴
- マルチセッションサポート
- エラー報告とステータス表示

**バッチCLI (`crane_skill`)**
- 単一コマンド実行
- シナリオファイル処理
- マルチロボット協調
- リターンコードベースの自動化

#### 2. コアエンジン

**スキル実行マネージャー**
```cpp
class SkillExecutionManager {
    // アクションクライアントのライフサイクルを管理
    // ゴール送信と結果処理を処理
    // フィードバック集約を提供
    // タイムアウトと再試行ロジックを実装
};
```

**パラメータタイプシステム**
```cpp
// パラメータの自動タイプ解決
struct ParameterResolver {
    static NamedValueArray resolveParameters(
        const std::map<std::string, std::string>& params);
    
    // タイプ検出階層:
    // 1. Boolean (true/false)
    // 2. Integer (整数)
    // 3. Float (小数)
    // 4. String (その他すべて)
};
```

**マルチロボットコーディネーター**
```python
class MultiRobotCoordinator:
    """複数ロボット間での同時スキル実行を管理"""
    
    def execute_parallel(self, skill_name: str, robot_ids: List[int], 
                        parameters: Dict[str, Any]) -> bool:
        # 同期を伴う並列実行
        # 失敗処理と部分成功報告
        # 進捗追跡とステータス集約
```

#### 3. シナリオシステム

**JSONスキーマ**
```json
{
  "type": "object",
  "properties": {
    "name": {"type": "string"},
    "description": {"type": "string"},
    "skills": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "name": {"type": "string"},
          "robot_id": {"type": "integer", "minimum": 0, "maximum": 15},
          "parameters": {"type": "object"},
          "delay": {"type": "number", "minimum": 0},
          "description": {"type": "string"}
        },
        "required": ["name", "robot_id"]
      }
    }
  },
  "required": ["name", "skills"]
}
```

**実行エンジン**
```python
class ScenarioExecutor:
    """スキルシナリオを処理および実行"""
    
    def validate_scenario(self, scenario: Dict) -> List[str]:
        # JSONスキーマ検証
        # スキル名検証
        # パラメータタイプチェック
        # ロボットID境界チェック
    
    def execute_scenario(self, scenario: Dict) -> ExecutionResult:
        # 順次スキル実行
        # スキル間の遅延処理
        # 進捗報告
        # エラー回復戦略
```

## 技術的設計決定

### 1. CLI第一アプローチ

**決定**: グラフィカルインターフェースよりもコマンドラインインターフェースを優先

**根拠**:
- **リモート開発**: SSHフレンドリーな開発ワークフロー
- **自動化**: スクリプトやCI/CDシステムとの簡単な統合
- **パフォーマンス**: GUIアプリケーションよりも低いリソース消費
- **一貫性**: 異なるプラットフォーム間での統一された体験
- **バージョン管理**: シナリオファイルをgitで追跡可能

**トレードオフ**:
- GUIと比較して視覚的フィードバックが少ない
- グラフィカルインターフェースに慣れたユーザーの学習曲線
- リアルタイム可視化機能の制限

### 2. タイプセーフパラメータシステム

**決定**: 自動パラメータタイプ検出と検証を実装

**実装**:
```cpp
void addParameter(const std::string& key, const std::string& value) {
    // タイプ検出カスケード
    if (isBooleanString(value)) {
        goal_msg.parameter.bool_values.push_back(
            createNamedBool(key, parseBool(value)));
    } else if (isIntegerString(value)) {
        goal_msg.parameter.int_values.push_back(
            createNamedInt(key, parseInt(value)));
    } else if (isFloatString(value)) {
        goal_msg.parameter.float_values.push_back(
            createNamedFloat(key, parseFloat(value)));
    } else {
        goal_msg.parameter.string_values.push_back(
            createNamedString(key, value));
    }
}
```

**利点**:
- **タイプセーフティ**: ランタイムタイプエラーを防止
- **ユーザー利便性**: 文字列入力からの自動変換
- **検証**: パラメータ問題の早期検出
- **互換性**: 既存のcrane_msgs構造との動作

### 3. シナリオベーステスト

**決定**: JSONベースのテストシナリオ定義

**スキーマ設計**:
```json
{
  "skills": [
    {
      "name": "EmplaceRobot",
      "robot_id": 0,
      "parameters": {
        "target_x": 1.0,
        "target_y": 2.0,
        "target_theta": 0.5
      },
      "delay": 2.0,
      "description": "Position robot at starting location"
    }
  ]
}
```

**利点**:
- **再現性**: シナリオはバージョン管理と共有が可能
- **複雑さ管理**: 複雑なテストを管理可能なステップに分解
- **ドキュメント化**: 説明付きの自己文書化テストケース
- **自動化**: CI/CDシステムとの簡単な統合

### 4. マルチロボット協調

**決定**: 並列マルチロボット操作のネイティブサポート

**実装戦略**:
```python
async def execute_multi_robot_skill(skill_name: str, robot_ids: List[int], 
                                   parameters: Dict[str, Any]) -> Dict[int, bool]:
    """複数ロボットでスキルを並列実行"""
    
    # 各ロボットのアクションクライアントを作成
    clients = {robot_id: create_action_client(robot_id) for robot_id in robot_ids}
    
    # 並列でゴールを送信
    futures = {}
    for robot_id, client in clients.items():
        goal = create_goal(skill_name, robot_id, parameters)
        futures[robot_id] = client.send_goal_async(goal)
    
    # すべての結果を待機
    results = {}
    for robot_id, future in futures.items():
        try:
            result = await asyncio.wait_for(future, timeout=30.0)
            results[robot_id] = result.success
        except asyncio.TimeoutError:
            results[robot_id] = False
    
    return results
```

**利点**:
- **効率性**: 並列実行により総テスト時間を短縮
- **現実性**: 実際のマルチロボット協調シナリオをテスト
- **スケーラビリティ**: 様々な数のロボットでのテストをサポート
- **失敗分離**: 個々のロボットの失敗がテスト全体を停止させない

## エラーハンドリングと検証

### パラメータ検証

```cpp
class ParameterValidator {
public:
    static ValidationResult validate(const std::string& skill_name,
                                   const ParameterMap& parameters) {
        auto schema = getSkillSchema(skill_name);
        ValidationResult result;
        
        for (const auto& [key, value] : parameters) {
            if (!schema.hasParameter(key)) {
                result.addWarning("Unknown parameter: " + key);
                continue;
            }
            
            auto expected_type = schema.getParameterType(key);
            if (!isValidType(value, expected_type)) {
                result.addError("Invalid type for " + key + 
                              ": expected " + typeToString(expected_type));
            }
        }
        
        // 必須パラメータをチェック
        for (const auto& required_param : schema.getRequiredParameters()) {
            if (parameters.find(required_param) == parameters.end()) {
                result.addError("Missing required parameter: " + required_param);
            }
        }
        
        return result;
    }
};
```

### 実行エラーハンドリング

```python
class ExecutionErrorHandler:
    """スキル実行のための包括的エラーハンドリング"""
    
    def handle_execution_error(self, error: Exception, context: ExecutionContext) -> RecoveryAction:
        if isinstance(error, ActionServerUnavailableError):
            return RecoveryAction.RETRY_WITH_BACKOFF
        elif isinstance(error, ParameterValidationError):
            return RecoveryAction.ABORT_WITH_MESSAGE
        elif isinstance(error, TimeoutError):
            return RecoveryAction.RETRY_ONCE
        else:
            return RecoveryAction.ABORT_AND_LOG
```

## パフォーマンス考慮事項

### リソース使用量最適化

**メモリ管理**:
- スキルスキーマの遅延読み込み
- アクションクライアントの接続プーリング
- 制限されたログバッファ
- 完了した操作の自動クリーンアップ

**実行効率**:
```cpp
class SkillExecutionPool {
    // 接続再利用
    std::unordered_map<uint8_t, ActionClient::SharedPtr> client_pool_;
    
    // バッチ操作
    void executeBatch(const std::vector<SkillRequest>& requests) {
        // クライアント作成を最小化するためにロボットIDでグループ化
        auto grouped = groupByRobotId(requests);
        
        // スレッドプールでの並列実行
        std::vector<std::future<Result>> futures;
        for (const auto& [robot_id, robot_requests] : grouped) {
            futures.push_back(std::async(std::launch::async,
                [this, robot_id, robot_requests]() {
                    return executeRobotBatch(robot_id, robot_requests);
                }));
        }
        
        // 結果を収集
        for (auto& future : futures) {
            future.wait();
        }
    }
};
```

### スケーラビリティ設計

**並行実行**:
- 非同期アクションクライアント操作
- スレッドセーフな結果集約
- ノンブロッキングユーザーインターフェース更新
- 設定可能なタイムアウトと再試行ポリシー

**大規模テスト**:
```python
class ScalabilityManager:
    """大規模ロボットチームの実行スケーリングを管理"""
    
    def __init__(self, max_concurrent_robots: int = 16):
        self.semaphore = asyncio.Semaphore(max_concurrent_robots)
        self.execution_queue = asyncio.Queue()
    
    async def execute_with_scaling(self, requests: List[SkillRequest]) -> List[Result]:
        # リソース枯渇を防ぐために並行実行を制限
        async with self.semaphore:
            return await self._execute_batch(requests)
```

## 拡張性とプラグインアーキテクチャ

### スキルプラグインシステム

```cpp
class SkillPlugin {
public:
    virtual ~SkillPlugin() = default;
    virtual std::string getName() const = 0;
    virtual ParameterSchema getParameterSchema() const = 0;
    virtual ValidationResult validateParameters(const ParameterMap& params) const = 0;
    virtual ExecutionHints getExecutionHints() const { return {}; }
};

class SkillRegistry {
    std::unordered_map<std::string, std::unique_ptr<SkillPlugin>> plugins_;
    
public:
    void registerPlugin(std::unique_ptr<SkillPlugin> plugin) {
        plugins_[plugin->getName()] = std::move(plugin);
    }
    
    const SkillPlugin* getPlugin(const std::string& skill_name) const {
        auto it = plugins_.find(skill_name);
        return it != plugins_.end() ? it->second.get() : nullptr;
    }
};
```

### カスタムコマンド拡張

```python
class CommandExtension:
    """カスタムコマンド拡張のベースクラス"""
    
    def get_command_name(self) -> str:
        raise NotImplementedError
    
    def get_description(self) -> str:
        raise NotImplementedError
    
    def add_arguments(self, parser: argparse.ArgumentParser):
        raise NotImplementedError
    
    def execute(self, args: argparse.Namespace) -> int:
        raise NotImplementedError

# 拡張の例
class BenchmarkExtension(CommandExtension):
    def get_command_name(self) -> str:
        return "benchmark"
    
    def execute(self, args: argparse.Namespace) -> int:
        # カスタムベンチマークロジック
        return self.run_performance_tests(args.skill_name, args.iterations)
```

## 将来のアーキテクチャ考慮事項

### Webインターフェース統合

```typescript
// 将来のWebインターフェースアーキテクチャ
interface WebBridgeAPI {
    // リアルタイムスキル実行
    executeSkill(skillName: string, robotId: number, parameters: ParameterMap): Promise<ExecutionResult>;
    
    // ライブデータストリーミング
    subscribeToWorldModel(callback: (worldModel: WorldModel) => void): Subscription;
    subscribeToRobotCommands(callback: (commands: RobotCommand[]) => void): Subscription;
    
    // シナリオ管理
    uploadScenario(scenario: Scenario): Promise<string>;
    executeScenario(scenarioId: string): Promise<ExecutionResult>;
}
```

### 分析と監視

```python
class PerformanceAnalyzer:
    """パフォーマンス監視と分析"""
    
    def analyze_execution_metrics(self, execution_history: List[ExecutionRecord]) -> AnalysisReport:
        # 実行時間分析
        # 成功率計算  
        # パラメータ相関分析
        # ロボットパフォーマンス比較
        pass
    
    def generate_optimization_suggestions(self, analysis: AnalysisReport) -> List[Suggestion]:
        # パラメータチューニング推奨事項
        # スキル選択最適化
        # リソース割り当て提案
        pass
```

この設計概要は、crane_debug_toolsアーキテクチャの包括的な理解を提供し、開発者がシステムを効果的に使用、拡張、保守できるようにします。