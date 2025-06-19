# Design Overview: Crane Debug Tools

This document provides a comprehensive overview of the design philosophy, architecture, and technical decisions behind the crane_debug_tools package.

## Design Philosophy

### Core Principles

1. **Simplicity First**: Prefer simple, composable tools over complex monolithic interfaces
2. **Automation-Ready**: Every feature should be scriptable and automatable
3. **Developer-Centric**: Optimize for developer productivity and workflow efficiency
4. **Platform Agnostic**: Work consistently across different development environments
5. **Performance Oriented**: Minimize resource usage and execution overhead

### Design Goals

- **Replace Qt Dependencies**: Eliminate heavy GUI framework dependencies
- **Enable Remote Development**: Support development over SSH and remote connections
- **Improve Testing Workflows**: Provide better tools for automated and regression testing
- **Multi-Robot Focus**: Native support for coordinated multi-robot testing
- **CI/CD Integration**: First-class support for continuous integration workflows

## Architecture Overview

### System Architecture

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

### Component Architecture

#### 1. CLI Interface Layer

**Interactive CLI (`crane_skill_cli`)**
- Real-time skill execution with feedback
- Tab completion and command history
- Multi-session support
- Error reporting and status display

**Batch CLI (`crane_skill`)**
- Single-command execution
- Scenario file processing
- Multi-robot coordination
- Return code-based automation

#### 2. Core Engine

**Skill Execution Manager**
```cpp
class SkillExecutionManager {
    // Manages action client lifecycle
    // Handles goal submission and result processing
    // Provides feedback aggregation
    // Implements timeout and retry logic
};
```

**Parameter Type System**
```cpp
// Automatic type resolution for parameters
struct ParameterResolver {
    static NamedValueArray resolveParameters(
        const std::map<std::string, std::string>& params);
    
    // Type detection hierarchy:
    // 1. Boolean (true/false)
    // 2. Integer (whole numbers)
    // 3. Float (decimal numbers)
    // 4. String (everything else)
};
```

**Multi-Robot Coordinator**
```python
class MultiRobotCoordinator:
    """Manages simultaneous skill execution across multiple robots"""
    
    def execute_parallel(self, skill_name: str, robot_ids: List[int], 
                        parameters: Dict[str, Any]) -> bool:
        # Parallel execution with synchronization
        # Failure handling and partial success reporting
        # Progress tracking and status aggregation
```

#### 3. Scenario System

**JSON Schema**
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

**Execution Engine**
```python
class ScenarioExecutor:
    """Processes and executes skill scenarios"""
    
    def validate_scenario(self, scenario: Dict) -> List[str]:
        # JSON schema validation
        # Skill name verification
        # Parameter type checking
        # Robot ID bounds checking
    
    def execute_scenario(self, scenario: Dict) -> ExecutionResult:
        # Sequential skill execution
        # Delay handling between skills
        # Progress reporting
        # Error recovery strategies
```

## Technical Design Decisions

### 1. CLI-First Approach

**Decision**: Prioritize command-line interfaces over graphical interfaces

**Rationale**:
- **Remote Development**: SSH-friendly development workflows
- **Automation**: Easy integration with scripts and CI/CD systems
- **Performance**: Lower resource consumption than GUI applications
- **Consistency**: Uniform experience across different platforms
- **Version Control**: Scenario files can be tracked in git

**Trade-offs**:
- Less visual feedback compared to GUI
- Learning curve for users accustomed to graphical interfaces
- Limited real-time visualization capabilities

### 2. Type-Safe Parameter System

**Decision**: Implement automatic parameter type detection and validation

**Implementation**:
```cpp
void addParameter(const std::string& key, const std::string& value) {
    // Type detection cascade
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

**Benefits**:
- **Type Safety**: Prevents runtime type errors
- **User Convenience**: Automatic conversion from string input
- **Validation**: Early detection of parameter issues
- **Compatibility**: Works with existing crane_msgs structure

### 3. Scenario-Based Testing

**Decision**: JSON-based test scenario definitions

**Schema Design**:
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

**Advantages**:
- **Reproducibility**: Scenarios can be version controlled and shared
- **Complexity Management**: Break down complex tests into manageable steps
- **Documentation**: Self-documenting test cases with descriptions
- **Automation**: Easy integration with CI/CD systems

### 4. Multi-Robot Coordination

**Decision**: Native support for parallel multi-robot operations

**Implementation Strategy**:
```python
async def execute_multi_robot_skill(skill_name: str, robot_ids: List[int], 
                                   parameters: Dict[str, Any]) -> Dict[int, bool]:
    """Execute skill on multiple robots in parallel"""
    
    # Create action clients for each robot
    clients = {robot_id: create_action_client(robot_id) for robot_id in robot_ids}
    
    # Submit goals in parallel
    futures = {}
    for robot_id, client in clients.items():
        goal = create_goal(skill_name, robot_id, parameters)
        futures[robot_id] = client.send_goal_async(goal)
    
    # Await all results
    results = {}
    for robot_id, future in futures.items():
        try:
            result = await asyncio.wait_for(future, timeout=30.0)
            results[robot_id] = result.success
        except asyncio.TimeoutError:
            results[robot_id] = False
    
    return results
```

**Benefits**:
- **Efficiency**: Parallel execution reduces total test time
- **Realism**: Tests actual multi-robot coordination scenarios
- **Scalability**: Supports testing with varying numbers of robots
- **Failure Isolation**: Individual robot failures don't stop entire test

## Error Handling and Validation

### Parameter Validation

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
        
        // Check required parameters
        for (const auto& required_param : schema.getRequiredParameters()) {
            if (parameters.find(required_param) == parameters.end()) {
                result.addError("Missing required parameter: " + required_param);
            }
        }
        
        return result;
    }
};
```

### Execution Error Handling

```python
class ExecutionErrorHandler:
    """Comprehensive error handling for skill execution"""
    
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

## Performance Considerations

### Resource Usage Optimization

**Memory Management**:
- Lazy loading of skill schemas
- Connection pooling for action clients
- Bounded logging buffers
- Automatic cleanup of completed operations

**Execution Efficiency**:
```cpp
class SkillExecutionPool {
    // Connection reuse
    std::unordered_map<uint8_t, ActionClient::SharedPtr> client_pool_;
    
    // Batch operations
    void executeBatch(const std::vector<SkillRequest>& requests) {
        // Group by robot ID to minimize client creation
        auto grouped = groupByRobotId(requests);
        
        // Parallel execution with thread pool
        std::vector<std::future<Result>> futures;
        for (const auto& [robot_id, robot_requests] : grouped) {
            futures.push_back(std::async(std::launch::async,
                [this, robot_id, robot_requests]() {
                    return executeRobotBatch(robot_id, robot_requests);
                }));
        }
        
        // Collect results
        for (auto& future : futures) {
            future.wait();
        }
    }
};
```

### Scalability Design

**Concurrent Execution**:
- Asynchronous action client operations
- Thread-safe result aggregation
- Non-blocking user interface updates
- Configurable timeout and retry policies

**Large-Scale Testing**:
```python
class ScalabilityManager:
    """Manages execution scaling for large robot teams"""
    
    def __init__(self, max_concurrent_robots: int = 16):
        self.semaphore = asyncio.Semaphore(max_concurrent_robots)
        self.execution_queue = asyncio.Queue()
    
    async def execute_with_scaling(self, requests: List[SkillRequest]) -> List[Result]:
        # Throttle concurrent executions to prevent resource exhaustion
        async with self.semaphore:
            return await self._execute_batch(requests)
```

## Extensibility and Plugin Architecture

### Skill Plugin System

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

### Custom Command Extensions

```python
class CommandExtension:
    """Base class for custom command extensions"""
    
    def get_command_name(self) -> str:
        raise NotImplementedError
    
    def get_description(self) -> str:
        raise NotImplementedError
    
    def add_arguments(self, parser: argparse.ArgumentParser):
        raise NotImplementedError
    
    def execute(self, args: argparse.Namespace) -> int:
        raise NotImplementedError

# Example extension
class BenchmarkExtension(CommandExtension):
    def get_command_name(self) -> str:
        return "benchmark"
    
    def execute(self, args: argparse.Namespace) -> int:
        # Custom benchmarking logic
        return self.run_performance_tests(args.skill_name, args.iterations)
```

## Future Architecture Considerations

### Web Interface Integration

```typescript
// Future web interface architecture
interface WebBridgeAPI {
    // Real-time skill execution
    executeSkill(skillName: string, robotId: number, parameters: ParameterMap): Promise<ExecutionResult>;
    
    // Live data streaming
    subscribeToWorldModel(callback: (worldModel: WorldModel) => void): Subscription;
    subscribeToRobotCommands(callback: (commands: RobotCommand[]) => void): Subscription;
    
    // Scenario management
    uploadScenario(scenario: Scenario): Promise<string>;
    executeScenario(scenarioId: string): Promise<ExecutionResult>;
}
```

### Analytics and Monitoring

```python
class PerformanceAnalyzer:
    """Performance monitoring and analytics"""
    
    def analyze_execution_metrics(self, execution_history: List[ExecutionRecord]) -> AnalysisReport:
        # Execution time analysis
        # Success rate calculation  
        # Parameter correlation analysis
        # Robot performance comparison
        pass
    
    def generate_optimization_suggestions(self, analysis: AnalysisReport) -> List[Suggestion]:
        # Parameter tuning recommendations
        # Skill selection optimization
        # Resource allocation suggestions
        pass
```

This design overview provides a comprehensive understanding of the crane_debug_tools architecture, enabling developers to effectively use, extend, and maintain the system.