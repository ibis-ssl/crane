# API Reference: Crane Debug Tools

This document provides a comprehensive API reference for developers working with or extending the crane_debug_tools package.

## Table of Contents
1. [Core Classes](#core-classes)
2. [CLI Interface API](#cli-interface-api)
3. [Scenario System API](#scenario-system-api)
4. [Parameter System API](#parameter-system-api)
5. [Extension Points](#extension-points)
6. [ROS 2 Integration](#ros-2-integration)
7. [Error Handling](#error-handling)

## Core Classes

### SkillTesterCLI (C++)

Main class for the interactive CLI interface.

```cpp
class SkillTesterCLI : public rclcpp::Node
{
public:
    using SkillExecutionAction = crane_msgs::action::SkillExecution;
    using SkillExecutionClient = rclcpp_action::Client<SkillExecutionAction>;

    // Constructor
    SkillTesterCLI();
    
    // Main execution loop
    void run();

private:
    // Skill execution
    void executeSkill(const std::string& skill_name, int robot_id, 
                     const std::map<std::string, std::string>& params);
    
    // Command processing
    void processCommand(const std::string& input);
    
    // UI helpers
    void printWelcomeMessage();
    void printHelp();
    void listSkills();

    // Members
    SkillExecutionClient::SharedPtr client_;
    std::vector<std::string> available_skills_;
    int num_robots_;
};
```

**Key Methods:**

#### `SkillTesterCLI()`
Constructor that initializes the ROS 2 node and action client.

**Parameters:** None  
**Returns:** N/A  
**Example:**
```cpp
auto node = std::make_shared<SkillTesterCLI>();
```

#### `void run()`
Starts the interactive CLI loop.

**Parameters:** None  
**Returns:** void  
**Behavior:** Blocks until user exits  
**Example:**
```cpp
node->run();
```

#### `void executeSkill(...)`
Executes a skill with given parameters.

**Parameters:**
- `skill_name` (const std::string&): Name of the skill to execute
- `robot_id` (int): Target robot ID (0-15)
- `params` (const std::map<std::string, std::string>&): Skill parameters

**Returns:** void  
**Throws:** std::runtime_error on invalid parameters  
**Example:**
```cpp
std::map<std::string, std::string> params = {
    {"target_x", "1.0"},
    {"target_y", "2.0"},
    {"kick_power", "5.0"}
};
tester.executeSkill("Kick", 0, params);
```

### SkillCLI (Python)

Python class for the command-line script interface.

```python
class SkillCLI(Node):
    """Command-line interface for crane robot skills"""
    
    AVAILABLE_SKILLS = [
        "Sleep", "Idle", "Kick", "Receive", "Goalie", 
        "Attacker", "SubAttacker", "StealBall", ...
    ]
    
    def __init__(self):
        """Initialize the CLI node"""
        
    def wait_for_server(self, timeout: float = 10.0) -> bool:
        """Wait for the action server to be available"""
        
    def execute_skill(self, skill_name: str, robot_id: int, 
                     parameters: Optional[Dict[str, Any]] = None) -> bool:
        """Execute a skill with given parameters"""
```

**Key Methods:**

#### `__init__()`
Initializes the ROS 2 node and action client.

**Parameters:** None  
**Example:**
```python
cli = SkillCLI()
```

#### `wait_for_server(timeout: float = 10.0) -> bool`
Waits for the action server to become available.

**Parameters:**
- `timeout` (float): Maximum wait time in seconds

**Returns:** bool - True if server is available, False otherwise  
**Example:**
```python
if cli.wait_for_server():
    print("Server ready")
```

#### `execute_skill(...) -> bool`
Executes a skill and waits for completion.

**Parameters:**
- `skill_name` (str): Name of the skill
- `robot_id` (int): Target robot ID
- `parameters` (Optional[Dict[str, Any]]): Skill parameters

**Returns:** bool - True if execution succeeded  
**Example:**
```python
success = cli.execute_skill("Kick", 0, {
    "target_x": 1.0,
    "target_y": 2.0,
    "kick_power": 5.0
})
```

## CLI Interface API

### Command Line Interface

The `crane_skill` script provides a unified command-line interface.

#### Command Structure
```bash
crane_skill <command> [arguments] [options]
```

### Available Commands

#### `list`
Lists all available skills.

**Syntax:** `crane_skill list`  
**Returns:** 0 on success  
**Output:** Skill names, one per line  
**Example:**
```bash
$ crane_skill list
Sleep
Idle
Kick
Receive
...
```

#### `run`
Executes a single skill.

**Syntax:** `crane_skill run <skill> <robot_id> [param:value ...]`

**Parameters:**
- `skill` (str): Skill name (must be in available skills list)
- `robot_id` (int): Robot ID (0-15)
- `param:value` (str): Parameter pairs in key:value format

**Returns:** 0 on success, 1 on failure  
**Example:**
```bash
crane_skill run Kick 0 target_x:1.0 target_y:2.0 kick_power:5.0
```

#### `multi`
Executes a skill on multiple robots.

**Syntax:** `crane_skill multi <skill> <robot_ids> [param:value ...]`

**Parameters:**
- `skill` (str): Skill name
- `robot_ids` (str): Comma-separated robot IDs (e.g., "0,1,2")
- `param:value` (str): Parameter pairs

**Returns:** 0 if all robots succeed, 1 if any fail  
**Example:**
```bash
crane_skill multi Attacker 0,1,2
crane_skill multi EmplaceRobot 1,2,3 target_x:1.0 target_y:0.0
```

#### `scenario`
Executes a skill scenario from JSON file.

**Syntax:** `crane_skill scenario <file.json>`

**Parameters:**
- `file.json` (str): Path to scenario file

**Returns:** 0 if all skills succeed, 1 if any fail  
**Example:**
```bash
crane_skill scenario scenarios/basic_test.json
```

## Scenario System API

### JSON Schema

Scenario files must conform to this JSON schema:

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "type": "object",
  "properties": {
    "name": {
      "type": "string",
      "description": "Human-readable scenario name"
    },
    "description": {
      "type": "string", 
      "description": "Detailed scenario description"
    },
    "skills": {
      "type": "array",
      "minItems": 1,
      "items": {
        "type": "object",
        "properties": {
          "name": {
            "type": "string",
            "description": "Skill name"
          },
          "robot_id": {
            "type": "integer",
            "minimum": 0,
            "maximum": 15,
            "description": "Target robot ID"
          },
          "parameters": {
            "type": "object",
            "description": "Skill parameters as key-value pairs"
          },
          "delay": {
            "type": "number",
            "minimum": 0,
            "description": "Delay in seconds after skill completion"
          },
          "description": {
            "type": "string",
            "description": "Human-readable step description"
          }
        },
        "required": ["name", "robot_id"]
      }
    }
  },
  "required": ["name", "skills"]
}
```

### Scenario Functions

#### `load_scenario(scenario_file: str) -> List[Dict[str, Any]]`
Loads and validates a scenario file.

**Parameters:**
- `scenario_file` (str): Path to JSON scenario file

**Returns:** List of skill dictionaries  
**Raises:** ValueError on invalid format  
**Example:**
```python
skills = load_scenario("test_scenario.json")
for skill in skills:
    print(f"Skill: {skill['name']}, Robot: {skill['robot_id']}")
```

#### `execute_scenario(cli: SkillCLI, skills: List[Dict[str, Any]]) -> bool`
Executes a sequence of skills.

**Parameters:**
- `cli` (SkillCLI): Initialized CLI instance
- `skills` (List[Dict]): List of skill definitions

**Returns:** bool - True if all skills succeed  
**Example:**
```python
cli = SkillCLI()
skills = load_scenario("my_test.json")
success = execute_scenario(cli, skills)
```

### Scenario Validation

#### `validate_scenario_schema(scenario: Dict) -> List[str]`
Validates scenario against JSON schema.

**Parameters:**
- `scenario` (Dict): Parsed scenario dictionary

**Returns:** List of validation error messages (empty if valid)  
**Example:**
```python
with open("scenario.json") as f:
    scenario = json.load(f)

errors = validate_scenario_schema(scenario)
if errors:
    print("Validation errors:", errors)
```

## Parameter System API

### Parameter Types

The system supports four parameter types with automatic detection:

| Type | Python Type | ROS Message | Detection Pattern |
|------|-------------|-------------|-------------------|
| Boolean | `bool` | `NamedBool` | `"true"`, `"false"` (case-insensitive) |
| Integer | `int` | `NamedInt` | Whole numbers without decimal point |
| Float | `float` | `NamedFloat` | Numbers with decimal point |
| String | `str` | `NamedString` | All other values |

### Parameter Processing Functions

#### `parse_parameters(param_strings: List[str]) -> Dict[str, Any]`
Parses command-line parameter strings.

**Parameters:**
- `param_strings` (List[str]): List of "key:value" strings

**Returns:** Dictionary with typed values  
**Example:**
```python
params = parse_parameters([
    "target_x:1.5",      # -> {"target_x": 1.5}
    "kick_power:5.0",    # -> {"kick_power": 5.0}
    "enable_chip:true",  # -> {"enable_chip": True}
    "mode:attack"        # -> {"mode": "attack"}
])
```

#### Parameter Type Conversion

**C++ Implementation:**
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

**Python Implementation:**
```python
def convert_parameters(parameters: Dict[str, Any]) -> NamedValueArray:
    """Convert parameters to ROS message format"""
    
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
        # ... similar for float and string
    
    return result
```

### Type Detection Utilities

#### `isBooleanString(const std::string& value) -> bool`
Detects boolean values.

**Parameters:**
- `value` (const std::string&): String to test

**Returns:** bool - True if string represents a boolean  
**Accepted Values:** "true", "false", "TRUE", "FALSE", "True", "False"  
**Example:**
```cpp
assert(isBooleanString("true") == true);
assert(isBooleanString("1.5") == false);
```

#### `isFloatString(const std::string& value) -> bool`
Detects floating-point values.

**Parameters:**
- `value` (const std::string&): String to test

**Returns:** bool - True if string represents a float  
**Example:**
```cpp
assert(isFloatString("1.5") == true);
assert(isFloatString("42") == false);  // Integer, not float
assert(isFloatString("abc") == false);
```

## Extension Points

### Custom Skill Plugins

Create custom skill handlers by implementing the skill plugin interface:

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
        
        // Custom validation logic
        if (params.find("custom_param") != params.end()) {
            float value = std::stof(params.at("custom_param"));
            if (value < 0.0 || value > 10.0) {
                result.addError("custom_param must be between 0.0 and 10.0");
            }
        }
        
        return result;
    }
};
```

### Custom Command Extensions

Extend the CLI with custom commands:

```python
class BenchmarkCommand(CommandExtension):
    """Custom command for performance benchmarking"""
    
    def get_command_name(self) -> str:
        return "benchmark"
    
    def get_description(self) -> str:
        return "Run performance benchmarks on skills"
    
    def add_arguments(self, parser: argparse.ArgumentParser):
        parser.add_argument("skill", help="Skill to benchmark")
        parser.add_argument("--iterations", type=int, default=10,
                          help="Number of iterations")
        parser.add_argument("--robots", default="0",
                          help="Comma-separated robot IDs")
    
    def execute(self, args: argparse.Namespace) -> int:
        # Custom benchmarking logic
        return self.run_benchmark(args.skill, args.iterations, args.robots)

# Register extension
CLI_EXTENSIONS.register(BenchmarkCommand())
```

### Scenario Processors

Create custom scenario processors:

```python
class ConditionalScenarioProcessor:
    """Process scenarios with conditional logic"""
    
    def process_scenario(self, scenario: Dict) -> Dict:
        """Add conditional execution logic"""
        
        processed_skills = []
        for skill in scenario["skills"]:
            # Add conditional logic
            if self.should_execute_skill(skill):
                processed_skills.append(skill)
        
        scenario["skills"] = processed_skills
        return scenario
    
    def should_execute_skill(self, skill: Dict) -> bool:
        """Determine if skill should be executed based on conditions"""
        # Custom logic here
        return True
```

## ROS 2 Integration

### Action Interface

The system uses the standard ROS 2 action pattern:

```cpp
// Action definition (from crane_msgs)
action SkillExecution {
    # Goal
    uint8 robot_id
    string name
    NamedValueArray parameter
    ---
    # Result  
    int16 result
    ---
    # Feedback
    string message
}
```

### Action Client Usage

```cpp
// Create action client
auto client = rclcpp_action::create_client<SkillExecutionAction>(
    node, "/simple_ai/skill_execution");

// Send goal
auto goal_msg = SkillExecutionAction::Goal();
goal_msg.robot_id = robot_id;
goal_msg.name = skill_name;
goal_msg.parameter = convertParameters(parameters);

auto send_goal_options = rclcpp_action::Client<SkillExecutionAction>::SendGoalOptions();

send_goal_options.goal_response_callback = 
    [](const auto& goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(logger, "Goal was rejected");
        }
    };

send_goal_options.feedback_callback = 
    [](const auto&, const auto& feedback) {
        RCLCPP_INFO(logger, "Feedback: %s", feedback->message.c_str());
    };

send_goal_options.result_callback = 
    [](const auto& result) {
        RCLCPP_INFO(logger, "Result: %d", result.result->result);
    };

client->async_send_goal(goal_msg, send_goal_options);
```

### Topic Subscriptions

Monitor system state through topic subscriptions:

```python
# World model subscription
def world_model_callback(msg):
    """Process world model updates"""
    ball_position = (msg.ball_info.position.x, msg.ball_info.position.y)
    robot_positions = [(r.pose.position.x, r.pose.position.y) 
                      for r in msg.robot_info_ours]

world_model_sub = node.create_subscription(
    WorldModel, '/world_model', world_model_callback, 10)

# Robot commands subscription  
def robot_commands_callback(msg):
    """Monitor robot command outputs"""
    for cmd in msg.robot_commands:
        print(f"Robot {cmd.robot_id}: target=({cmd.target_x}, {cmd.target_y})")

commands_sub = node.create_subscription(
    RobotCommands, '/robot_commands', robot_commands_callback, 10)
```

## Error Handling

### Exception Hierarchy

```python
class CraneDebugError(Exception):
    """Base exception for crane debug tools"""
    pass

class SkillExecutionError(CraneDebugError):
    """Error during skill execution"""
    def __init__(self, skill_name: str, robot_id: int, message: str):
        self.skill_name = skill_name
        self.robot_id = robot_id
        super().__init__(f"Skill {skill_name} failed on robot {robot_id}: {message}")

class ParameterValidationError(CraneDebugError):
    """Invalid skill parameters"""
    def __init__(self, parameter: str, message: str):
        self.parameter = parameter
        super().__init__(f"Parameter {parameter}: {message}")

class ActionServerUnavailableError(CraneDebugError):
    """Action server not available"""
    pass

class ScenarioValidationError(CraneDebugError):
    """Invalid scenario format"""
    pass
```

### Error Handling Patterns

```python
try:
    success = cli.execute_skill("Kick", 0, {"target_x": 1.0, "target_y": 2.0})
    if not success:
        raise SkillExecutionError("Kick", 0, "Execution failed")
        
except ActionServerUnavailableError:
    print("Error: crane_robot_skills action server not available")
    print("Please ensure the crane system is running")
    return 1
    
except ParameterValidationError as e:
    print(f"Parameter error: {e}")
    return 1
    
except SkillExecutionError as e:
    print(f"Execution error: {e}")
    return 1
```

### Return Codes

The CLI tools use standard Unix return codes:

| Code | Meaning | Description |
|------|---------|-------------|
| 0 | Success | All operations completed successfully |
| 1 | General Error | Skill execution failed or invalid parameters |
| 2 | Usage Error | Invalid command syntax or missing arguments |
| 3 | Server Error | Action server unavailable or communication failure |
| 4 | Validation Error | Invalid scenario format or parameter validation failed |

**Example Usage:**
```bash
crane_skill run Kick 0 target_x:1.0 target_y:2.0
echo $?  # Check return code (0 = success, non-zero = error)
```

This API reference provides comprehensive documentation for developers working with crane_debug_tools, enabling effective usage and extension of the system.