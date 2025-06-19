# User Guide: Crane Debug Tools

This comprehensive guide covers everything you need to know to effectively use the crane_debug_tools for robot skill testing and debugging.

## Table of Contents
1. [Getting Started](#getting-started)
2. [CLI Interface](#cli-interface)
3. [Batch Operations](#batch-operations)
4. [Scenario Testing](#scenario-testing)
5. [Multi-Robot Coordination](#multi-robot-coordination)
6. [Advanced Usage](#advanced-usage)
7. [Troubleshooting](#troubleshooting)
8. [Best Practices](#best-practices)

## Getting Started

### Prerequisites

Before using crane_debug_tools, ensure you have:

1. **ROS 2 Jazzy** properly installed and configured
2. **Crane workspace** built and sourced
3. **crane_robot_skills** action server running

### Installation and Setup

```bash
# Build the debug tools
colcon build --packages-select crane_debug_tools

# Source the workspace
source install/local_setup.bash

# Verify installation
crane_skill --help
```

### First Steps

1. **Check available skills:**
   ```bash
   crane_skill list
   ```

2. **Test basic functionality:**
   ```bash
   crane_skill run Sleep 0 duration:1.0
   ```

3. **Launch interactive mode:**
   ```bash
   ros2 run crane_debug_tools crane_skill_cli
   ```

## CLI Interface

### Interactive CLI Mode

The interactive CLI provides a conversational interface for skill testing:

```bash
ros2 run crane_debug_tools crane_skill_cli
```

**Available Commands:**
- `help, h` - Show help information
- `list, l` - List available skills
- `run <skill> <robot_id> [params]` - Execute a skill
- `robots <n>` - Set number of active robots
- `quit, exit, q` - Exit the program

**Example Session:**
```
=== Crane Skill Tester CLI ===
> list
Available skills:
  Sleep     Idle      Kick      Receive
  Goalie    Attacker  SubAttacker  StealBall
  ...

> robots 6
Set number of robots to 6

> run EmplaceRobot 0 target_x:1.0 target_y:2.0 target_theta:0.5
Executing skill 'EmplaceRobot' on robot 0 with parameters: target_x=1.0 target_y=2.0 target_theta=0.5
Goal accepted by server, executing...
Skill execution succeeded with result: 0

> run Kick 0 target_x:3.0 target_y:1.0 kick_power:5.0
Executing skill 'Kick' on robot 0 with parameters: target_x=3.0 target_y=1.0 kick_power=5.0
Goal accepted by server, executing...
Feedback: Ball detected, adjusting position
Feedback: Kick trajectory calculated
Skill execution succeeded with result: 0
```

### Direct CLI Commands

For quick operations and scripting, use the direct command interface:

```bash
# Basic syntax
crane_skill <command> [arguments]
```

**Available Commands:**

#### `list` - Show Available Skills
```bash
crane_skill list
```

#### `run` - Execute Single Skill
```bash
crane_skill run <skill_name> <robot_id> [param1:value1] [param2:value2] ...
```

**Examples:**
```bash
# Simple skill without parameters
crane_skill run Idle 0

# Skill with parameters
crane_skill run Kick 0 target_x:1.5 target_y:0.5 kick_power:4.0

# Robot positioning
crane_skill run EmplaceRobot 1 target_x:2.0 target_y:1.0 target_theta:1.57

# Test motion
crane_skill run TestMotionPosition 0 target_x:1.0 target_y:1.0
```

#### `multi` - Multi-Robot Execution
```bash
crane_skill multi <skill_name> <robot_ids> [parameters]
```

**Examples:**
```bash
# Execute skill on multiple robots
crane_skill multi Idle 0,1,2

# Multi-robot positioning
crane_skill multi EmplaceRobot 0,1,2,3 target_x:1.0 target_y:0.0

# Formation setup
crane_skill multi Attacker 1,2,3
```

#### `scenario` - Execute Test Scenarios
```bash
crane_skill scenario <scenario_file.json>
```

**Example:**
```bash
crane_skill scenario scenarios/basic_skills_test.json
```

## Batch Operations

### Parameter Types and Formats

The system automatically detects parameter types:

| Type | Format | Example |
|------|--------|---------|
| **Boolean** | `true`/`false` | `enable_kick:true` |
| **Integer** | Whole numbers | `robot_count:5` |
| **Float** | Decimal numbers | `target_x:1.5`, `kick_power:3.14` |
| **String** | Text | `mode:attack`, `strategy:defensive` |

### Common Parameter Patterns

**Position Parameters:**
```bash
# 2D position
target_x:1.0 target_y:2.0

# 3D position with orientation
target_x:1.0 target_y:2.0 target_theta:0.5

# Velocity
velocity_x:0.5 velocity_y:0.3
```

**Skill-Specific Parameters:**
```bash
# Kick parameters
kick_power:5.0 chip_enable:false

# Motion parameters
max_velocity:2.0 acceleration:1.0

# Timing parameters
duration:3.0 delay:1.5
```

### Scripting Integration

**Bash Script Example:**
```bash
#!/bin/bash
# Formation setup script

echo "Setting up defensive formation..."

# Position goalkeeper
crane_skill run EmplaceRobot 0 target_x:-3.0 target_y:0.0 target_theta:0.0

# Position defenders
crane_skill multi EmplaceRobot 1,2 target_x:-1.5 target_y:1.0
crane_skill run EmplaceRobot 2 target_x:-1.5 target_y:-1.0 target_theta:0.0

# Activate defensive behaviors
crane_skill run Goalie 0
crane_skill multi SecondThreatDefender 1,2

echo "Defensive formation ready!"
```

**Python Script Example:**
```python
#!/usr/bin/env python3
import subprocess
import time

def execute_skill(skill, robot_id, **params):
    """Execute a skill with parameters"""
    cmd = ['crane_skill', 'run', skill, str(robot_id)]
    for key, value in params.items():
        cmd.append(f"{key}:{value}")
    
    result = subprocess.run(cmd, capture_output=True, text=True)
    return result.returncode == 0

# Test sequence
robots = [0, 1, 2]

# Position robots
for i, robot in enumerate(robots):
    success = execute_skill('EmplaceRobot', robot, 
                          target_x=i*1.0, target_y=0.0, target_theta=0.0)
    if success:
        print(f"Robot {robot} positioned successfully")
    time.sleep(1)

# Activate behaviors
execute_skill('Goalie', 0)
execute_skill('Attacker', 1)
execute_skill('SubAttacker', 2)
```

## Scenario Testing

### Scenario File Format

Scenarios are defined in JSON format with the following structure:

```json
{
  "name": "Scenario Name",
  "description": "Description of what this scenario tests",
  "skills": [
    {
      "name": "SkillName",
      "robot_id": 0,
      "parameters": {
        "param1": "value1",
        "param2": 123.45
      },
      "delay": 2.0,
      "description": "Description of this step"
    }
  ]
}
```

### Creating Custom Scenarios

**Basic Scenario Example:**
```json
{
  "name": "Ball Handling Test",
  "description": "Test basic ball handling skills",
  "skills": [
    {
      "name": "EmplaceRobot",
      "robot_id": 0,
      "parameters": {
        "target_x": 0.0,
        "target_y": 0.0,
        "target_theta": 0.0
      },
      "delay": 2,
      "description": "Position robot near ball"
    },
    {
      "name": "Kick",
      "robot_id": 0,
      "parameters": {
        "target_x": 2.0,
        "target_y": 0.0,
        "kick_power": 3.0
      },
      "delay": 3,
      "description": "Kick ball forward"
    },
    {
      "name": "Receive",
      "robot_id": 0,
      "parameters": {},
      "delay": 0,
      "description": "Switch to receive mode"
    }
  ]
}
```

**Complex Multi-Robot Scenario:**
```json
{
  "name": "Coordinated Attack",
  "description": "Test coordinated attacking behavior",
  "skills": [
    {
      "name": "EmplaceRobot",
      "robot_id": 0,
      "parameters": {"target_x": -2.0, "target_y": 0.0, "target_theta": 0.0},
      "delay": 0,
      "description": "Position goalkeeper"
    },
    {
      "name": "EmplaceRobot", 
      "robot_id": 1,
      "parameters": {"target_x": 0.0, "target_y": 0.0, "target_theta": 0.0},
      "delay": 0,
      "description": "Position main attacker"
    },
    {
      "name": "EmplaceRobot",
      "robot_id": 2,
      "parameters": {"target_x": -0.5, "target_y": 1.0, "target_theta": 0.0},
      "delay": 0,
      "description": "Position support attacker"
    },
    {
      "name": "EmplaceRobot",
      "robot_id": 3,
      "parameters": {"target_x": -0.5, "target_y": -1.0, "target_theta": 0.0},
      "delay": 2,
      "description": "Position second support"
    },
    {
      "name": "Goalie",
      "robot_id": 0,
      "parameters": {},
      "delay": 0,
      "description": "Activate goalkeeper"
    },
    {
      "name": "Attacker",
      "robot_id": 1,
      "parameters": {},
      "delay": 0,
      "description": "Activate main attacker"
    },
    {
      "name": "SubAttacker",
      "robot_id": 2,
      "parameters": {},
      "delay": 0,
      "description": "Activate support attacker"
    },
    {
      "name": "SubAttacker",
      "robot_id": 3,
      "parameters": {},
      "delay": 5,
      "description": "Activate second support"
    }
  ]
}
```

### Scenario Execution

```bash
# Execute scenario
crane_skill scenario my_scenario.json

# With verbose output
crane_skill scenario my_scenario.json --verbose

# Continue on errors (if implemented)
crane_skill scenario my_scenario.json --continue-on-error
```

**Expected Output:**
```
--- Executing skill 1/4 ---
Executing skill 'EmplaceRobot' on robot 0
Parameters: target_x:0.0 target_y:0.0 target_theta:0.0
✓ Skill execution succeeded

Waiting 2 seconds before next skill...

--- Executing skill 2/4 ---
Executing skill 'Kick' on robot 0
Parameters: target_x:2.0 target_y:0.0 kick_power:3.0
✓ Skill execution succeeded

--- Scenario completed: 4/4 skills succeeded ---
```

## Multi-Robot Coordination

### Parallel Execution

Execute the same skill on multiple robots simultaneously:

```bash
# Basic multi-robot command
crane_skill multi <skill_name> <robot_list> [parameters]

# Examples
crane_skill multi Idle 0,1,2,3,4,5
crane_skill multi EmplaceRobot 1,2,3 target_x:1.0 target_y:0.0
crane_skill multi Attacker 0,1,2
```

### Formation Testing

**Defensive Formation:**
```bash
# Quick defensive setup
crane_skill run EmplaceRobot 0 target_x:-3.0 target_y:0.0 target_theta:0.0  # Goalie
crane_skill multi EmplaceRobot 1,2 target_x:-1.5 target_y:1.0               # Defenders
crane_skill run Goalie 0
crane_skill multi SecondThreatDefender 1,2
```

**Offensive Formation:**
```bash
# Attacking formation
crane_skill multi EmplaceRobot 0,1,2 target_x:1.0 target_y:0.0    # Forward line
crane_skill multi EmplaceRobot 3,4 target_x:0.0 target_y:1.0      # Midfield
crane_skill multi Attacker 0,1,2
crane_skill multi SubAttacker 3,4
```

### Coordination Scenarios

**Passing Sequence:**
```json
{
  "name": "Passing Chain",
  "skills": [
    {
      "name": "EmplaceRobot",
      "robot_id": 0,
      "parameters": {"target_x": -1.0, "target_y": 0.0},
      "delay": 0
    },
    {
      "name": "EmplaceRobot", 
      "robot_id": 1,
      "parameters": {"target_x": 0.0, "target_y": 1.0},
      "delay": 0
    },
    {
      "name": "EmplaceRobot",
      "robot_id": 2,
      "parameters": {"target_x": 1.0, "target_y": 0.0},
      "delay": 2
    },
    {
      "name": "Kick",
      "robot_id": 0,
      "parameters": {"target_x": 0.0, "target_y": 1.0, "kick_power": 2.0},
      "delay": 1
    },
    {
      "name": "Receive",
      "robot_id": 1,
      "parameters": {},
      "delay": 2
    },
    {
      "name": "Kick",
      "robot_id": 1,
      "parameters": {"target_x": 1.0, "target_y": 0.0, "kick_power": 2.0},
      "delay": 1
    },
    {
      "name": "Receive",
      "robot_id": 2,
      "parameters": {},
      "delay": 0
    }
  ]
}
```

## Advanced Usage

### Environment Variables

Configure behavior through environment variables:

```bash
# Set default robot count
export CRANE_DEBUG_ROBOT_COUNT=6

# Set action server timeout
export CRANE_DEBUG_TIMEOUT=30

# Enable verbose logging
export CRANE_DEBUG_VERBOSE=1

# Use custom action server topic
export CRANE_DEBUG_ACTION_TOPIC="/custom/skill_execution"
```

### Custom Parameter Validation

For skills requiring specific parameter combinations:

```bash
# Kick with validation
crane_skill run Kick 0 target_x:3.0 target_y:0.0 kick_power:5.0 chip_enable:false

# Motion with constraints
crane_skill run TestMotionPosition 0 target_x:1.0 target_y:1.0 max_velocity:2.0
```

### Integration with ROS 2 Tools

**Monitor execution:**
```bash
# Watch action server
ros2 action list | grep skill_execution

# Monitor topics
ros2 topic echo /world_model
ros2 topic echo /robot_commands

# Check node graph
ros2 node list | grep crane
```

**Parameter inspection:**
```bash
# Check robot skills node parameters
ros2 param list /crane_robot_skills

# Get parameter values
ros2 param get /crane_robot_skills use_world_model
```

### Performance Monitoring

**Execution timing:**
```bash
# Time skill execution
time crane_skill run Kick 0 target_x:1.0 target_y:2.0

# Scenario timing
time crane_skill scenario scenarios/complex_test.json
```

**Resource monitoring:**
```bash
# Monitor during execution
htop &
crane_skill scenario scenarios/stress_test.json
```

## Troubleshooting

### Common Issues

#### 1. Action Server Not Available

**Error:** `Action server not available after waiting`

**Solutions:**
```bash
# Check if crane_robot_skills is running
ros2 node list | grep crane_robot_skills

# Check action server
ros2 action list | grep skill_execution

# Restart crane system
ros2 launch crane_bringup crane.launch.py
```

#### 2. Parameter Type Errors

**Error:** `Invalid parameter type`

**Solutions:**
```bash
# Check parameter format
crane_skill run Kick 0 target_x:1.0  # Correct (float)
crane_skill run Kick 0 target_x:1    # May be interpreted as int

# Use explicit decimal notation for floats
crane_skill run Kick 0 kick_power:5.0  # Not just "5"
```

#### 3. Robot ID Out of Range

**Error:** `Robot ID must be between 0 and 15`

**Solutions:**
```bash
# Check valid robot range
crane_skill list  # Shows valid robots

# Use valid robot IDs
crane_skill run Kick 0 target_x:1.0    # Robot 0 (valid)
crane_skill run Kick 16 target_x:1.0   # Robot 16 (invalid)
```

#### 4. Skill Execution Timeout

**Error:** `Skill execution timed out`

**Solutions:**
```bash
# Check robot status
ros2 topic echo /world_model --once

# Verify robot is visible and responding
ros2 topic echo /robot_commands --once

# Try simpler skill first
crane_skill run Idle 0
```

### Debugging Commands

**Check system status:**
```bash
# ROS 2 environment
ros2 node list
ros2 topic list
ros2 action list

# Crane specific
ros2 topic echo /world_model --once
ros2 param list /crane_robot_skills
```

**Verbose execution:**
```bash
# Enable debug output (if available)
export ROS_LOG_LEVEL=DEBUG
crane_skill run Kick 0 target_x:1.0 target_y:2.0
```

**Network diagnostics:**
```bash
# Check ROS 2 discovery
ros2 doctor

# Network connectivity
ros2 multicast send
ros2 multicast receive
```

## Best Practices

### Skill Testing Workflow

1. **Start Simple:**
   ```bash
   crane_skill run Idle 0
   crane_skill run Sleep 0 duration:1.0
   ```

2. **Test Motion:**
   ```bash
   crane_skill run EmplaceRobot 0 target_x:0.0 target_y:0.0
   crane_skill run TestMotionPosition 0 target_x:1.0 target_y:1.0
   ```

3. **Test Game Skills:**
   ```bash
   crane_skill run Kick 0 target_x:2.0 target_y:0.0 kick_power:3.0
   crane_skill run Receive 0
   ```

4. **Test Complex Behaviors:**
   ```bash
   crane_skill run Attacker 0
   crane_skill run Goalie 1
   ```

### Scenario Development

1. **Incremental Testing:**
   - Start with single robot scenarios
   - Add complexity gradually
   - Test each step individually

2. **Documentation:**
   - Use descriptive names and descriptions
   - Comment complex parameter choices
   - Include expected outcomes

3. **Error Handling:**
   - Plan for failure cases
   - Use appropriate delays between skills
   - Validate robot positions before complex maneuvers

### Multi-Robot Testing

1. **Coordination:**
   - Test individual robots first
   - Gradually increase team size
   - Monitor for interference between robots

2. **Timing:**
   - Use appropriate delays for robot movement
   - Account for communication latency
   - Test synchronization scenarios

3. **Validation:**
   - Verify robot positions before group maneuvers
   - Check for collision avoidance
   - Monitor team coordination effectiveness

This user guide provides comprehensive coverage of crane_debug_tools usage, from basic operations to advanced multi-robot coordination testing.