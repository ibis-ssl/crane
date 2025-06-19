# Examples: Crane Debug Tools

This document provides practical examples and use cases for crane_debug_tools, from basic operations to advanced scenarios.

## Table of Contents
1. [Basic Usage Examples](#basic-usage-examples)
2. [Skill Testing Workflows](#skill-testing-workflows)
3. [Multi-Robot Scenarios](#multi-robot-scenarios)
4. [Automated Testing](#automated-testing)
5. [Real-World Use Cases](#real-world-use-cases)
6. [Integration Examples](#integration-examples)
7. [Custom Scenarios](#custom-scenarios)

## Basic Usage Examples

### Getting Started

#### Example 1: List and Test Basic Skills
```bash
# List all available skills
$ crane_skill list
Sleep
Idle
Kick
Receive
Goalie
Attacker
...

# Test the simplest skill
$ crane_skill run Idle 0
Executing skill 'Idle' on robot 0
✓ Skill execution succeeded

# Test with parameters
$ crane_skill run Sleep 0 duration:2.0
Executing skill 'Sleep' on robot 0 with parameters: duration=2.0
✓ Skill execution succeeded
```

#### Example 2: Robot Positioning
```bash
# Move robot to specific position
$ crane_skill run EmplaceRobot 0 target_x:1.0 target_y:2.0 target_theta:0.5
Executing skill 'EmplaceRobot' on robot 0
Parameters: target_x:1.0 target_y:2.0 target_theta:0.5
✓ Skill execution succeeded

# Test motion without orientation
$ crane_skill run TestMotionPosition 0 target_x:0.5 target_y:1.5
Executing skill 'TestMotionPosition' on robot 0
Parameters: target_x:0.5 target_y:1.5
✓ Skill execution succeeded
```

#### Example 3: Ball Interaction
```bash
# Simple kick
$ crane_skill run Kick 0 target_x:3.0 target_y:0.0 kick_power:4.0
Executing skill 'Kick' on robot 0
Parameters: target_x:3.0 target_y:0.0 kick_power:4.0
✓ Skill execution succeeded

# Receive mode
$ crane_skill run Receive 0
Executing skill 'Receive' on robot 0
✓ Skill execution succeeded
```

### Interactive CLI Session

```bash
$ ros2 run crane_debug_tools crane_skill_cli
=== Crane Skill Tester CLI ===
Type 'help' for commands, 'list' for available skills, 'quit' to exit

> help
Available commands:
  help, h          - Show this help
  list, l          - List available skills
  run <skill> <robot_id> [params] - Execute skill
  robots <n>       - Set number of robots for multi-robot tests (1-16)
  quit, exit, q    - Exit the program

> robots 6
Set number of robots to 6

> list
Available skills:
  Sleep           Idle            Kick            Receive
  Goalie          Attacker        SubAttacker     StealBall
  ...

> run EmplaceRobot 0 target_x:1.0 target_y:0.0 target_theta:0.0
Executing skill 'EmplaceRobot' on robot 0 with parameters: target_x=1.0 target_y=0.0 target_theta=0.0
Goal accepted by server, executing...
Skill execution succeeded with result: 0

> run Kick 0 target_x:2.0 target_y:1.0 kick_power:3.0
Executing skill 'Kick' on robot 0 with parameters: target_x=2.0 target_y=1.0 kick_power=3.0
Goal accepted by server, executing...
Feedback: Ball detected, adjusting position
Feedback: Kick trajectory calculated
Skill execution succeeded with result: 0

> quit
```

## Skill Testing Workflows

### Workflow 1: New Robot Testing

When testing a newly configured robot:

```bash
#!/bin/bash
# new_robot_test.sh - Test sequence for new robot validation

ROBOT_ID=0

echo "Testing Robot $ROBOT_ID - Basic Functionality"

# Test 1: Basic responsiveness
echo "1. Testing basic responsiveness..."
crane_skill run Idle $ROBOT_ID
if [ $? -eq 0 ]; then
    echo "✅ Robot responds to commands"
else
    echo "❌ Robot not responding"
    exit 1
fi

# Test 2: Position control
echo "2. Testing position control..."
crane_skill run EmplaceRobot $ROBOT_ID target_x:0.0 target_y:0.0 target_theta:0.0
sleep 2
crane_skill run EmplaceRobot $ROBOT_ID target_x:1.0 target_y:1.0 target_theta:1.57
sleep 2
crane_skill run EmplaceRobot $ROBOT_ID target_x:0.0 target_y:0.0 target_theta:0.0

# Test 3: Motion control
echo "3. Testing motion control..."
crane_skill run TestMotionPosition $ROBOT_ID target_x:0.5 target_y:0.5
sleep 3

# Test 4: Ball handling (if ball available)
echo "4. Testing ball handling..."
crane_skill run Kick $ROBOT_ID target_x:1.0 target_y:0.0 kick_power:2.0
sleep 2
crane_skill run Receive $ROBOT_ID

echo "✅ Basic robot tests completed"
```

### Workflow 2: Skill Development Testing

When developing or modifying skills:

```bash
#!/bin/bash
# skill_development_test.sh - Test new skill implementations

SKILL_NAME="Attacker"
ROBOT_ID=0

echo "Testing Skill: $SKILL_NAME"

# Preparation
echo "Preparing test environment..."
crane_skill run EmplaceRobot $ROBOT_ID target_x:0.0 target_y:0.0 target_theta:0.0
sleep 2

# Test iterations
for i in {1..5}; do
    echo "Test iteration $i/5"
    
    # Execute skill
    crane_skill run $SKILL_NAME $ROBOT_ID
    
    if [ $? -eq 0 ]; then
        echo "✅ Iteration $i successful"
    else
        echo "❌ Iteration $i failed"
    fi
    
    # Brief pause between tests
    sleep 3
    
    # Reset position
    crane_skill run EmplaceRobot $ROBOT_ID target_x:0.0 target_y:0.0 target_theta:0.0
    sleep 2
done

echo "Skill development testing completed"
```

### Workflow 3: Parameter Tuning

```bash
#!/bin/bash
# parameter_tuning.sh - Test different parameter values

ROBOT_ID=0

echo "Parameter Tuning for Kick Skill"

# Test different kick powers
for power in 1.0 2.0 3.0 4.0 5.0; do
    echo "Testing kick power: $power"
    
    # Position robot
    crane_skill run EmplaceRobot $ROBOT_ID target_x:0.0 target_y:0.0 target_theta:0.0
    sleep 1
    
    # Test kick
    crane_skill run Kick $ROBOT_ID target_x:2.0 target_y:0.0 kick_power:$power
    
    echo "Kick power $power completed. Press Enter to continue..."
    read
done

# Test different target positions
positions=(
    "1.0 0.0"
    "2.0 1.0"
    "1.5 -1.0"
    "3.0 0.5"
)

for pos in "${positions[@]}"; do
    read -r target_x target_y <<< "$pos"
    echo "Testing kick to position: ($target_x, $target_y)"
    
    crane_skill run EmplaceRobot $ROBOT_ID target_x:0.0 target_y:0.0 target_theta:0.0
    sleep 1
    
    crane_skill run Kick $ROBOT_ID target_x:$target_x target_y:$target_y kick_power:3.0
    
    echo "Position ($target_x, $target_y) completed. Press Enter to continue..."
    read
done

echo "Parameter tuning completed"
```

## Multi-Robot Scenarios

### Example 1: Formation Setup

```bash
#!/bin/bash
# formation_setup.sh - Set up standard team formation

echo "Setting up 4-3-3 formation"

# Goalkeepers
echo "Positioning goalkeeper..."
crane_skill run EmplaceRobot 0 target_x:-3.0 target_y:0.0 target_theta:0.0

# Defenders
echo "Positioning defenders..."
crane_skill multi EmplaceRobot 1,2,3 target_x:-1.5 target_y:0.0
crane_skill run EmplaceRobot 1 target_x:-1.5 target_y:1.0 target_theta:0.0
crane_skill run EmplaceRobot 2 target_x:-1.5 target_y:0.0 target_theta:0.0
crane_skill run EmplaceRobot 3 target_x:-1.5 target_y:-1.0 target_theta:0.0

# Midfielders
echo "Positioning midfielders..."
crane_skill run EmplaceRobot 4 target_x:0.0 target_y:1.0 target_theta:0.0
crane_skill run EmplaceRobot 5 target_x:0.0 target_y:0.0 target_theta:0.0
crane_skill run EmplaceRobot 6 target_x:0.0 target_y:-1.0 target_theta:0.0

# Attackers (if available)
echo "Positioning attackers..."
if [ $(crane_skill list | wc -l) -gt 7 ]; then
    crane_skill run EmplaceRobot 7 target_x:1.5 target_y:0.5 target_theta:0.0
    crane_skill run EmplaceRobot 8 target_x:1.5 target_y:0.0 target_theta:0.0
    crane_skill run EmplaceRobot 9 target_x:1.5 target_y:-0.5 target_theta:0.0
fi

echo "Formation setup complete"
sleep 3

# Activate behaviors
echo "Activating behaviors..."
crane_skill run Goalie 0
crane_skill multi SecondThreatDefender 1,2,3
crane_skill multi SubAttacker 4,5,6

if [ $(crane_skill list | wc -l) -gt 7 ]; then
    crane_skill multi Attacker 7,8,9
fi

echo "Team ready for play!"
```

### Example 2: Coordinated Attack

```bash
#!/bin/bash
# coordinated_attack.sh - Simulate coordinated attacking play

echo "Executing coordinated attack sequence"

# Initial positioning
echo "Phase 1: Initial positioning"
crane_skill run EmplaceRobot 0 target_x:0.0 target_y:0.0 target_theta:0.0     # Ball carrier
crane_skill run EmplaceRobot 1 target_x:1.0 target_y:1.0 target_theta:0.0     # Support 1
crane_skill run EmplaceRobot 2 target_x:1.0 target_y:-1.0 target_theta:0.0    # Support 2
crane_skill run EmplaceRobot 3 target_x:2.0 target_y:0.0 target_theta:0.0     # Target

sleep 3

# Start coordinated movement
echo "Phase 2: Coordinated advance"
crane_skill multi Attacker 0,1,2
crane_skill run Receive 3

sleep 5

# Execute pass
echo "Phase 3: Pass execution"
crane_skill run Kick 0 target_x:2.0 target_y:0.0 kick_power:4.0

sleep 2

# Follow-up positioning
echo "Phase 4: Follow-up positioning"
crane_skill multi SubAttacker 0,1,2
crane_skill run Attacker 3

echo "Coordinated attack completed"
```

### Example 3: Defensive Drill

```bash
#!/bin/bash
# defensive_drill.sh - Practice defensive scenarios

echo "Defensive positioning drill"

# Set up defensive line
echo "Setting up defensive line..."
robots=(1 2 3 4)
positions=("-1.0 1.5" "-1.0 0.5" "-1.0 -0.5" "-1.0 -1.5")

for i in ${!robots[@]}; do
    robot=${robots[$i]}
    pos=${positions[$i]}
    read -r x y <<< "$pos"
    crane_skill run EmplaceRobot $robot target_x:$x target_y:$y target_theta:0.0
done

# Position goalkeeper
crane_skill run EmplaceRobot 0 target_x:-2.5 target_y:0.0 target_theta:0.0

sleep 3

# Activate defensive behaviors
echo "Activating defensive behaviors..."
crane_skill run Goalie 0
crane_skill multi SecondThreatDefender 1,2,3,4

sleep 5

# Simulate threat response
echo "Simulating threat from right side..."
crane_skill run EmplaceRobot 5 target_x:0.0 target_y:-2.0 target_theta:1.57  # Attacker

# Defensive adjustment
sleep 2
crane_skill run StealBall 4  # Closest defender responds

sleep 3

echo "Defensive drill completed"
```

## Automated Testing

### Example 1: Regression Test Suite

```json
{
  "name": "Regression Test Suite",
  "description": "Comprehensive test of all basic skills",
  "skills": [
    {
      "name": "Sleep",
      "robot_id": 0,
      "parameters": {"duration": 0.5},
      "delay": 0,
      "description": "Test basic skill execution"
    },
    {
      "name": "Idle",
      "robot_id": 0,
      "parameters": {},
      "delay": 1,
      "description": "Test parameter-less skill"
    },
    {
      "name": "EmplaceRobot",
      "robot_id": 0,
      "parameters": {
        "target_x": 1.0,
        "target_y": 1.0,
        "target_theta": 0.785
      },
      "delay": 2,
      "description": "Test positioning skill"
    },
    {
      "name": "TestMotionPosition",
      "robot_id": 0,
      "parameters": {
        "target_x": 0.0,
        "target_y": 0.0
      },
      "delay": 2,
      "description": "Test motion control"
    },
    {
      "name": "Kick",
      "robot_id": 0,
      "parameters": {
        "target_x": 2.0,
        "target_y": 0.0,
        "kick_power": 3.0
      },
      "delay": 2,
      "description": "Test ball interaction"
    },
    {
      "name": "Receive",
      "robot_id": 0,
      "parameters": {},
      "delay": 1,
      "description": "Test ball receiving"
    }
  ]
}
```

Run the regression test:
```bash
crane_skill scenario regression_test.json
```

### Example 2: Performance Benchmark

```bash
#!/bin/bash
# performance_benchmark.sh - Measure skill execution performance

ROBOT_ID=0
ITERATIONS=10

echo "Performance Benchmark - $ITERATIONS iterations"

# Test different skills
skills=("Sleep" "Idle" "EmplaceRobot" "TestMotionPosition" "Kick")

for skill in "${skills[@]}"; do
    echo "Benchmarking skill: $skill"
    total_time=0
    
    for i in $(seq 1 $ITERATIONS); do
        echo -n "  Iteration $i/$ITERATIONS... "
        
        start_time=$(date +%s.%N)
        
        case $skill in
            "Sleep")
                crane_skill run Sleep $ROBOT_ID duration:0.1 > /dev/null 2>&1
                ;;
            "Idle")
                crane_skill run Idle $ROBOT_ID > /dev/null 2>&1
                ;;
            "EmplaceRobot")
                crane_skill run EmplaceRobot $ROBOT_ID target_x:0.0 target_y:0.0 target_theta:0.0 > /dev/null 2>&1
                ;;
            "TestMotionPosition")
                crane_skill run TestMotionPosition $ROBOT_ID target_x:0.5 target_y:0.5 > /dev/null 2>&1
                ;;
            "Kick")
                crane_skill run Kick $ROBOT_ID target_x:1.0 target_y:0.0 kick_power:2.0 > /dev/null 2>&1
                ;;
        esac
        
        end_time=$(date +%s.%N)
        execution_time=$(echo "$end_time - $start_time" | bc)
        total_time=$(echo "$total_time + $execution_time" | bc)
        
        printf "%.3fs\n" $execution_time
    done
    
    average_time=$(echo "scale=3; $total_time / $ITERATIONS" | bc)
    echo "  Average execution time: ${average_time}s"
    echo
done

echo "Performance benchmark completed"
```

### Example 3: Stress Test

```bash
#!/bin/bash
# stress_test.sh - Test system under load

echo "Stress test - Multiple rapid executions"

# Test rapid execution
echo "Phase 1: Rapid execution test"
for i in {1..50}; do
    echo -n "Execution $i/50... "
    crane_skill run Sleep 0 duration:0.1 > /dev/null 2>&1
    if [ $? -eq 0 ]; then
        echo "✅"
    else
        echo "❌"
    fi
done

# Test multi-robot load
echo "Phase 2: Multi-robot load test"
for round in {1..10}; do
    echo "Round $round/10: Multi-robot execution"
    crane_skill multi Idle 0,1,2,3,4,5 > /dev/null 2>&1
    if [ $? -eq 0 ]; then
        echo "✅ Round $round successful"
    else
        echo "❌ Round $round failed"
    fi
    sleep 1
done

echo "Stress test completed"
```

## Real-World Use Cases

### Use Case 1: Match Preparation

```bash
#!/bin/bash
# match_preparation.sh - Pre-match robot testing

echo "Match Preparation Checklist"

# Test all robots individually
robots=(0 1 2 3 4 5)

for robot in "${robots[@]}"; do
    echo "Testing Robot $robot"
    
    # Basic responsiveness
    crane_skill run Idle $robot
    if [ $? -ne 0 ]; then
        echo "❌ Robot $robot not responding!"
        continue
    fi
    
    # Position control
    crane_skill run EmplaceRobot $robot target_x:0.0 target_y:0.0 target_theta:0.0
    sleep 1
    
    # Motion test
    crane_skill run TestMotionPosition $robot target_x:0.5 target_y:0.5
    sleep 2
    
    # Return to start
    crane_skill run EmplaceRobot $robot target_x:0.0 target_y:0.0 target_theta:0.0
    
    echo "✅ Robot $robot ready"
done

# Team coordination test
echo "Testing team coordination..."
crane_skill multi EmplaceRobot 0,1,2,3,4,5 target_x:1.0 target_y:0.0

sleep 3

crane_skill multi Idle 0,1,2,3,4,5

echo "✅ Match preparation completed"
```

### Use Case 2: Skill Calibration

```bash
#!/bin/bash
# skill_calibration.sh - Calibrate skill parameters

ROBOT_ID=0

echo "Skill Calibration Session"

# Kick power calibration
echo "Calibrating kick power..."
read -p "Place ball at robot position and press Enter..."

for power in 1.0 2.0 3.0 4.0 5.0; do
    echo "Testing kick power: $power"
    crane_skill run Kick $ROBOT_ID target_x:2.0 target_y:0.0 kick_power:$power
    
    read -p "Measure ball distance. Good power level? (y/n): " response
    if [ "$response" = "y" ]; then
        echo "Optimal kick power: $power"
        break
    fi
    
    read -p "Reset ball position and press Enter..."
done

# Position accuracy calibration
echo "Calibrating position accuracy..."
positions=("1.0 0.0" "0.0 1.0" "-1.0 0.0" "0.0 -1.0")

for pos in "${positions[@]}"; do
    read -r x y <<< "$pos"
    echo "Testing position: ($x, $y)"
    
    crane_skill run EmplaceRobot $ROBOT_ID target_x:$x target_y:$y target_theta:0.0
    
    read -p "Check robot position. Accurate? (y/n): " response
    if [ "$response" = "n" ]; then
        echo "Position inaccuracy detected at ($x, $y)"
    fi
done

echo "Calibration session completed"
```

### Use Case 3: Tournament Debugging

```bash
#!/bin/bash
# tournament_debug.sh - Quick debugging during tournament

echo "Tournament Quick Debug"

# Check all critical systems
echo "1. System health check..."
if ! ros2 action list | grep -q skill_execution; then
    echo "❌ Critical: Action server not available!"
    exit 1
fi

echo "2. Robot communication test..."
failed_robots=()

for robot in {0..5}; do
    echo -n "Testing robot $robot... "
    if crane_skill run Idle $robot > /dev/null 2>&1; then
        echo "✅"
    else
        echo "❌"
        failed_robots+=($robot)
    fi
done

if [ ${#failed_robots[@]} -gt 0 ]; then
    echo "⚠️  Failed robots: ${failed_robots[*]}"
else
    echo "✅ All robots responsive"
fi

# Quick formation test
echo "3. Formation test..."
crane_skill multi EmplaceRobot 0,1,2,3,4,5 target_x:0.0 target_y:0.0

if [ $? -eq 0 ]; then
    echo "✅ Formation control working"
else
    echo "❌ Formation control issues"
fi

# Quick ball handling test
echo "4. Ball handling test..."
crane_skill run Kick 0 target_x:1.0 target_y:0.0 kick_power:2.0

if [ $? -eq 0 ]; then
    echo "✅ Ball handling working"
else
    echo "❌ Ball handling issues"
fi

echo "Quick debug completed"
```

## Integration Examples

### Example 1: CI/CD Pipeline Integration

```yaml
# .github/workflows/crane_test.yml
name: Crane Skills Test

on: [push, pull_request]

jobs:
  test:
    runs-on: ubuntu-latest
    container: ros:jazzy
    
    steps:
    - uses: actions/checkout@v3
    
    - name: Build workspace
      run: |
        source /opt/ros/jazzy/setup.bash
        colcon build --packages-select crane_debug_tools
        
    - name: Test basic functionality
      run: |
        source install/local_setup.bash
        crane_skill list
        
    - name: Run regression tests
      run: |
        source install/local_setup.bash
        # Note: Requires running crane system for full test
        # crane_skill scenario tests/regression_test.json
```

### Example 2: Docker Integration

```dockerfile
# Dockerfile.crane_debug
FROM ros:jazzy

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Copy workspace
COPY . /workspace
WORKDIR /workspace

# Build
RUN source /opt/ros/jazzy/setup.bash && \
    colcon build --packages-select crane_debug_tools

# Setup environment
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "source /workspace/install/local_setup.bash" >> ~/.bashrc

# Default command
CMD ["/bin/bash"]
```

Usage:
```bash
# Build image
docker build -f Dockerfile.crane_debug -t crane_debug .

# Run container
docker run -it --rm crane_debug

# Inside container
crane_skill list
```

### Example 3: Python Script Integration

```python
#!/usr/bin/env python3
# integration_example.py - Use crane_debug_tools from Python

import subprocess
import json
import time
import sys

class CraneDebugInterface:
    """Python interface to crane_debug_tools"""
    
    def execute_skill(self, skill_name, robot_id, parameters=None):
        """Execute a skill and return success status"""
        cmd = ['crane_skill', 'run', skill_name, str(robot_id)]
        
        if parameters:
            for key, value in parameters.items():
                cmd.append(f"{key}:{value}")
        
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
            return result.returncode == 0, result.stdout, result.stderr
        except subprocess.TimeoutExpired:
            return False, "", "Timeout"
    
    def execute_scenario(self, scenario_file):
        """Execute a scenario file"""
        cmd = ['crane_skill', 'scenario', scenario_file]
        
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=300)
            return result.returncode == 0, result.stdout, result.stderr
        except subprocess.TimeoutExpired:
            return False, "", "Timeout"
    
    def get_available_skills(self):
        """Get list of available skills"""
        cmd = ['crane_skill', 'list']
        
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
            if result.returncode == 0:
                return result.stdout.strip().split('\n')
            else:
                return []
        except subprocess.TimeoutExpired:
            return []

def main():
    """Example usage of CraneDebugInterface"""
    debug = CraneDebugInterface()
    
    # Test basic functionality
    print("Testing crane debug interface...")
    
    # Get available skills
    skills = debug.get_available_skills()
    print(f"Available skills: {len(skills)}")
    
    # Test simple skill
    success, stdout, stderr = debug.execute_skill("Sleep", 0, {"duration": 1.0})
    if success:
        print("✅ Sleep skill test passed")
    else:
        print(f"❌ Sleep skill test failed: {stderr}")
    
    # Test positioning
    success, stdout, stderr = debug.execute_skill("EmplaceRobot", 0, {
        "target_x": 1.0,
        "target_y": 2.0,
        "target_theta": 0.5
    })
    if success:
        print("✅ Position skill test passed")
    else:
        print(f"❌ Position skill test failed: {stderr}")
    
    print("Integration example completed")

if __name__ == "__main__":
    main()
```

## Custom Scenarios

### Example 1: Penalty Kick Scenario

```json
{
  "name": "Penalty Kick Practice",
  "description": "Practice penalty kick execution and goalkeeper response",
  "skills": [
    {
      "name": "EmplaceRobot",
      "robot_id": 0,
      "parameters": {
        "target_x": -3.0,
        "target_y": 0.0,
        "target_theta": 0.0
      },
      "delay": 0,
      "description": "Position goalkeeper"
    },
    {
      "name": "EmplaceRobot",
      "robot_id": 1,
      "parameters": {
        "target_x": -0.5,
        "target_y": 0.0,
        "target_theta": 0.0
      },
      "delay": 2,
      "description": "Position penalty kicker"
    },
    {
      "name": "Goalie",
      "robot_id": 0,
      "parameters": {},
      "delay": 1,
      "description": "Activate goalkeeper behavior"
    },
    {
      "name": "PenaltyKick",
      "robot_id": 1,
      "parameters": {},
      "delay": 5,
      "description": "Execute penalty kick"
    },
    {
      "name": "Idle",
      "robot_id": 0,
      "parameters": {},
      "delay": 0,
      "description": "Stop goalkeeper after attempt"
    },
    {
      "name": "Idle",
      "robot_id": 1,
      "parameters": {},
      "delay": 0,
      "description": "Stop kicker after attempt"
    }
  ]
}
```

### Example 2: Corner Kick Scenario

```json
{
  "name": "Corner Kick Setup",
  "description": "Practice corner kick execution and positioning",
  "skills": [
    {
      "name": "EmplaceRobot",
      "robot_id": 0,
      "parameters": {
        "target_x": 2.8,
        "target_y": 1.8,
        "target_theta": -2.356
      },
      "delay": 0,
      "description": "Position corner kick taker"
    },
    {
      "name": "EmplaceRobot",
      "robot_id": 1,
      "parameters": {
        "target_x": 1.5,
        "target_y": 0.0,
        "target_theta": 0.0
      },
      "delay": 0,
      "description": "Position target player"
    },
    {
      "name": "EmplaceRobot",
      "robot_id": 2,
      "parameters": {
        "target_x": 0.5,
        "target_y": -1.0,
        "target_theta": 0.0
      },
      "delay": 0,
      "description": "Position support player"
    },
    {
      "name": "EmplaceRobot",
      "robot_id": 3,
      "parameters": {
        "target_x": -2.5,
        "target_y": 0.0,
        "target_theta": 0.0
      },
      "delay": 2,
      "description": "Position goalkeeper"
    },
    {
      "name": "Receive",
      "robot_id": 1,
      "parameters": {},
      "delay": 0,
      "description": "Prepare target to receive"
    },
    {
      "name": "Receive",
      "robot_id": 2,
      "parameters": {},
      "delay": 0,
      "description": "Prepare support to receive"
    },
    {
      "name": "Goalie",
      "robot_id": 3,
      "parameters": {},
      "delay": 1,
      "description": "Activate goalkeeper"
    },
    {
      "name": "Kick",
      "robot_id": 0,
      "parameters": {
        "target_x": 1.5,
        "target_y": 0.0,
        "kick_power": 4.0
      },
      "delay": 3,
      "description": "Execute corner kick"
    },
    {
      "name": "Attacker",
      "robot_id": 1,
      "parameters": {},
      "delay": 0,
      "description": "Switch to attacking"
    },
    {
      "name": "SubAttacker",
      "robot_id": 2,
      "parameters": {},
      "delay": 0,
      "description": "Support the attack"
    }
  ]
}
```

### Example 3: Full Match Simulation

```json
{
  "name": "Mini Match Simulation",
  "description": "Simulate a brief match scenario with multiple phases",
  "skills": [
    {
      "name": "EmplaceRobot",
      "robot_id": 0,
      "parameters": {"target_x": -3.0, "target_y": 0.0, "target_theta": 0.0},
      "delay": 0,
      "description": "Position Team A goalkeeper"
    },
    {
      "name": "EmplaceRobot",
      "robot_id": 1,
      "parameters": {"target_x": -1.0, "target_y": 0.0, "target_theta": 0.0},
      "delay": 0,
      "description": "Position Team A defender"
    },
    {
      "name": "EmplaceRobot",
      "robot_id": 2,
      "parameters": {"target_x": 0.0, "target_y": 0.0, "target_theta": 0.0},
      "delay": 0,
      "description": "Position Team A midfielder"
    },
    {
      "name": "EmplaceRobot",
      "robot_id": 3,
      "parameters": {"target_x": 3.0, "target_y": 0.0, "target_theta": 3.14159},
      "delay": 0,
      "description": "Position Team B goalkeeper"
    },
    {
      "name": "EmplaceRobot",
      "robot_id": 4,
      "parameters": {"target_x": 1.0, "target_y": 0.0, "target_theta": 3.14159},
      "delay": 0,
      "description": "Position Team B defender"
    },
    {
      "name": "EmplaceRobot",
      "robot_id": 5,
      "parameters": {"target_x": 0.5, "target_y": 0.0, "target_theta": 3.14159},
      "delay": 3,
      "description": "Position Team B midfielder"
    },
    {
      "name": "Goalie",
      "robot_id": 0,
      "parameters": {},
      "delay": 0,
      "description": "Activate Team A goalkeeper"
    },
    {
      "name": "Goalie",
      "robot_id": 3,
      "parameters": {},
      "delay": 1,
      "description": "Activate Team B goalkeeper"
    },
    {
      "name": "SimpleKickOff",
      "robot_id": 2,
      "parameters": {},
      "delay": 3,
      "description": "Team A kickoff"
    },
    {
      "name": "Attacker",
      "robot_id": 2,
      "parameters": {},
      "delay": 0,
      "description": "Team A attacks"
    },
    {
      "name": "SecondThreatDefender",
      "robot_id": 1,
      "parameters": {},
      "delay": 0,
      "description": "Team A supports"
    },
    {
      "name": "StealBall",
      "robot_id": 4,
      "parameters": {},
      "delay": 0,
      "description": "Team B defends"
    },
    {
      "name": "SecondThreatDefender",
      "robot_id": 5,
      "parameters": {},
      "delay": 10,
      "description": "Team B supports defense"
    },
    {
      "name": "Attacker",
      "robot_id": 4,
      "parameters": {},
      "delay": 0,
      "description": "Team B counter-attacks"
    },
    {
      "name": "SubAttacker",
      "robot_id": 5,
      "parameters": {},
      "delay": 0,
      "description": "Team B supports attack"
    },
    {
      "name": "StealBall",
      "robot_id": 1,
      "parameters": {},
      "delay": 8,
      "description": "Team A regains possession"
    },
    {
      "name": "Idle",
      "robot_id": 0,
      "parameters": {},
      "delay": 0,
      "description": "End simulation - Team A goalkeeper"
    },
    {
      "name": "Idle",
      "robot_id": 1,
      "parameters": {},
      "delay": 0,
      "description": "End simulation - Team A defender"
    },
    {
      "name": "Idle",
      "robot_id": 2,
      "parameters": {},
      "delay": 0,
      "description": "End simulation - Team A midfielder"
    },
    {
      "name": "Idle",
      "robot_id": 3,
      "parameters": {},
      "delay": 0,
      "description": "End simulation - Team B goalkeeper"
    },
    {
      "name": "Idle",
      "robot_id": 4,
      "parameters": {},
      "delay": 0,
      "description": "End simulation - Team B defender"
    },
    {
      "name": "Idle",
      "robot_id": 5,
      "parameters": {},
      "delay": 0,
      "description": "End simulation - Team B midfielder"
    }
  ]
}
```

Execute scenarios:
```bash
# Run individual scenarios
crane_skill scenario penalty_kick.json
crane_skill scenario corner_kick.json
crane_skill scenario mini_match.json

# Time scenario execution
time crane_skill scenario mini_match.json
```

These examples provide comprehensive coverage of crane_debug_tools usage patterns, from simple skill testing to complex multi-robot scenarios and real-world applications.