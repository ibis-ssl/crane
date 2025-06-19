# Crane Debug Tools

Modern debugging and testing tools for the Crane robot soccer system, designed to replace the Qt-based `crane_simple_ai` with more flexible and modern alternatives.

## Features

### 🌐 Web-Based Interface
- Modern, responsive web interface accessible from any device
- Real-time visualization of robot positions and ball tracking
- Interactive skill execution with parameter configuration
- Live execution logs and status monitoring

### 🖥️ Command Line Interface (CLI)
- Lightweight CLI tool for quick skill testing
- Batch execution with scenario files
- Multi-robot coordination testing
- Scriptable and automation-friendly

### 🔧 Enhanced Functionality
- Support for all crane robot skills
- Real-time parameter adjustment
- Multi-robot coordination testing
- Automated test scenario execution
- Performance monitoring and logging

## Quick Start

### Installation

The tools are built automatically with the crane workspace:

```bash
# Build the tools
colcon build --packages-select crane_debug_tools

# Source the workspace
source install/local_setup.bash
```

### Web Interface

1. **Launch the debug tools:**
   ```bash
   ros2 launch crane_debug_tools debug_tools.launch.py enable_web:=true
   ```

2. **Open your web browser and navigate to:**
   ```
   http://localhost:8081
   ```

3. **Start using the interface:**
   - Select a skill from the left panel
   - Configure robot ID and parameters
   - Click "Execute Skill" to run

### CLI Interface

1. **Interactive CLI mode:**
   ```bash
   ros2 run crane_debug_tools crane_skill_cli
   ```

2. **Direct skill execution:**
   ```bash
   # Execute a skill with parameters
   crane_skill run Kick 0 target_x:1.0 target_y:2.0 kick_power:5.0
   
   # Execute on multiple robots
   crane_skill multi Attacker 0,1,2
   
   # List available skills
   crane_skill list
   ```

3. **Scenario execution:**
   ```bash
   crane_skill scenario test_sequence.json
   ```

## Usage Examples

### Basic Skill Testing

**Web Interface:**
1. Select "Kick" skill from the skills list
2. Set robot ID to 0
3. Configure parameters: target_x=1.0, target_y=2.0, kick_power=5.0
4. Click "Execute Skill"

**CLI:**
```bash
crane_skill run Kick 0 target_x:1.0 target_y:2.0 kick_power:5.0
```

### Multi-Robot Coordination

**CLI example:**
```bash
# Execute Attacker skill on robots 0, 1, and 2
crane_skill multi Attacker 0,1,2
```

### Automated Test Scenarios

Create a JSON scenario file (`test_sequence.json`):
```json
{
  "skills": [
    {
      "name": "EmplaceRobot",
      "robot_id": 0,
      "parameters": {"target_x": 1.0, "target_y": 0.0, "target_theta": 0.0},
      "delay": 2
    },
    {
      "name": "Kick",
      "robot_id": 0,
      "parameters": {"target_x": 3.0, "target_y": 1.0, "kick_power": 5.0},
      "delay": 1
    }
  ]
}
```

Execute the scenario:
```bash
crane_skill scenario test_sequence.json
```

## Available Skills

The debug tools support all crane robot skills:

**Basic Skills:**
- `Sleep` - Pause robot for specified duration
- `Idle` - Keep robot in idle state
- `EmplaceRobot` - Move robot to specific position and orientation

**Game Skills:**
- `Kick` - Kick ball toward target
- `Receive` - Position to receive ball
- `Goalie` - Goalkeeper behavior
- `Attacker` - Attacking behavior
- `SubAttacker` - Supporting attacker behavior
- `StealBall` - Aggressive ball retrieval

**Formation Skills:**
- `SingleBallPlacement` - Ball placement for set pieces
- `GoalKick` - Goal kick execution
- `SimpleKickOff` - Basic kickoff behavior
- `KickOffAttack` - Attacking kickoff
- `KickOffSupport` - Supporting kickoff

**Testing Skills:**
- `TestMotionPosition` - Test position control
- `TestMotionVelocity` - Test velocity control
- `Marker` - Visual marker for debugging
- `Teleop` - Manual robot control

## Configuration

### Launch Parameters

```bash
# Custom web port
ros2 launch crane_debug_tools debug_tools.launch.py web_port:=9090

# Enable only CLI tools
ros2 launch crane_debug_tools debug_tools.launch.py enable_web:=false enable_cli:=true
```

### Web Interface Configuration

The web interface automatically connects to the WebSocket server on port 8080. If you change the port, update the JavaScript configuration in `web/app.js`.

## Architecture

### Web Bridge Server
- **Purpose:** Bridge between ROS 2 and web interface
- **Technology:** WebSocket server with JSON messaging
- **Features:** Real-time data streaming, skill execution, parameter management

### CLI Tools
- **crane_skill_cli:** Interactive CLI for skill testing
- **crane_skill:** Python script for batch operations and automation

### Communication Flow
```
Web Interface ←→ WebSocket Bridge ←→ ROS 2 Actions ←→ crane_robot_skills
CLI Tools ←→ ROS 2 Actions ←→ crane_robot_skills
```

## Integration with Crane System

The debug tools integrate seamlessly with the existing crane system:

1. **Action Interface:** Uses the same `SkillExecution` action as `crane_simple_ai`
2. **World Model:** Subscribes to world model updates for visualization
3. **Robot Commands:** Monitors robot commands for debugging
4. **Launch Integration:** Can be enabled/disabled in main crane launch

### Replacing crane_simple_ai

To use the new debug tools instead of the Qt-based simple AI:

```bash
# Old way (Qt GUI)
ros2 launch crane_bringup crane.launch.py simple_ai:=true

# New way (Web interface)
ros2 launch crane_bringup crane.launch.py
ros2 launch crane_debug_tools debug_tools.launch.py enable_web:=true
```

## Development

### Dependencies
- ROS 2 Jazzy
- WebSocket++ (for web bridge)
- nlohmann/json (for JSON handling)
- Bootstrap 5 (for web UI)

### Building
```bash
colcon build --packages-select crane_debug_tools
```

### Testing
```bash
colcon test --packages-select crane_debug_tools
```

## Advantages Over crane_simple_ai

1. **Cross-platform:** Web interface works on any device with a browser
2. **Remote debugging:** Access debug tools from anywhere on the network
3. **Modern UI:** Responsive, mobile-friendly interface
4. **Automation:** CLI tools support scripting and batch operations
5. **Multi-robot:** Built-in support for coordinating multiple robots
6. **Real-time:** Live updates of robot positions and game state
7. **Extensible:** Easy to add new features and visualization

## Troubleshooting

### Web Interface Not Loading
- Check that the web server is running on port 8081
- Ensure WebSocket bridge is running on port 8080
- Check browser console for error messages

### CLI Commands Not Working
- Verify ROS 2 environment is sourced
- Check that crane_robot_skills action server is running
- Ensure robot skills action server is available at `/simple_ai/skill_execution`

### Skills Not Executing
- Verify connection to skill execution action server
- Check ROS 2 node graph: `ros2 node list`
- Monitor action server status: `ros2 action list`

## Contributing

When adding new skills or parameters:

1. Update the skills list in both web and CLI interfaces
2. Add parameter definitions in the web interface
3. Update documentation and examples
4. Test with both interface types