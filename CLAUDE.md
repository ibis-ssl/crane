# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Crane is a ROS 2-based autonomous robotics system for RoboCup Small Size League (SSL) competitions. It's an AI framework for controlling a team of small autonomous robots in soccer matches, built by the ibis-ssl team using ROS 2 Jazzy.

## Build and Development Commands

### Initial Setup

```bash
# Clone dependencies and setup workspace
vcs import src < src/crane/dependency_jazzy.repos
rosdep install -riy --from-paths src

# Build with symlink install for development
colcon build --symlink-install

# Source environment
source install/local_setup.bash
```

### Development Build Commands

```bash
# Standard development build
colcon build --symlink-install

# Release build with coverage (CI configuration)
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --mixin coverage-gcc coverage-pytest compile-commands

# Build specific packages
colcon build --packages-select crane_world_model_publisher crane_planner_plugins
```

### Testing

```bash
# Run all tests
colcon test --event-handlers console_cohesion+

# Run tests for specific packages
colcon test --packages-select crane_basics crane_sender --event-handlers console_cohesion+

# Run individual test by name (using regex)
colcon test --packages-select crane_basics --event-handlers console_cohesion+ --ctest-args -R test_ball_msg_conversion

# Run scenario tests (Python integration tests)
cd scenario_test
python3 emit_from_penalty_01.py
python3 STOP_ROBOT_SPEED.py

# Build before testing (required for changes)
colcon build --packages-select <package_name>
source install/local_setup.bash
```

### Launching the System

```bash
# Main system launch with simulation
ros2 launch crane_bringup crane.launch.py sim:=true

# Communication components only
ros2 launch robocup_ssl_comm comm.launch.py

# Data processing pipeline
ros2 launch crane_bringup data.launch.py
```

## Architecture Overview

### Core Components

- **crane_session_controller**: Main match orchestration and game state management
- **crane_planner_plugins**: Strategy planners (offense, defense, formations) using plugin architecture
- **crane_robot_skills**: Individual robot behaviors (goalie, attacker, defender skills)
- **crane_local_planner**: Real-time path planning with RVO2-based collision avoidance
- **crane_world_model_publisher**: World state estimation and ball/robot tracking
- **crane_play_switcher**: Game situation analysis and automatic play selection

### Message Flow Architecture

The system uses a distributed ROS 2 node architecture where each component runs as a separate node:

1. SSL vision/referee data flows through `robocup_ssl_comm`
2. World model processes and publishes game state via `crane_world_model_publisher`
3. Session controller orchestrates high-level match flow
4. Planner plugins generate robot assignments and strategies
5. Robot skills translate strategies into low-level robot commands
6. Commands are sent via `crane_sender` to simulation or real robots

### Key Directories

- `session/` - High-level match control and strategy planners
- `utility/` - Shared utilities including `crane_basics` for geometric operations
- `consai_ros2/` - SSL protocol communication (vision, referee, commands)
- `crane_msgs/` - Custom ROS 2 message definitions
- `3rdparty/` - Third-party dependencies (RVO2, matplotlib, etc.)

## Development Environment

### Docker Simulation Environment

```bash
# Start simulation environment with grSim and game controller
cd docker/sim
docker compose up -d

# Access simulation interfaces:
# - Game Controller: http://localhost:8081  
# - Vision Client: http://localhost:8082
# - Status Board: http://localhost:8083
```

### Real Robot Environment

```bash
cd docker/real
docker compose up -d
```

## Code Conventions

### Build System

- Uses `ament_cmake_auto` for automatic CMake configuration
- Each package has standardized CMakeLists.txt structure
- Custom linting via `crane_lint_common` package
- C++20 standard with compiler flags: `-Wall -Wextra -Wpedantic -g`

### Testing Structure

- Unit tests in `test/` directories within each package using GTest
- Integration tests in `scenario_test/` using Python RCST framework
- CI/CD runs comprehensive test suites including scenario tests
- Pre-commit hooks with clang-format, cpplint, ruff, and ROS-specific linting

### Message Definitions

- Custom messages in `crane_msgs/` package
- SSL protocol messages in `consai_ros2/robocup_ssl_msgs`
- Visualization messages in `crane_visualization_interfaces`
- Ball struct and BallInfo.msg conversion using position.z/velocity.z for 3D coordinates

### ROS 2 Package Dependencies

Core dependency hierarchy:

1. **Message Layer**: `crane_msgs`, `robocup_ssl_msgs`, `crane_visualization_interfaces`
2. **Utility Layer**: `crane_basics` (geometry, physics), `crane_msg_wrappers`
3. **Component Layer**: `crane_world_model_publisher`, `crane_game_analyzer`, `crane_robot_skills`
4. **Planning Layer**: `crane_session_controller`, `crane_planner_plugins`, `crane_local_planner`
5. **Integration Layer**: `crane_bringup`, `crane_sender`, `robocup_ssl_comm`

## Special Development Considerations

### Real-time Constraints

- System operates under real-time constraints for robot control
- Ball physics simulation requires accurate prediction models
- Multi-robot coordination uses RVO2 algorithm for collision avoidance
- Network communication timing is critical for SSL protocol compliance

### Coordinate Systems

- Field coordinate system follows SSL specifications
- Geometric operations use custom Vector2d/Vector3d classes (not Eigen)
- Ball model includes physics simulation with configurable parameters

### Ball Physics and Message Conversion

- Ball struct implements state-aware physics (STOPPED, ROLLING, FLYING)
- 3D parabolic motion for flying balls with air resistance and gravity
- Template conversion functions `toMsg()` and `fromMsg()` for ROS 2 message compatibility
- Uses `position.z` and `velocity.z` from geometry_msgs/Vector3 for 3D coordinates
- Ball state estimation based on velocity and height for autonomous tracking

### Plugin Architecture

- Planner strategies implement plugin interface for modularity
- Skills system allows composable robot behaviors
- Configuration-driven parameter management throughout system

### Documentation

- Main documentation in `docs/` folder (Japanese)
- Architecture details in individual component documentation
- Development logs maintained in `docs/logs/`
