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
# Standard development build (最適化済み)
colcon build --symlink-install

# 最適化されたビルドスクリプト使用
./src/crane/scripts/optimized_build.bash

# クリーンビルド（最適化済み）
./src/crane/scripts/optimized_build.bash clean

# Release build with coverage (CI configuration)
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --mixin coverage-gcc coverage-pytest compile-commands

# Build specific packages
colcon build --packages-select crane_world_model_publisher crane_planner_plugins

# ベンチマークビルド（最適化前後の比較）
./src/crane/scripts/optimized_build.bash benchmark
```

### ビルド時間最適化

このリポジトリには以下のビルド時間最適化が実装されています：

- **colcon.meta**: パッケージ別並列ビルド設定とコンパイラ最適化
- **Git shallow clone**: vendor パッケージの高速クローン
- **警告抑制**: 不要な警告を抑制してログを簡素化
- **最適化スクリプト**: `scripts/optimized_build.bash` で自動化

**ビルド時間**:

- 最適化前: 7分18秒（33パッケージ）
- 最適化後: 目標5分30秒（20-30%削減）

**詳細**: `docs/logs/portal/build_optimization_guide.md` を参照

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

## Git Repository Management

### CRITICAL: Files to NEVER Commit

**ALWAYS verify these directories/files are NOT committed to git:**

- `build/` - Contains all CMake build artifacts, object files, executables
- `install/` - Contains ROS 2 installation files and symlinks
- `log/` - Contains build and test logs
- `.idea/` - IntelliJ IDEA/CLion IDE configuration files
- `.vscode/` - Visual Studio Code IDE configuration files
- `cmake-build-*/` - CLion build directories
- `*.o`, `*.so`, `*.a` - Compiled object files and libraries
- `CMakeCache.txt`, `CMakeFiles/` - CMake cache and generated files

### .gitignore Verification

Before any commit, ensure `.gitignore` properly excludes:

```gitignore
# Build directories
build/
install/
log/

# IDE specific
.vscode/
.idea/
**/cmake-build-debug/
**/cmake-build-*/

# CMake
CMakeCache.txt
CMakeFiles/
cmake_install.cmake
*.cmake
CTestConfiguration.ini
CTestCustom.cmake
CTestTestfile.cmake

# Compiled Object files
*.o
*.obj

# Libraries
*.lib
*.a
*.la
*.lo
*.so
*.so.*
*.dylib

# Executables
*.exe
*.out
*.app

# Testing
Testing/

# Ament
ament_cmake_*/
```

### Pre-Commit Checks

**MANDATORY before every commit:**

```bash
# 1. Check git status for unwanted files
git status

# 2. Verify no build artifacts are staged
git diff --cached --name-only | grep -E "(build/|install/|log/|\.idea|\.vscode|\.o$|\.so$|CMakeCache\.txt)"

# 3. If any build artifacts found, remove them:
git rm -r --cached build/ install/ log/ .idea/ .vscode/ || true
git reset HEAD -- build/ install/ log/ .idea/ .vscode/ || true

# 4. Clean workspace if needed
rm -rf build/ install/ log/
```

### Emergency Cleanup (if build artifacts were committed)

If build artifacts were accidentally committed:

```bash
# Remove from current commit
git rm -r --cached build/ install/ log/ .idea/ .vscode/
git commit -m "Remove build artifacts and IDE settings from git tracking"

# For past commits (USE WITH CAUTION - rewrites history)
FILTER_BRANCH_SQUELCH_WARNING=1 git filter-branch --force --index-filter \
  'git rm -rf --cached --ignore-unmatch build install log .idea .vscode cmake-build-* */cmake-build-*' \
  --prune-empty HEAD~20..HEAD
```

## Commit Message Standards

### Language Policy

**ALL commit messages in this repository MUST be written in Japanese.**

### Commit Message Format

Use the following format for all commits:

```
[カテゴリ]概要（50文字以内）

詳細説明（任意、72文字で改行）
- 具体的な変更内容
- 影響範囲や理由の説明
- 必要に応じて参考情報

🤖 Generated with [Claude Code](https://claude.ai/code)

Co-Authored-By: Claude <noreply@anthropic.com>
```

### Commit Categories (カテゴリ)

Use these standardized categories in Japanese:

- **機能追加**: 新機能の追加 (feat)
- **バグ修正**: バグの修正 (fix)
- **リファクタリング**: コード改善（機能変更なし）
- **ドキュメント**: ドキュメントの更新
- **テスト**: テストの追加・修正
- **ビルド**: ビルドシステムの変更
- **CI/CD**: CI/CDの設定変更
- **設定**: 設定ファイルの変更
- **翻訳**: 言語ファイルの翻訳
- **クリーンアップ**: 不要ファイル削除、整理

### Good Commit Examples

**機能追加の例:**

```
機能追加: crane_debug_toolsにWebインターフェースを追加

- WebSocketサーバーによるリアルタイム通信機能
- ブラウザベースのスキル実行インターフェース
- ロボット位置の可視化機能
- CLI との併用をサポート

🤖 Generated with [Claude Code](https://claude.ai/code)

Co-Authored-By: Claude <noreply@anthropic.com>
```

**ドキュメントの例:**

```
ドキュメント: crane_debug_toolsの日本語翻訳

全てのAPIリファレンスとユーザーガイドを英語から日本語に翻訳
- 技術用語の統一と日本語開発者向けアクセシビリティ向上
- コードブロックとコマンド例は元のまま保持

🤖 Generated with [Claude Code](https://claude.ai/code)

Co-Authored-By: Claude <noreply@anthropic.com>
```

**バグ修正の例:**

```
バグ修正: ロボットスキル実行時のタイムアウト問題を解決

ActionServerのタイムアウト値を10秒から30秒に変更
- 複雑なスキル実行時の予期しない中断を防止
- crane_robot_skills.hpp:45でタイムアウト定数を更新

🤖 Generated with [Claude Code](https://claude.ai/code)

Co-Authored-By: Claude <noreply@anthropic.com>
```

### Commit Granularity (コミット粒度)

**Fine-grained commits preferred:**

- One logical change per commit
- Separate different types of changes (code vs docs vs config)
- Each commit should be buildable and functional

**Examples of proper granularity:**

```bash
# Good: Separate commits for different aspects
git commit -m "機能追加: 新しいKickスキルのパラメータ検証機能"
git commit -m "テスト: Kickスキルのパラメータ検証テストケース追加"
git commit -m "ドキュメント: Kickスキルパラメータ仕様を更新"

# Bad: Everything in one commit
git commit -m "Kickスキルの実装、テスト、ドキュメント"
```

### Mandatory Elements

**Every commit MUST include:**

1. **Japanese title** (50 characters or less)
2. **Category prefix** from the standardized list
3. **Claude Code attribution** (footer)

**Optional but recommended:**

- Detailed description in Japanese
- Bullet points for multiple changes
- References to issues or related work

### Commit Verification Checklist

Before committing, verify:

- [ ] Message is in Japanese
- [ ] Uses standardized category prefix
- [ ] Title is descriptive and under 50 characters
- [ ] No build artifacts are included (see Git Repository Management)
- [ ] Code changes are functional and tested
- [ ] Claude Code attribution is included

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
