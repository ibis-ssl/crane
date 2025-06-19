# Changelog

All notable changes to the crane_debug_tools package will be documented in this file.

## [0.1.0] - 2024-12-19

### Added
- **CLI Skill Tester**: Interactive command-line interface for skill testing
- **crane_skill Script**: Python-based CLI tool for batch operations and automation
- **Automated Scenarios**: JSON-based test scenario execution
- **Multi-Robot Support**: Native support for coordinating multiple robots
- **Example Scenarios**: Pre-built test scenarios for common use cases
  - `basic_skills_test.json` - Basic robot skills testing
  - `multi_robot_formation.json` - Formation and coordination testing
  - `kickoff_sequence.json` - Kickoff execution testing
  - `defense_test.json` - Defensive positioning testing
  - `goal_kick_test.json` - Goal kick scenario testing

### Features
- **Cross-Platform**: Works on any system with ROS2
- **Remote Development**: Full functionality over SSH
- **Type-Safe Parameters**: Automatic parameter type detection and validation
- **Scriptable**: Full automation support for CI/CD integration
- **Performance**: Minimal resource usage compared to Qt GUI

### Supported Skills
All crane robot skills are supported:
- Basic: Sleep, Idle, EmplaceRobot
- Motion: TestMotionPosition, TestMotionVelocity
- Game: Kick, Receive, Goalie, Attacker, SubAttacker
- Formation: StealBall, SingleBallPlacement, GoalKick
- Kickoff: SimpleKickOff, KickOffAttack, KickOffSupport
- Defense: SecondThreatDefender, FreekickSaver
- Special: Marker, PenaltyKick, Teleop

### Technical Details
- **ROS2 Integration**: Uses standard SkillExecution action interface
- **Parameter Handling**: Support for NamedBool, NamedInt, NamedFloat, NamedString
- **Error Handling**: Comprehensive error reporting and validation
- **Documentation**: Complete migration guide and usage examples

### Known Limitations
- **Web Interface**: Temporarily disabled due to WebSocket++ compatibility issues
- **Real-time Visualization**: Currently CLI-only, web visualization planned for future release

### Migration from crane_simple_ai
- **Compatibility**: Full compatibility with existing skill interface
- **Performance**: Significantly faster execution and lower resource usage
- **Automation**: New capabilities not available in Qt GUI
- **Multi-robot**: Native support vs. single-robot limitation

### Development
- **Build System**: Standard ament_cmake with auto-discovery
- **Testing**: Automated test scripts and scenario validation
- **CI/CD Ready**: Scriptable interface suitable for automated testing

### Future Roadmap
- Web-based interface with real-time visualization
- Enhanced scenario editor with GUI
- Performance monitoring and analytics
- Integration with ROS2 testing framework
- Advanced multi-robot coordination testing