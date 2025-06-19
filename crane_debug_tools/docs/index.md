# Crane Debug Tools Documentation

Welcome to the comprehensive documentation for crane_debug_tools - the modern replacement for crane_simple_ai, providing powerful command-line and web-based tools for testing and debugging robot skills in the Crane robot soccer system.

## What is crane_debug_tools?

crane_debug_tools is a next-generation debugging and testing suite designed to replace the Qt-based crane_simple_ai with more flexible, automatable, and developer-friendly tools. It provides both command-line interfaces and web-based debugging capabilities for the Crane robot soccer system.

### Key Features

- **🖥️ Command-Line Interface**: Interactive and batch CLI tools for quick testing
- **🌐 Web Interface**: Modern browser-based debugging (planned)
- **🤖 Multi-Robot Support**: Native coordination testing for robot teams
- **📝 Scenario Testing**: JSON-based automated test sequences
- **🔧 Cross-Platform**: Works on any system with ROS 2
- **⚡ High Performance**: Minimal resource usage compared to GUI alternatives
- **🚀 Automation-Ready**: Full CI/CD integration support

## Quick Start

### Installation
```bash
# Build the package
colcon build --packages-select crane_debug_tools

# Source workspace
source install/local_setup.bash
```

### Basic Usage
```bash
# List available skills
crane_skill list

# Execute a skill
crane_skill run Kick 0 target_x:1.0 target_y:2.0 kick_power:5.0

# Multi-robot execution
crane_skill multi Attacker 0,1,2

# Run test scenario
crane_skill scenario scenarios/basic_test.json
```

## Documentation Sections

### 📖 Core Documentation

#### [User Guide](USER_GUIDE.md)
Comprehensive guide covering all aspects of using crane_debug_tools, from basic operations to advanced multi-robot scenarios.

**Contents:**
- Getting started and installation
- CLI interface usage
- Parameter systems and formatting
- Multi-robot coordination
- Scenario testing
- Best practices and workflows

#### [Migration Guide](MIGRATION_GUIDE.md)
Step-by-step guide for transitioning from crane_simple_ai to crane_debug_tools.

**Contents:**
- Feature comparison
- Command mapping
- Workflow migration
- Performance improvements
- Troubleshooting migration issues

### 🔧 Technical Documentation

#### [Design Overview](DESIGN_OVERVIEW.md)
In-depth technical documentation covering the architecture and design decisions behind crane_debug_tools.

**Contents:**
- System architecture
- Design philosophy
- Component interaction
- Performance considerations
- Extensibility framework

#### [API Reference](API_REFERENCE.md)
Complete API documentation for developers working with or extending crane_debug_tools.

**Contents:**
- Core classes and interfaces
- CLI command reference
- Parameter system API
- Extension points
- ROS 2 integration details

### 🛠️ Practical Guides

#### [Examples](EXAMPLES.md)
Extensive collection of practical examples and real-world use cases.

**Contents:**
- Basic skill testing
- Multi-robot scenarios
- Automated testing workflows
- Integration examples
- Custom scenario development

#### [Troubleshooting](TROUBLESHOOTING.md)
Comprehensive troubleshooting guide for common issues and their solutions.

**Contents:**
- Quick diagnostics
- Common error resolution
- System dependencies
- Network configuration
- Performance optimization

## Architecture Overview

### System Components

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
└───────────────────────┼────────────────────────────────────┘
                        │
        ┌───────────────┼───────────────┐
        │           ROS 2 Layer         │
        │                               │
        │ • Action Client Communication │
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

### Available Tools

#### Command-Line Tools

1. **`crane_skill_cli`** - Interactive CLI for real-time skill testing
2. **`crane_skill`** - Batch CLI script for automation and scripting

#### Supported Skills

All crane robot skills are supported:

**Basic Skills:**
- `Sleep`, `Idle`, `EmplaceRobot`
- `TestMotionPosition`, `TestMotionVelocity`

**Game Skills:**
- `Kick`, `Receive`, `Goalie`
- `Attacker`, `SubAttacker`, `StealBall`

**Formation Skills:**
- `SingleBallPlacement`, `GoalKick`
- `SimpleKickOff`, `KickOffAttack`, `KickOffSupport`

**Advanced Skills:**
- `SecondThreatDefender`, `FreekickSaver`
- `PenaltyKick`, `Marker`, `Teleop`

## Quick Examples

### Basic Skill Testing
```bash
# Simple skill execution
crane_skill run Sleep 0 duration:2.0

# Robot positioning
crane_skill run EmplaceRobot 0 target_x:1.0 target_y:2.0 target_theta:0.5

# Ball interaction
crane_skill run Kick 0 target_x:3.0 target_y:1.0 kick_power:4.0
```

### Multi-Robot Scenarios
```bash
# Formation setup
crane_skill multi EmplaceRobot 0,1,2,3 target_x:1.0 target_y:0.0

# Coordinate team behaviors
crane_skill run Goalie 0
crane_skill multi Attacker 1,2,3
```

### Automated Testing
```json
{
  "name": "Basic Test",
  "skills": [
    {
      "name": "EmplaceRobot",
      "robot_id": 0,
      "parameters": {"target_x": 1.0, "target_y": 0.0},
      "delay": 2
    },
    {
      "name": "Kick",
      "robot_id": 0,
      "parameters": {"target_x": 3.0, "kick_power": 5.0},
      "delay": 1
    }
  ]
}
```

Execute with:
```bash
crane_skill scenario basic_test.json
```

## Comparison with crane_simple_ai

| Feature | crane_simple_ai | crane_debug_tools |
|---------|-----------------|-------------------|
| **Interface** | Qt5 GUI | CLI + Web (planned) |
| **Automation** | Manual only | Full scripting support |
| **Multi-robot** | Single robot | Native multi-robot |
| **Performance** | ~50-100MB memory | ~10-20MB memory |
| **Platform** | Linux only | Cross-platform |
| **Remote access** | X11 forwarding required | SSH-friendly |
| **CI/CD** | Not supported | Full integration |

## Getting Help

### Documentation Navigation

- **New Users**: Start with [User Guide](USER_GUIDE.md)
- **Migrating**: See [Migration Guide](MIGRATION_GUIDE.md)
- **Developers**: Check [API Reference](API_REFERENCE.md)
- **Troubleshooting**: Visit [Troubleshooting Guide](TROUBLESHOOTING.md)
- **Examples**: Browse [Examples Collection](EXAMPLES.md)

### Support Resources

- **Issues**: Report bugs and request features on the project repository
- **Documentation**: This comprehensive documentation set
- **Examples**: Extensive example scenarios and scripts
- **API Reference**: Complete technical documentation

### Community and Contributing

crane_debug_tools is part of the open-source Crane robot soccer system. Contributions are welcome in the form of:

- Bug reports and feature requests
- Documentation improvements
- Example scenarios and use cases
- Performance optimizations
- New tool integrations

## What's Next?

### Current Status
- ✅ CLI interface complete and tested
- ✅ Multi-robot coordination support
- ✅ Scenario testing framework
- ✅ Comprehensive documentation
- ⏳ Web interface (planned for future release)

### Future Roadmap
- 🔮 Web-based real-time visualization
- 🔮 Enhanced performance monitoring
- 🔮 Advanced scenario editor
- 🔮 Integration with ROS 2 testing framework
- 🔮 Machine learning integration for parameter optimization

### Getting Started Today

1. **Install**: Build and source the package
2. **Try**: Run basic skills with the CLI
3. **Explore**: Test multi-robot scenarios
4. **Automate**: Create custom test scenarios
5. **Integrate**: Add to your development workflow

Welcome to the future of robot skill debugging with crane_debug_tools! 🚀