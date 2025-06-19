# Migration Guide: From crane_simple_ai to crane_debug_tools

This guide helps you transition from the Qt-based `crane_simple_ai` to the new modern `crane_debug_tools`.

## Quick Comparison

| Feature | crane_simple_ai | crane_debug_tools |
|---------|-----------------|-------------------|
| **Interface** | Qt5 GUI | CLI + Web (planned) |
| **Platform** | Linux only | Cross-platform |
| **Automation** | Manual only | CLI scripting + scenarios |
| **Multi-robot** | Single robot focus | Native multi-robot support |
| **Remote access** | Local only | CLI works over SSH |
| **Dependencies** | Qt5, heavy GUI deps | Minimal, ROS2 only |

## Basic Usage Migration

### Starting the Interface

**Old way (crane_simple_ai):**
```bash
ros2 launch crane_bringup crane.launch.py simple_ai:=true
```

**New way (crane_debug_tools):**
```bash
# For CLI interface
ros2 run crane_debug_tools crane_skill_cli

# Or use the standalone script
crane_skill list
crane_skill run Kick 0 target_x:1.0 target_y:2.0
```

### Skill Execution

**Old way:** 
- Select skill from dropdown
- Set robot ID in spinner
- Configure parameters in table
- Click execute button

**New way:**
```bash
# Direct execution
crane_skill run <skill_name> <robot_id> [param1:value1] [param2:value2]

# Examples
crane_skill run Kick 0 target_x:1.0 target_y:2.0 kick_power:5.0
crane_skill run EmplaceRobot 1 target_x:2.0 target_y:1.5 target_theta:0.5
crane_skill run Sleep 0 duration:2.0
```

## Advanced Features

### Multi-Robot Coordination

**Old:** Not supported - had to execute skills one by one

**New:** Native multi-robot support
```bash
# Execute same skill on multiple robots
crane_skill multi Attacker 0,1,2

# Execute with parameters
crane_skill multi EmplaceRobot 0,1,2 target_x:1.0 target_y:0.0
```

### Automated Testing Scenarios

**Old:** Not supported - manual execution only

**New:** JSON-based scenario execution
```bash
# Execute pre-defined test sequences
crane_skill scenario scenarios/basic_skills_test.json
crane_skill scenario scenarios/multi_robot_formation.json
```

Example scenario file:
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

## Skill Parameter Mapping

The parameter system has been improved for better type safety:

### crane_simple_ai Parameters
- All parameters were strings in a table
- No type validation
- Manual parameter entry

### crane_debug_tools Parameters
- Automatic type detection (float, int, bool, string)
- Command-line friendly format: `key:value`
- Support for complex scenarios

### Common Skills Migration

**Kick Skill:**
```bash
# Old: Set in GUI table
# target_x: 1.0
# target_y: 2.0
# kick_power: 5.0

# New: Command line
crane_skill run Kick 0 target_x:1.0 target_y:2.0 kick_power:5.0
```

**EmplaceRobot Skill:**
```bash
# Old: GUI parameter table
# New: Direct command
crane_skill run EmplaceRobot 0 target_x:2.0 target_y:1.5 target_theta:0.5
```

## Workflow Migration

### Development Testing Workflow

**Old Workflow:**
1. Launch crane system with `simple_ai:=true`
2. Open Qt GUI
3. Manually select skills and set parameters
4. Execute one by one
5. Manually observe results

**New Workflow:**
1. Launch crane system normally
2. Use CLI for quick testing:
   ```bash
   crane_skill run TestMotionPosition 0 target_x:1.0 target_y:1.0
   ```
3. Create scenario files for complex tests
4. Automate regression testing with scripts

### Integration Testing Workflow

**Old:** Manual, error-prone, not repeatable

**New:** Automated and scriptable
```bash
# Create test script
cat > test_formation.sh << 'EOF'
#!/bin/bash
echo "Testing robot formation..."
crane_skill multi EmplaceRobot 0,1,2 target_x:1.0
crane_skill run Sleep 0 duration:2.0
crane_skill multi Attacker 0,1,2
EOF

chmod +x test_formation.sh
./test_formation.sh
```

## Troubleshooting Migration Issues

### Common Issues and Solutions

**Issue:** `crane_skill` command not found
```bash
# Solution: Source the workspace
source install/local_setup.bash
```

**Issue:** Action server not available
```bash
# Check if crane_robot_skills is running
ros2 action list | grep skill_execution

# If not found, ensure crane system is fully launched
ros2 launch crane_bringup crane.launch.py
```

**Issue:** Skill parameters not working
```bash
# Check parameter format - use colon separator
crane_skill run Kick 0 target_x:1.0  # Correct
crane_skill run Kick 0 target_x=1.0  # Wrong
```

### Debugging Tips

1. **Verbose output:** Add debug prints to understand what's happening
2. **Step-by-step execution:** Test skills individually before scenarios
3. **Parameter validation:** Use `crane_skill list` to verify available skills

## Performance Comparison

| Metric | crane_simple_ai | crane_debug_tools |
|--------|-----------------|-------------------|
| **Startup time** | ~3-5 seconds (Qt load) | ~0.5 seconds (CLI) |
| **Memory usage** | ~50-100 MB (Qt overhead) | ~10-20 MB (minimal) |
| **Execution speed** | GUI interaction delay | Instant command execution |
| **Batch operations** | Not supported | Native support |

## Integration with Existing Workflows

### CI/CD Integration

**Old:** Not possible with GUI

**New:** Full CI/CD support
```yaml
# Example GitHub Actions workflow
- name: Test Robot Skills
  run: |
    source install/local_setup.bash
    crane_skill scenario tests/ci_skills_test.json
```

### Remote Development

**Old:** Requires X11 forwarding for GUI

**New:** Works over SSH natively
```bash
# Remote development over SSH
ssh robot_computer
crane_skill run Kick 0 target_x:1.0 target_y:2.0
```

## Future Migration Path

The `crane_debug_tools` is designed to be the long-term replacement for `crane_simple_ai`. 

### Planned Features
- Web-based interface for visual debugging
- Enhanced scenario editor
- Performance monitoring and analytics
- Integration with ROS2 testing framework

### Deprecation Timeline
- **Phase 1 (Current):** Both systems available, crane_debug_tools recommended for new development
- **Phase 2 (Future):** crane_simple_ai marked deprecated
- **Phase 3 (Future):** crane_simple_ai removed from codebase

## Getting Help

If you encounter issues during migration:

1. **Check the documentation:** `/crane_debug_tools/README.md`
2. **Run example tests:** `examples/simple_test.py`
3. **Compare scenarios:** Look at `scenarios/` directory for examples
4. **Ask for help:** Create an issue with your specific use case

## Summary

The migration from `crane_simple_ai` to `crane_debug_tools` provides:

✅ **Better automation** - Script and automate testing workflows  
✅ **Multi-robot support** - Test coordination scenarios easily  
✅ **Cross-platform compatibility** - Works on any system with ROS2  
✅ **Remote development** - Debug over SSH without GUI forwarding  
✅ **CI/CD integration** - Automated testing in pipelines  
✅ **Performance** - Faster execution, lower resource usage  

The new tools maintain all the functionality of the original GUI while adding powerful automation and multi-robot capabilities that make development and testing more efficient.