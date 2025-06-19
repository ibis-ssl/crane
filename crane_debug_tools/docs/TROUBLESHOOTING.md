# Troubleshooting Guide: Crane Debug Tools

This guide helps you diagnose and resolve common issues when using crane_debug_tools.

## Table of Contents
1. [Quick Diagnostics](#quick-diagnostics)
2. [Common Issues](#common-issues)
3. [System Dependencies](#system-dependencies)
4. [Network and Communication](#network-and-communication)
5. [Performance Issues](#performance-issues)
6. [Integration Problems](#integration-problems)
7. [Advanced Debugging](#advanced-debugging)

## Quick Diagnostics

### Health Check Script

Create a quick health check to verify system status:

```bash
#!/bin/bash
# crane_debug_health_check.sh

echo "=== Crane Debug Tools Health Check ==="

# Check ROS 2 environment
echo "1. Checking ROS 2 environment..."
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ ROS_DISTRO not set. Source your ROS 2 installation."
    exit 1
else
    echo "✅ ROS_DISTRO: $ROS_DISTRO"
fi

# Check workspace
echo "2. Checking workspace..."
if [ -z "$AMENT_PREFIX_PATH" ]; then
    echo "❌ Workspace not sourced. Run: source install/local_setup.bash"
    exit 1
else
    echo "✅ Workspace sourced"
fi

# Check crane_debug_tools package
echo "3. Checking crane_debug_tools package..."
if ! ros2 pkg list | grep -q crane_debug_tools; then
    echo "❌ crane_debug_tools package not found. Build the package first."
    exit 1
else
    echo "✅ crane_debug_tools package available"
fi

# Check crane_skill command
echo "4. Checking crane_skill command..."
if ! command -v crane_skill &> /dev/null; then
    echo "❌ crane_skill command not found. Check installation."
    exit 1
else
    echo "✅ crane_skill command available"
fi

# Check action server
echo "5. Checking action server..."
if ! timeout 5s ros2 action list | grep -q skill_execution; then
    echo "⚠️  Action server not available. Start crane_robot_skills."
else
    echo "✅ Action server available"
fi

# Check basic functionality
echo "6. Testing basic functionality..."
if timeout 10s crane_skill list > /dev/null 2>&1; then
    echo "✅ Basic functionality working"
else
    echo "❌ Basic functionality test failed"
fi

echo "=== Health check complete ==="
```

### Quick Test Commands

```bash
# Test basic CLI functionality
crane_skill list

# Test ROS 2 connectivity
ros2 node list
ros2 action list

# Test skill execution (if server available)
crane_skill run Sleep 0 duration:0.1
```

## Common Issues

### Issue 1: `crane_skill` Command Not Found

**Symptoms:**
```bash
$ crane_skill list
bash: crane_skill: command not found
```

**Causes and Solutions:**

#### Cause 1: Workspace not sourced
```bash
# Solution: Source the workspace
cd /path/to/your/workspace
source install/local_setup.bash

# Add to ~/.bashrc for persistent setup
echo "source /path/to/your/workspace/install/local_setup.bash" >> ~/.bashrc
```

#### Cause 2: Package not built
```bash
# Solution: Build the package
colcon build --packages-select crane_debug_tools

# If build fails, clean and rebuild
colcon build --packages-select crane_debug_tools --cmake-clean-cache
```

#### Cause 3: Script not executable
```bash
# Solution: Make script executable
chmod +x install/crane_debug_tools/lib/crane_debug_tools/crane_skill

# Check script location
find install/ -name crane_skill -type f
```

#### Cause 4: Python path issues
```bash
# Solution: Check Python path and dependencies
python3 -c "import rclpy; print('rclpy OK')"
python3 -c "import crane_msgs; print('crane_msgs OK')"

# If imports fail, check package installation
ros2 pkg list | grep crane_msgs
```

### Issue 2: Action Server Not Available

**Symptoms:**
```bash
$ crane_skill run Sleep 0 duration:1.0
Error: Action server not available after 10 seconds
```

**Diagnostic Steps:**

#### Step 1: Check if crane_robot_skills is running
```bash
# Check for crane nodes
ros2 node list | grep crane

# Expected output should include nodes like:
# /crane_robot_skills
# /crane_world_model_publisher
# /crane_session_controller
```

#### Step 2: Check action server specifically
```bash
# List all action servers
ros2 action list

# Look for skill execution action
ros2 action list | grep skill_execution

# Check action server info
ros2 action info /simple_ai/skill_execution
```

#### Step 3: Check system launch
```bash
# If action server missing, launch crane system
ros2 launch crane_bringup crane.launch.py

# Or start robot skills specifically
ros2 run crane_robot_skills crane_robot_skills_node
```

**Solutions:**

#### Solution 1: Launch crane system
```bash
# Standard launch
ros2 launch crane_bringup crane.launch.py

# With simulation
ros2 launch crane_bringup crane.launch.py sim:=true

# Check launch output for errors
```

#### Solution 2: Check dependencies
```bash
# Verify all crane packages are built
colcon list --packages-up-to crane_robot_skills

# Build missing packages
colcon build --packages-up-to crane_robot_skills
```

#### Solution 3: Network configuration
```bash
# Check ROS domain ID (should match across all nodes)
echo $ROS_DOMAIN_ID

# If using multiple machines, check network
ros2 doctor

# Reset network configuration if needed
unset ROS_DOMAIN_ID
ros2 daemon stop
ros2 daemon start
```

### Issue 3: Parameter Type Errors

**Symptoms:**
```bash
$ crane_skill run Kick 0 target_x:1.0 target_y:2.0 kick_power:abc
Error: Invalid parameter type for kick_power: expected number
```

**Common Parameter Issues:**

#### Issue: String interpreted as number
```bash
# Wrong: Missing decimal point for float
crane_skill run Kick 0 kick_power:5

# Correct: Explicit decimal
crane_skill run Kick 0 kick_power:5.0
```

#### Issue: Boolean format
```bash
# Wrong: Numeric boolean
crane_skill run EmplaceRobot 0 precise_positioning:1

# Correct: String boolean
crane_skill run EmplaceRobot 0 precise_positioning:true
```

#### Issue: Missing required parameters
```bash
# Check skill documentation for required parameters
ros2 interface show crane_msgs/action/SkillExecution

# Use appropriate parameters for each skill
crane_skill run Kick 0 target_x:1.0 target_y:2.0  # kick_power may be required
```

**Parameter Debugging:**

```bash
# Test parameter parsing
crane_skill run Sleep 0 duration:1.0  # Simple float parameter
crane_skill run Idle 0                # No parameters

# Check parameter format
echo "Testing parameter: key:value format required"
```

### Issue 4: Robot ID Out of Range

**Symptoms:**
```bash
$ crane_skill run Kick 16 target_x:1.0
Error: Robot ID must be between 0 and 15
```

**Solutions:**

```bash
# Use valid robot IDs (0-15)
crane_skill run Kick 0 target_x:1.0    # Valid
crane_skill run Kick 15 target_x:1.0   # Valid (max)

# For multi-robot, check ID format
crane_skill multi Idle 0,1,2           # Valid format
crane_skill multi Idle "0, 1, 2"       # Wrong: spaces not allowed
```

### Issue 5: Skill Execution Timeout

**Symptoms:**
```bash
$ crane_skill run Kick 0 target_x:1.0 target_y:2.0
Executing skill 'Kick' on robot 0...
Goal accepted by server, executing...
Skill execution timed out or failed
```

**Diagnostic Steps:**

#### Check robot status
```bash
# Monitor world model
ros2 topic echo /world_model --once

# Check robot positions
ros2 topic echo /world_model | grep -A 20 robot_info_ours

# Monitor robot commands
ros2 topic echo /robot_commands
```

#### Check system status
```bash
# Monitor system health
ros2 topic list | grep crane
ros2 node list | grep crane

# Check for error messages
ros2 topic echo /rosout | grep ERROR
```

**Solutions:**

#### Solution 1: Verify robot visibility
```bash
# Check if robots are detected in world model
ros2 topic echo /world_model --once | grep robot_info_ours

# If no robots detected, check vision system
ros2 topic echo /vision_data
```

#### Solution 2: Check simulation environment
```bash
# If using simulation, verify grSim is running
# Launch simulation environment if needed
ros2 launch crane_bringup crane.launch.py sim:=true

# Check simulation topics
ros2 topic list | grep sim
```

#### Solution 3: Try simpler skills first
```bash
# Test with skills that don't require complex execution
crane_skill run Idle 0
crane_skill run Sleep 0 duration:1.0

# Then progress to movement skills
crane_skill run EmplaceRobot 0 target_x:0.0 target_y:0.0
```

## System Dependencies

### ROS 2 Dependencies

#### Missing ROS 2 Packages
```bash
# Check required packages
rosdep check --from-paths src --ignore-src

# Install missing dependencies
rosdep install --from-paths src --ignore-src -y

# Verify crane_msgs is available
ros2 interface list | grep crane_msgs
```

#### Version Compatibility
```bash
# Check ROS 2 version
ros2 --version

# Verify package versions
ros2 pkg xml crane_debug_tools | grep version

# Check for version conflicts
colcon list --packages-up-to crane_debug_tools
```

### Python Dependencies

#### Missing Python Packages
```bash
# Test Python imports
python3 -c "
import rclpy
import crane_msgs
from crane_msgs.action import SkillExecution
from crane_msgs.msg import NamedValueArray
print('All imports successful')
"

# If imports fail, rebuild packages
colcon build --packages-select crane_msgs crane_debug_tools
```

#### Python Path Issues
```bash
# Check Python path includes ROS packages
python3 -c "import sys; print('\n'.join(sys.path))"

# Verify PYTHONPATH includes install directory
echo $PYTHONPATH | grep install

# Source workspace if PYTHONPATH is wrong
source install/local_setup.bash
```

### Build Dependencies

#### Missing Build Tools
```bash
# Install required build tools
sudo apt update
sudo apt install build-essential cmake python3-colcon-common-extensions

# For C++ development
sudo apt install g++ gdb

# For Python development  
sudo apt install python3-dev python3-pip
```

#### CMake Issues
```bash
# Clean CMake cache if build issues persist
colcon build --packages-select crane_debug_tools --cmake-clean-cache

# Force rebuild
rm -rf build/ install/
colcon build --packages-select crane_debug_tools
```

## Network and Communication

### ROS 2 Communication Issues

#### Domain ID Conflicts
```bash
# Check current domain ID
echo $ROS_DOMAIN_ID

# Set unique domain ID if conflicts exist
export ROS_DOMAIN_ID=42

# Reset ROS daemon with new domain
ros2 daemon stop
ros2 daemon start

# Test communication
ros2 node list
```

#### Multicast Issues
```bash
# Test multicast connectivity
ros2 multicast send

# In another terminal
ros2 multicast receive

# If multicast fails, check network configuration
ip route show
```

#### Network Interface Problems
```bash
# Check network interfaces
ip addr show

# Test ROS 2 discovery
ros2 doctor

# If using multiple interfaces, specify one
export ROS_NETWORK_INTERFACE=eth0
```

### Action Communication

#### Action Server Connection
```bash
# Monitor action server status
watch "ros2 action list | grep skill_execution"

# Check action server node
ros2 node info /crane_robot_skills

# Test action server directly
ros2 action send_goal /simple_ai/skill_execution crane_msgs/action/SkillExecution "{robot_id: 0, name: 'Sleep', parameter: {float_values: [{name: 'duration', value: 1.0}]}}"
```

#### Action Client Debugging
```bash
# Enable action client debug output
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"
export RCUTILS_LOGGING_USE_STDOUT=1
export RCUTILS_LOGGING_BUFFERED_STREAM=1

# Run with debug output
crane_skill run Sleep 0 duration:1.0
```

## Performance Issues

### Slow Execution

#### Symptoms
- Commands take long time to complete
- Delayed response from action server
- High CPU or memory usage

#### Diagnostic Steps
```bash
# Monitor system resources
htop
iotop

# Check ROS 2 performance
ros2 topic hz /world_model
ros2 topic bw /robot_commands

# Profile execution time
time crane_skill run Sleep 0 duration:1.0
```

#### Solutions

**Optimize ROS 2 Configuration:**
```bash
# Use faster DDS implementation
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp

# Adjust QoS settings if needed
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

# Reduce discovery traffic
export ROS_STATIC_PEERS=localhost
```

**System Optimization:**
```bash
# Increase process priority
nice -n -10 crane_skill run Kick 0 target_x:1.0 target_y:2.0

# Monitor and kill competing processes
ps aux | grep -v grep | grep ros
```

### Memory Issues

#### Memory Leaks
```bash
# Monitor memory usage over time
watch "ps aux | grep crane_skill | grep -v grep"

# Use memory profiling tools
valgrind --tool=memcheck --leak-check=full crane_skill run Sleep 0 duration:1.0
```

#### Large Scenario Files
```bash
# For large scenario files, split into smaller chunks
# Monitor memory during scenario execution
watch "free -h && echo '---' && ps aux | grep crane_skill"
```

## Integration Problems

### CI/CD Integration Issues

#### Pipeline Failures
```bash
# Test CI/CD commands locally
export CI=true
crane_skill scenario scenarios/basic_test.json

# Check return codes
crane_skill run Sleep 0 duration:1.0
echo "Exit code: $?"
```

#### Headless Environment
```bash
# Ensure no GUI dependencies
export DISPLAY=""
export HEADLESS=1

# Test in minimal environment
docker run --rm -v $(pwd):/workspace ubuntu:22.04 bash -c "
  cd /workspace && 
  source /opt/ros/jazzy/setup.bash && 
  crane_skill list
"
```

### Docker Integration

#### Container Issues
```bash
# Build container with debug tools
FROM ros:jazzy
COPY . /workspace
WORKDIR /workspace
RUN colcon build --packages-select crane_debug_tools
RUN echo 'source install/local_setup.bash' >> ~/.bashrc

# Test in container
docker run -it --rm my_crane_image crane_skill list
```

#### Network in Docker
```bash
# Use host networking for ROS 2
docker run --network host my_crane_image

# Or configure container networking
docker run -e ROS_DOMAIN_ID=0 -p 11311:11311 my_crane_image
```

## Advanced Debugging

### Debug Logging

#### Enable Detailed Logging
```bash
# Set log level for debugging
export RCUTILS_LOGGING_SEVERITY_THRESHOLD=DEBUG

# Enable specific logger
export RCUTILS_LOGGING_CONFIG_FILE=/path/to/logging.conf

# Create logging configuration
cat > logging.conf << EOF
logger_names=crane_debug_tools
logger.crane_debug_tools.level=DEBUG
EOF
```

#### Log Analysis
```bash
# Capture logs to file
crane_skill run Kick 0 target_x:1.0 2>&1 | tee debug.log

# Analyze log patterns
grep ERROR debug.log
grep WARNING debug.log
grep "skill" debug.log
```

### Core Dumps and Crashes

#### Enable Core Dumps
```bash
# Enable core dumps
ulimit -c unlimited

# Set core dump pattern
echo "/tmp/core.%e.%p" | sudo tee /proc/sys/kernel/core_pattern

# Run with core dump collection
crane_skill run Kick 0 target_x:1.0

# Analyze core dump (if crash occurs)
gdb crane_skill_cli core.crane_skill_cli.1234
```

#### Debugging with GDB
```bash
# Run under debugger
gdb --args crane_skill run Kick 0 target_x:1.0

# Set breakpoints and analyze
(gdb) break main
(gdb) run
(gdb) backtrace
```

### Network Debugging

#### Packet Analysis
```bash
# Capture ROS 2 traffic
sudo tcpdump -i any -w ros2_traffic.pcap port 7400-7500

# Analyze with Wireshark (if available)
wireshark ros2_traffic.pcap

# Or analyze with text tools
tcpdump -r ros2_traffic.pcap -A | grep skill_execution
```

#### DDS Debugging
```bash
# Enable DDS debugging
export CYCLONEDX_ENABLE_LOGGING=1
export CYCLONEDX_LOG_LEVEL=finest

# Check DDS configuration
ros2 doctor --report
```

### Custom Debug Tools

#### Create Debug Script
```bash
#!/bin/bash
# debug_crane_skill.sh

echo "=== Debug Information ==="
echo "Date: $(date)"
echo "User: $(whoami)"
echo "Working Directory: $(pwd)"
echo "ROS_DISTRO: $ROS_DISTRO"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"

echo -e "\n=== Node List ==="
ros2 node list

echo -e "\n=== Action List ==="
ros2 action list

echo -e "\n=== Topic List ==="
ros2 topic list | grep crane

echo -e "\n=== Skill Execution Test ==="
crane_skill run Sleep 0 duration:0.5

echo -e "\n=== Debug Complete ==="
```

#### Monitor Script
```bash
#!/bin/bash
# monitor_crane_debug.sh

echo "Monitoring crane debug tools..."
while true; do
    echo "$(date): Checking system status"
    
    # Check action server
    if ros2 action list | grep -q skill_execution; then
        echo "✅ Action server available"
    else
        echo "❌ Action server not available"
    fi
    
    # Check memory usage
    echo "Memory: $(free -h | grep Mem | awk '{print $3 "/" $2}')"
    
    # Check CPU usage
    echo "CPU: $(top -bn1 | grep "Cpu(s)" | sed "s/.*, *\([0-9.]*\)%* id.*/\1/" | awk '{print 100 - $1"%"}')"
    
    sleep 10
done
```

This troubleshooting guide provides comprehensive coverage of common issues and their solutions, enabling users to effectively diagnose and resolve problems with crane_debug_tools.