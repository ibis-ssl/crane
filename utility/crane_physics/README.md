# crane_physics

A physics simulation and robot modeling utility package for the crane robotics system. This package provides comprehensive physics calculations, robot dynamics modeling, and ball physics simulation for robot soccer applications.

## Package Overview

The `crane_physics` package is a header-only utility library that provides:

1. **Ball Physics Modeling** - Comprehensive 3D ball physics with state-based simulation
2. **Robot Dynamics** - Robot motion calculations and travel time predictions
3. **Control Systems** - PID controller implementation
4. **Strategic Analysis** - Pass analysis and position assignment algorithms
5. **Target Geometry** - Dynamic target specification system

## Architecture

This package integrates tightly with `crane_geometry` for mathematical operations and provides physics foundations used throughout the crane robotics system, particularly in:

- World model and state estimation
- Motion planning and trajectory generation
- Robot skill execution
- Game analysis and strategy

## Key Components

### 1. Ball Physics Model (`ball_info.hpp`)

#### Ball States

The ball physics system supports three distinct states:

- **STOPPED**: Ball is stationary (velocity below threshold)
- **ROLLING**: Ball is moving on the ground with friction deceleration
- **FLYING**: Ball is airborne following parabolic trajectory

#### Core Physics Parameters

```cpp
struct Ball {
    double deceleration = 0.5;    // Rolling deceleration (m/s²)
    double gravity = -9.81;       // Gravity acceleration (m/s²)
    double air_resistance = 0.0;  // Air resistance coefficient
    Point pos;                    // 2D position
    double pos_z;                 // Height (z-coordinate)
    Point vel;                    // 2D velocity
    double vel_z;                 // Vertical velocity
    State state;                  // Current physics state
};
```

#### Advanced Physics Calculations

**Rolling Physics**:

- Uses constant deceleration model
- Predicts stop time: `t_stop = v / deceleration`
- Maximum distance: `d_max = v²/(2·deceleration)`
- Position prediction: `x(t) = x₀ + v₀·t - 0.5·a·t²`

**Flying Physics (Parabolic Motion)**:

- 3D trajectory calculation with gravity
- Ground intersection detection
- Seamless transition from flying to rolling state
- Position: `x(t) = x₀ + v₀·t`, `z(t) = z₀ + vz₀·t + 0.5·g·t²`

**State Transitions**:

- Automatic state detection based on height and velocity thresholds
- Hysteresis-based state switching to prevent oscillation
- Supports complex multi-phase trajectories (flying → landing → rolling)

#### Key Methods

```cpp
// Motion state queries
bool isMoving(double threshold = 0.01) const;
bool isMovingTowards(const Point& target, double angle_threshold = 60.0) const;
bool isMovingAwayFrom(const Point& target, double angle_threshold = 60.0) const;

// Physics predictions
Point getPredictedPosition(double time_ahead) const;
Point getPredictedVelocity(double time_ahead) const;
double getStopTime() const;
double getMaxDistance() const;

// Trajectory analysis
std::optional<double> getTimeToReachClosestPointFrom(const Point& target) const;
Segment getTrajectorySegment(double time_horizon) const;
std::vector<std::pair<Point, double>> getBallSequence(double t_horizon, double t_step) const;
```

#### Advanced Features

**ParabolicPhysics Class**:

- Dedicated 3D parabolic motion calculator
- Ground intersection computation
- Initial velocity estimation from trajectory points
- Supports both forward prediction and parameter estimation

**Hysteresis System**:

- Prevents state oscillation with configurable thresholds
- Callback system for state transition events
- Used for robust ball speed and state detection

### 2. Robot Dynamics (`robot_info.hpp`)

#### Robot Information Structure

```cpp
struct RobotInfo {
    uint8_t id;                    // Robot identifier
    Pose2D pose;                   // Position and orientation
    Velocity2D vel;                // Linear and angular velocity
    bool available;                // Robot availability status
    rclcpp::Time vision_detection_stamp;  // Last vision update
    rclcpp::Time ball_sensor_stamp;       // Last ball sensor update
    bool ball_sensor;              // Ball sensor state
    BallContact ball_contact;      // Ball contact tracking
};
```

#### Physical Constants and Geometry

- Robot radius: 0.060m (for collision detection)
- Dribbler distance: 0.090m (from center to ball contact point)
- Integrated ball contact detection and timing

#### Key Methods

```cpp
Point kicker_center() const;           // Position of kicker/dribbler
Circle geometry() const;               // Robot collision geometry
double getDistance(const Point& pos) const;  // Distance calculations
bool getBallSensorAvailable(rclcpp::Time now) const;  // Sensor status
```

### 3. Travel Time Calculations (`travel_time.hpp`)

#### Simple Travel Time

```cpp
double getTravelTime(std::shared_ptr<RobotInfo> robot, Point target);
```

Basic calculation assuming constant velocity.

#### Trapezoidal Motion Profile

```cpp
double getTravelTimeTrapezoidal(
    std::shared_ptr<RobotInfo> robot,
    Point target,
    double max_acceleration,
    double max_velocity
);
```

**Motion Phases**:

1. **Acceleration Phase**: `t₁ = (v_max - v₀) / a`
2. **Cruise Phase**: `t₂ = d_remaining / v_max`
3. **Deceleration Phase**: `t₃ = v_max / a`

**Mathematical Model**:

- Handles cases with and without cruise phase
- Accounts for initial robot velocity in target direction
- Optimizes trajectory for minimum time

### 4. PID Controller (`pid_controller.hpp`)

#### Implementation

```cpp
class PIDController {
    double setGain(double p, double i, double d, double max_int = -1.0);
    double update(double error, double dt);
};
```

#### Features

- Standard PID control with proportional, integral, and derivative terms
- Integral windup protection with configurable limits
- High-precision control suitable for robot motion control

#### Mathematical Formula

```
output = Kp·error + Ki·∫error·dt + Kd·(d_error/dt)
```

### 5. Position Assignment System (`position_assignments.hpp`)

#### Hungarian Algorithm Implementation

Solves the assignment problem for optimal robot-target pairing:

```cpp
template<typename Geometry = Point>
std::vector<int> getOptimalAssignments(
    const std::vector<Point>& robot_positions,
    const std::vector<Geometry>& targets
);
```

#### Algorithm Details

- **Complexity**: O(n³) using Hungarian algorithm
- **Optimization**: Minimizes total euclidean distance
- **Flexibility**: Supports various target geometries (Point, Circle, etc.)
- **Use Cases**: Formation control, multi-robot coordination

### 6. Pass Analysis (`pass.hpp`)

#### Strategic Pass Evaluation

```cpp
struct Analysis {
    bool need_chip;                    // Whether chip kick is required
    double required_chip_distance;     // Minimum chip distance
};

Analysis getPassAnalysis(
    const Point& ball,
    const Point& target,
    std::vector<RobotInfo::SharedPtr> their_robots,
    double block_distance = 0.2
);
```

#### Analysis Process

1. **Path Construction**: Creates line segment from ball to target
2. **Obstacle Detection**: Finds closest opponent robots to pass line
3. **Blocking Assessment**: Determines if robots are within blocking distance
4. **Chip Recommendation**: Calculates required chip distance if blocked

### 7. Target Geometry System (`target_geometry.hpp`)

#### Dynamic Target Specification

Provides flexible target definition system using polymorphism:

```cpp
class TargetModule {
    static TargetModule buildBall();           // Target ball position
    static TargetModule buildFriend(uint8_t id);  // Target friendly robot
    static TargetModule buildEnemy(uint8_t id);   // Target enemy robot
    static TargetModule buildPoint(Point point);   // Target specific point
    Point getPoint(const WorldModelWrapper::SharedPtr& world_model);
};
```

#### Supported Target Types

- **TargetBall**: Dynamically tracks ball position
- **TargetFriendRobot**: Tracks friendly robot by ID
- **TargetEnemyRobot**: Tracks enemy robot by ID
- **TargetPoint**: Static point target
- **TargetBallLine**: Ball trajectory line segment

### 8. Ball Contact Detection (`ball_contact.hpp`)

#### Contact Tracking

```cpp
struct BallContact {
    std::chrono::system_clock::time_point last_contact_start_time;
    std::chrono::system_clock::time_point last_contact_end_time;

    void update(bool is_contacted);
    std::chrono::duration getContactDuration() const;
    bool findPastContact(double duration_sec) const;
};
```

#### Features

- Precise contact timing with microsecond resolution
- Contact duration measurement
- Historical contact detection within time windows
- Integration with robot dribbler systems

## Integration with crane_geometry

The package leverages `crane_geometry` for:

### Mathematical Types

```cpp
using Point = crane::Vector2d;          // 2D positions
using Point3D = crane::Vector3d;        // 3D positions
using Velocity2D = crane::Vector2d;     // 2D velocities
using Segment = bg::model::segment<Point>;  // Line segments
using Circle = crane::geometry::model::Circle<Point>;  // Circular geometry
```

### Geometric Operations

- Distance calculations
- Closest point computations
- Geometric intersections
- Boost.Geometry integration

## Usage in Crane System

### World Model Integration

The ball physics model is primarily used in:

- `crane_world_model_publisher` for state estimation
- Ball tracking and trajectory prediction
- Multi-hypothesis tracking systems

### Motion Planning

Robot dynamics calculations support:

- Local path planning (`crane_local_planner`)
- Skill execution timing (`crane_robot_skills`)
- Multi-robot coordination

### Game Analysis

Strategic calculations enable:

- Pass opportunity assessment
- Defensive positioning
- Formation optimization

## Testing

The package includes comprehensive unit tests:

### Test Coverage

- **Ball Physics**: State transitions, trajectory predictions, parabolic motion
- **Robot Dynamics**: Travel time calculations, motion profiles
- **PID Controller**: Gain settings, integral clamping, control output
- **Position Assignment**: Hungarian algorithm optimization
- **Message Conversions**: ROS2 message serialization/deserialization

### Running Tests

```bash
colcon test --packages-select crane_physics
colcon test-result --verbose
```

## Physical Constants and Calibration

### Ball Physics Parameters

- **Deceleration**: 0.5 m/s² (rolling friction on grass field)
- **Gravity**: -9.81 m/s² (standard Earth gravity)
- **Height Threshold**: 0.05m (flying vs. rolling detection)
- **Speed Thresholds**: 0.1 m/s (moving), 0.05 m/s (stopped)

### Robot Physical Properties

- **Robot Radius**: 0.060m (90mm regulation compliance)
- **Dribbler Distance**: 0.090m (center to ball contact)
- **Maximum Velocity**: ~4.0 m/s (typical robot capability)
- **Maximum Acceleration**: ~2.0 m/s² (typical robot capability)

## Dependencies

### Build Dependencies

- `ament_cmake_auto` - Build system
- `crane_geometry` - Mathematical operations and types

### Test Dependencies

- `ament_cmake_gtest` - Unit testing framework
- `crane_msgs` - ROS2 message definitions
- `rclcpp` - ROS2 C++ client library

## Performance Considerations

### Computational Complexity

- **Ball Predictions**: O(1) for simple states, O(n) for trajectory sampling
- **Hungarian Algorithm**: O(n³) for n robots
- **PID Controller**: O(1) per control cycle
- **Pass Analysis**: O(m) for m opponent robots

### Memory Usage

- Header-only design minimizes runtime overhead
- Efficient Eigen-based vector operations
- Minimal dynamic allocation in hot paths

### Real-time Performance

- Designed for 60-100Hz control loops
- Deterministic computation times
- Suitable for embedded robot controllers

## Future Enhancements

### Planned Features

1. **Advanced Air Resistance**: Quadratic drag model for flying balls
2. **Spin Dynamics**: Magnus effect and ball spin simulation
3. **Multi-Body Dynamics**: Robot-robot collision modeling
4. **Adaptive Parameters**: Environment-based parameter tuning
5. **GPU Acceleration**: CUDA support for large-scale simulations

### Research Areas

- Machine learning-based parameter estimation
- Probabilistic motion models
- Advanced trajectory optimization
- Multi-robot swarm dynamics

## License

MIT License - See LICENSE file for details.

## Contributing

Please refer to CONTRIBUTING.md for development guidelines and coding standards.
