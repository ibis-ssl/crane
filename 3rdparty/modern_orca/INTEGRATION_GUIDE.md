# Modern ORCA Library - Crane Integration Guide

This guide shows how to integrate the Modern ORCA library into the existing Crane ROS 2 SSL project as a replacement for the current RVO2 implementation.

## Quick Integration

### 1. Update crane_local_planner/CMakeLists.txt

```cmake
# Add modern_orca dependency
find_package(modern_orca REQUIRED)

# Link to the existing rvo2_planner target
target_link_libraries(crane_local_planner
  PRIVATE
    modern_orca::modern_orca
    # ... other dependencies
)
```

### 2. Create Modern ORCA Wrapper

Replace the existing `rvo2_planner.cpp` implementation with a modern wrapper:

```cpp
#include "modern_orca_planner.hpp"
#include <modern_orca/modern_orca.hpp>

class ModernORCAPlanner : public LocalPlannerBase {
public:
    ModernORCAPlanner(rclcpp::Node & node) : LocalPlannerBase("modern_orca_planner", node) {
        // Configuration from ROS parameters
        simulator_ = std::make_unique<modern_orca::CircularAgentSimulator>();
        setupConstraints();
    }

    auto calculateRobotCommand(const crane_msgs::msg::RobotCommands & msg, double theta_offset)
        -> crane_msgs::msg::RobotCommands override {

        updateAgents(msg);
        simulator_->step(1.0 / 60.0);
        return extractResults(msg, theta_offset);
    }

private:
    std::unique_ptr<modern_orca::CircularAgentSimulator> simulator_;
    std::unordered_map<int, modern_orca::AgentId> robot_to_agent_map_;
};
```

### 3. Migration Benefits

Compared to the existing RVO2 implementation, Modern ORCA provides:

#### Constraint Flexibility

```cpp
// OLD: Manual position override in overrideTargetPosition()
if (isInBox(penalty_area, target_pos, PENALTY_AREA_OFFSET)) {
    while (isInBox(penalty_area, target_pos, PENALTY_AREA_OFFSET)) {
        target_pos += (target_pos - goal_pos).normalized() * 0.05;
    }
}

// NEW: Direct velocity space constraints
simulator_->addConstraint<SSL_PenaltyAreaConstraint>(
    agent_id, penalty_center, penalty_size, margin
);
```

#### Custom SSL Constraints

```cpp
// Add SSL-specific constraints directly in velocity space
class SSL_BallPlacementConstraint : public modern_orca::ConstraintBase<CircularAgent> {
    auto generateHalfPlanes(const CircularAgent& agent, TimeStep dt) const
        -> std::vector<modern_orca::HalfPlaneD> override {
        // Direct half-plane constraint implementation
        // Much more accurate than position-space workarounds
    }
};
```

#### Type Safety

```cpp
// OLD: Manual agent management with integer indices
rvo_sim->setAgentPosition(command.robot_id, RVO::Vector2(pos.x, pos.y));

// NEW: Type-safe agent management
auto agent_id = simulator_->addAgent(position, preferred_vel, max_speed, radius);
simulator_->getAgent(agent_id).setPosition(new_position);
```

## Advanced Integration Features

### 1. Custom SSL Constraints

```cpp
// Replace manual penalty area avoidance with proper constraints
class CraneSSLConstraints {
public:
    static void addAllSSLConstraints(modern_orca::CircularAgentSimulator& sim,
                                   const WorldModel& world_model) {

        // Ball avoidance with game state awareness
        auto ball_distance = getSSLBallDistance(world_model.getGameState());
        sim.addGlobalConstraint<SSL_BallAvoidanceConstraint>(
            world_model.ball().pos, ball_distance);

        // Dynamic penalty area constraints
        sim.addGlobalConstraint<SSL_PenaltyAreaConstraint>(
            world_model.getOurPenaltyArea(),
            getSSLPenaltyMargin(world_model.getGameState()));

        // Ball placement area avoidance
        if (auto placement_area = world_model.getBallPlacementArea()) {
            sim.addGlobalConstraint<SSL_BallPlacementConstraint>(*placement_area);
        }
    }
};
```

### 2. Performance Improvements

```cpp
// Enable parallel processing for better performance
simulator_->setParallelExecution(true);

// Use optimal solver for better velocity solutions
auto optimal_solver = std::make_unique<modern_orca::OptimalLinearProgram2DSolver>(max_speed);
simulator_->setSolver(std::move(optimal_solver));
```

### 3. Integration with Existing Code

```cpp
// Convert between crane types and modern_orca types
modern_orca::Vector2D toModernORCA(const crane::Point& point) {
    return {point.x(), point.y()};
}

crane::Point fromModernORCA(const modern_orca::Vector2D& vec) {
    return crane::Point(vec.x(), vec.y());
}

// Maintain compatibility with existing robot command structure
crane_msgs::msg::RobotCommands extractResults(
    const crane_msgs::msg::RobotCommands& original_commands,
    double theta_offset) {

    crane_msgs::msg::RobotCommands result;
    for (const auto& cmd : original_commands.robot_commands) {
        auto agent_id = robot_to_agent_map_[cmd.robot_id];
        auto velocity = simulator_->getAgent(agent_id).velocity();

        // Convert back to original command format
        crane_msgs::msg::RobotCommand new_cmd = cmd;
        new_cmd.control_mode = crane_msgs::msg::RobotCommand::POLAR_VELOCITY_TARGET_MODE;
        // ... populate velocity fields

        result.robot_commands.push_back(new_cmd);
    }
    return result;
}
```

## Migration Strategy

### Phase 1: Drop-in Replacement

- Replace RVO2 library calls with Modern ORCA equivalents
- Maintain existing constraint logic initially
- Verify performance and behavior match

### Phase 2: Constraint Migration

- Convert position-space constraints to velocity-space constraints
- Implement SSL-specific constraint classes
- Remove manual position override logic

### Phase 3: Optimization

- Enable parallel processing
- Use optimal solvers
- Add custom constraints for advanced behaviors

## Performance Comparison

| Metric | Original RVO2 | Modern ORCA |
|--------|---------------|-------------|
| Constraint Types | 1 (ORCA only) | Unlimited extensible |
| Constraint Accuracy | Position-space workarounds | Direct velocity-space |
| Type Safety | Manual index management | Compile-time type safety |
| Parallel Processing | No | Yes (OpenMP) |
| Custom Constraints | Requires library modification | Plugin architecture |
| Memory Safety | Manual memory management | RAII with smart pointers |
| SSL Integration | Manual position hacks | Direct constraint API |

## Testing

```cpp
// Comprehensive test suite included
#include <modern_orca/modern_orca.hpp>

TEST_CASE("SSL Integration", "[ssl]") {
    modern_orca::CircularAgentSimulator sim;

    // Test SSL-specific constraints
    auto agent = sim.addAgent(Vector2D{0, 0}, Vector2D{1, 0}, 2.0, 0.09);
    sim.addConstraint<SSL_BallAvoidanceConstraint>(agent, Vector2D{0.3, 0}, 0.5);

    sim.step(1.0/60.0);

    // Verify ball avoidance behavior
    REQUIRE(distance(sim.getAgent(agent).position(), Vector2D{0.3, 0}) >= 0.5);
}
```

The Modern ORCA library provides a complete, type-safe, and extensible replacement for the existing RVO2 implementation while maintaining full compatibility with Crane's architecture.
