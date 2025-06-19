# Modern ORCA Library

A modern C++20 implementation of the Optimal Reciprocal Collision Avoidance (ORCA) algorithm with extensible constraints and plugin support.

## Features

- **Modern C++20**: Smart pointers, concepts, ranges, and type-safe design
- **Extensible Architecture**: Plugin system for custom constraints and derived methods
- **Half-plane Constraint API**: Direct interface for adding custom velocity constraints
- **Header-only**: Template-based implementation for optimal performance
- **Type-safe**: Strong typing with compile-time safety
- **Multi-threaded**: OpenMP support for parallel agent processing

## Quick Start

```cpp
#include <modern_orca/simulator.hpp>
#include <modern_orca/agents/circular_agent.hpp>
#include <modern_orca/constraints/orca_constraint.hpp>

using namespace modern_orca;

// Create simulator
Simulator simulator;

// Add agents
auto agent1 = simulator.addAgent<CircularAgent>(
    Vector2D{0.0, 0.0},    // position
    Vector2D{1.0, 0.0},    // preferred velocity
    0.1,                   // radius
    2.0                    // max speed
);

auto agent2 = simulator.addAgent<CircularAgent>(
    Vector2D{2.0, 0.0},    // position
    Vector2D{-1.0, 0.0},   // preferred velocity
    0.1,                   // radius
    2.0                    // max speed
);

// Add custom constraint
simulator.addConstraint<CustomHalfPlaneConstraint>(
    agent1, 
    Vector2D{0.0, 1.0},    // normal vector
    Vector2D{0.0, 0.5}     // point on line
);

// Run simulation
for (int step = 0; step < 100; ++step) {
    simulator.step(1.0 / 60.0);  // 60 FPS
    
    // Get results
    auto pos1 = simulator.getAgent(agent1).position();
    auto vel1 = simulator.getAgent(agent1).velocity();
}
```

## Architecture

### Core Components

- **Agents**: Template-based agent classes (CircularAgent, PolygonAgent)
- **Constraints**: Extensible constraint system with plugin support
- **Simulator**: Main simulation orchestrator with multi-threading
- **Solvers**: Pluggable LP solvers for velocity optimization

### Extensibility

```cpp
// Custom constraint
class MyConstraint : public Constraint {
public:
    auto generateHalfPlanes(const Agent& agent, TimeStep dt) const 
        -> std::vector<HalfPlane> override {
        // Your custom constraint logic
        return {HalfPlane{normal, point}};
    }
};

// Register and use
ConstraintRegistry::register<MyConstraint>("my_constraint");
simulator.addConstraint<MyConstraint>(agent_id, /* parameters */);
```

## Building

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

## Integration

### CMake Integration

```cmake
find_package(modern_orca REQUIRED)
target_link_libraries(your_target modern_orca::modern_orca)
```

### ROS 2 Integration

See `examples/ros2_wrapper.cpp` for a complete ROS 2 wrapper implementation.

## Performance

- **Header-only**: Zero-cost abstractions and inlining
- **SIMD**: Vectorized operations where possible
- **Parallel**: Multi-threaded agent processing
- **Memory-efficient**: Custom allocators and object pooling

## License

MIT License - see LICENSE file for details.