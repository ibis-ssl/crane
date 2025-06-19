#include <modern_orca/modern_orca.hpp>
#include <iostream>
#include <iomanip>

using namespace modern_orca;

int main() {
    std::cout << "Modern ORCA Library - Basic Example\n";
    std::cout << "===================================\n\n";
    
    CircularAgentSimulator simulator;
    
    auto agent1 = simulator.addAgent(
        Vector2D{-2.0, 0.0},   // position
        Vector2D{2.0, 0.0},    // preferred velocity  
        2.0,                   // max speed
        0.5                    // radius
    );
    
    auto agent2 = simulator.addAgent(
        Vector2D{2.0, 0.0},    // position
        Vector2D{-2.0, 0.0},   // preferred velocity
        2.0,                   // max speed
        0.5                    // radius
    );
    
    auto agent3 = simulator.addAgent(
        Vector2D{0.0, -2.0},   // position
        Vector2D{0.0, 2.0},    // preferred velocity
        2.0,                   // max speed
        0.5                    // radius
    );
    
    simulator.addORCAConstraints();
    
    simulator.addGlobalConstraint<BoundaryConstraint<CircularAgent>>(
        Vector2D{-5.0, -5.0},  // min bounds
        Vector2D{5.0, 5.0},    // max bounds
        0.1                    // margin
    );
    
    simulator.addConstraint<CircularObstacleConstraint<CircularAgent>>(
        agent1,
        Vector2D{0.0, 0.0},    // obstacle center
        0.3                    // obstacle radius
    );
    
    std::cout << "Initial Configuration:\n";
    std::cout << "Agent 1: pos=" << simulator.getAgent(agent1).position().x() 
              << "," << simulator.getAgent(agent1).position().y() << "\n";
    std::cout << "Agent 2: pos=" << simulator.getAgent(agent2).position().x() 
              << "," << simulator.getAgent(agent2).position().y() << "\n";
    std::cout << "Agent 3: pos=" << simulator.getAgent(agent3).position().x() 
              << "," << simulator.getAgent(agent3).position().y() << "\n\n";
    
    std::cout << "Simulation Progress:\n";
    std::cout << std::fixed << std::setprecision(2);
    
    for (int step = 0; step < 100; ++step) {
        simulator.step(1.0 / 60.0);  // 60 FPS
        
        if (step % 10 == 0) {
            std::cout << "Step " << std::setw(3) << step << ": ";
            std::cout << "Agent1(" << std::setw(5) << simulator.getAgent(agent1).position().x() 
                     << "," << std::setw(5) << simulator.getAgent(agent1).position().y() << ") ";
            std::cout << "Agent2(" << std::setw(5) << simulator.getAgent(agent2).position().x() 
                     << "," << std::setw(5) << simulator.getAgent(agent2).position().y() << ") ";
            std::cout << "Agent3(" << std::setw(5) << simulator.getAgent(agent3).position().x() 
                     << "," << std::setw(5) << simulator.getAgent(agent3).position().y() << ")\n";
        }
    }
    
    auto stats = simulator.getStatistics();
    std::cout << "\nSimulation Statistics:\n";
    std::cout << "Agent count: " << stats.agent_count << "\n";
    std::cout << "Total constraints: " << stats.total_constraints << "\n";
    std::cout << "Final time: " << stats.current_time << "s\n";
    
    std::cout << "\nExample completed successfully!\n";
    return 0;
}