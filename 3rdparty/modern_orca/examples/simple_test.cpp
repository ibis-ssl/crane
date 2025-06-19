// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <iostream>
#include <modern_orca/modern_orca.hpp>

int main()
{
  std::cout << "Modern ORCA Library - Simple Test\n";
  std::cout << "=================================\n\n";

  // Test basic types
  modern_orca::Vector2D v1(1.0, 2.0);
  modern_orca::Vector2D v2(3.0, 4.0);
  auto sum = v1 + v2;

  std::cout << "Vector test: (" << v1.x() << "," << v1.y() << ") + (" << v2.x() << "," << v2.y()
            << ") = (" << sum.x() << "," << sum.y() << ")\n";

  // Test agents
  modern_orca::CircularAgent agent1(
    0, modern_orca::Vector2D{0, 0}, modern_orca::Vector2D{1, 0}, 2.0, 0.1);
  modern_orca::CircularAgent agent2(
    1, modern_orca::Vector2D{1, 0}, modern_orca::Vector2D{-1, 0}, 2.0, 0.1);

  std::cout << "Agent 1 position: (" << agent1.position().x() << "," << agent1.position().y()
            << ")\n";
  std::cout << "Agent 2 position: (" << agent2.position().x() << "," << agent2.position().y()
            << ")\n";

  // Test basic simulation without parallel execution
  modern_orca::CircularAgentSimulator simulator;
  simulator.setParallelExecution(false);  // Disable parallel processing

  auto id1 =
    simulator.addAgent(modern_orca::Vector2D{-1, 0}, modern_orca::Vector2D{1, 0}, 2.0, 0.1);
  auto id2 =
    simulator.addAgent(modern_orca::Vector2D{1, 0}, modern_orca::Vector2D{-1, 0}, 2.0, 0.1);

  simulator.addORCAConstraints();

  std::cout << "\nSimulation test (10 steps):\n";
  for (int i = 0; i < 10; ++i) {
    simulator.step(0.1);
    auto pos1 = simulator.getAgent(id1).position();
    auto pos2 = simulator.getAgent(id2).position();

    std::cout << "Step " << i << ": Agent1(" << pos1.x() << "," << pos1.y() << ") Agent2("
              << pos2.x() << "," << pos2.y() << ")\n";
  }

  auto stats = simulator.getStatistics();
  std::cout << "\nStatistics:\n";
  std::cout << "Agents: " << stats.agent_count << "\n";
  std::cout << "Constraints: " << stats.total_constraints << "\n";
  std::cout << "Time: " << stats.current_time << "s\n";

  std::cout << "\nBasic functionality test completed successfully!\n";
  return 0;
}
