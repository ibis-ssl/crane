// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <chrono>
#include <iomanip>
#include <iostream>
#include <modern_orca/modern_orca.hpp>

class ROS2CompatibleAgent : public modern_orca::CircularAgent
{
public:
  ROS2CompatibleAgent(
    modern_orca::AgentId id, const modern_orca::Vector2D & position,
    const modern_orca::Vector2D & preferred_velocity, modern_orca::Scalar max_speed,
    modern_orca::Scalar radius, int robot_id = -1)
  : modern_orca::CircularAgent(id, position, preferred_velocity, max_speed, radius),
    robot_id_(robot_id)
  {
  }

  auto getRobotId() const noexcept -> int { return robot_id_; }
  void setRobotId(int robot_id) { robot_id_ = robot_id; }

  auto clone() const
    -> std::unique_ptr<modern_orca::AgentBase<modern_orca::CircularCollisionModel>> override
  {
    return std::make_unique<ROS2CompatibleAgent>(*this);
  }

private:
  int robot_id_;
};

class CraneSSLFieldConstraint : public modern_orca::ConstraintBase<ROS2CompatibleAgent>
{
public:
  CraneSSLFieldConstraint(
    modern_orca::Scalar field_length = 12.0, modern_orca::Scalar field_width = 9.0,
    modern_orca::Scalar boundary_margin = 0.2, int priority = 95)
  : field_length_(field_length),
    field_width_(field_width),
    boundary_margin_(boundary_margin),
    priority_(priority)
  {
  }

  auto generateHalfPlanes(const ROS2CompatibleAgent & agent, modern_orca::TimeStep dt) const
    -> std::vector<modern_orca::HalfPlaneD> override
  {
    std::vector<modern_orca::HalfPlaneD> constraints;
    const auto pos = agent.position();
    const auto radius = agent.radius();
    const auto margin = boundary_margin_ + radius;

    const auto half_length = field_length_ * 0.5;
    const auto half_width = field_width_ * 0.5;

    if (pos.x() < -half_length + margin) {
      constraints.emplace_back(
        modern_orca::Vector2D{1.0, 0.0}, modern_orca::Vector2D{-half_length + margin, pos.y()});
    }
    if (pos.x() > half_length - margin) {
      constraints.emplace_back(
        modern_orca::Vector2D{-1.0, 0.0}, modern_orca::Vector2D{half_length - margin, pos.y()});
    }
    if (pos.y() < -half_width + margin) {
      constraints.emplace_back(
        modern_orca::Vector2D{0.0, 1.0}, modern_orca::Vector2D{pos.x(), -half_width + margin});
    }
    if (pos.y() > half_width - margin) {
      constraints.emplace_back(
        modern_orca::Vector2D{0.0, -1.0}, modern_orca::Vector2D{pos.x(), half_width - margin});
    }

    return constraints;
  }

  auto priority() const noexcept -> int override { return priority_; }
  auto name() const -> std::string override { return "CraneSSLFieldConstraint"; }

  auto clone() const -> std::unique_ptr<modern_orca::ConstraintBase<ROS2CompatibleAgent>> override
  {
    return std::make_unique<CraneSSLFieldConstraint>(*this);
  }

private:
  modern_orca::Scalar field_length_;
  modern_orca::Scalar field_width_;
  modern_orca::Scalar boundary_margin_;
  int priority_;
};

class CraneModernORCAPlanner
{
public:
  CraneModernORCAPlanner()
  : simulator_(std::make_unique<modern_orca::OptimalLinearProgram2DSolver>(4.0))
  {
    simulator_.setParallelExecution(true);
  }

  auto addRobot(
    int robot_id, const modern_orca::Vector2D & position, modern_orca::Scalar max_speed = 3.0,
    modern_orca::Scalar radius = 0.09) -> modern_orca::AgentId
  {
    auto agent_id =
      simulator_.addAgent(position, modern_orca::Vector2D{0, 0}, max_speed, radius, robot_id);

    simulator_.addConstraint<CraneSSLFieldConstraint>(agent_id);

    return agent_id;
  }

  void setupMultiRobotORCA() { simulator_.addORCAConstraints(); }

  void setTargetVelocity(
    modern_orca::AgentId agent_id, const modern_orca::Vector2D & target_velocity)
  {
    auto & agent = simulator_.getAgent(agent_id);
    agent.setPreferredVelocity(target_velocity);
  }

  void addBallAvoidance(
    const modern_orca::Vector2D & ball_position, modern_orca::Scalar avoidance_distance = 0.5)
  {
    for (auto & [agent_id, agent] : simulator_.getAllAgents()) {
      simulator_.addConstraint<SSL_BallAvoidanceConstraint>(
        agent_id, ball_position, avoidance_distance);
    }
  }

  void addPenaltyAreaAvoidance(const modern_orca::Vector2D & goal_center, bool is_our_goal = false)
  {
    const modern_orca::Vector2D penalty_size{1.8, 3.6};          // SSL penalty area size
    const modern_orca::Scalar margin = is_our_goal ? 0.1 : 0.2;  // Closer to our own goal

    for (auto & [agent_id, agent] : simulator_.getAllAgents()) {
      simulator_.addConstraint<SSL_PenaltyAreaConstraint>(
        agent_id, goal_center, penalty_size, margin);
    }
  }

  auto step(modern_orca::TimeStep dt) -> std::vector<modern_orca::Vector2D>
  {
    simulator_.step(dt);

    std::vector<modern_orca::Vector2D> velocities;
    for (const auto & [agent_id, agent] : simulator_.getAllAgents()) {
      velocities.push_back(agent->velocity());
    }

    return velocities;
  }

  auto getPositions() const -> std::vector<modern_orca::Vector2D>
  {
    std::vector<modern_orca::Vector2D> positions;
    for (const auto & [agent_id, agent] : simulator_.getAllAgents()) {
      positions.push_back(agent->position());
    }
    return positions;
  }

  auto getStatistics() const -> auto { return simulator_.getStatistics(); }

  void clear() { simulator_.clear(); }

private:
  modern_orca::Simulator<ROS2CompatibleAgent> simulator_;

  class SSL_BallAvoidanceConstraint : public modern_orca::ConstraintBase<ROS2CompatibleAgent>
  {
  public:
    SSL_BallAvoidanceConstraint(
      const modern_orca::Vector2D & ball_position, modern_orca::Scalar avoidance_distance)
    : ball_position_(ball_position), avoidance_distance_(avoidance_distance)
    {
    }

    auto generateHalfPlanes(const ROS2CompatibleAgent & agent, modern_orca::TimeStep dt) const
      -> std::vector<modern_orca::HalfPlaneD> override
    {
      std::vector<modern_orca::HalfPlaneD> constraints;

      const auto agent_pos = agent.position();
      const auto distance_to_ball = modern_orca::distance(agent_pos, ball_position_);
      const auto required_distance = avoidance_distance_ + agent.radius();

      if (distance_to_ball < required_distance + 0.1) {
        auto direction = (agent_pos - ball_position_).normalized();
        if (direction.isZero()) {
          direction = modern_orca::Vector2D{1.0, 0.0};
        }

        auto constraint_point = ball_position_ + direction * required_distance;
        constraints.emplace_back(direction, constraint_point);
      }

      return constraints;
    }

    auto priority() const noexcept -> int override { return 80; }
    auto name() const -> std::string override { return "SSL_BallAvoidanceConstraint"; }

    auto clone() const -> std::unique_ptr<modern_orca::ConstraintBase<ROS2CompatibleAgent>> override
    {
      return std::make_unique<SSL_BallAvoidanceConstraint>(*this);
    }

  private:
    modern_orca::Vector2D ball_position_;
    modern_orca::Scalar avoidance_distance_;
  };

  class SSL_PenaltyAreaConstraint : public modern_orca::ConstraintBase<ROS2CompatibleAgent>
  {
  public:
    SSL_PenaltyAreaConstraint(
      const modern_orca::Vector2D & penalty_center, const modern_orca::Vector2D & penalty_size,
      modern_orca::Scalar margin)
    : penalty_center_(penalty_center), penalty_size_(penalty_size), margin_(margin)
    {
    }

    auto generateHalfPlanes(const ROS2CompatibleAgent & agent, modern_orca::TimeStep dt) const
      -> std::vector<modern_orca::HalfPlaneD> override
    {
      std::vector<modern_orca::HalfPlaneD> constraints;

      const auto agent_pos = agent.position();
      const auto agent_radius = agent.radius();

      const auto half_width = penalty_size_.x() * 0.5 + margin_ + agent_radius;
      const auto half_height = penalty_size_.y() * 0.5 + margin_ + agent_radius;

      const auto min_x = penalty_center_.x() - half_width;
      const auto max_x = penalty_center_.x() + half_width;
      const auto min_y = penalty_center_.y() - half_height;
      const auto max_y = penalty_center_.y() + half_height;

      if (
        agent_pos.x() > min_x && agent_pos.x() < max_x && agent_pos.y() > min_y &&
        agent_pos.y() < max_y) {
        const auto dx_left = agent_pos.x() - min_x;
        const auto dx_right = max_x - agent_pos.x();
        const auto dy_bottom = agent_pos.y() - min_y;
        const auto dy_top = max_y - agent_pos.y();

        const auto min_dist = std::min({dx_left, dx_right, dy_bottom, dy_top});

        if (min_dist == dx_left) {
          constraints.emplace_back(
            modern_orca::Vector2D{-1, 0}, modern_orca::Vector2D{min_x, agent_pos.y()});
        } else if (min_dist == dx_right) {
          constraints.emplace_back(
            modern_orca::Vector2D{1, 0}, modern_orca::Vector2D{max_x, agent_pos.y()});
        } else if (min_dist == dy_bottom) {
          constraints.emplace_back(
            modern_orca::Vector2D{0, -1}, modern_orca::Vector2D{agent_pos.x(), min_y});
        } else {
          constraints.emplace_back(
            modern_orca::Vector2D{0, 1}, modern_orca::Vector2D{agent_pos.x(), max_y});
        }
      }

      return constraints;
    }

    auto priority() const noexcept -> int override { return 90; }
    auto name() const -> std::string override { return "SSL_PenaltyAreaConstraint"; }

    auto clone() const -> std::unique_ptr<modern_orca::ConstraintBase<ROS2CompatibleAgent>> override
    {
      return std::make_unique<SSL_PenaltyAreaConstraint>(*this);
    }

  private:
    modern_orca::Vector2D penalty_center_;
    modern_orca::Vector2D penalty_size_;
    modern_orca::Scalar margin_;
  };
};

int main()
{
  std::cout << "Modern ORCA Library - Crane Integration Example\n";
  std::cout << "==============================================\n\n";

  CraneModernORCAPlanner planner;

  std::vector<modern_orca::AgentId> robot_agents;
  for (int i = 0; i < 6; ++i) {
    const auto start_pos = modern_orca::Vector2D{
      -4.0 + i * 0.5,       // Spread along x-axis
      (i % 2) * 0.5 - 0.25  // Alternate y positions
    };

    robot_agents.push_back(planner.addRobot(i, start_pos, 3.0, 0.09));
  }

  planner.setupMultiRobotORCA();

  planner.addPenaltyAreaAvoidance(modern_orca::Vector2D{-6.0, 0.0}, true);  // Our goal
  planner.addPenaltyAreaAvoidance(modern_orca::Vector2D{6.0, 0.0}, false);  // Their goal

  planner.addBallAvoidance(modern_orca::Vector2D{0.0, 0.0}, 0.5);

  for (std::size_t i = 0; i < robot_agents.size(); ++i) {
    const auto target_velocity = modern_orca::Vector2D{
      1.5 * std::cos(2.0 * modern_orca::PI * i / 6.0),
      1.0 * std::sin(2.0 * modern_orca::PI * i / 6.0)};
    planner.setTargetVelocity(robot_agents[i], target_velocity);
  }

  std::cout << "Simulating SSL robot coordination scenario:\n";
  std::cout << "- 6 robots with ORCA collision avoidance\n";
  std::cout << "- SSL field boundary constraints\n";
  std::cout << "- Penalty area avoidance\n";
  std::cout << "- Ball avoidance (0.5m radius)\n";
  std::cout << "- Modern C++ implementation with parallel processing\n\n";

  const auto start_time = std::chrono::high_resolution_clock::now();

  for (int step = 0; step < 300; ++step) {
    auto velocities = planner.step(1.0 / 60.0);
    auto positions = planner.getPositions();

    if (step % 60 == 0) {
      const auto t = step * (1.0 / 60.0);
      std::cout << std::fixed << std::setprecision(2);
      std::cout << "t=" << std::setw(4) << t << "s: ";

      for (std::size_t i = 0; i < std::min(size_t(3), positions.size()); ++i) {
        std::cout << "R" << i << "(" << std::setw(5) << positions[i].x() << "," << std::setw(5)
                  << positions[i].y() << ") ";
      }

      std::cout << " |v|=[";
      for (std::size_t i = 0; i < std::min(size_t(3), velocities.size()); ++i) {
        std::cout << std::setw(4) << velocities[i].norm();
        if (i < std::min(size_t(3), velocities.size()) - 1) std::cout << ",";
      }
      std::cout << "]\n";
    }
  }

  const auto end_time = std::chrono::high_resolution_clock::now();
  const auto duration =
    std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

  auto stats = planner.getStatistics();
  std::cout << "\nPerformance Results:\n";
  std::cout << "Total simulation time: " << duration.count() << "ms\n";
  std::cout << "Agents: " << stats.agent_count << "\n";
  std::cout << "Total constraints: " << stats.total_constraints << "\n";
  std::cout << "Simulated time: " << stats.current_time << "s\n";
  std::cout << "Real-time factor: " << std::fixed << std::setprecision(1)
            << (stats.current_time * 1000.0) / duration.count() << "x\n";

  std::cout << "\nKey Advantages of Modern ORCA Library:\n";
  std::cout << "✓ Type-safe constraint system with compile-time checks\n";
  std::cout << "✓ Extensible plugin architecture for custom constraints\n";
  std::cout << "✓ Modern C++20 features (concepts, ranges, smart pointers)\n";
  std::cout << "✓ Header-only library for easy integration\n";
  std::cout << "✓ Parallel processing for multi-agent scenarios\n";
  std::cout << "✓ Direct half-plane constraint API for derived methods\n";
  std::cout << "✓ Drop-in replacement for existing RVO2 usage\n";

  std::cout << "\nCrane integration example completed successfully!\n";
  return 0;
}
