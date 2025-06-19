#include <modern_orca/modern_orca.hpp>
#include <iostream>
#include <cmath>

using namespace modern_orca;

class SSL_BallAvoidanceConstraint : public ConstraintBase<CircularAgent> {
public:
    SSL_BallAvoidanceConstraint(const Vector2D& ball_position, Scalar avoidance_distance = 0.5, 
                               int priority = 80)
        : ball_position_(ball_position), avoidance_distance_(avoidance_distance), 
          priority_(priority) {}
    
    auto generateHalfPlanes(const CircularAgent& agent, TimeStep dt) const 
        -> std::vector<HalfPlaneD> override {
        std::vector<HalfPlaneD> constraints;
        
        const auto agent_pos = agent.position();
        const auto distance_to_ball = distance(agent_pos, ball_position_);
        const auto required_distance = avoidance_distance_ + agent.radius();
        
        if (distance_to_ball < required_distance + 0.1) {
            auto direction = (agent_pos - ball_position_).normalized();
            if (direction.isZero()) {
                direction = Vector2D{1.0, 0.0};
            }
            
            auto constraint_point = ball_position_ + direction * required_distance;
            constraints.emplace_back(direction, constraint_point);
        }
        
        return constraints;
    }
    
    auto priority() const noexcept -> int override { return priority_; }
    auto name() const -> std::string override { return "SSL_BallAvoidanceConstraint"; }
    
    auto clone() const -> std::unique_ptr<ConstraintBase<CircularAgent>> override {
        return std::make_unique<SSL_BallAvoidanceConstraint>(*this);
    }
    
    void setBallPosition(const Vector2D& position) { ball_position_ = position; }
    auto getBallPosition() const noexcept -> const Vector2D& { return ball_position_; }

private:
    Vector2D ball_position_;
    Scalar avoidance_distance_;
    int priority_;
};

class SSL_PenaltyAreaConstraint : public ConstraintBase<CircularAgent> {
public:
    SSL_PenaltyAreaConstraint(const Vector2D& penalty_center, const Vector2D& penalty_size,
                             Scalar margin = 0.2, int priority = 90)
        : penalty_center_(penalty_center), penalty_size_(penalty_size), 
          margin_(margin), priority_(priority) {}
    
    auto generateHalfPlanes(const CircularAgent& agent, TimeStep dt) const 
        -> std::vector<HalfPlaneD> override {
        std::vector<HalfPlaneD> constraints;
        
        const auto agent_pos = agent.position();
        const auto agent_radius = agent.radius();
        
        const auto half_width = penalty_size_.x() * 0.5 + margin_ + agent_radius;
        const auto half_height = penalty_size_.y() * 0.5 + margin_ + agent_radius;
        
        const auto min_x = penalty_center_.x() - half_width;
        const auto max_x = penalty_center_.x() + half_width;
        const auto min_y = penalty_center_.y() - half_height;
        const auto max_y = penalty_center_.y() + half_height;
        
        if (agent_pos.x() > min_x && agent_pos.x() < max_x &&
            agent_pos.y() > min_y && agent_pos.y() < max_y) {
            
            const auto dx_left = agent_pos.x() - min_x;
            const auto dx_right = max_x - agent_pos.x();
            const auto dy_bottom = agent_pos.y() - min_y;
            const auto dy_top = max_y - agent_pos.y();
            
            const auto min_dist = std::min({dx_left, dx_right, dy_bottom, dy_top});
            
            if (min_dist == dx_left) {
                constraints.emplace_back(Vector2D{-1, 0}, Vector2D{min_x, agent_pos.y()});
            } else if (min_dist == dx_right) {
                constraints.emplace_back(Vector2D{1, 0}, Vector2D{max_x, agent_pos.y()});
            } else if (min_dist == dy_bottom) {
                constraints.emplace_back(Vector2D{0, -1}, Vector2D{agent_pos.x(), min_y});
            } else {
                constraints.emplace_back(Vector2D{0, 1}, Vector2D{agent_pos.x(), max_y});
            }
        }
        
        return constraints;
    }
    
    auto priority() const noexcept -> int override { return priority_; }
    auto name() const -> std::string override { return "SSL_PenaltyAreaConstraint"; }
    
    auto clone() const -> std::unique_ptr<ConstraintBase<CircularAgent>> override {
        return std::make_unique<SSL_PenaltyAreaConstraint>(*this);
    }

private:
    Vector2D penalty_center_;
    Vector2D penalty_size_;
    Scalar margin_;
    int priority_;
};

class DynamicFormationConstraint : public ConstraintBase<CircularAgent> {
public:
    DynamicFormationConstraint(const Vector2D& formation_center, Scalar formation_radius,
                              Scalar time_period = 5.0, int priority = 30)
        : formation_center_(formation_center), formation_radius_(formation_radius),
          time_period_(time_period), priority_(priority) {}
    
    auto generateHalfPlanes(const CircularAgent& agent, TimeStep dt) const 
        -> std::vector<HalfPlaneD> override {
        std::vector<HalfPlaneD> constraints;
        
        const auto current_time = dt * agent.id();
        const auto rotation_angle = 2.0 * PI * current_time / time_period_;
        
        const auto target_offset = Vector2D{
            formation_radius_ * std::cos(rotation_angle),
            formation_radius_ * std::sin(rotation_angle)
        };
        const auto target_position = formation_center_ + target_offset;
        
        const auto agent_pos = agent.position();
        const auto direction_to_target = (target_position - agent_pos).normalized();
        
        if (!direction_to_target.isZero()) {
            const auto constraint_point = agent_pos + direction_to_target * 0.1;
            constraints.emplace_back(direction_to_target, constraint_point);
        }
        
        return constraints;
    }
    
    auto priority() const noexcept -> int override { return priority_; }
    auto name() const -> std::string override { return "DynamicFormationConstraint"; }
    
    auto clone() const -> std::unique_ptr<ConstraintBase<CircularAgent>> override {
        return std::make_unique<DynamicFormationConstraint>(*this);
    }

private:
    Vector2D formation_center_;
    Scalar formation_radius_;
    Scalar time_period_;
    int priority_;
};

int main() {
    std::cout << "Modern ORCA Library - Custom Constraints Example\n";
    std::cout << "===============================================\n\n";
    
    CircularAgentSimulator simulator;
    
    std::vector<AgentId> agents;
    for (int i = 0; i < 6; ++i) {
        const auto angle = 2.0 * PI * i / 6.0;
        const auto start_pos = Vector2D{
            3.0 * std::cos(angle),
            3.0 * std::sin(angle)
        };
        
        agents.push_back(simulator.addAgent(
            start_pos,             // position
            Vector2D{0.0, 0.0},    // preferred velocity (formation will control)
            1.5,                   // max speed
            0.08                   // radius (small robots)
        ));
    }
    
    simulator.addORCAConstraints();
    
    const Vector2D ball_position{1.0, 1.0};
    simulator.addGlobalConstraint<SSL_BallAvoidanceConstraint>(
        ball_position,
        0.5  // SSL ball avoidance distance
    );
    
    const Vector2D penalty_center{-4.0, 0.0};
    const Vector2D penalty_size{2.0, 3.0};
    simulator.addGlobalConstraint<SSL_PenaltyAreaConstraint>(
        penalty_center,
        penalty_size,
        0.2  // margin
    );
    
    simulator.addGlobalConstraint<DynamicFormationConstraint>(
        Vector2D{0.0, 0.0},    // formation center
        2.0,                   // formation radius
        10.0                   // rotation period
    );
    
    simulator.addGlobalConstraint<BoundaryConstraint<CircularAgent>>(
        Vector2D{-6.0, -4.0},  // field bounds
        Vector2D{6.0, 4.0}
    );
    
    std::cout << "Custom constraints demonstration:\n";
    std::cout << "- SSL Ball Avoidance (0.5m from ball)\n";
    std::cout << "- SSL Penalty Area Avoidance\n";
    std::cout << "- Dynamic Formation Control (rotating circle)\n";
    std::cout << "- Field Boundary Constraints\n";
    std::cout << "- ORCA Collision Avoidance\n\n";
    
    auto* ball_constraint = simulator.getConstraintManager(agents[0])
        .template getConstraintsOfType<SSL_BallAvoidanceConstraint>()[0];
    
    std::cout << "Simulation with moving ball:\n";
    std::cout << std::fixed << std::setprecision(2);
    
    for (int step = 0; step < 200; ++step) {
        const auto t = step * (1.0 / 60.0);
        
        const auto new_ball_pos = Vector2D{
            1.0 + 1.5 * std::sin(t * 0.5),
            1.0 + 1.0 * std::cos(t * 0.3)
        };
        
        for (auto agent_id : agents) {
            auto ball_constraints = simulator.getConstraintManager(agent_id)
                .template getConstraintsOfType<SSL_BallAvoidanceConstraint>();
            for (auto* constraint : ball_constraints) {
                constraint->setBallPosition(new_ball_pos);
            }
        }
        
        simulator.step(1.0 / 60.0);
        
        if (step % 30 == 0) {
            std::cout << "t=" << std::setw(4) << t << "s: Ball(" 
                     << new_ball_pos.x() << "," << new_ball_pos.y() << ") ";
            std::cout << "Agent0(" << simulator.getAgent(agents[0]).position().x() 
                     << "," << simulator.getAgent(agents[0]).position().y() << ")\n";
        }
    }
    
    auto stats = simulator.getStatistics();
    std::cout << "\nFinal Statistics:\n";
    std::cout << "Agents: " << stats.agent_count << "\n";
    std::cout << "Total constraints: " << stats.total_constraints << "\n";
    std::cout << "Simulation time: " << stats.current_time << "s\n";
    
    std::cout << "\nCustom constraints example completed!\n";
    return 0;
}