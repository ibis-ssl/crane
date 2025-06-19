#pragma once

#include "../types.hpp"
#include "../concepts.hpp"
#include <vector>
#include <memory>
#include <optional>
#include <unordered_map>
#include <functional>
#include <string>

namespace modern_orca {

template<typename SolverType>
class SolverBase {
public:
    virtual ~SolverBase() = default;
    
    virtual auto solve(const std::vector<HalfPlaneD>& constraints, 
                      const Vector2D& preferred_velocity) -> Vector2D = 0;
    
    virtual auto feasible() const noexcept -> bool = 0;
    virtual auto iterations() const noexcept -> std::size_t = 0;
    virtual auto solutionQuality() const noexcept -> Scalar { return 0.0; }
    
    virtual void reset() {}
    virtual auto name() const -> std::string = 0;
    
    virtual auto clone() const -> std::unique_ptr<SolverBase> = 0;

protected:
    SolverBase() = default;
};

class LinearProgram2DSolver : public SolverBase<LinearProgram2DSolver> {
public:
    LinearProgram2DSolver(Scalar max_speed = 10.0, std::size_t max_iterations = 100)
        : max_speed_(max_speed), max_iterations_(max_iterations), 
          feasible_(false), iterations_(0) {}
    
    auto solve(const std::vector<HalfPlaneD>& constraints, 
              const Vector2D& preferred_velocity) -> Vector2D override {
        reset();
        
        if (constraints.empty()) {
            feasible_ = true;
            auto result = preferred_velocity;
            if (result.norm() > max_speed_) {
                result = result.normalized() * max_speed_;
            }
            return result;
        }
        
        Vector2D solution = preferred_velocity;
        
        if (solution.norm() > max_speed_) {
            solution = solution.normalized() * max_speed_;
        }
        
        for (std::size_t iter = 0; iter < max_iterations_; ++iter) {
            iterations_++;
            bool all_satisfied = true;
            
            for (const auto& constraint : constraints) {
                if (!constraint.contains(solution)) {
                    all_satisfied = false;
                    solution = projectOntoConstraint(solution, constraint);
                    
                    if (solution.norm() > max_speed_ + EPSILON) {
                        solution = solution.normalized() * max_speed_;
                    }
                }
            }
            
            if (all_satisfied) {
                feasible_ = true;
                return solution;
            }
        }
        
        feasible_ = false;
        return solution;
    }
    
    auto feasible() const noexcept -> bool override { return feasible_; }
    auto iterations() const noexcept -> std::size_t override { return iterations_; }
    auto name() const -> std::string override { return "LinearProgram2DSolver"; }
    
    auto clone() const -> std::unique_ptr<SolverBase<LinearProgram2DSolver>> override {
        return std::make_unique<LinearProgram2DSolver>(*this);
    }
    
    void reset() override {
        feasible_ = false;
        iterations_ = 0;
    }
    
    auto maxSpeed() const noexcept -> Scalar { return max_speed_; }
    void setMaxSpeed(Scalar max_speed) { max_speed_ = std::max(Scalar{0}, max_speed); }

private:
    Scalar max_speed_;
    std::size_t max_iterations_;
    bool feasible_;
    std::size_t iterations_;
    
    auto projectOntoConstraint(const Vector2D& point, const HalfPlaneD& constraint) const -> Vector2D {
        const auto distance = constraint.signedDistance(point);
        if (distance >= 0) {
            return point;
        }
        return point - distance * constraint.normal;
    }
};

class OptimalLinearProgram2DSolver : public SolverBase<OptimalLinearProgram2DSolver> {
public:
    OptimalLinearProgram2DSolver(Scalar max_speed = 10.0)
        : max_speed_(max_speed), feasible_(false), iterations_(0) {}
    
    auto solve(const std::vector<HalfPlaneD>& constraints, 
              const Vector2D& preferred_velocity) -> Vector2D override {
        reset();
        
        if (constraints.empty()) {
            feasible_ = true;
            auto result = preferred_velocity;
            if (result.norm() > max_speed_) {
                result = result.normalized() * max_speed_;
            }
            return result;
        }
        
        Vector2D solution = preferred_velocity;
        if (solution.norm() > max_speed_) {
            solution = solution.normalized() * max_speed_;
        }
        
        std::vector<HalfPlaneD> active_constraints;
        active_constraints.reserve(constraints.size() + 1);
        
        for (const auto& constraint : constraints) {
            active_constraints.push_back(constraint);
        }
        
        active_constraints.emplace_back(Vector2D{0, 0}, Vector2D{0, 0});
        
        for (std::size_t i = 0; i < active_constraints.size(); ++i) {
            iterations_++;
            
            if (i == active_constraints.size() - 1) {
                if (solution.norm() <= max_speed_ + EPSILON) {
                    break;
                }
                
                solution = solution.normalized() * max_speed_;
                continue;
            }
            
            const auto& constraint = active_constraints[i];
            if (constraint.contains(solution)) {
                continue;
            }
            
            if (i == 0) {
                solution = projectOntoConstraint(solution, constraint);
                continue;
            }
            
            Vector2D intersection;
            bool found_intersection = false;
            
            for (std::size_t j = 0; j < i; ++j) {
                const auto& other_constraint = active_constraints[j];
                auto intersection_result = findIntersection(constraint, other_constraint);
                
                if (intersection_result.has_value()) {
                    intersection = intersection_result.value();
                    found_intersection = true;
                    
                    bool valid = true;
                    for (std::size_t k = 0; k < i; ++k) {
                        if (k != j && !active_constraints[k].contains(intersection)) {
                            valid = false;
                            break;
                        }
                    }
                    
                    if (valid && intersection.norm() <= max_speed_ + EPSILON) {
                        solution = intersection;
                        break;
                    }
                }
            }
            
            if (!found_intersection) {
                solution = projectOntoConstraint(solution, constraint);
            }
        }
        
        feasible_ = true;
        for (const auto& constraint : constraints) {
            if (!constraint.contains(solution, EPSILON)) {
                feasible_ = false;
                break;
            }
        }
        
        return solution;
    }
    
    auto feasible() const noexcept -> bool override { return feasible_; }
    auto iterations() const noexcept -> std::size_t override { return iterations_; }
    auto name() const -> std::string override { return "OptimalLinearProgram2DSolver"; }
    
    auto clone() const -> std::unique_ptr<SolverBase<OptimalLinearProgram2DSolver>> override {
        return std::make_unique<OptimalLinearProgram2DSolver>(*this);
    }
    
    void reset() override {
        feasible_ = false;
        iterations_ = 0;
    }

private:
    Scalar max_speed_;
    bool feasible_;
    std::size_t iterations_;
    
    auto projectOntoConstraint(const Vector2D& point, const HalfPlaneD& constraint) const -> Vector2D {
        const auto distance = constraint.signedDistance(point);
        if (distance >= 0) {
            return point;
        }
        return point - distance * constraint.normal;
    }
    
    auto findIntersection(const HalfPlaneD& plane1, const HalfPlaneD& plane2) const 
        -> std::optional<Vector2D> {
        const auto& n1 = plane1.normal;
        const auto& n2 = plane2.normal;
        const auto& p1 = plane1.point;
        const auto& p2 = plane2.point;
        
        const auto det = cross(n1, n2);
        if (std::abs(det) < EPSILON) {
            return std::nullopt;
        }
        
        const auto c1 = dot(n1, p1);
        const auto c2 = dot(n2, p2);
        
        const auto intersection = Vector2D{
            (c1 * n2.y() - c2 * n1.y()) / det,
            (c2 * n1.x() - c1 * n2.x()) / det
        };
        
        return intersection;
    }
};

class SolverRegistry {
public:
    using SolverFactory = std::function<std::unique_ptr<SolverBase<void>>()>;
    
    static auto instance() -> SolverRegistry& {
        static SolverRegistry registry;
        return registry;
    }
    
    template<typename SolverType, typename... Args>
    static void registerSolver(const std::string& name) {
        auto& registry = instance();
        registry.factories_[name] = []() -> std::unique_ptr<SolverBase<void>> {
            return std::make_unique<SolverType>();
        };
    }
    
    static auto createSolver(const std::string& name) -> std::unique_ptr<SolverBase<void>> {
        auto& registry = instance();
        auto it = registry.factories_.find(name);
        if (it != registry.factories_.end()) {
            return it->second();
        }
        return nullptr;
    }

private:
    std::unordered_map<std::string, SolverFactory> factories_;
};

}  // namespace modern_orca