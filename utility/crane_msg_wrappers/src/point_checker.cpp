// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_msg_wrappers/point_checker.hpp"

#include <cmath>
#include <crane_msg_wrappers/world_model_wrapper.hpp>

namespace crane
{

PointChecker::PointChecker(std::shared_ptr<WorldModelWrapper> & world_model)
: world_model_(world_model.get())
{
}

PointChecker::PointChecker(WorldModelWrapper * world_model) : world_model_(world_model) {}

auto PointChecker::isFieldInside(const Point & p, double offset) const -> bool
{
  Box field_box;
  field_box.min_corner() << -world_model_->fieldSize().x() / 2.f - offset,
    -world_model_->fieldSize().y() / 2.f - offset;
  field_box.max_corner() << world_model_->fieldSize().x() / 2.f + offset,
    world_model_->fieldSize().y() / 2.f + offset;
  return isInBox(field_box, p);
}

auto PointChecker::addFieldInsideChecker(double offset) -> void
{
  checkers_.emplace_back([this, offset](const Point & p) { return isFieldInside(p, offset); });
}

auto PointChecker::addFieldOutsideChecker(double offset) -> void
{
  checkers_.emplace_back([this, offset](const Point & p) { return not isFieldInside(p, offset); });
}

auto PointChecker::isBallPlacementArea(const Point & p, double offset) const -> bool
{
  if (auto area = world_model_->getBallPlacementArea(offset)) {
    return bg::distance(area.value(), p) < 0.001;
  } else {
    return false;
  }
}

auto PointChecker::addBallPlacementAreaInsideChecker(double offset) -> void
{
  checkers_.emplace_back(
    [this, offset](const Point & p) { return isBallPlacementArea(p, offset); });
}

auto PointChecker::addBallPlacementAreaOutsideChecker(double offset) -> void
{
  checkers_.emplace_back(
    [this, offset](const Point & p) { return not isBallPlacementArea(p, offset); });
}

auto PointChecker::isEnemyPenaltyArea(const Point & p, double offset) const -> bool
{
  return isInBox(world_model_->getTheirPenaltyArea(), p, offset);
}

auto PointChecker::isFriendPenaltyArea(const Point & p, double offset) const -> bool
{
  return isInBox(world_model_->getOurPenaltyArea(), p, offset);
}

auto PointChecker::isPenaltyArea(const Point & p, double offset) const -> bool
{
  return isFriendPenaltyArea(p, offset) || isEnemyPenaltyArea(p, offset);
}

auto PointChecker::addEnemyPenaltyAreaInsideChecker(double offset) -> void
{
  checkers_.emplace_back([this, offset](const Point & p) { return isEnemyPenaltyArea(p, offset); });
}

auto PointChecker::addEnemyPenaltyAreaOutsideChecker(double offset) -> void
{
  checkers_.emplace_back(
    [this, offset](const Point & p) { return not isEnemyPenaltyArea(p, offset); });
}

auto PointChecker::addFriendPenaltyAreaInsideChecker(double offset) -> void
{
  checkers_.emplace_back(
    [this, offset](const Point & p) { return isFriendPenaltyArea(p, offset); });
}

auto PointChecker::addFriendPenaltyAreaOutsideChecker(double offset) -> void
{
  checkers_.emplace_back(
    [this, offset](const Point & p) { return not isFriendPenaltyArea(p, offset); });
}

auto PointChecker::addPenaltyAreaInsideChecker(double offset) -> void
{
  checkers_.emplace_back([this, offset](const Point & p) { return isPenaltyArea(p, offset); });
}

auto PointChecker::addPenaltyAreaOutsideChecker(double offset) -> void
{
  checkers_.emplace_back([this, offset](const Point & p) { return not isPenaltyArea(p, offset); });
}

auto PointChecker::isFieldInsideAndOutsidePenaltyArea(
  const Point & p, double field_offset, double penalty_offset) const -> bool
{
  return isFieldInside(p, field_offset) && !isPenaltyArea(p, penalty_offset);
}

auto PointChecker::isInOurHalf(const Point & p, double offset) const -> bool
{
  return p.x() * world_model_->getOurSideSign() > offset;
}

auto PointChecker::addInOurHalfChecker(double offset) -> void
{
  checkers_.emplace_back([this, offset](const Point & p) { return isInOurHalf(p, offset); });
}

auto PointChecker::checkDistance(
  const Point & p, const Point & target, double threshold, Rule rule) const -> bool
{
  double distance = (p - target).norm();
  switch (rule) {
    case Rule::EQUAL_TO:
      return std::abs(distance - threshold) < 1e-9;
    case Rule::NOT_EQUAL_TO:
      return std::abs(distance - threshold) >= 1e-9;
    case Rule::LESS_THAN:
      return distance < threshold;
    case Rule::GREATER_THAN:
      return distance > threshold;
    case Rule::LESS_THAN_OR_EQUAL_TO:
      return distance <= threshold;
    case Rule::GREATER_THAN_OR_EQUAL_TO:
      return distance >= threshold;
    default:
      return false;
  }
}

auto PointChecker::addDistanceChecker(const Point & target, double threshold, Rule rule) -> void
{
  checkers_.emplace_back([this, target, threshold, rule](const Point & p) {
    return checkDistance(p, target, threshold, rule);
  });
}

auto PointChecker::checkDistanceFromBall(const Point & p, double threshold, Rule rule) const -> bool
{
  return checkDistance(p, world_model_->ball().pos, threshold, rule);
}

auto PointChecker::addDistanceFromBallChecker(double threshold, Rule rule) -> void
{
  checkers_.emplace_back(
    [this, threshold, rule](const Point & p) { return checkDistanceFromBall(p, threshold, rule); });
}

auto PointChecker::checkDistanceFromRobot(
  const Point & p, RobotIdentifier id, double threshold, Rule rule) const -> bool
{
  return checkDistance(p, world_model_->getRobot(id)->pose.pos, threshold, rule);
}

auto PointChecker::addDistanceFromRobotChecker(RobotIdentifier id, double threshold, Rule rule)
  -> void
{
  checkers_.emplace_back([this, id, threshold, rule](const Point & p) {
    return checkDistanceFromRobot(p, id, threshold, rule);
  });
}

auto PointChecker::checkDistanceFromRobot(
  const Point & p, std::shared_ptr<RobotInfo> robot, double threshold, Rule rule) const -> bool
{
  return checkDistance(p, robot->pose.pos, threshold, rule);
}

auto PointChecker::checkDistanceFromRobots(
  const Point & p, const RobotList & robots, double threshold, Rule rule) const -> bool
{
  for (const auto & robot : robots) {
    if (not checkDistance(p, robot->pose.pos, threshold, rule)) {
      return false;
    }
  }
  return true;
}

auto PointChecker::addDistanceFromRobotsChecker(
  const RobotList & robots, double threshold, Rule rule) -> void
{
  checkers_.emplace_back([this, robots, threshold, rule](const Point & p) {
    return checkDistanceFromRobots(p, robots, threshold, rule);
  });
}

auto PointChecker::checkDistanceFromOurRobots(const Point & p, double threshold, Rule rule) const
  -> bool
{
  return checkDistanceFromRobots(
    p, world_model_->ours().robotsWhere().available().get(), threshold, rule);
}

auto PointChecker::addDistanceFromOurRobotsChecker(double threshold, Rule rule) -> void
{
  checkers_.emplace_back([this, threshold, rule](const Point & p) {
    return checkDistanceFromOurRobots(p, threshold, rule);
  });
}

auto PointChecker::checkDistanceFromTheirRobots(const Point & p, double threshold, Rule rule) const
  -> bool
{
  return checkDistanceFromRobots(
    p, world_model_->theirs().robotsWhere().available().get(), threshold, rule);
}

auto PointChecker::addDistanceFromTheirRobotsChecker(double threshold, Rule rule) -> void
{
  checkers_.emplace_back([this, threshold, rule](const Point & p) {
    return checkDistanceFromTheirRobots(p, threshold, rule);
  });
}

auto PointChecker::addCustomChecker(std::function<bool(const Point &)> checker) -> void
{
  checkers_.emplace_back(checker);
}

auto PointChecker::operator()(const Point & p) const -> bool
{
  return std::ranges::all_of(checkers_, [p](auto & check) { return check(p); });
}

auto PointChecker::buildStandard(std::shared_ptr<WorldModelWrapper> world_model) -> PointChecker
{
  PointChecker checker(world_model);
  checker.addFieldInsideChecker();
  checker.addPenaltyAreaOutsideChecker();
  checker.addBallPlacementAreaOutsideChecker();
  return checker;
}

}  // namespace crane
