// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_msg_wrappers/point_checker.hpp"

#include <crane_msg_wrappers/world_model_wrapper.hpp>

namespace crane
{

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

auto PointChecker::isInOurHalf(const Point & p, double offset) const -> bool
{
  return p.x() * world_model_->getOurSideSign() > offset;
}

}  // namespace crane
