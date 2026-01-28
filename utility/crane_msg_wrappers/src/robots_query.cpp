// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_msg_wrappers/robots_query.hpp"

#include <algorithm>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/filter.hpp>
#include <range/v3/view/transform.hpp>

namespace crane
{

RobotsQuery::RobotsQuery(const RobotList & robots, uint8_t goalie_id)
: robots_(robots), goalie_id_(goalie_id)
{
}

auto RobotsQuery::available() -> RobotsQuery &
{
  predicates_.emplace_back([](const RobotInfo::SharedPtr & robot) { return robot->available(); });
  return *this;
}

auto RobotsQuery::availableStrict() -> RobotsQuery &
{
  predicates_.emplace_back(
    [](const RobotInfo::SharedPtr & robot) { return robot->availableStrict(); });
  return *this;
}

auto RobotsQuery::availableLoose() -> RobotsQuery &
{
  predicates_.emplace_back(
    [](const RobotInfo::SharedPtr & robot) { return robot->availableLoose(); });
  return *this;
}

auto RobotsQuery::excludeId(uint8_t id) -> RobotsQuery &
{
  predicates_.emplace_back([id](const RobotInfo::SharedPtr & robot) { return robot->id != id; });
  return *this;
}

auto RobotsQuery::excludeIds(const std::vector<uint8_t> & ids) -> RobotsQuery &
{
  predicates_.emplace_back([ids](const RobotInfo::SharedPtr & robot) {
    return std::find(ids.begin(), ids.end(), robot->id) == ids.end();
  });
  return *this;
}

auto RobotsQuery::excludeGoalie() -> RobotsQuery &
{
  predicates_.emplace_back([goalie_id = goalie_id_](const RobotInfo::SharedPtr & robot) {
    return robot->id != goalie_id;
  });
  return *this;
}

auto RobotsQuery::where(Predicate pred) -> RobotsQuery &
{
  predicates_.emplace_back(pred);
  return *this;
}

auto RobotsQuery::get() const -> RobotList
{
  return robots_ | ranges::views::filter([this](const RobotInfo::SharedPtr & robot) {
           return std::all_of(
             predicates_.begin(), predicates_.end(),
             [&robot](const Predicate & pred) { return pred(robot); });
         }) |
         ranges::to<std::vector>();
}

auto RobotsQuery::getIds() const -> std::vector<uint8_t>
{
  auto robots = get();
  return robots |
         ranges::views::transform([](const RobotInfo::SharedPtr & robot) { return robot->id; }) |
         ranges::to<std::vector>();
}

auto RobotsQuery::getView() const -> decltype(auto)
{
  return robots_ | ranges::views::filter([this](const RobotInfo::SharedPtr & robot) {
           return std::all_of(
             predicates_.begin(), predicates_.end(),
             [&robot](const Predicate & pred) { return pred(robot); });
         });
}

auto RobotsQuery::count() const -> size_t { return get().size(); }

auto RobotsQuery::empty() const -> bool { return count() == 0; }

}  // namespace crane
