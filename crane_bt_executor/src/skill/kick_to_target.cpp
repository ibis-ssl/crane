//
// Created by hans on 5/6/20.
//
#include "crane_bt_executor/skill/kick_to_target.hpp"
#include "crane_bt_executor/behavior_tree/status_converter/always.hpp"
#include "crane_bt_executor/behavior_tree/parallel_one.hpp"
//#include "crane_bt_executor/robot_io.hpp"
#include "crane_bt_executor/utils/tool.hpp"
#include "crane_bt_executor/utils/target.hpp"
#include "crane_bt_executor/skill/spin_at_target.hpp"
#include "crane_bt_executor/skill/move.hpp"
#include "crane_bt_executor/skill/kick.hpp"
#include "crane_bt_executor/skill/face.hpp"
//#include "crane_bt_executor/skill/stop.hpp"

KickToTarget::KickToTarget(TargetModule target, float power)
: target_(target), kick_power_(power)
{
  auto go_ball = std::make_shared<ParallelOne>();
  go_ball->addChild(std::make_shared<Move>(TargetModule::buildBall(), 0.5f));
  auto always_running1 = std::make_shared<AlwaysRunning>(std::make_shared<Face>(target));
  go_ball->addChild(always_running1);
  addChild(go_ball);
//    auto over_target = TargetModule(std::make_shared<TargetOperation<std::plus<Point>>>(TargetModule::buildBall(),dir));
  auto pivot_turn = std::make_shared<ParallelOne>();
  auto dir = TargetModule(std::make_shared<TargetOperation<std::minus<Point>>>(TargetModule::buildBall(), target));
  pivot_turn->addChild(std::make_shared<SpinAtTarget>(TargetModule::buildBall(), TargetModule(std::make_shared<TargetOperation<std::plus<Point>>>(TargetModule::buildBall(),dir)),0.02f));
  auto always_running2 = std::make_shared<AlwaysRunning>(std::make_shared<Face>(target_));
  pivot_turn->addChild(always_running2);

  addChild(pivot_turn);
//    addChild(std::make_shared<Stop>());
//    addChild(std::make_shared<Face>(TargetModule::buildBall()));
  addChild(std::make_shared<Kick>());
  addChild(std::make_shared<Move>(TargetModule::buildBall(),0.1f));
}