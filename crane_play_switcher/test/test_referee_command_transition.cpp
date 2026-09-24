// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <crane_play_switcher/referee_command_transition.hpp>
#include <optional>

using crane::decideCommandTransition;
using crane_msgs::msg::PlaySituation;
using robocup_ssl_msgs::msg::Referee;
using robocup_ssl_msgs::msg::RefereeCommand;

namespace
{
// 自チームは黄色として判定する
constexpr bool IS_YELLOW = true;

Referee makeReferee(int command, uint32_t counter, std::optional<int> next_command = std::nullopt)
{
  Referee msg;
  msg.command.value = command;
  msg.command_counter = counter;
  if (next_command) {
    msg.has_field |= Referee::NEXT_COMMAND_FIELD_SET;
    msg.next_command.value = *next_command;
  }
  return msg;
}

// タイムアウト（STOP へ強制遷移）から msg で復帰したときの遷移先
int recoverFromTimeout(int state_before_timeout, const Referee & latest_raw, const Referee & msg)
{
  const auto transition =
    decideCommandTransition(PlaySituation::STOP, latest_raw, msg, IS_YELLOW, state_before_timeout);
  EXPECT_TRUE(transition.has_value());
  return transition ? transition->next_play_situation : -1;
}
}  // namespace

// 途切れている間に新しいコマンドが出ていなければ（同一 RAW・同一 counter）、
// タイムアウト前の状態に戻る
TEST(RefereeCommandTransition, SameRawRecoveryRestoresStateBeforeTimeout)
{
  struct Case
  {
    int raw;
    std::optional<int> next_command;
    int state_before_timeout;
  };
  const Case cases[] = {
    {RefereeCommand::PREPARE_KICKOFF_YELLOW, std::nullopt, PlaySituation::OUR_KICKOFF_PREPARATION},
    {RefereeCommand::PREPARE_PENALTY_BLUE, std::nullopt, PlaySituation::THEIR_PENALTY_PREPARATION},
    {RefereeCommand::NORMAL_START, std::nullopt, PlaySituation::OUR_KICKOFF_START},
    {RefereeCommand::NORMAL_START, std::nullopt, PlaySituation::INPLAY},
    {RefereeCommand::FORCE_START, std::nullopt, PlaySituation::INPLAY},
    {RefereeCommand::STOP, RefereeCommand::DIRECT_FREE_YELLOW,
     PlaySituation::STOP_PRE_OUR_DIRECT_FREE},
  };
  for (const auto & c : cases) {
    const auto raw = makeReferee(c.raw, 7, c.next_command);
    const auto transition =
      decideCommandTransition(PlaySituation::STOP, raw, raw, IS_YELLOW, c.state_before_timeout);
    ASSERT_TRUE(transition.has_value()) << "raw=" << c.raw;
    EXPECT_EQ(transition->next_play_situation, c.state_before_timeout) << "raw=" << c.raw;
    EXPECT_TRUE(transition->restored_before_timeout) << "raw=" << c.raw;
  }
}

// PREPARE 中に途切れ、その間に NORMAL_START が出ていたら、
// タイムアウト前の PREPARATION を起点に START へ進む
TEST(RefereeCommandTransition, NormalStartRecoveryAfterPrepareStartsPlay)
{
  EXPECT_EQ(
    recoverFromTimeout(
      PlaySituation::OUR_KICKOFF_PREPARATION,
      makeReferee(RefereeCommand::PREPARE_KICKOFF_YELLOW, 7),
      makeReferee(RefereeCommand::NORMAL_START, 8)),
    PlaySituation::OUR_KICKOFF_START);
  EXPECT_EQ(
    recoverFromTimeout(
      PlaySituation::THEIR_PENALTY_PREPARATION,
      makeReferee(RefereeCommand::PREPARE_PENALTY_BLUE, 7),
      makeReferee(RefereeCommand::NORMAL_START, 8)),
    PlaySituation::THEIR_PENALTY_START);
}

// 同じ RAW でも counter が進んでいれば新しいコマンドとして判定し直す
TEST(RefereeCommandTransition, SameRawWithNewCounterIsReevaluated)
{
  const auto transition = decideCommandTransition(
    PlaySituation::STOP, makeReferee(RefereeCommand::DIRECT_FREE_YELLOW, 7),
    makeReferee(RefereeCommand::DIRECT_FREE_YELLOW, 10), IS_YELLOW, PlaySituation::INPLAY);
  ASSERT_TRUE(transition.has_value());
  EXPECT_EQ(transition->next_play_situation, PlaySituation::OUR_DIRECT_FREE);
  EXPECT_FALSE(transition->restored_before_timeout);
}

// 途切れている間に出た STOP・HALT で復帰したら、そのコマンドに従う
TEST(RefereeCommandTransition, StopAndHaltRecoveryFollowNewCommand)
{
  const auto inplay_raw = makeReferee(RefereeCommand::FORCE_START, 7);
  EXPECT_EQ(
    recoverFromTimeout(PlaySituation::INPLAY, inplay_raw, makeReferee(RefereeCommand::HALT, 8)),
    PlaySituation::HALT);
  EXPECT_EQ(
    recoverFromTimeout(
      PlaySituation::INPLAY, inplay_raw,
      makeReferee(RefereeCommand::STOP, 8, RefereeCommand::PREPARE_KICKOFF_BLUE)),
    PlaySituation::STOP_PRE_THEIR_KICKOFF_PREPARATION);
}

// タイムアウトしていなければ、RAW が変わったときだけ遷移する（従来どおり）
TEST(RefereeCommandTransition, WithoutTimeoutOnlyRawChangeTransitions)
{
  const auto prepare = makeReferee(RefereeCommand::PREPARE_KICKOFF_YELLOW, 7);
  EXPECT_FALSE(decideCommandTransition(
                 PlaySituation::OUR_KICKOFF_PREPARATION, prepare, prepare, IS_YELLOW, std::nullopt)
                 .has_value());

  const auto transition = decideCommandTransition(
    PlaySituation::OUR_KICKOFF_PREPARATION, prepare, makeReferee(RefereeCommand::NORMAL_START, 8),
    IS_YELLOW, std::nullopt);
  ASSERT_TRUE(transition.has_value());
  EXPECT_EQ(transition->next_play_situation, PlaySituation::OUR_KICKOFF_START);
}
