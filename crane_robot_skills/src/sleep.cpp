// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_robot_skills/sleep.hpp>

namespace crane::skills
{
Sleep::Sleep(RobotCommandWrapperBase::SharedPtr & base)
: SkillBase("Sleep", base), is_started(getContextReference<bool>("is_started", false))
{
  setParameter("duration", 0.0);
}

Status Sleep::update([[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer)
{
  if (not is_started) {
    start_time = std::chrono::steady_clock::now();
    is_started = true;
  }

  auto elapsed_time = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time);
  if (elapsed_time.count() > getParameter<double>("duration")) {
    return Status::SUCCESS;
  } else {
    return Status::RUNNING;
  }
}
void Sleep::print(std::ostream & os) const { os << "[Sleep] 残り時間: " << getRestTime() << "秒"; }

double Sleep::getRestTime() const
{
  auto elapsed_time = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time);
  return getParameter<double>("duration") - elapsed_time.count();
}
}  // namespace crane::skills
