// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_robot_skills/sleep.hpp>

namespace crane::skills
{
Status Sleep::update()
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

double Sleep::getRestTime() const
{
  auto elapsed_time = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time);
  return getParameter<double>("duration") - elapsed_time.count();
}
}  // namespace crane::skills
