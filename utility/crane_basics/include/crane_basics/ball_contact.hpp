// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_BASICS__BALL_CONTACT_HPP_
#define CRANE_BASICS__BALL_CONTACT_HPP_

#include <chrono>

namespace crane
{
struct BallContact
{
  std::chrono::system_clock::time_point last_contact_end_time;
  std::chrono::system_clock::time_point last_contact_start_time;

  void update(bool is_contacted);

  [[nodiscard]] auto getContactDuration() const
  {
    return (last_contact_end_time - last_contact_start_time);
  }

  [[nodiscard]] auto findPastContact(double duration_sec) const
  {
    auto past = std::chrono::system_clock::now() - std::chrono::duration<double>(duration_sec);
    return past < last_contact_end_time;
  }

private:
  bool is_contacted_pre_frame = false;
};
}  // namespace crane

#endif  // CRANE_BASICS__BALL_CONTACT_HPP_
