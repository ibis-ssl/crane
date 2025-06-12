// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

export module crane_basics:ball_contact;

// <chrono> is removed, assumed from crane_basics.cppm global fragment.

export namespace crane
{
export struct BallContact
{
  std::chrono::system_clock::time_point last_contact_end_time;
  std::chrono::system_clock::time_point last_contact_start_time;

  auto update(bool is_contacted) -> void; // Definition expected elsewhere

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
