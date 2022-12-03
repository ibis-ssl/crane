// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_MSG_WRAPPERS__MESSAGE_WRAPPER_HPP_
#define CRANE_MSG_WRAPPERS__MESSAGE_WRAPPER_HPP_

namespace crane
{
template <typename TMsg>
class MessageWrapper
{
public:
  virtual void update(const TMsg & msg) = 0;
};
}  // namespace crane

#endif  // CRANE_MSG_WRAPPERS__MESSAGE_WRAPPER_HPP_
