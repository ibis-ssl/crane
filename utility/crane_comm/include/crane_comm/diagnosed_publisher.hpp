// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_COMM__DIAGNOSED_PUBLISHER_HPP_
#define CRANE_COMM__DIAGNOSED_PUBLISHER_HPP_

#include <diagnostic_updater/publisher.hpp>
#include <string>

namespace crane
{
template <typename MessageT>
class DiagnosedPublisher
{
public:
  template <typename NodePointerT>
  DiagnosedPublisher(
    const NodePointerT node, const std::string & topic_name, const size_t qos_history_depth,
    double min_update_frequency, double max_update_frequency)
  : frequency_status_param{min_update_frequency, max_update_frequency},
    publisher(node->template create_publisher<MessageT>(topic_name, qos_history_depth)),
    diagnostics_updater(node),
    clock(node->get_clock()),
    topic_diagnostic(
      publisher->get_topic_name(), diagnostics_updater,
      diagnostic_updater::FrequencyStatusParam(
        &frequency_status_param.min_update_frequency, &frequency_status_param.max_update_frequency),
      diagnostic_updater::TimeStampStatusParam(), clock)
  {
    diagnostics_updater.setHardwareID(topic_name);
  }

  auto publish(const MessageT & message) -> void
  {
    topic_diagnostic.tick(clock->now());
    publisher->publish(message);
  }

private:
  // TopicDiagnostic はこの 2 値へのポインタを持つので、topic_diagnostic より先に宣言する
  struct FrequencyStatusParam
  {
    double min_update_frequency;
    double max_update_frequency;
  } frequency_status_param;

  typename rclcpp::Publisher<MessageT>::SharedPtr publisher;

  diagnostic_updater::Updater diagnostics_updater;

  rclcpp::Clock::SharedPtr clock;

  diagnostic_updater::TopicDiagnostic topic_diagnostic;
};

}  // namespace crane

#endif  // CRANE_COMM__DIAGNOSED_PUBLISHER_HPP_
