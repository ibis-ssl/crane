// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_BASICS__DIAGNOSED_PUBLISHER_HPP_
#define CRANE_BASICS__DIAGNOSED_PUBLISHER_HPP_

#include <boost/asio.hpp>
#include <diagnostic_updater/publisher.hpp>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace crane
{
template <typename MessageT>
class DiagnosedPublisher
{
public:
  template <typename NodePointerT>
  DiagnosedPublisher(
    const NodePointerT node, const std::string & topic_name, const size_t qos_history_depth,
    double min_update_frequency, double max_update_frequency,
    const diagnostic_updater::TimeStampStatusParam & stamp =
      diagnostic_updater::TimeStampStatusParam())
  : frequency_status_param{min_update_frequency, max_update_frequency},
    publisher(node->template create_publisher<MessageT>(topic_name, qos_history_depth)),
    diagnostics_updater(node),
    clock(node->get_clock()),
    topic_diagnostic(
      publisher->get_topic_name(), diagnostics_updater,
      diagnostic_updater::FrequencyStatusParam(
        &frequency_status_param.min_update_frequency, &frequency_status_param.max_update_frequency),
      stamp, clock)
  {
    diagnostics_updater.setHardwareID(topic_name);
  }

  struct FrequencyStatusParam
  {
    double min_update_frequency;
    double max_update_frequency;
  } frequency_status_param;

  void publish(typename MessageT::UniquePtr message)
  {
    topic_diagnostic.tick(clock->now());
    publisher->publish(std::move(message));
  }

  void publish(const MessageT & message)
  {
    topic_diagnostic.tick(clock->now());
    publisher->publish(message);
  }

private:
  typename rclcpp::Publisher<MessageT>::SharedPtr publisher;

  diagnostic_updater::Updater diagnostics_updater;

  rclcpp::Clock::SharedPtr clock;

  diagnostic_updater::TopicDiagnostic topic_diagnostic;
};

}  // namespace crane

#endif  // CRANE_BASICS__DIAGNOSED_PUBLISHER_HPP_
