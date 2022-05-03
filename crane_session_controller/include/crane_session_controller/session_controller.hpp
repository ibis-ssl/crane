// Copyright (c) 2022 ibis-ssl
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef CRANE_SESSION_CONTROLLER__SESSION_CONTROLLER_HPP_
#define CRANE_SESSION_CONTROLLER__SESSION_CONTROLLER_HPP_

#include <chrono>
#include <deque>
#include <memory>
#include <optional>

#include "crane_msg_wrappers/world_model_wrapper.hpp"
#include "crane_msgs/msg/world_model.hpp"
#include "crane_msgs/srv/robot_select.hpp"
#include "crane_session_controller/visibility_control.h"
#include "rclcpp/rclcpp.hpp"

namespace crane {
    class SessionModule {
    public:
        using SharedPtr = std::shared_ptr<SessionModule>;
        SessionModule(std::string name, rclcpp::Node::SharedPtr node) : name(name), node(node) {}

        void construct() {
            client = node->create_client<crane_msgs::srv::RobotSelect>("robot_select");
            using namespace std::chrono_literals;
            while (!client->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(
                            rclcpp::get_logger(name.c_str()), "Interrupted while waiting for the service. Exiting.");
                    break;
                }
                RCLCPP_INFO(rclcpp::get_logger(name.c_str()), "service not available, waiting again...");
            }
        }

        std::optional<crane_msgs::srv::RobotSelect::Response> sendRequest(
                crane_msgs::srv::RobotSelect::Request::SharedPtr request) {
            auto result = client->async_send_request(request);
            // Wait for the result.
            if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
                return std::optional<crane_msgs::srv::RobotSelect::Response>(*result.get());
            } else {
                return std::nullopt;
            }
        }

    private:
        rclcpp::Client<crane_msgs::srv::RobotSelect>::SharedPtr client;
        const std::string name;
        const rclcpp::Node::SharedPtr node;
    };

    class SessionContoller : public rclcpp::Node {
    public:
        COMPOSITION_PUBLIC
        explicit SessionContoller(const rclcpp::NodeOptions &options)
                : rclcpp::Node("session_controller", options) {

            // example of adding planner
            session_planners_["replace"]  = std::make_shared<SessionModule>("replace", shared_from_this());
            for(auto &planner : session_planners_){
                planner.second->construct();
            }

            // example for ball replacement
            // TODO : load from config file
            // TODO:
            auto replace_map = std::make_shared<std::vector<std::pair<std::string, int>>>();
            replace_map->emplace_back("goalie",1);
            replace_map->emplace_back("replace",2);
            replace_map->emplace_back("waiter",100);
            robot_selection_priority_map["ball_replacement"] = replace_map;

            using namespace std::chrono_literals;
            timer_ = create_wall_timer(1s, std::bind(&SessionContoller::timerCallback, this));
        }

        void timerCallback() {}
        void reassignRequestCallback(){
            request("replace");
        }

        void request(std::string situation) {
            // TODO: reflect actual robot ids
            std::vector<int> available_robot_ids = {1,2,3,4};
            auto map = robot_selection_priority_map[situation];
            if(map){
                for(auto p : *map){
                    auto req = std::make_shared<crane_msgs::srv::RobotSelect::Request>();
                    req->robot_num = p.second;
                    for(auto id : available_robot_ids){
                        req->robot_ids.emplace_back(id);
                    }
                    session_planners_[p.first]->sendRequest(req);
                }
            }
        }

    private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<crane_msgs::msg::WorldModel>::SharedPtr sub_world_model_;
        std::shared_ptr<WorldModelWrapper> world_model_;
        std::deque<crane_msgs::srv::RobotSelect::Request> query_queue_;
        rclcpp::Client<crane_msgs::srv::RobotSelect>::SharedPtr robot_select_client_;
        std::unordered_map<std::string, SessionModule::SharedPtr> session_planners_;

        std::unordered_map<std::string, std::shared_ptr<std::vector<std::pair<std::string, int>>>> robot_selection_priority_map;

        void world_model_callback(const crane_msgs::msg::WorldModel::SharedPtr msg) {
            world_model_->update(*msg);
        }
    };

}  // namespace crane
#endif  // CRANE_SESSION_CONTROLLER__SESSION_CONTROLLER_HPP_
