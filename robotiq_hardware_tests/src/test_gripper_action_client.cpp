// Copyright (c) 2022 PickNik, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <chrono>
#include <memory>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "robotiq_hardware_tests/action/gripper_control.hpp"

using GripperControlAction = robotiq_hardware_tests::action::GripperControl;
using GoalHandleGripperControl = rclcpp_action::ClientGoalHandle<GripperControlAction>;

class GripperActionClient : public rclcpp::Node
{
public:
  GripperActionClient() : Node("gripper_action_client")
  {
    // Create action client
    action_client_ = rclcpp_action::create_client<GripperControlAction>(
        this, "hande_gripper_control");
  }

  void send_goal(double position)
  {
    using namespace std::placeholders;

    if (!action_client_->wait_for_action_server(std::chrono::seconds(10)))
    {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      return;
    }

    auto goal_msg = GripperControlAction::Goal();
    goal_msg.position = position;

    RCLCPP_INFO(this->get_logger(), "Sending goal to move gripper to %.4f m", position);

    auto send_goal_options = rclcpp_action::Client<GripperControlAction>::SendGoalOptions();
    
    send_goal_options.goal_response_callback =
        std::bind(&GripperActionClient::goal_response_callback, this, _1);
    
    send_goal_options.feedback_callback =
        std::bind(&GripperActionClient::feedback_callback, this, _1, _2);
    
    send_goal_options.result_callback =
        std::bind(&GripperActionClient::result_callback, this, _1);

    action_client_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  void goal_response_callback(const GoalHandleGripperControl::SharedPtr & goal_handle)
  {
    if (!goal_handle)
    {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
      GoalHandleGripperControl::SharedPtr,
      const std::shared_ptr<const GripperControlAction::Feedback> feedback)
  {
    RCLCPP_INFO(this->get_logger(), "Feedback: %s (%.4f m)", 
                feedback->status.c_str(), feedback->current_position);
  }

  void result_callback(const GoalHandleGripperControl::WrappedResult & result)
  {
    switch (result.code)
    {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Goal succeeded! %s", result.result->message.c_str());
        RCLCPP_INFO(this->get_logger(), "Final position: %.4f m", result.result->final_position);
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted: %s", result.result->message.c_str());
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(this->get_logger(), "Goal was canceled: %s", result.result->message.c_str());
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        break;
    }
    
    // Exit after receiving result
    rclcpp::shutdown();
  }

  rclcpp_action::Client<GripperControlAction>::SharedPtr action_client_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  if (argc != 2)
  {
    std::cout << "Usage: test_gripper_action_client <position_in_meters>" << std::endl;
    std::cout << "Position range: 0.0 (fully closed) to 0.025 (fully open)" << std::endl;
    return 1;
  }

  double position = std::stod(argv[1]);
  
  if (position < 0.0 || position > 0.025)
  {
    std::cout << "Position out of range! Must be between 0.0 and 0.025 meters" << std::endl;
    return 1;
  }

  auto action_client = std::make_shared<GripperActionClient>();
  action_client->send_goal(position);

  rclcpp::spin(action_client);
  
  return 0;
} 