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
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robotiq_driver/default_driver.hpp>
#include <robotiq_driver/default_serial.hpp>
#include "robotiq_hardware_tests/action/gripper_control.hpp"

constexpr auto kComPort = "/tmp/ttyUR";
constexpr auto kBaudRate = 115200;
constexpr auto kTimeout = 1;
constexpr auto kSlaveAddress = 0x09;

// Position conversion constants
constexpr double MIN_POSITION_M = 0.0;      // Minimum position in meters
constexpr double MAX_POSITION_M = 0.025;    // Maximum position in meters  
constexpr uint8_t MIN_GRIPPER_POS = 0x00;   // Fully open gripper position
constexpr uint8_t MAX_GRIPPER_POS = 0xFF;   // Fully closed gripper position
constexpr double GOAL_TOLERANCE_M = 0.001;  // Goal tolerance in meters

using robotiq_driver::DefaultDriver;
using robotiq_driver::DefaultSerial;
using GripperControlAction = robotiq_hardware_tests::action::GripperControl;
using GoalHandleGripperControl = rclcpp_action::ServerGoalHandle<GripperControlAction>;

class HandeGripperControlNode : public rclcpp::Node
{
public:
  HandeGripperControlNode() : Node("hande_gripper_control_node")
  {
    // Initialize gripper driver
    if (!initialize_gripper())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize gripper");
      return;
    }

    // Create action server
    action_server_ = rclcpp_action::create_server<GripperControlAction>(
        this,
        "hande_gripper_control",
        std::bind(&HandeGripperControlNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&HandeGripperControlNode::handle_cancel, this, std::placeholders::_1),
        std::bind(&HandeGripperControlNode::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Hande gripper control action server ready");
  }

private:
  bool initialize_gripper()
  {
    try
    {
      auto serial = std::make_unique<DefaultSerial>();
      serial->set_port(kComPort);
      serial->set_baudrate(kBaudRate);
      serial->set_timeout(std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::duration<double>(kTimeout)));

      driver_ = std::make_unique<DefaultDriver>(std::move(serial));
      driver_->set_slave_address(kSlaveAddress);

      RCLCPP_INFO(this->get_logger(), "Connecting to gripper at %s...", kComPort);
      
      const bool connected = driver_->connect();
      if (!connected)
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to connect to gripper");
        return false;
      }

      RCLCPP_INFO(this->get_logger(), "Gripper connected. Activating...");
      
      // Deactivate first to ensure clean state
      driver_->deactivate();
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      
      // Activate gripper
      driver_->activate();
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));

      RCLCPP_INFO(this->get_logger(), "Gripper activated and ready");
      return true;
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Exception during gripper initialization: %s", e.what());
      return false;
    }
  }

  uint8_t position_to_gripper_value(double position_m)
  {
    // Clamp position to valid range
    position_m = std::max(MIN_POSITION_M, std::min(MAX_POSITION_M, position_m));
    
    // Convert from meters to gripper position (inverted scale)
    // 0.0m -> 0xFF (closed), 0.025m -> 0x00 (open)
    double normalized = (position_m - MIN_POSITION_M) / (MAX_POSITION_M - MIN_POSITION_M);
    uint8_t gripper_pos = static_cast<uint8_t>((1.0 - normalized) * (MAX_GRIPPER_POS - MIN_GRIPPER_POS) + MIN_GRIPPER_POS);
    
    return gripper_pos;
  }

  double gripper_value_to_position(uint8_t gripper_pos)
  {
    // Convert from gripper position to meters (inverted scale)
    // 0xFF -> 0.0m (closed), 0x00 -> 0.025m (open)
    double normalized = 1.0 - (static_cast<double>(gripper_pos - MIN_GRIPPER_POS) / (MAX_GRIPPER_POS - MIN_GRIPPER_POS));
    double position_m = normalized * (MAX_POSITION_M - MIN_POSITION_M) + MIN_POSITION_M;
    
    return position_m;
  }

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const GripperControlAction::Goal> goal)
  {
    (void)uuid;
    
    // Validate position range
    if (goal->position < MIN_POSITION_M || goal->position > MAX_POSITION_M)
    {
      RCLCPP_WARN(this->get_logger(), "Position out of range. Must be between %.4f and %.4f meters", 
                  MIN_POSITION_M, MAX_POSITION_M);
      return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(this->get_logger(), "Accepting goal to move gripper to %.4f m", goal->position);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleGripperControl> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleGripperControl> goal_handle)
  {
    // Execute in a separate thread to not block the action server
    std::thread{std::bind(&HandeGripperControlNode::execute, this, goal_handle)}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleGripperControl> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<GripperControlAction::Feedback>();
    auto result = std::make_shared<GripperControlAction::Result>();

    try
    {
      // Convert position to gripper value
      uint8_t target_gripper_pos = position_to_gripper_value(goal->position);
      
      RCLCPP_INFO(this->get_logger(), "Moving gripper to position %.4f m (0x%02X)", 
                  goal->position, target_gripper_pos);

      // Send initial command to gripper
      driver_->set_speed(0x0F);
      driver_->set_force(0x0A);
      driver_->set_gripper_position(target_gripper_pos);

      feedback->status = "Moving to target position";
      feedback->current_position = gripper_value_to_position(driver_->get_gripper_position());
      goal_handle->publish_feedback(feedback);

      // Monitor gripper movement with while loop as in gripper_interface_test
      uint8_t current_gripper_pos = driver_->get_gripper_position();
      double current_position_m = gripper_value_to_position(current_gripper_pos);
      
      while (!driver_->gripper_detected_while_closing() && 
             std::abs(current_position_m - goal->position) > GOAL_TOLERANCE_M)
      {
        // Check if goal was cancelled
        if (goal_handle->is_canceling())
        {
          result->success = false;
          result->message = "Goal was cancelled";
          result->final_position = gripper_value_to_position(current_gripper_pos);
          goal_handle->canceled(result);
          RCLCPP_INFO(this->get_logger(), "Goal cancelled");
          return;
        }

        current_gripper_pos = driver_->get_gripper_position();
        current_position_m = gripper_value_to_position(current_gripper_pos);
        
        // Publish feedback
        feedback->current_position = current_position_m;
        feedback->status = "Moving... at position " + std::to_string(feedback->current_position) + " m";
        goal_handle->publish_feedback(feedback);
        
        RCLCPP_DEBUG(this->get_logger(), "Gripper moving... @%d (%.4f m), error: %.4f m", 
                     static_cast<int>(current_gripper_pos), current_position_m,
                     std::abs(current_position_m - goal->position));
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }

      // Check if object was detected or target reached
      if (driver_->gripper_detected_while_closing())
      {
        feedback->status = "Object detected, confirming grip";
        goal_handle->publish_feedback(feedback);
        
        RCLCPP_INFO(this->get_logger(), "Object detected at position %d (%.4f m)", 
                    static_cast<int>(current_gripper_pos), 
                    gripper_value_to_position(current_gripper_pos));
      }
      else
      {
        feedback->status = "Target position reached";
        goal_handle->publish_feedback(feedback);
        
        RCLCPP_INFO(this->get_logger(), "Target position reached at %d (%.4f m)", 
                    static_cast<int>(current_gripper_pos), 
                    gripper_value_to_position(current_gripper_pos));
      }

      // Send gripper command again at the detected/final position as requested
      driver_->set_gripper_position(current_gripper_pos);
      
      feedback->status = "Finalizing position";
      goal_handle->publish_feedback(feedback);
      
      // Wait a bit for the command to be processed
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      
      // Get final position
      uint8_t final_gripper_pos = driver_->get_gripper_position();
      double final_position_m = gripper_value_to_position(final_gripper_pos);

      result->success = true;
      result->final_position = final_position_m;
      
      if (driver_->gripper_detected_while_closing())
      {
        result->message = "Object successfully grasped at position " + std::to_string(final_position_m) + " m";
      }
      else
      {
        result->message = "Gripper moved to target position " + std::to_string(final_position_m) + " m";
      }

      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Action completed: %s", result->message.c_str());
    }
    catch (const std::exception& e)
    {
      result->success = false;
      result->message = "Failed to control gripper: " + std::string(e.what());
      result->final_position = gripper_value_to_position(driver_->get_gripper_position());
      
      goal_handle->abort(result);
      RCLCPP_ERROR(this->get_logger(), "Action failed: %s", result->message.c_str());
    }
  }

  std::unique_ptr<DefaultDriver> driver_;
  rclcpp_action::Server<GripperControlAction>::SharedPtr action_server_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HandeGripperControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
} 