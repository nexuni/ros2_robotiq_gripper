// Copyright (c) 2022 PickNik, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions of binary form must reproduce the above copyright
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
#include <robotiq_hardware_tests/srv/gripper_control.hpp>

using namespace std::chrono_literals;

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  if (argc != 2)
  {
    std::cout << "Usage: test_gripper_client <position_in_meters>" << std::endl;
    std::cout << "Position should be between 0.0 (closed) and 0.025 (open) meters" << std::endl;
    return 1;
  }

  double position = std::atof(argv[1]);
  
  auto node = rclcpp::Node::make_shared("gripper_test_client");
  auto client = node->create_client<robotiq_hardware_tests::srv::GripperControl>("hande_gripper_control");

  // Wait for service to be available
  std::cout << "Waiting for service to be available..." << std::endl;
  while (!client->wait_for_service(1s))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for service");
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Service not available, waiting...");
  }

  // Create request
  auto request = std::make_shared<robotiq_hardware_tests::srv::GripperControl::Request>();
  request->position = position;

  std::cout << "Sending gripper position request: " << position << " meters" << std::endl;

  // Send request
  auto result = client->async_send_request(request);

  // Wait for response
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = result.get();
    std::cout << "Service response:" << std::endl;
    std::cout << "  Success: " << (response->success ? "true" : "false") << std::endl;
    std::cout << "  Message: " << response->message << std::endl;
    
    if (response->success)
    {
      std::cout << "Gripper command sent successfully!" << std::endl;
      return 0;
    }
    else
    {
      std::cout << "Gripper command failed!" << std::endl;
      return 1;
    }
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to call service");
    return 1;
  }

  rclcpp::shutdown();
  return 0;
} 