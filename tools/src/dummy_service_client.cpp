// Copyright 2021 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include <drone_srv_server_client/dummy_service_client.hpp>


using namespace std::chrono_literals;
DummyServiceClient::DummyServiceClient(
  std::string drone_id, std::string drone_namespace)
: Node("drone_subscriber"),
  drone_id_(drone_id),
  drone_namespace_(drone_namespace),
  count_(0)
{
  gps_waypoint_client_ = this->create_client<fog_msgs::srv::Vec4>(
    "/" + drone_namespace_ + "/gps_waypoint");

  timer_ = this->create_wall_timer(
    2s, std::bind(&DummyServiceClient::timer_callback_, this));
}

void DummyServiceClient::timer_callback_()
{
  if (!gps_waypoint_client_->service_is_ready()){
    RCLCPP_ERROR(this->get_logger(), "GPS waypoint service is not ready, waiting");
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Preparing request to gps waypoint service");
  auto req = std::make_shared<fog_msgs::srv::Vec4::Request>();
  std::vector<double> vec = {static_cast<double>(count_), 0.0, 1.0, 1.0};
  req->goal = vec;
  RCLCPP_INFO(this->get_logger(), "Sending request to gps waypoint service");
  gps_waypoint_client_->async_send_request(req,
    std::bind(&DummyServiceClient::gpsWaypointCallback, this, _1));
  count_++;
}
