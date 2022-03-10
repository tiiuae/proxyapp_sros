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


#include <chrono>
#include <memory>
#include <string>
#include <random>

#include "rclcpp/rclcpp.hpp"

#include <drone_srv_server_client/dummy_service_server.hpp>


using namespace std::chrono_literals;

DummyServiceServer::DummyServiceServer(
  std::string drone_id, std::string drone_namespace)
: Node("drone_dummy_service"),
  drone_id_(drone_id),
  drone_namespace_(drone_namespace),
  count_(0)
{
  gps_waypoint_service_ =
    this->create_service<fog_msgs::srv::Vec4>(
    "/" + drone_namespace_ + "/navigation/gps_waypoint",
    std::bind(&DummyServiceServer::gpsWaypointCallback, this, _1, _2));
}

void DummyServiceServer::gpsWaypointCallback(
  [[ maybe_unused ]]const std::shared_ptr<fog_msgs::srv::Vec4::Request> request,
  std::shared_ptr<fog_msgs::srv::Vec4::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Received gps waypoint");
    response->success = true;

}
