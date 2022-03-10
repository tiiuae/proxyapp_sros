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

/**
 * @file DummyServiceServer.hpp
 *
 */

#ifndef _PUBSUB_APP_DUMMY_SERVICE_SERVER_HPP_
#define _PUBSUB_APP_DUMMY_SERVICE_SERVER_HPP_

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <fog_msgs/srv/vec4.hpp>
#include <fog_msgs/srv/path.hpp>
#include <nav_msgs/msg/path.hpp>

using namespace std::placeholders;
class DummyServiceServer : public rclcpp::Node
{
public:
  DummyServiceServer(
    std::string drone_id, std::string drone_namespace);

private:
  // service callbacks
  void gotoTriggerCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void hoverCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void localWaypointCallback(
    const std::shared_ptr<fog_msgs::srv::Vec4::Request> request,
    std::shared_ptr<fog_msgs::srv::Vec4::Response> response);
  void localPathCallback(
    const std::shared_ptr<fog_msgs::srv::Path::Request> request,
    std::shared_ptr<fog_msgs::srv::Path::Response> response);
  void gpsWaypointCallback(
    const std::shared_ptr<fog_msgs::srv::Vec4::Request> request,
    std::shared_ptr<fog_msgs::srv::Vec4::Response> response);
  void gpsPathCallback(
    const std::shared_ptr<fog_msgs::srv::Path::Request> request,
    std::shared_ptr<fog_msgs::srv::Path::Response> response);

  void gotoCallback(const nav_msgs::msg::Path::UniquePtr msg);
  void timer_callback_();

  int random_(
    int min,
    int max);

  rclcpp::TimerBase::SharedPtr timer_;
  std::string drone_id_, drone_namespace_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr goto_trigger_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr hover_service_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr goto_subscriber_;
  rclcpp::Service<fog_msgs::srv::Vec4>::SharedPtr local_waypoint_service_;
  rclcpp::Service<fog_msgs::srv::Path>::SharedPtr local_path_service_;
  rclcpp::Service<fog_msgs::srv::Vec4>::SharedPtr gps_waypoint_service_;
  rclcpp::Service<fog_msgs::srv::Path>::SharedPtr gps_path_service_;
  size_t count_;
};

#endif /* _PUBSUB_APP_DUMMY_SERVICE_SERVER_HPP_ */
