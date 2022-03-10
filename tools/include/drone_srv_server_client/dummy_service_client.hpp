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

#ifndef _PUBSUB_APP_DUMMY_client_CLIENT_HPP_
#define _PUBSUB_APP_DUMMY_client_CLIENT_HPP_
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <fog_msgs/srv/vec4.hpp>
#include <fog_msgs/srv/path.hpp>
#include <nav_msgs/msg/path.hpp>

using std::placeholders::_1;

class DummyServiceClient : public rclcpp::Node
{
public:
  DummyServiceClient(
    std::string drone_id, std::string drone_namespace);

private:
  // service callbacks
  void gotoTriggerCallback(
    const rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
  {
    RCLCPP_INFO(this->get_logger(), "gotoTriggerCallback");
    std::shared_ptr<std_srvs::srv::Trigger_Response> response = future.get();
    if (response->success) {
      RCLCPP_INFO(this->get_logger(), "gotoTriggerCallback: success");
    } else {
      RCLCPP_INFO(this->get_logger(), "gotoTriggerCallback: fail");
    }
  }
  void hoverCallback(
    const rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
  {
    RCLCPP_INFO(this->get_logger(), "hoverCallback");
    std::shared_ptr<std_srvs::srv::Trigger_Response> response = future.get();
    if (response->success) {
      RCLCPP_INFO(this->get_logger(), "hoverCallback: success");
    } else {
      RCLCPP_INFO(this->get_logger(), "hoverCallback: fail");
    }
  }
  void localWaypointCallback(
    const rclcpp::Client<fog_msgs::srv::Vec4>::SharedFuture future)
  {
    std::shared_ptr<fog_msgs::srv::Vec4_Response> response = future.get();
    RCLCPP_INFO(this->get_logger(), "localWaypointCallback");
    if (response->success) {
      RCLCPP_INFO(this->get_logger(), "localWaypointCallback: success");
    } else {
      RCLCPP_INFO(this->get_logger(), "localWaypointCallback: fail");
    }
  }
  void localPathCallback(
    const rclcpp::Client<fog_msgs::srv::Path>::SharedFuture future)
  {
    RCLCPP_INFO(this->get_logger(), "localPathCallback");
    std::shared_ptr<fog_msgs::srv::Path_Response> response = future.get();
    if (response->success) {
      RCLCPP_INFO(this->get_logger(), "localPathCallback: success");
    } else {
      RCLCPP_INFO(this->get_logger(), "localPathCallback: fail");
    }
  }
  void gpsWaypointCallback(
    const rclcpp::Client<fog_msgs::srv::Vec4>::SharedFuture future)
  {
    RCLCPP_INFO(this->get_logger(), "gpsWaypointCallback");
    std::shared_ptr<fog_msgs::srv::Vec4_Response> response = future.get();
    if (response->success) {
      RCLCPP_INFO(this->get_logger(), "gpsWaypointCallback: success");
    } else {
      RCLCPP_INFO(this->get_logger(), "gpsWaypointCallback: fail");
    }
  }
  void gpsPathCallback(
    const rclcpp::Client<fog_msgs::srv::Path>::SharedFuture future)
  {
    RCLCPP_INFO(this->get_logger(), "gpsPathCallback");
    std::shared_ptr<fog_msgs::srv::Path_Response> response = future.get();
    if (response->success) {
      RCLCPP_INFO(this->get_logger(), "gpsPathCallback: success");
    } else {
      RCLCPP_INFO(this->get_logger(), "gpsPathCallback: fail");
    }
  }


  void timer_callback_();
  rclcpp::TimerBase::SharedPtr timer_;
  std::string drone_id_, drone_namespace_;
  size_t count_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr goto_path_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr goto_trigger_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr hover_client_;
  rclcpp::Client<fog_msgs::srv::Vec4>::SharedPtr local_waypoint_client_;
  rclcpp::Client<fog_msgs::srv::Path>::SharedPtr local_path_client_;
  rclcpp::Client<fog_msgs::srv::Vec4>::SharedPtr gps_waypoint_client_;
  rclcpp::Client<fog_msgs::srv::Path>::SharedPtr gps_path_client_;
};
#endif /* _PUBSUB_APP_DUMMY_client_CLIENT_HPP_ */
