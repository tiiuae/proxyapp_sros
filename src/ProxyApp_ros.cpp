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

#include <map>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include <fog_msgs/srv/vec4.hpp>
// #include "node_thread.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;
namespace proxyapp_ros
{

/**
* @class proxyapp_ros::NodeThread
* @brief A background thread to process node/executor callbacks
*/
class NodeThread
{
public:
  /**
   * @brief A background thread to process node callbacks constructor
   * @param node_base Interface to Node to spin in thread
   */
  explicit NodeThread(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base);

  /**
   * @brief A background thread to process executor's callbacks constructor
   * @param executor Interface to executor to spin in thread
   */
  explicit NodeThread(rclcpp::executors::SingleThreadedExecutor::SharedPtr executor);

  /**
   * @brief A destructor
   */
  ~NodeThread();

protected:
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_;
  std::unique_ptr<std::thread> thread_;
  rclcpp::Executor::SharedPtr executor_;
};

NodeThread::NodeThread(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base)
: node_(node_base)
{
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  thread_ = std::make_unique<std::thread>(
    [&]()
    {
      executor_->add_node(node_);
      executor_->spin();
      executor_->remove_node(node_);
    });
}

NodeThread::NodeThread(rclcpp::executors::SingleThreadedExecutor::SharedPtr executor)
: executor_(executor)
{
  thread_ = std::make_unique<std::thread>([&]() {executor_->spin();});
}

NodeThread::~NodeThread()
{
  executor_->cancel();
  thread_->join();
}

class UnsecureServiceClient : public rclcpp::Node
{
public:
  UnsecureServiceClient(
    std::string drone_id, std::string drone_namespace,
    const rclcpp::NodeOptions & options)
  : Node("unsecure_service_client", options),
    drone_id_(drone_id),
    drone_namespace_(drone_namespace),
    count_(0)
  {
    gps_waypoint_client_ = this->create_client<fog_msgs::srv::Vec4>(
      "/" + drone_namespace_ + "/navigation/gps_waypoint");
  }
  virtual ~UnsecureServiceClient() {}

  bool relayRequest(
    const std::shared_ptr<fog_msgs::srv::Vec4::Request> request,
    std::shared_ptr<fog_msgs::srv::Vec4::Response> & response)
  {
    if (!gps_waypoint_client_->service_is_ready()) {
      RCLCPP_ERROR(this->get_logger(), "GPS waypoint service is not ready, waiting");
      return false;
    }
    auto client_future = gps_waypoint_client_->async_send_request(request);
    client_future.wait();
    auto resp = client_future.get();
    // RCLCPP_INFO(
    //   this->get_logger(), "GPS waypoint service response: %s",
    //   resp->success ? "success" : "fail");
    response->success = resp->success;
    response->message = resp->message;
    return true;
  }

private:
  std::string drone_id_, drone_namespace_;
  size_t count_;
  rclcpp::Client<fog_msgs::srv::Vec4>::SharedPtr gps_waypoint_client_;
};


class SecureServiceServer : public rclcpp::Node
{
public:
  SecureServiceServer()
  : Node("secure_service_server"),
    drone_id_(""),
    drone_namespace_(""),
    count_(0)
  {
    declare_parameter("drone_id", std::string("sad04"));
    declare_parameter("drone_namespace", std::string("sad04"));
    drone_id_ = get_parameter("drone_id").as_string();
    drone_namespace_ = get_parameter("drone_namespace").as_string();
    rclcpp::NodeOptions options = rclcpp::NodeOptions();
    std::vector<std::string> new_args = options.arguments();
    new_args.push_back("--ros-args");
    new_args.push_back("-r");
    new_args.push_back(std::string("__node:=") + this->get_name() + "_rclcpp_node");
    new_args.push_back("--");
    unsecure_client_ = std::make_shared<UnsecureServiceClient>(
      drone_id_, drone_namespace_, rclcpp::NodeOptions(options).arguments(new_args));
    rclcpp_thread_ = std::make_unique<NodeThread>(unsecure_client_->get_node_base_interface());

    gps_waypoint_service_ =
      this->create_service<fog_msgs::srv::Vec4>(
      "/" + drone_namespace_ + "/secure/navigation/gps_waypoint",
      std::bind(&SecureServiceServer::gpsWaypointCallback, this, _1, _2));
  }

  ~SecureServiceServer()
  {
    rclcpp_thread_.reset();
  }

private:
  // service callbacks
  void gpsWaypointCallback(
    const std::shared_ptr<fog_msgs::srv::Vec4::Request> request,
    std::shared_ptr<fog_msgs::srv::Vec4::Response> response)
  {

    RCLCPP_INFO(this->get_logger(), "Received gps waypoint");
    if (unsecure_client_->relayRequest(request, response)) {
      RCLCPP_INFO(this->get_logger(), "Relayed gps waypoint");
    } else {
      RCLCPP_INFO(this->get_logger(), "Failed to relay gps waypoint");
    }
    RCLCPP_INFO(
      this->get_logger(), "GPS waypoint service response: %s",
      response->success ? "success" : "fail");
  }

  int random_(
    int min,
    int max);

  rclcpp::TimerBase::SharedPtr timer_;
  std::string drone_id_, drone_namespace_;
  // This is the node to relay messages to
  std::shared_ptr<UnsecureServiceClient> unsecure_client_;
  // When creating a local node, this class will launch a separate thread created to spin the node
  std::unique_ptr<NodeThread> rclcpp_thread_;

  rclcpp::Service<fog_msgs::srv::Vec4>::SharedPtr gps_waypoint_service_;
  size_t count_;
};


} // namespace proxyapp_ros

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<proxyapp_ros::SecureServiceServer>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
