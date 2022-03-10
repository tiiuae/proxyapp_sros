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
#include "std_msgs/msg/string.hpp"
#include "fog_msgs/srv/vec4.hpp"

#include <drone_srv_server_client/dummy_service_client.hpp>
#include <drone_srv_server_client/dummy_service_server.hpp>


int main(
        int argc,
        char* argv[])
{
    int type = 0;
    std::string drone_id = "";

    std::string drone_namespace = "";

    if (argc > 1)
    {
        if (strcmp(argv[1], "server") == 0)
        {
            type = 1;
        }
        else if (strcmp(argv[1], "client") == 0)
        {
            type = 2;
        }
        else
        {
            std::cout << "'server' or 'client' argument needed" << std::endl;
            return 0;
        }

        if (argc > 2)
        {
            drone_id = argv[2];
        }
        else
        {
            std::cout << "The drone cannot have an empty id" << std::endl;
            return 0;
        }
        if (argc > 3)
        {
            drone_namespace = argv[3];
        }
        else
        {
            std::cout << "The drone namespace cannot have an empty id" << std::endl;
            return 0;
        }
    }
    else
    {
        std::cout << "'server' or 'client' argument needed" << std::endl;
        return 0;
    }

    switch(type)
    {
        case 1:
            {
                std::cout << "[" << drone_id.c_str() << "] Starting service server..." << std::endl;
                rclcpp::init(argc, argv);
                rclcpp::spin(std::make_shared<DummyServiceServer>(drone_id, drone_namespace));
                break;
            }
        case 2:
            {
                std::cout << "[" << drone_id.c_str() << "] Starting server client..." << std::endl;
                rclcpp::init(argc, argv);
                rclcpp::spin(std::make_shared<DummyServiceClient>(drone_id, drone_namespace));
                break;
            }
    }

    rclcpp::shutdown();
    return 0;
}
