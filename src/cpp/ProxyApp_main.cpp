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
 * @file ProxyApp_main.cpp
 *
 */

#include <iostream>
#include <string>

#include "proxyapp/ProxyAppParticipant_impl.hpp"
#include "types/fog_msgs/Vec4PubSubTypes.h"

void run()
{
    std::cout << "Waiting for Data, press Enter to stop the ProxyApp." << std::endl;
    std::cin.ignore();
    std::cout << "Shutting down the ProxyApp." << std::endl;
}

int main(
        int argc,
        char** argv)
{
    std::cout << "Starting ProxyApp" << std::endl;

    std::string whitelist_ip;
    std::string topic_name;
    
    int type = 0;
    if (argc > 2)
    {
        topic_name = argv[1];
        whitelist_ip = argv[2];
    }
    else
    {
        std::cout << "Two arguments required (Vec4 service type default)" << std::endl;
        std::cout << "1: topic name" << std::endl;
        std::cout << "2: Whitelist ip 127.0.0.1" << std::endl;
        std::cout << "Example ProxyApp navigation/gps_waypoint 127.0.0.1" << std::endl;
        return(0);
    }
    typedef fog_msgs::srv::Vec4_RequestPubSubType ReqPubSubT;
    typedef fog_msgs::srv::Vec4_Request ReqT;
    proxy_app::ProxyAppParticipant<ReqPubSubT, ReqT> internal_participant_request;
    proxy_app::ProxyAppParticipant<ReqPubSubT, ReqT> secure_participant_request;

    typedef fog_msgs::srv::Vec4_ResponsePubSubType ResPubSubT;
    typedef fog_msgs::srv::Vec4_Response ResT;
    proxy_app::ProxyAppParticipant<ResPubSubT, ResT> internal_participant_response;
    proxy_app::ProxyAppParticipant<ResPubSubT, ResT> secure_participant_response;
    
    // Add secure tag only for the secure participants, the proxyapp will publish on normal topic name
    bool internal_participant_request_initialized = internal_participant_request.init(whitelist_ip, "rq/" + topic_name + "Request", false);
    bool secure_participant_request_initialized = secure_participant_request.init("", "rq/secure/" + topic_name + "Request", true);
    bool internal_participant_response_initialized = internal_participant_response.init(whitelist_ip, "rr/" + topic_name + "Reply", false);
    bool secure_participant_response_initialized = secure_participant_response.init("", "rr/secure/" + topic_name + "Reply", true);

    if (internal_participant_request_initialized && secure_participant_request_initialized &&
        internal_participant_response_initialized && secure_participant_response_initialized)
    {
        // Give access to each Participant to the other Participant's DataWriter.
        // It is required if the data received in one Participant is to be republished in the other one.
        internal_participant_request.assign_remote_datawriter(secure_participant_request.get_local_datawriter());
        secure_participant_request.assign_remote_datawriter(internal_participant_request.get_local_datawriter());
        // Assign the service response participants
        internal_participant_response.assign_remote_datawriter(secure_participant_response.get_local_datawriter());
        secure_participant_response.assign_remote_datawriter(internal_participant_response.get_local_datawriter());
        run();
    }
    else if (!internal_participant_request_initialized)
    {
        std::cout << "[ERROR] Failure initializing internal request Participant" << std::endl;
    }
    else if (!secure_participant_request_initialized)
    {
        std::cout << "[ERROR] Failure initializing secure request Participant" << std::endl;
    }
    else if (!internal_participant_response_initialized)
    {
        std::cout << "[ERROR] Failure initializing internal response Participant" << std::endl;
    }
    else if (!secure_participant_response_initialized)
    {
        std::cout << "[ERROR] Failure initializing secure response Participant" << std::endl;
    }
    else
    {
        std::cout << "[ERROR] Failure initializing secure Participant" << std::endl;
    }

    return 0;
}
