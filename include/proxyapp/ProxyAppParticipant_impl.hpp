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
 * @file ProxyAppParticipant.cpp
 *
 */

#include "proxyapp/ProxyAppParticipant.hpp"

#include <fstream>
#include <memory>
#include <mutex>
#include <string>

#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/domain/qos/DomainParticipantQos.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>
#include <fastdds/rtps/attributes/RTPSParticipantAttributes.h>
#include <fastdds/rtps/transport/UDPv4TransportDescriptor.h>
#include <fastrtps/types/TypesBase.h>

#include "types/fog_msgs/Vec4PubSubTypes.h"

namespace proxy_app{

using namespace eprosima::fastdds::dds;
using namespace eprosima::fastdds::rtps;
using namespace eprosima::fastrtps::rtps;

template <class PubSubT, class MsgT>
ProxyAppParticipant<PubSubT, MsgT>::ProxyAppParticipant()
    : participant_(nullptr)
    , publisher_(nullptr)
    , subscriber_(nullptr)
    , writer_(nullptr)
    , reader_(nullptr)
    , topic_(nullptr)
    , type_(new PubSubT())
    , stop_thread_(false)
{
    std::unique_lock<std::mutex> lock(thread_mutex_);
    if (!thread_)
    {
        thread_.reset(new std::thread(&ProxyAppParticipant::thread_routine, this));
    }
}

template <class PubSubT, class MsgT>
ProxyAppParticipant<PubSubT, MsgT>::~ProxyAppParticipant()
{
    stop_thread();

    // Delete DDS entities
    if (nullptr != reader_)
    {
        subscriber_->delete_datareader(reader_);
    }
    if (nullptr != writer_)
    {
        publisher_->delete_datawriter(writer_);
    }
    if (nullptr != subscriber_)
    {
        participant_->delete_subscriber(subscriber_);
    }
    if (nullptr != publisher_)
    {
        participant_->delete_publisher(publisher_);
    }
    if (nullptr != topic_)
    {
        participant_->delete_topic(topic_);
    }
    DomainParticipantFactory::get_instance()->delete_participant(participant_);
}

template <class PubSubT, class MsgT>
bool ProxyAppParticipant<PubSubT, MsgT>::init(
        const std::string& whitelist_ip,
        const std::string& topic_name, bool security)
{
    DomainParticipantQos pqos;
    uint32_t domain_id;
    // Set security options
    if (security)
    {
        /*
        With DDS-RPC -> DDS Service (RequesTopic, ReplyTopic):
        ddsServiceName = “rs/” + rosServiceName
        
        With no DDS-RPC support -> user RequesTopic, ReplyTopic) pair:
        ddsRequestTopic = “rq/” + rosServiceName
        ddsReplyTopic = “rr/” + rosServiceName
        */
        domain_id = 1;
        std::cout << "[INFO] Secure domain ID: " << domain_id << std::endl;

        // Read ROS_SECURITY_KEYSTORE environment file
        char* data;
        std::string keystore_path;
        data = getenv("ROS_SECURITY_KEYSTORE");
        if (nullptr != data)
        {
            keystore_path = data;
        }
        else
        {
            std::cout << "[ERROR] ROS_SECURITY_KEYSTORE environment variable not set" << std::endl;
            return false;
        }

        // Read PKCS#11 URI
        std::ifstream key_p11_file;
        std::string pkcs11_uri;
        key_p11_file.open(keystore_path + "/enclaves/fog/proxyapp/key.p11");
        key_p11_file >> pkcs11_uri;
        key_p11_file.close();

        pqos.name("secure_participant");

        /* Fast DDS Participant security configuration */

        // Authentication plugin:
        // https://fast-dds.docs.eprosima.com/en/latest/fastdds/security/auth_plugin/auth_plugin.html
        pqos.properties().properties().emplace_back("dds.sec.auth.plugin", "builtin.PKI-DH");
        pqos.properties().properties().emplace_back("dds.sec.auth.builtin.PKI-DH.identity_ca",
                "file://" + keystore_path + "/enclaves/fog/proxyapp/identity_ca.cert.pem");
        pqos.properties().properties().emplace_back("dds.sec.auth.builtin.PKI-DH.identity_certificate",
                "file://" + keystore_path + "/enclaves/fog/proxyapp/cert.pem");
        pqos.properties().properties().emplace_back("dds.sec.auth.builtin.PKI-DH.private_key", pkcs11_uri);
        // Access control plugin:
        // https://fast-dds.docs.eprosima.com/en/latest/fastdds/security/access_control_plugin/access_control_plugin.html
        pqos.properties().properties().emplace_back("dds.sec.access.plugin", "builtin.Access-Permissions");
        pqos.properties().properties().emplace_back("dds.sec.access.builtin.Access-Permissions.permissions_ca",
                "file://" + keystore_path + "/enclaves/fog/proxyapp/permissions_ca.cert.pem");
        pqos.properties().properties().emplace_back("dds.sec.access.builtin.Access-Permissions.governance",
                "file://" + keystore_path + "/enclaves/fog/proxyapp/governance.p7s");
        pqos.properties().properties().emplace_back("dds.sec.access.builtin.Access-Permissions.permissions",
                "file://" + keystore_path + "/enclaves/fog/proxyapp/permissions.p7s");
        // Cryptographic plugin:
        // https://fast-dds.docs.eprosima.com/en/latest/fastdds/security/crypto_plugin/crypto_plugin.html
        pqos.properties().properties().emplace_back("dds.sec.crypto.plugin", "builtin.AES-GCM-GMAC");

        // Enable secure logging plugin:
        // https://fast-dds.docs.eprosima.com/en/latest/fastdds/security/logging_plugin/logging_plugin.html
        pqos.properties().properties().emplace_back("dds.sec.log.plugin", "builtin.DDS_LogTopic");
        pqos.properties().properties().emplace_back("dds.sec.log.builtin.DDS_LogTopic.logging_level", "EMERGENCY_LEVEL");
        pqos.properties().properties().emplace_back("dds.sec.log.builtin.DDS_LogTopic.log_file", "security.log");
    }
    else
    {
        domain_id = 0;
        pqos.name("non_secure_participant");

        // Whitelist
        auto udp_transport = std::make_shared<UDPv4TransportDescriptor>();
        udp_transport->interfaceWhiteList.emplace_back(whitelist_ip);
        pqos.transport().user_transports.push_back(udp_transport);
        pqos.transport().use_builtin_transports = false;
        std::cout << "Whitelisted " << whitelist_ip << std::endl;
    }

    participant_ = DomainParticipantFactory::get_instance()->create_participant(domain_id, pqos);
    if (nullptr == participant_)
    {
        std::cout << "[ERROR] Failure creating participant" << std::endl;
        return false;
    }

    // Register the type
    type_.register_type(participant_);

    // Create the publisher
    publisher_ = participant_->create_publisher(PUBLISHER_QOS_DEFAULT);
    if (nullptr == publisher_)
    {
        std::cout << "[ERROR] Failure creating publisher" << std::endl;
        return false;
    }

    // Create the subscriber
    subscriber_ = participant_->create_subscriber(SUBSCRIBER_QOS_DEFAULT);
    if (nullptr == subscriber_)
    {
        std:: cout << "[ERROR] Failure creating subscriber" << std::endl;
        return false;
    }

    // Create the topic
    topic_ = participant_->create_topic(topic_name, type_.get_type_name(), TOPIC_QOS_DEFAULT);
    if (nullptr == topic_)
    {
        std::cout << "[ERROR] Failure creating topic " << topic_name << std::endl;
        return false;
    }

    // Create the datawriter
    writer_ = publisher_->create_datawriter(topic_, DATAWRITER_QOS_DEFAULT);
    if (nullptr == writer_)
    {
        std::cout << "[ERROR] Failure creating datawriter in topic " << topic_name << std::endl;
        return false;
    }

    // Create the datareader
    reader_ = subscriber_->create_datareader(topic_, DATAREADER_QOS_DEFAULT, this);
    if (nullptr == reader_)
    {
        std::cout << "[ERROR] creating datareader in topic " << topic_name << std::endl;
        return false;
    }

    return true;
}

template <class PubSubT, class MsgT>
void ProxyAppParticipant<PubSubT, MsgT>::on_data_available(
        DataReader* reader)
{
    SampleInfo info;
    auto topic_name = reader->get_topicdescription()->get_name();
    MsgT msg;
    if (ReturnCode_t::RETCODE_OK == reader->take_next_sample(&msg, &info))
    {
        // Discard data comming from the own DataWriter: avoid feedback
        if (info.valid_data && info.publication_handle != writer_->get_instance_handle())
        {
            std::cout << "Received data in topic "
                    << reader->get_topicdescription()->get_name()
                    << std::endl;
            std::unique_lock<std::mutex> lock(thread_mutex_);
            data_queue_.push(msg);
            lock.unlock();
            thread_cv_.notify_one();
        }
    }
}

template <class PubSubT, class MsgT>
void ProxyAppParticipant<PubSubT, MsgT>::on_subscription_matched(
        DataReader* reader,
        const SubscriptionMatchedStatus& info)
{
    if (info.current_count_change == 1)
    {
        std::cout << "DataReader matched in topic " << reader->get_topicdescription()->get_name() << std::endl;
        std::cout << "\tTotal DataWriters matched: " << info.total_count << std::endl;
    }
}

template <class PubSubT, class MsgT>
void ProxyAppParticipant<PubSubT, MsgT>::stop_thread()
{
    stop_thread_.store(true);
    thread_cv_.notify_one();
    thread_->join();
    thread_.reset();
}

template <class PubSubT, class MsgT>
void ProxyAppParticipant<PubSubT, MsgT>::thread_routine()
{
    while (!stop_thread_)
    {
        // Wait until queue has data to process or the thread has been commanded to stop
        std::unique_lock<std::mutex> lock(thread_mutex_);
        thread_cv_.wait(lock, [&]()
                {
                    return stop_thread_ || !data_queue_.empty();
                });

        // Stop thread if it has been commanded
        if (stop_thread_)
        {
            return;
        }

        // Process data if the remote_writer_ has been assigned and the queue has data to process
        if (remote_writer_)
        {
            while (!data_queue_.empty())
            {
                MsgT sample = data_queue_.front();
                lock.unlock();
                remote_writer_->write(&sample);
                std::cout << "Publishing data in topic " 
                        << remote_writer_->get_topic()->get_name()
                        << std::endl;
                lock.lock();
                data_queue_.pop();
            }
        }
    }
}

template <class PubSubT, class MsgT>
void ProxyAppParticipant<PubSubT, MsgT>::assign_remote_datawriter(
        DataWriter* writer)
{
    remote_writer_ = writer;
}

template <class PubSubT, class MsgT>
DataWriter* ProxyAppParticipant<PubSubT, MsgT>::get_local_datawriter()
{
    return writer_;
}

} // namespace proxy_app