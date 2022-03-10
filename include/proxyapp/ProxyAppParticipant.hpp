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
 * @file ProxyAppParticipant.hpp
 *
 */

#ifndef _EPROSIMA_PROXY_APP_PARTICIPANT_
#define _EPROSIMA_PROXY_APP_PARTICIPANT_

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/DataWriterListener.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>

#include "types/fog_msgs/Vec4PubSubTypes.h"

namespace proxy_app{

using namespace eprosima::fastdds::dds;

template <class PubSubT, class MsgT>
class ProxyAppParticipant : public DataReaderListener
{
public:

    //! Constructor
    ProxyAppParticipant();

    //! Destructor
    ~ProxyAppParticipant();

    //! Initialize DDS entities
    bool init(
            const std::string& whitelist_ip,
            const std::string& topic_name, bool security = false);

    //! Custom callback
    void on_data_available(
            DataReader* reader) override;

    void on_subscription_matched(
            DataReader* reader,
            const SubscriptionMatchedStatus& info) override;

    //! Assign DataWriter
    void assign_remote_datawriter(
            DataWriter* writer);

    //! Get local DataWriter
    DataWriter* get_local_datawriter();

private:

    //! DDS entities pointers
    DomainParticipant* participant_;
    Publisher* publisher_;
    Subscriber* subscriber_;
    DataWriter* writer_;
    DataWriter* remote_writer_;
    DataReader* reader_;
    Topic* topic_;
    TypeSupport type_;

    //! Stop thread flag
    std::atomic_bool stop_thread_;
    //! Mutex to block thread waiting data
    std::mutex thread_mutex_;
    //! Condition variable to notify a data has been received
    std::condition_variable thread_cv_;
    //! Waiting data thread
    std::unique_ptr<std::thread> thread_;
    //! Data samples queue
    std::queue<MsgT> data_queue_;

    //! Thread routine: copy data from the unsecure topic to the secure one and viceversa
    void thread_routine();

    //! Stop data thread
    void stop_thread();

};

} // namespace proxy_app

#endif // _EPROSIMA_PROXY_APP_PARTICIPANT_
