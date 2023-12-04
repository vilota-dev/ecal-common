#pragma once

#include <memory>
#include <mutex>
#include <vector>
#include <map>
#include <string>
#include <iostream>

#include <capnp/message.h>
#include <tbb/concurrent_queue.h>

namespace vk
{

  template <typename T>
  using ItemPtr = std::shared_ptr<T>;

  template <typename T>
  using MessageQueuePtr = std::shared_ptr<tbb::concurrent_bounded_queue<ItemPtr<T>>>;

  // Wrapper around a map of topics -> tbb queues
  //
  // { "topic1": vector[
  //    shared_ptr<queue<shared_ptr<T>>>,
  //    ... ], ... }
  template <typename T>
  class GenericTbbMiddleware
  {
    public:
      // to subscribe to a topic, pass a queue to which the messages will be pushed
      // to unsubscribe, simply do queue.reset(nullptr)
      void registerSubscriber(const std::string& topic, MessageQueuePtr<T> queue) {
          std::lock_guard<std::mutex> lock(m_mutexSubs);
          m_subQueues[topic].push_back(queue);
      }

      void sendMsg(const std::string& topic, ItemPtr<T> item) {
        std::lock_guard<std::mutex> lock(m_mutexSubs);
        auto& subQueues = m_subQueues[topic];

        std::cout << "[TBB Middleware] sending msg to " << subQueues.size() << " sub queues of topic: " << topic << std::endl;
        for (auto it = subQueues.begin(); it != subQueues.end(); it++) {
            if (it->get() && it->use_count() > 1) {
                std::cout << "[TBB Middleware] pushing to sub queue..." << std::endl;
                (*it)->push(item);
            } else {
                std::cout << "[TBB Middleware] clearing sub queue..." << std::endl;
                subQueues.erase(it);
            }
        }
      }

    private:
      std::map<std::string, std::vector<MessageQueuePtr<T>>> m_subQueues;
      std::mutex m_mutexSubs;
  };

} // namespace vk
