#pragma once

#include <memory>
#include <mutex>
#include <string>

#include "GenericTbbTypes.hpp"

namespace vk
{

  template <typename T>
  class GenericTbbPublisher
  {
    public:
      explicit GenericTbbPublisher(const std::string& id) : pubId(id + "_pub") {}
      GenericTbbPublisher() = default;
      GenericTbbPublisher(const GenericTbbPublisher&) = default;

      void setId(const std::string& id) {
          pubId = id + "_pub";
      }

      void registerSubscriber(MessageQueuePtr<T>& queue) {
          std::lock_guard<std::mutex> lock(m_mutexQueue);
          m_subQueues.push_back(queue);
      }

      void sendMsg(ItemPtr<T> item) {
          std::lock_guard<std::mutex> lock(m_mutexQueue);

          for (auto it = m_subQueues.begin(); it != m_subQueues.end(); it++) {
              if (it->get()) {
                  (*it)->push(item);
              } else {
                  m_subQueues.erase(it);
              }
          }
      }

    private:
      std::string pubId;
      std::vector<MessageQueuePtr<T>> m_subQueues;
      std::mutex m_mutexQueue;
  };

} // namespace vk
