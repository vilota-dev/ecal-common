#pragma once

#include <string>

#include "GenericTbbTypes.hpp"
#include "GenericTbbPublisher.hpp"

namespace vk
{

  template <typename T>
  class GenericTbbSubscriber
  {
    public:
      void setId(const std::string& id) {
          subId = id + "_sub";
      }

      void linkPublisher(const std::string& topic, std::shared_ptr<GenericTbbPublisher<T>> pub) {
          pub->registerSubscriber(topic, queuePtr);
      }
      MessageQueuePtr<T> queuePtr;

    private:
      std::string subId = "unknown_sub";
  };

} // namespace vk
