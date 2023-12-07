#pragma once

#include <vector>
#include <tbb/concurrent_queue.h>

namespace vk
{

    template <typename T>
    using ItemPtr = std::shared_ptr<T>;

    template <typename T>
    using MessageQueuePtr = std::shared_ptr<tbb::concurrent_bounded_queue<ItemPtr<T>>>;

} // namespace vk