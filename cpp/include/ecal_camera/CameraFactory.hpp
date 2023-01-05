#pragma once

#include "CameraInterface.hpp"


namespace vk 
{

    class CameraFactory
    {
      public:
        static CameraInterface::Ptr getCameraHandler();
    };
} // namespace vk

