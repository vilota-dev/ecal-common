#include <ecal_camera/CameraFactory.hpp>

#include "CameraInternal.hpp"

namespace vk 
{

CameraInterface::Ptr CameraFactory::getCameraHandler()
{
    return std::make_shared<CameraInternal>();
}

} // namespace vk

