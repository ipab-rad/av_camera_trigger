#ifndef AV_CAMERA_TRIGGER__CAMERA_TRIGGER_NODE_HPP_
#define AV_CAMERA_TRIGGER__CAMERA_TRIGGER_NODE_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <type_traits>

#include "rclcpp/rclcpp.hpp"

#include "camera_trigger_msgs/msg/trigger.hpp"
#include "av_camera_trigger/device_interface.hpp"
#include "av_camera_trigger/hid_device_interface.hpp"

namespace av_camera_trigger
{

struct CameraParams
{
    std::string topic;
    int framerate;
    long int portNumber;
    std::string alias;
    bool attached;
};

class CameraTriggerNode : public rclcpp::Node
{
  public:
    using CameraContainer = std::vector<CameraParams>;

    CameraTriggerNode(const rclcpp::NodeOptions & options);

  private:
    static rclcpp::Time getTimeNow();
    bool loadParams();
    bool connectToFirmware();

    void attachCameras();
    void printTimeOffset();
    bool synchFirmwareTime();

    const std::string& description(const rclcpp::Duration& offset);
    std::chrono::nanoseconds getTimeOffset();

    CameraContainer    m_cameras;
    bool               m_applyTimeCorrection;
    int                m_frameRate;
    bool               m_monitorTimeOffset;

    IDevice::SharedPtr           m_triggerDevice;

    std::chrono::milliseconds    m_maxFirmwareQueryPeriod;

    std::string    m_firmwareSerialDevice;
    std::string    m_firmwareLogFilePath;
};

}  // namespace av_camera_trigger
#endif  // AV_CAMERA_TRIGGER__CAMERA_TRIGGER_NODE_HPP_
