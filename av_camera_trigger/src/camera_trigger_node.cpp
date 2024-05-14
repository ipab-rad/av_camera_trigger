#include "av_camera_trigger/camera_trigger_node.hpp"

// using namespace std::chrono_literals;


CameraTriggerNode::CameraTriggerNode() : Node("camera_trigger_node"),
    m_triggerDevice{std::make_shared<HIDDevice>(constants::VENDOR_ID,
                                                constants::PRODUCT_ID)}
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Camera Trigger Node startup");
    if (this->connectToFirmware())
    {
        this->attachCameras();
        this->printTimeOffset();
    }
    else
    {
        rclcpp::shutdown();
    }
}

void CameraTriggerNode::attachCameras()
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),  "Stopping all cameras ...");
    m_triggerDevice->stopCameras();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),  "Detaching all cameras ...");
    m_triggerDevice->detachCameras();

    // for (const auto& camera : getAttachedCameras())
    // {
    int camera_framerate = 10;
    int camera_portnumber = 0;
    std::string camera_alias = "RSPR_L";
    m_triggerDevice->attachCamera(camera_framerate, camera_portnumber, camera_alias);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),  
        "Camera identified by alias " << camera_alias <<
        " on connector " << camera_portnumber << " has been attached");
    // }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),  "Firmware is starting all attached cameras...");
    m_triggerDevice->startCameras();
}

bool CameraTriggerNode::connectToFirmware()
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Attempting to connect to firmware");       
    std::future<bool> result = std::async(std::launch::async, [this]{return m_triggerDevice->connect();});

    const auto connected = result.get();

    if (connected)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Connection with the firmware established");            
    }
    else 
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Couldn't connect to the firmware");

    }
    return connected;
}

std::chrono::nanoseconds CameraTriggerNode::getTimeOffset()
{
    // get current system time
    using namespace std::chrono;
    const auto initialTime = system_clock::now();
    const nanoseconds deviceTime = m_triggerDevice->getDeviceTime();
    const auto currentTime = system_clock::now();

    const nanoseconds sendDuration = (currentTime - initialTime) / 2;
    const nanoseconds offset = (initialTime.time_since_epoch() - deviceTime) - sendDuration;

    return offset;
}

const std::string& CameraTriggerNode::description(const rclcpp::Duration& offset)
{
    if (offset.nanoseconds() < 0)
    {
        static const std::string s{"Host behind firmware clock"};
        return s;
    }
    else if (offset.nanoseconds() > 0)
    {
        static const std::string s{"Host ahead firmware clock"};
        return s;
    }

    static const std::string s{"Host and firmware clocks are in synch"};
    return s;
}

void CameraTriggerNode::printTimeOffset()
{
    rclcpp::Duration m_offset(getTimeOffset());
    RCLCPP_INFO_STREAM(this->get_logger(), "Clocks status: " << this->description(m_offset) << " by " << std::abs(m_offset.nanoseconds()) << " ns");
}