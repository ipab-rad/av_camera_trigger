#include "av_camera_trigger/camera_trigger_node.hpp"

namespace av_camera_trigger
{
CameraTriggerNode::CameraTriggerNode(const rclcpp::NodeOptions & options) : Node("camera_trigger_node", options),
    m_triggerDevice{std::make_shared<HIDDevice>(constants::VENDOR_ID,
                                                constants::PRODUCT_ID)}
{
    RCLCPP_INFO(this->get_logger(), "Camera Trigger Node startup");

    if (this->loadParams() and this->connectToFirmware())
    {
        this->synchFirmwareTime();
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
    RCLCPP_INFO(this->get_logger(),  "Stopping all cameras ...");
    m_triggerDevice->stopCameras();

    RCLCPP_INFO(this->get_logger(),  "Detaching all cameras ...");
    m_triggerDevice->detachCameras();

    for (auto& camera : m_cameras)
    {
        m_triggerDevice->attachCamera(camera.framerate,
                                      camera.portNumber,
                                      camera.alias);
        camera.attached = true;
        RCLCPP_INFO(this->get_logger(),
            "Camera identified by alias %s on connector %li has been attached", camera.alias.c_str(), camera.portNumber);
    }

    RCLCPP_INFO(this->get_logger(),  "Firmware is starting all attached cameras...");
    m_triggerDevice->startCameras();
}

bool CameraTriggerNode::connectToFirmware()
{
    RCLCPP_INFO(this->get_logger(), "Attempting to connect to firmware");
    std::future<bool> result = std::async(std::launch::async, [this]{return m_triggerDevice->connect();});

    const auto connected = result.get();

    if (connected)
    {
        RCLCPP_INFO(this->get_logger(), "Connection with the firmware established");
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Couldn't connect to the firmware");

    }
    return connected;
}

std::chrono::nanoseconds CameraTriggerNode::getTimeOffset()
{
    using namespace std::chrono;
    const auto initialTime = system_clock::now();
    const nanoseconds deviceTime = m_triggerDevice->getDeviceTime();
    const auto currentTime = system_clock::now();

    const nanoseconds sendDuration = (currentTime - initialTime) / 2;
    const nanoseconds offset =
        (initialTime.time_since_epoch() - deviceTime) - sendDuration;

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

bool CameraTriggerNode::loadParams()
{
    m_frameRate = this->declare_parameter("frame_rate", 15);

    const auto cams = this->declare_parameter(
        "cameras", std::vector<std::string>({}));
    const auto ports = this->declare_parameter(
        "ports", std::vector<int>({}));
    const auto topics = this->declare_parameter(
        "topics", std::vector<std::string>({}));

    // If parameters loaded and correctly formatted, store in structs
    if ((cams.size() != 0) and
        (cams.size() == ports.size() and ports.size() == topics.size()))
    {
        for (unsigned int i = 0; i < cams.size(); i++)
        {
            const std::string full_topic =
            "/sensor/camera/" + topics[i] + "/trigger";
            m_cameras.push_back(
                {full_topic, m_frameRate, ports[i], cams[i], false}
                );
        }
        RCLCPP_INFO(this->get_logger(),
            "Camera parameters loaded correctly");
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(),
            "Cameras incorrectly parameterised");
        return false;
    }

    using namespace std::chrono;
    constexpr auto expectedPeriod =
        milliseconds(1000) / constants::LOOP_RATE;
    const auto period = milliseconds{1000} / m_frameRate;
    //75% of the frame rate period
    m_maxFirmwareQueryPeriod = period * 3 / 4;

    RCLCPP_INFO_STREAM(this->get_logger(),
        "Expected firmware query period " <<
        expectedPeriod.count() << " ms");
    RCLCPP_INFO_STREAM(this->get_logger(),
        "Max allowed firmware query period " <<
        m_maxFirmwareQueryPeriod.count() << " ms");

    m_applyTimeCorrection = this->declare_parameter(
        "apply_time_correction", false);
    RCLCPP_INFO_STREAM(this->get_logger(),
        "Time correction is " <<
        (m_applyTimeCorrection ? "ENABLED" : "disabled"));

    m_monitorTimeOffset = this->declare_parameter("monitor_fw_host_time_offset", true);
    RCLCPP_INFO_STREAM(this->get_logger(),
        "Time offset monitoring between host and firmware is " <<
        (m_monitorTimeOffset ? "ENABLED" : "disabled"));

    m_firmwareSerialDevice = this->declare_parameter(
        "firmware_serial_device", std::string("/dev/ttyACM0"));
    RCLCPP_INFO_STREAM(this->get_logger(),
        "Firmware logs will be read from " << m_firmwareSerialDevice);

    m_firmwareLogFilePath = this->declare_parameter(
        "firmware_log_file_path",
        std::string("/tmp/camera_trigger_firmware.log"));
    RCLCPP_INFO_STREAM(this->get_logger(),
        "Firmware logs will be written to " << m_firmwareLogFilePath);

    return true;
}


void CameraTriggerNode::printTimeOffset()
{
    rclcpp::Duration m_offset(getTimeOffset());
    RCLCPP_INFO_STREAM(this->get_logger(),
        "Clocks status: " << this->description(m_offset) <<
        " by " << std::abs(m_offset.nanoseconds()) << " ns");
}

bool CameraTriggerNode::synchFirmwareTime()
{
    RCLCPP_INFO(this->get_logger(), "Syncing time between Firmware and Host");
    try
    {
        m_triggerDevice->setDeviceTime(rclcpp::Clock(RCL_ROS_TIME).now().nanoseconds());
    }
    catch (...)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to set device time");
        return (false);
    }

    time_t diff = std::abs(m_triggerDevice->getDeviceTime().count() / std::nano::den - rclcpp::Clock(RCL_ROS_TIME).now().nanoseconds());

    if (diff < 1)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Synched! Time diff: " << diff);
        return true;
    }
    else
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Not synched! Time diff: " << diff);
        return false;
    }
}
}  // namespace av_camera_trigger

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(av_camera_trigger::CameraTriggerNode)
