#include "av_camera_trigger/camera_trigger_node.hpp"


CameraTriggerNode::CameraTriggerNode() : Node("camera_trigger_node"),
    m_triggerDevice{std::make_shared<HIDDevice>(constants::VENDOR_ID,
                                                constants::PRODUCT_ID)}
{
    RCLCPP_INFO(this->get_logger(), "Camera Trigger Node startup");
    this->loadParams();
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
    RCLCPP_INFO(this->get_logger(),  "Stopping all cameras ...");
    m_triggerDevice->stopCameras();

    RCLCPP_INFO(this->get_logger(),  "Detaching all cameras ...");
    m_triggerDevice->detachCameras();

    int camera_framerate = m_frameRate;
    int camera_portnumber = 0;
    std::string camera_alias = "RSPR_L";
    m_triggerDevice->attachCamera(camera_framerate, camera_portnumber, camera_alias);
    RCLCPP_INFO_STREAM(this->get_logger(),
        "Camera identified by alias " << camera_alias <<
        " on connector " << camera_portnumber << " has been attached");
    // for (const auto& camera : getAttachedCameras())
    // {
    //     m_triggerDevice->attachCamera(camera.framerate,
    //                                   camera.portNumber,
    //                                   camera.alias);
    //     NODELET_INFO_STREAM("Camera identified by alias " << camera.alias <<
    //                         " on connector " << camera.portNumber << " has been attached");
    // }

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

void CameraTriggerNode::loadParams()
{
    // try
    // {
        // XmlRpc::XmlRpcValue paramList;
        // ros::param::get("~camera_params", paramList);

        // if (paramList.getType() != XmlRpc::XmlRpcValue::Type::TypeArray)
        // {
        //     throw XmlRpc::XmlRpcException{"Wrong camera_params type"};
        // }

        m_frameRate = this->declare_parameter("~frame_rate", 15);
        // for (unsigned int i = 0; i < paramList.size(); i++)
        // {
        // //     XmlRpc::XmlRpcValue& rpcValue = paramList[i];

        // //     const auto attach = getValue<bool>(rpcValue, "attach");
        // //     const auto topic = getValue<std::string>(rpcValue, "topic");
        // //     const auto portNumber = getValue<int>(rpcValue, "port_number");
        // //     const auto cameraAlias = getValue<std::string>(rpcValue, "camera_alias");

        //     m_cameras.push_back({topic, m_frameRate, portNumber, cameraAlias, attach});

        //     RCLCPP_INFO_STREAM(this->get_logger(), "Camera " << cameraAlias << ": "
        //                         << "connector: " << portNumber
        //                         << ", frame rate: " << m_frameRate
        //                         << ", topic: " << topic);
        // }

        using namespace std::chrono;
        constexpr auto expectedPeriod = milliseconds(1000) / constants::LOOP_RATE;
        const auto period = milliseconds{1000} / m_frameRate;
        m_maxFirmwareQueryPeriod = period * 3 / 4; //75% of the frame rate period
        RCLCPP_INFO_STREAM(this->get_logger(), "Expected firmware query period " << expectedPeriod.count() << " ms");
        RCLCPP_INFO_STREAM(this->get_logger(), "Max allowed firmware query period " << m_maxFirmwareQueryPeriod.count() << " ms");

        m_applyTimeCorrection = this->declare_parameter("~apply_time_correction", false);
        RCLCPP_INFO_STREAM(this->get_logger(), "Time correction is " << (m_applyTimeCorrection ? "ENABLED" : "disabled"));

        m_monitorTimeOffset = this->declare_parameter("~monitor_fw_host_time_offset", true);
        RCLCPP_INFO_STREAM(this->get_logger(), "Time offset monitoring between host and firmware is " << (m_monitorTimeOffset ? "ENABLED" : "disabled"));

        m_firmwareSerialDevice = this->declare_parameter("~firmware_serial_device", std::string("/dev/ttyACM0"));
        RCLCPP_INFO_STREAM(this->get_logger(), "Firmware logs will be read from " << m_firmwareSerialDevice);
        m_firmwareLogFilePath = this->declare_parameter("~firmware_log_file_path", std::string("/tmp/camera_trigger_firmware.log"));
        RCLCPP_INFO_STREAM(this->get_logger(), "Firmware logs will be written to " << m_firmwareLogFilePath);
    // }
    // catch (const XmlRpc::XmlRpcException& ex)
    // {
    //     RCLCPP_ERROR_STREAM(this->get_logger(), "Error whilst loading parameters: " << ex.getMessage());
    //     throw node::Error{ErrorCode::ConfigError};
    // }
}


void CameraTriggerNode::printTimeOffset()
{
    rclcpp::Duration m_offset(getTimeOffset());
    RCLCPP_INFO_STREAM(this->get_logger(), "Clocks status: " << this->description(m_offset) << " by " << std::abs(m_offset.nanoseconds()) << " ns");
}
