#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

// #include "FirmwareLogger.hpp"
#include "camera_trigger_msgs/msg/trigger.hpp"
#include "av_camera_trigger/device_interface.hpp"
#include "av_camera_trigger/hid_device_interface.hpp"

#include <chrono>
#include <type_traits>

#include <boost/range/adaptor/filtered.hpp>

enum class ErrorCode
{
    ConfigError = 0,
    GenericError,
    VersionMismatchError,
    DeviceError,
    NoPpsError, // this is not actually an error!
    CameraError,
    FirmwareLoggerError
};


struct CameraParams
{
    std::string topic;
    int framerate;
    int portNumber;
    std::string alias;
    bool attached;

    CameraParams(const std::string& topic, int framerate,
                 int portNumber, const std::string& alias,
                 bool attached) :
        topic(topic),
        framerate(framerate),
        portNumber(portNumber),
        alias{alias},
        attached{attached}
    {}
};


namespace
{
template <typename T>
inline T toRos(std::chrono::nanoseconds dur)
{
    using namespace std::chrono;
    const auto secs = duration_cast<seconds>(dur);
    const T ret(secs.count(), (dur - secs).count());
    return ret;
}

inline rclcpp::Duration toRosDuration(std::chrono::nanoseconds dur)
{
    return toRos<rclcpp::Duration>(dur);
}

inline rclcpp::Time toRosTime(std::chrono::nanoseconds dur)
{
    return toRos<rclcpp::Time>(dur);
}

}

class CameraTriggerNode : public rclcpp::Node
{
  public:
    using CameraContainer = std::vector<CameraParams>;

    CameraTriggerNode();

  private:

    // std::string getNodeVersion() override;
    // static rclcpp::Time getTimeNow();
    void loadParams();
    // void makeCameraBonds();
    // void initialiseDiags();
    bool connectToFirmware();
    // void checkVersion();
    void checkFirmwareConfiguration() const;
    void attachCameras();
    void printTimeOffset();

    const std::string& description(const rclcpp::Duration& offset);
    // void publishErrorMessages();
    // void getCameraInfoAndPublish();

    // Return the offset between the system clock of this machine hosting this
    // ROS node and the firmware, in this particular or order. Throws std::logic_error.
    std::chrono::nanoseconds getTimeOffset();
    // void monitorTimeOffset();
    // void monitorPpsStatus();

    // void applyTimeCorrection(camera_trigger_msgs::trigger& message);

    // void monitorCameraInfoRetrievalFrequency() const;

    struct IsAttached
    {
        bool operator()(const CameraParams& camera) const
        {
            return camera.attached;
        }
    };

    boost::filtered_range<IsAttached, const CameraContainer>
    getAttachedCameras() const
    {
        return boost::adaptors::filter(m_cameras, IsAttached{});
    }

    boost::filtered_range<IsAttached, CameraContainer>
    getAttachedCameras()
    {
        return boost::adaptors::filter(m_cameras, IsAttached{});
    }

    // void forEachAttachedCamera(std::function<void(const CameraParams&, std::size_t diagIndex)> action);

    // struct CameraBond
    // {
    //     rclcpp::Publisher pub;
    //     std::size_t    index;
    //     const CameraParams*  cameraParams;
    // };

    // CameraBond& getCameraBondByPortNumber(int portNumber);

    // void reset();

    // void safeCall(std::function<void()> call);

    // std::vector<CameraBond>             m_cameraBonds;
    // std::vector<camera_vals>    m_lastCameraInfo;

    CameraContainer    m_cameras;
    bool               m_applyTimeCorrection;
    int                m_frameRate;
    bool               m_monitorTimeOffset;
    // message::PpsStatus m_lastPpsStatus;
    // std::chrono::nanoseconds m_lastFwHostClockOffset;

    
    IDevice::SharedPtr           m_triggerDevice;
    // rclcpp::Duration m_offset;
    // std::mutex                   m_diagnosticsMutex;
    std::chrono::milliseconds    m_maxFirmwareQueryPeriod;

    std::string    m_firmwareSerialDevice;
    std::string    m_firmwareLogFilePath;
    // FirmwareLogger m_firmwareLogger;
};

