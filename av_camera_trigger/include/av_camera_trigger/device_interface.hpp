#pragma once

#include <memory>
#include <stdint.h>
#include <string>
#include <sstream>
#include <vector>
#include <cmath>
#include <chrono>

#include "av_camera_trigger/camera_trigger_messaging.h"

#ifdef major
#undef major
#endif

#ifdef minor
#undef minor
#endif

struct version
{
    int major;
    int minor;
    int patch;
    int hardware_rev;

    version(int major, int minor, int patch, int hardware_rev) :
        major(major),
        minor(minor),
        patch(patch),
        hardware_rev(hardware_rev) {};

    version(std::string versionStr)
    {
        std::sscanf(versionStr.c_str(), "%d.%d.%d", &major, &minor, &patch);
        hardware_rev = 0;
    };

    std::string toString() const
    {
        std::stringstream ss;
        ss << major << "." << minor << "." << patch;
        return ss.str();
    };
};

struct camera_vals
{
    std::string state;
    uint8_t counter;
    type::Tick start_timestamp;
    uint32_t start_timestamp_sec;
    type::Tick end_timestamp;
    uint32_t end_timestamp_sec;
    int port_number;

    camera_vals() :
        state(""),
        counter(0),
        start_timestamp(0),
        start_timestamp_sec(0),
        end_timestamp(0),
        end_timestamp_sec(0),
        port_number{-1}
    {}

    camera_vals(std::string state,
                uint8_t counter,
                uint32_t start_timestamp,
                uint32_t start_timestamp_sec,
                uint32_t end_timestamp,
                uint32_t end_timestamp_sec,
                int port_number) :
        state(state),
        counter(counter),
        start_timestamp(start_timestamp),
        start_timestamp_sec(start_timestamp_sec),
        end_timestamp(end_timestamp),
        end_timestamp_sec(end_timestamp_sec),
        port_number{port_number}
    {}

};

union CameraReport
{
    uint32_t data[3];
    struct
    {
        uint32_t startMessage;
        uint32_t endMessage;
        uint32_t seconds;
    };
};

/**
 * @brief Interface for camera trigger device.
 *
 * @detailed provides both lower level read and write operations and higher level queries
 */
class IDevice
{
protected:
    const int MESSAGE_ATTEMPTS = 5;

public:
    using SharedPtr = std::shared_ptr<IDevice>;

    /**
     * @brief Attempts to connect to the firmware if not already connected.
     * @return true if successful, false otherwise
     */
    virtual bool connect() = 0;

    /**
      * Disconnects from the firmware if an active connection exists,
      * otherwise does nothing.
      */
    virtual void disconnect() = 0;

    /**
     * @brief Check whether device is connected
     *
     * @return true if connected, false otherwise
     */
    virtual bool isConnected() const = 0;

    /**
     * @brief read from the device
     *
     * @param timeout   Number of milliseconds to wait if no data is available
     *
     * @return vector of bytes read, empty if an error occurred
     */
    virtual std::vector<uint8_t> readFromDevice(std::chrono::milliseconds timeout = std::chrono::milliseconds{3}) = 0;

    /**
     * @brief write to the device
     *
     * @param buf   buffer containing data to write to device
     */

    virtual void writeToDevice(const std::vector<uint8_t>& buf) = 0;

    /**
     * @brief write a query to the device and check for a response
     * @param query     query to write
     *
     */
    void queryDevice(message::Query query);

    /**
     * @brief write an argumentless query to the device and retrieve a result
     * @param query     query to write
     * @param result    buffer for device response
     */
    void queryDevice(message::Query query, std::vector<uint8_t>& result)
    {
        return queryDevice(query, result, {});
    }

    /**
     * @brief write a query with arguments to the device and retrieve a result
     * @param query     query to write
     * @param result    buffer for device response
     * @param args      query arguments
     *
     */
    virtual void queryDevice(message::Query query, std::vector<uint8_t>& result,
                            const std::vector<uint8_t>& args) = 0;

    /**
     * @brief Set the device time
     * @param time  unix time in seconds
     */
    void setDeviceTime(time_t time);

    /**
     * @brief Get the current device UTC time which should, in theory, coincide
     * with the time of the machine hosting this ROS node.
     *
     * @return time in nanoseconds if successful, otherwise throws
     *         @class std::logic_error
     */
    std::chrono::nanoseconds getDeviceTime();

    /**
     * @brief Get the most recent PPS measurement from the device
     *
     * @return time in seconds if successful, -1 if there was an error,
     *         -2 if the query succeeds but the device doesn't recognise it
     */
    message::PpsStatus getPpsStatus();

    /**
     * @brief Get the camera information reports
     * (start time and end time inc seconds and nanoseconds, counter and status for each camera)
     *
     * @param buf   buffer for returned data
     */
    void getCameraInfo(std::vector<uint8_t>& buf);

    /**
     * @brief Get the camera info reports
     * (start time and end time inc seconds and nanoseconds, counter and status for each camera)
     *
     * @return Reports as a vector of camera_vals (empty if query failed)
     */

    std::vector<camera_vals> getCameraInfo();

    /**
     * @brief Add a camera to a specified port on the device
     *
     * @param framerate  required framerate
     * @param port       port on which to add the camera
     */
    void attachCamera(uint8_t framerate, uint8_t port, const std::string& alias);
    void detachCameras();

    /**
     * @brief Get the firmware version from the device
     *
     * @return firmware version, all -1s if error
     */
    version getVersion();

    /**
     * Tell the firmware to start/stop triggering the active cameras.
     */
    void startCameras();
    void stopCameras();

    /**
     * The @param cmd consists of two strings separated by delimiter which
     * is currently set to =. The first string represents the name of the
     * parameter whilst the right hand side one designates its value.
     */
    void persistParameter(const std::string& cmd);

    /**
     * Interogates the firmware for the string representation of the
     * parameter identified by @param name.
     *
     * Returns the string representation of the requested value.
     */
    std::string retrieveParameter(const std::string& name);

    /**
     * Same as the non-template version and additionally it casts
     * the string representation to the type specified by the template
     * parameter.
     *
     * Throws if the conversion fails.
     */
    template <typename T>
    T retrieveParameter(const std::string& name)
    {
        const std::string str = retrieveParameter(name);
        std::istringstream ss{str};
        ss.exceptions(std::ios_base::failbit);
        T val{};
        ss >> std::boolalpha >> val;
        return val;
    }

    void reset();
};
