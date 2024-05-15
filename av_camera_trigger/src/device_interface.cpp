
#include "av_camera_trigger/device_interface.hpp"
#include "av_camera_trigger/camera_trigger_messaging.h"
#include "av_camera_trigger/Exceptions.hpp"

#include <rclcpp/logging.hpp>

#include <ratio>
#include <iostream>

void IDevice::setDeviceTime(time_t seconds)
{
    RCLCPP_INFO_STREAM(rclcpp::get_logger("device_interface"), "Sending set device time (" << std::to_string(seconds) << " sec) request to the firmware...");
    std::vector<uint8_t> args(4);

    args[0] = seconds & 0xff;
    args[1] = seconds >> 8 & 0xff;
    args[2] = seconds >> 16 & 0xff;
    args[3] = seconds >> 24 & 0xff;

    std::vector<uint8_t> result(message::reportLength);

    queryDevice(message::Query::SetDeviceTime, result, args);
}

std::chrono::nanoseconds IDevice::getDeviceTime()
{
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("device_interface"), "Sending get device time request to the firmware...");
    std::vector<uint8_t> result(message::reportLength);

    queryDevice(message::Query::GetDeviceTime, result);

    using T = std::uint32_t;
    if (result.size() != 3 * sizeof(T))
    {
        throw except::Logic{"Failed to retrieve time from firmware due to response being too short " + std::to_string(result.size())};
    }

    const std::uint32_t* cameraReport = reinterpret_cast<const T*>(result.data());
    const std::chrono::seconds seconds{cameraReport[1]};
    const std::chrono::nanoseconds nanoseconds{cameraReport[2]};

    return seconds + nanoseconds;
}

message::PpsStatus IDevice::getPpsStatus()
{
    using namespace message;

    std::vector<uint8_t> result;

    queryDevice(Query::GetPpsStatus, result);

    using T = std::uint32_t;
    if (result.size() != 2 * sizeof(T))
    {
        throw except::Logic{"Failed to retrieve PPS status from firmware due to response being too short " + std::to_string(result.size())};
    }

    const T* cameraReport = reinterpret_cast<const T*>(result.data());
    const PpsStatus status = castTo<PpsStatus>(cameraReport[1]);

    return status;
}

void IDevice::getCameraInfo(std::vector<uint8_t>& buf)
{
    queryDevice(message::Query::GetCameraInfo, buf);
}

std::vector<camera_vals> IDevice::getCameraInfo()
{

    std::vector<uint8_t> buf;
    std::vector<camera_vals> result;
    result.reserve(message::numPorts);

    queryDevice(message::Query::GetCameraInfo, buf);

    const auto actual = buf.size();
    const auto expected = message::HID_REPORT_MAX_SIZE_BYTES;
    if (actual != expected)
    {
        throw except::BrokenChannel{"Incomplete HID camera info report. Expected size " +
                                    std::to_string(expected) + ", actual " + std::to_string(actual)};
    }

    const CameraReport* cameraReport = reinterpret_cast<const CameraReport*>(&buf.data()[8]);

    for (int i = 0; i < message::numPorts; i++)
    {
        const uint32_t camera_start_info = cameraReport[i].startMessage;
        const uint32_t camera_end_info   = cameraReport[i].endMessage;

        std::string state;
        const auto status = ((camera_start_info >> 24) & message::StatusMask);
        switch (status)
        {
        case message::OK:    state = message::STATE_OK; break;
        case message::Error: state = message::STATE_ERROR; break;
        case message::StatusMask:
        {
            // Both bits are set. This is impossible and it can only happen
            // if the firmware is bogus.
            throw except::Logic{"Camera wired to connector " + std::to_string(i) +
                                   " has been found in an undefined state. Most certainly this is"
                                   " an indication of either firmware bug or communication error "
                                   "- see RAV-562."};
        }
        break;
        default:
        {
            // Neither bit is set. This is a legitimate case and means the camera
            // info is not available due to the physical camera not being connected.
            continue;
        }
        }
        const uint8_t start_counter = (camera_start_info >> 24 & message::CounterMask) >> 4;
        const uint8_t end_counter   = (camera_end_info >> 24 & message::CounterMask) >> 4;
        const uint32_t start_timestamp = camera_start_info & message::TimerMask;
        const uint32_t end_timestamp   = camera_end_info & message::TimerMask;
        const uint32_t timestamp_sec   = cameraReport[i].seconds;

        if (end_counter != start_counter)
        {
            const auto msg = "The counters corresponding to start/end exposure times for camera connected to port " +
                              std::to_string(i) + " do not match: startCounter=" + std::to_string(start_counter) +
                              " endCounter=" + std::to_string(end_counter) +
                              " . Most certainly this is an indication of either firmware bug or communication"
                              " error which at the moment cannot be detected - see RAV-562.";
            throw except::Logic{msg};
        }

        result.push_back({state, start_counter,
                          start_timestamp, timestamp_sec,
                          end_timestamp, timestamp_sec,
                          i});
    }

    return result;
}

void IDevice::attachCamera(uint8_t framerate, uint8_t port, const std::string& alias)
{
    RCLCPP_INFO_STREAM(rclcpp::get_logger("device_interface"), "Sending attach camera " << alias << " request to the firmware...");
    std::vector<uint8_t> args = {framerate, port};
    // truncate the string to MAX_CHAR_COUNT_CAMERA_ALIAS chars
    const auto charCount = std::min(alias.size(), static_cast<std::size_t>(constants::MAX_CHAR_COUNT_CAMERA_ALIAS));
    args.push_back(charCount);
    std::copy(alias.begin(), alias.begin() + charCount, std::back_inserter(args));

    std::vector<uint8_t> result(message::reportLength);

    queryDevice(message::Query::AttachCamera, result, args);
}

void IDevice::detachCameras()
{
    queryDevice(message::Query::DetachCameras);
}

void IDevice::startCameras()
{
    queryDevice(message::Query::StartCameras);
}
void IDevice::stopCameras()
{
    queryDevice(message::Query::StopCameras);
}

void IDevice::persistParameter(const std::string& cmd)
{
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("device_interface"), "Sending parameter persistence request to the firmware..." << cmd);
    std::vector<uint8_t> args;
    args.push_back(cmd.size());
    std::copy(cmd.begin(), cmd.end(), std::back_inserter(args));

    std::vector<uint8_t> result;

    queryDevice(message::Query::PersistParameter, result, args);
}

std::string IDevice::retrieveParameter(const std::string& name)
{
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("device_interface"), "Sending parameter " << name << " retrieval request to the firmware...");
    std::vector<uint8_t> args;
    args.push_back(name.size());
    std::copy(name.begin(), name.end(), std::back_inserter(args));

    std::vector<uint8_t> result;

    queryDevice(message::Query::RetrieveParameter, result, args);

    const int byteCount = result[4];
    const std::string str{&result[5], &result[5 + byteCount]};

    return str;
}

version IDevice::getVersion()
{
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("device_interface"), "Sending version request to the firmware");
    std::vector<uint8_t> result(message::reportLength);

    queryDevice(message::Query::GetVersion, result);
    return version(result[4], result[5], result[6], result[7]);
}

void IDevice::reset()
{
    // Use INFO level such that we always record this message
    RCLCPP_INFO_STREAM(rclcpp::get_logger("device_interface"), "Sending reset request to the firmware");
    queryDevice(message::Query::Reset);
}

void IDevice::queryDevice(message::Query query)
{
    std::vector<uint8_t> result;
    queryDevice(query, result);
}
