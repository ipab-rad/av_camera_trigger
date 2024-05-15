#pragma once

#include <string>
#include <vector>
#include <algorithm>
#include <stdexcept>
#include <chrono>
#include <map>

#define FIRMWARE_MINIMUM_REQUIRED "2.1.0"

namespace constants
{
    // FW specific
    // TODO: move these into the FW
    constexpr uint32_t PPS_COUNTER_FREQUENCY = 108e+6;

    // host specific
    // TODO: move these into the host
    const int LOOP_RATE = 120; // loop rate for ROS node
    constexpr std::chrono::milliseconds MAX_CLOCKS_OFFSET{20};

    // shared between FW and host
    constexpr uint16_t VENDOR_ID  = 0x1234;
    constexpr uint16_t PRODUCT_ID = 0x0006;
    constexpr int MAX_CHAR_COUNT_CAMERA_ALIAS = 16;
} //namespace Constants

namespace type
{
    using Tick = std::chrono::duration<std::int64_t, std::ratio<1, constants::PPS_COUNTER_FREQUENCY>>;
}

namespace message
{
    enum class Query : uint8_t
    {
        // DO NOT forget to update isValid and toString functions
        SetDeviceTime     = 0x04,
        GetDeviceTime     = 0x08,
        GetCameraInfo     = 0x0B,
        AttachCamera      = 0x44,
        DetachCameras     = 0x46,
        Unknown           = 0xB4,
        GetVersion        = 0xB8, // this must not be changed for compatibility issues
        GetPpsStatus      = 0xBB,
        StartCameras      = 0x01,
        StopCameras       = 0x02,
        PersistParameter  = 0x05,
        RetrieveParameter = 0x06,
        Reset             = 0x07
        // DO NOT forget to update isValid and toString functions
    };

    template <typename> inline bool isValid(int);
    template <typename T> inline T castTo(int);

    template <>
    inline bool isValid<Query>(int query)
    {
        static std::vector<Query> all =
        {
            Query::SetDeviceTime,
            Query::GetDeviceTime,
            Query::GetCameraInfo,
            Query::AttachCamera,
            Query::DetachCameras,
            Query::Unknown,
            Query::GetVersion,
            Query::GetPpsStatus,
            Query::StartCameras,
            Query::StopCameras,
            Query::PersistParameter,
            Query::RetrieveParameter,
            Query::Reset
        };
        return std::any_of(all.cbegin(), all.cend(),
                           [query](Query q){return query == static_cast<int>(q);});
    }

    inline const std::string& toString(Query q)
    {
        static std::map<Query, std::string> m =
        {
            {Query::SetDeviceTime,      "SetDeviceTime"},
            {Query::GetDeviceTime,      "GetDeviceTime"},
            {Query::GetCameraInfo,      "GetCameraInfo"},
            {Query::AttachCamera,       "AttachCamera"},
            {Query::DetachCameras,      "DetachCameras"},
            {Query::Unknown,            "Unknown"},
            {Query::GetVersion,         "GetVersion"},
            {Query::GetPpsStatus,       "GetPpsStatus"},
            {Query::StartCameras,       "StartCameras"},
            {Query::StopCameras,        "StopCameras"},
            {Query::PersistParameter,   "PersistParameter"},
            {Query::RetrieveParameter,  "RetrieveParameter"},
            {Query::Reset,              "Reset"}
        };
        return m.at(q);
    }

    enum class PpsStatus
    {
        UNKNOWN,
        ACQUIRED,
        LOST
    };

    template <>
    inline bool isValid<PpsStatus>(int status)
    {
        static std::vector<PpsStatus> all =
        {
            PpsStatus::UNKNOWN,
            PpsStatus::ACQUIRED,
            PpsStatus::LOST
        };
        return std::any_of(all.cbegin(), all.cend(),
                           [status](PpsStatus st){return status == static_cast<int>(st);});
    }

    template <>
    inline PpsStatus castTo<PpsStatus>(int val)
    {
        if (! isValid<PpsStatus>(val))
        {
            throw std::runtime_error{"Invalid cast of value " + std::to_string(val) + " to PpsStatus"};
        }
        return static_cast<PpsStatus>(val);
    }

    inline const std::string& toString(PpsStatus status)
    {
        static std::vector<std::string> arr =
        {
            "UNKNOWN",
            "ACQUIRED",
            "LOST"
        };
        return arr[static_cast<int>(status)];
    }

    constexpr uint8_t Status0 = 6;
    constexpr uint8_t Status1 = 7;
    constexpr uint8_t StatusMask = ( 1 << Status0 ) | ( 1 << Status1 );
    constexpr uint8_t Error = (1 << Status0 );
    constexpr uint8_t OK = (1 << Status1);

    constexpr uint8_t Counter0 = 4;
    constexpr uint8_t Counter1 = 5;
    constexpr uint8_t CounterMask = ( 1 << Counter0) | ( 1 << Counter1 );

    constexpr uint32_t TimerMask = ~(0xF << 28);

    constexpr int reportLength = 64;
    constexpr int queryLength = 64;
    constexpr int numPorts = 14;
    constexpr std::uint32_t HID_REPORT_MAX_SIZE_BYTES = 3 * reportLength;

    const std::string STATE_NODEVICE="No trigger device available";
    const std::string STATE_OK="OK";
    const std::string STATE_ERROR="Error";

} //namespace Message
