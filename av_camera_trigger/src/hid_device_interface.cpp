#include "stdio.h"
#include "stdint.h"
#include "ctime"
#include <unistd.h>
#include <sstream>
#include <assert.h>

#include <rclcpp/logging.hpp>

#include "av_camera_trigger/hid_device_interface.hpp"

HIDDevice::HIDDevice(uint16_t vendor_id, uint16_t product_id) :
    m_handle{nullptr},
    m_vendor_id(vendor_id),
    m_product_id(product_id)
{
}

HIDDevice::~HIDDevice()
{
    disconnect();
}

void HIDDevice::disconnect()
{
    if (m_handle)
    {
        // close HID device
        hid_close(m_handle);

        // Free static HIDAPI objects.
        hid_exit();

        m_handle = nullptr;
    }
}

bool HIDDevice::connect()
{
    if (isConnected())
    {
        return true;
    }

    RCLCPP_INFO_STREAM(rclcpp::get_logger("hid_device_interface"), "Connecting to HID device identified by " <<
                    "vendor id " << std::hex << m_vendor_id << " and " <<
                    "product id " << std::hex << m_product_id
                    << std::dec);
    m_handle = hid_open(m_vendor_id, m_product_id, nullptr);

    return isConnected();
}

bool HIDDevice::connect(uint16_t vid, uint16_t pid)
{
    m_vendor_id = vid;
    m_product_id = pid;

    return connect();
}

bool HIDDevice::isConnected() const
{
    return m_handle != nullptr;
}

std::vector<uint8_t> HIDDevice::readFromDevice(std::chrono::milliseconds timeout)
{
    std::vector<uint8_t> buf;

    int length = 64;
    int totalBytesRead = 0;
    int attemptsRemaining = 5;

    while (true)
    {
        buf.resize(totalBytesRead + 64);
        const auto bytesRead = ::hid_read_timeout(m_handle, buf.data() + totalBytesRead,
                                                  message::reportLength, timeout.count());

        if (bytesRead == 0)
        {
            if (attemptsRemaining == 0)
            {
                break;
            }
        }
        else if (bytesRead < 0)
        {
            // Read error, device isn't connected
            disconnect();
            throw except::BrokenChannel{"Failed to read " + std::to_string(message::reportLength) +
                                        " bytes from the HID. Device disconnected?"};
        }

        totalBytesRead += bytesRead;

        if(bytesRead > 0 && bytesRead <= 64)
        {
            length = buf[1];
            if (buf[0] == static_cast<uint8_t>(message::Query::GetVersion) && bytesRead < 8)
            {
                // old message format, manually shift
                buf[7] = buf[4];
                buf[6] = buf[3];
                buf[5] = buf[2];
                buf[4] = buf[1];

                buf.resize(8);
                return buf;
            }
        }

        if (totalBytesRead >= length)
        {
            break;
        }

        --attemptsRemaining;
    }

    buf.resize(totalBytesRead);
    return buf;
}

void HIDDevice::writeToDevice(const std::vector<uint8_t>& buf)
{
    const int bytesWritten = ::hid_write(m_handle, buf.data(), message::queryLength);
    if (bytesWritten <= 0)
    {
        //Error, device isn't connected anymore.
        disconnect();
        throw except::BrokenChannel{"Could not write " + std::to_string(buf.size()) +
                                    " bytes to the HID. Device diconnected?"};
    }
}

void HIDDevice::queryDevice(message::Query query, std::vector<uint8_t>& result,
                           const std::vector<uint8_t>& args)
{
    const auto totalQueryLength = args.size() + 2; //account for the first two bytes
    if (totalQueryLength > message::queryLength)
    {
        throw except::Logic{"Query too long " + std::to_string(totalQueryLength)};
    }

    std::vector<uint8_t> buffer;
    buffer.reserve(message::queryLength);

    // NB: see description of hid_write in https://github.com/libusb/hidapi/blob/master/hidapi/hidapi.h
    //     to understand why the first byte must be 0x0
    buffer.push_back(0x0);
    buffer.push_back(static_cast<uint8_t>(query));

    for(const auto& arg: args)
    {
        buffer.push_back(arg);
    }

    // Ensure the buffer size is precisely queryLength.
    buffer.resize(message::queryLength);

    writeToDevice(buffer);

    constexpr std::chrono::milliseconds TIMEOUT{10};
    for(int i=0; i< MESSAGE_ATTEMPTS; i++)
    {
        result = readFromDevice(TIMEOUT);

        if (result[0] == static_cast<uint8_t>(query))
        {
            return;
        }
        else if (result[0] == static_cast<uint8_t>(message::Query::Unknown)) //query not recognised
        {
            throw except::Logic{"Query " + message::toString(query) + " not recognised by firmware"};
        }
    }

    throw except::BrokenChannel{"Firmware did not repond to query " + message::toString(query) +
                                " even after " + std::to_string(MESSAGE_ATTEMPTS) + " attempts, each taking " +
                                std::to_string(TIMEOUT.count()) + " ms"};
}


