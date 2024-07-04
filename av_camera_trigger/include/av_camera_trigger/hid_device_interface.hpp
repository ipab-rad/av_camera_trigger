#pragma once

#include "hidapi/hidapi.h"
#include "device_interface.hpp"
#include "camera_trigger_messaging.hpp"
#include "Exceptions.hpp"

#include <chrono>

class HIDDevice : public IDevice
{
public:
    HIDDevice(uint16_t vid, uint16_t pid);
    ~HIDDevice();

    bool connect() override;
    bool connect(uint16_t, uint16_t);
    void disconnect() override;

    bool isConnected() const override;

    std::vector<uint8_t> readFromDevice(std::chrono::milliseconds timeout) override;

    void writeToDevice(const std::vector<uint8_t>& buf) override;

    void queryDevice(message::Query query, std::vector<uint8_t>& result,
                    const std::vector<uint8_t>& args) override;

protected:
    hid_device* m_handle;

private:
    uint16_t m_vendor_id;
    uint16_t m_product_id;
};
