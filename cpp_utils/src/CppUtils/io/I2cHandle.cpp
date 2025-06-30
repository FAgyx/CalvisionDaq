#include "I2cHandle.h"

#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

I2cHandle::I2cHandle()
    : DeviceHandle()
{}

I2cHandle::I2cHandle(uint8_t bus_id, uint8_t dev_id, OpenMode mode) 
    : DeviceHandle()
{
    open(bus_id, dev_id, mode);
}

I2cHandle::~I2cHandle()
{}

void I2cHandle::open(uint8_t bus_id, uint8_t dev_id, OpenMode mode) {
    DeviceHandle::open("/dev/i2c-" + std::to_string(bus_id), mode);
    if (ioctl(fd_, I2C_SLAVE, dev_id) == -1) {
        throw std::runtime_error("Couldn't open I2C Bus " + std::to_string(bus_id) + " device " + std::to_string(dev_id));
    }
}
