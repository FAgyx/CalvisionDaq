#pragma once

#include "DeviceHandle.h"

class I2cHandle : public DeviceHandle {
public:
    I2cHandle();
    I2cHandle(uint8_t bus_id, uint8_t dev_id, OpenMode mode);

    virtual ~I2cHandle();

protected:
    void open(uint8_t bus_id, uint8_t dev_id, OpenMode mode);
};

using I2cReader = BinaryReaderTemplate<I2cHandle>;
using I2cWriter = BinaryWriterTemplate<I2cHandle>;
using I2cReaderWriter = BinaryReaderWriterTemplate<I2cHandle>;
