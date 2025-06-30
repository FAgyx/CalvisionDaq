#pragma once

#include "BinaryReader.h"
#include "BinaryWriter.h"

template <typename T>
class BinaryReaderWriterTemplate : public T, public BinaryReader, public BinaryWriter {
public:
    template <typename... Args>
    BinaryReaderWriterTemplate(Args&&... args)
        : T(std::forward<Args>(args)...), BinaryWriter(), BinaryReader()
    {}

    virtual ~BinaryReaderWriterTemplate() {}

protected:
    virtual void write_impl(const uint8_t* buffer, size_t N) override {
        T::_write(buffer, N);
    }

    virtual void read_impl(uint8_t* buffer, size_t N) override {
        T::_read(buffer, N);
    }

    virtual size_t var_read_impl(uint8_t* buffer, size_t N) override {
        return T::_var_read(buffer, N);
    }
};
