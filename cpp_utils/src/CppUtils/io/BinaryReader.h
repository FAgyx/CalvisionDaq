#pragma once

#include <cstdint>
#include <string>

class BinaryReader {
public:
    // ----- fixed length read
    // fails if the specified length of data is not read.

    template <typename T>
    void read(T& t) {
        read<T, 1>(&t);
    }

    template <typename T, size_t N>
    void read(T* buffer) {
        read<T>(buffer, N);
    }

    template <typename T>
    void read(T* buffer, size_t N) {
        this->read_impl((uint8_t*) buffer, sizeof(T) * N);
    }

    template <typename T>
    void read_buffer(T& buffer) {
        read(buffer.data(), buffer.size());
    }

    template <typename T>
    std::string read_string(T& buffer) {
        read_buffer(buffer);
        return std::string(buffer.data(), buffer.size());
    }

    // ----- variable length reading
    // returns the length of data read; may be smaller than buffer size

    template <typename T, size_t N>
    size_t var_read(T* buffer) {
        return var_read(buffer, N);
    }

    template <typename T>
    size_t var_read(T* buffer, size_t N) {
        return this->var_read_impl((uint8_t*) buffer, sizeof(T) * N);
    }

    template <typename T>
    size_t var_read_buffer(T& buffer) {
        return var_read(buffer.data(), buffer.size());
    }

protected:
    virtual void read_impl(uint8_t* buffer, size_t N) = 0;
    virtual size_t var_read_impl(uint8_t* buffer, size_t N) = 0;
};


template <typename T>
class BinaryReaderTemplate : public T, public BinaryReader {
public:
    template <typename... Args>
    BinaryReaderTemplate(Args&&... args)
        : T(std::forward<Args>(args)...), BinaryReader()
    {}

    virtual ~BinaryReaderTemplate() {}

protected:
    virtual void read_impl(uint8_t* buffer, size_t N) override {
        T::_read(buffer, N);
    }

    virtual size_t var_read_impl(uint8_t* buffer, size_t N) override {
        return T::_var_read(buffer, N);
    }
};


