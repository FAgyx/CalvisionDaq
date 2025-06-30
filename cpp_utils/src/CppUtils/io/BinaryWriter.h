#pragma once

#include <cstdint>
#include <string>

class BinaryWriter {
public:
    template <typename T>
    void write(const T& t) {
        write<T, 1>(&t);
    }

    void write_string(const std::string& s) {
        write<char>((char*) s.data(), (size_t) s.length());
    }

    template <typename T>
    void write(const T* buffer, size_t N) {
        this->write_impl((const uint8_t*) buffer, N * sizeof(T));
    }

    template <typename T, size_t N>
    void write(const T* buffer) {
        write(buffer, N);
    }

    template <typename T>
    void write_buffer(const T& buffer) {
        write(buffer.data(), buffer.size());
    }

protected:
    virtual void write_impl(const uint8_t* buffer, size_t N) = 0;
};


template <typename T>
class BinaryWriterTemplate : public T, public BinaryWriter {
public:
    template <typename... Args>
    BinaryWriterTemplate(Args&&... args)
        : T(std::forward<Args>(args)...), BinaryWriter()
    {}

    virtual ~BinaryWriterTemplate() {}

protected:
    virtual void write_impl(const uint8_t* buffer, size_t N) override {
        T::_write(buffer, N);
    }
};


