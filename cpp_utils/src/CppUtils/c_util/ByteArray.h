#pragma once

#include "BitManip.h"

#include "CppUtils/container/ArrayView.h"

#include <iostream>


enum class Endianness {
    Little,
    Big,
};


template <Endianness endian, size_t N_bytes>
class ByteArray {
public:
    using Self = ByteArray<endian, N_bytes>;

    constexpr ByteArray(uint8_t* data)
        : data_(data)
    {}

    constexpr ByteArray(const ArrayView<uint8_t, N_bytes>& data)
        : data_(data)
    {}

    uint8_t& operator[](size_t i) {
        return data_[index(i)];
    }

    constexpr uint8_t operator[](size_t i) const {
        return data_[index(i)];
    }

    constexpr static size_t size() { return N_bytes; }

    constexpr static bool empty() { return N_bytes == 0; }

    uint8_t& lsb() {
        return data_[index(0)];
    }

    constexpr uint8_t lsb() const {
        return data_[index(0)];
    }

    uint8_t& msb() {
        return data_[index(N_bytes-1)];
    }

    constexpr uint8_t msb() const {
        return data_[index(N_bytes-1)];
    }

    friend std::ostream& operator<<(std::ostream& out, const Self& self) {
        std::ios_base::fmtflags flags(out.flags());

        out << std::hex << "0x";
        for (size_t i = 0; i < N_bytes; i++) {
            out << static_cast<unsigned>(self[N_bytes-1-i]);
        }

        out.flags(flags);
        return out;
    }

private:
    constexpr size_t index(size_t i) const {
        switch(endian) {
            case Endianness::Little:
                return i;
            case Endianness::Big:
                return (N_bytes - 1) - i;
        }
    }

    ArrayView<uint8_t, N_bytes> data_;
};

template <Endianness endian, size_t N, typename T, typename U>
ByteArray<endian, sizeof(U)> make_byte_array(ArrayView<T, N> buffer, U value) {
    static_assert(sizeof(T) * N == sizeof(U));
    ByteArray<endian, sizeof(U)> bytes((uint8_t*) buffer.data());
    T lsb_mask = interval_mask<bit_sizeof<T>()-n_bits_per_byte, n_bits_per_byte,0,T>();
    for (size_t i = 0; i < sizeof(U); i++) {
        bytes[i] = static_cast<uint8_t>(lsb_mask & value);
        value = value >> n_bits_per_byte;
    }
    return bytes;
}


