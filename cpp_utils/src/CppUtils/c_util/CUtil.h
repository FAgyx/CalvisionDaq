#pragma once

#include <cstring>
#include <cstdint>
#include <type_traits>

template <typename T, typename U>
constexpr bool raw_buffers_aligned(size_t N_Ts, size_t N_Us) {
    return N_Ts * sizeof(T) == N_Us * sizeof(U);
}

/*
 * Helper to initialize C-structs to zero.
 */
template <typename T>
void initialize_zero(T& t) {
    std::memset((void*) &t, 0, sizeof(T));
}

template <typename T>
void zero_buffer(T* t, size_t N) {
    std::memset((void*) t, 0, N * sizeof(T));
}

template <size_t N, typename T>
void zero_buffer(T* t) {
    zero_buffer(t, N);
}

template <typename T>
void copy_buffer(T* destination, const T* source, size_t N) {
    copy_raw_buffer(destination, source, N);
}

template <typename T, typename U>
void copy_raw_buffer(T* destination, const U* source, size_t N) {
    std::memcpy((void*) destination, (void*) source, N * sizeof(U));
}

template <typename DestType, typename SourceType>
constexpr size_t num_reinterpreted(size_t N) {
    return (sizeof(SourceType) * N) / sizeof(DestType);
}


template <typename T>
constexpr size_t sizeof_bits() {
    return (sizeof(T) * 8) / sizeof(uint8_t);
}

template <size_t N_Bits>
using ContainingUintType =
    std::conditional_t<N_Bits <= 8, uint8_t,
    std::conditional_t<N_Bits <= 16, uint16_t,
    std::conditional_t<N_Bits <= 32, uint32_t,
    std::conditional_t<N_Bits <= 64, uint64_t,
    void > > > >;

template <size_t N_Bits, typename SourceType>
constexpr bool narrowed_type_fits(SourceType s) {
    if (sizeof_bits<SourceType>() <= N_Bits) return true;

    uint8_t* us = reinterpret_cast<uint8_t*>(&s);
    constexpr size_t length = num_reinterpreted<uint8_t, SourceType>(1);
    constexpr size_t index = N_Bits / sizeof_bits<uint8_t>();
    constexpr size_t offset = N_Bits % sizeof_bits<uint8_t>();

    uint8_t max_value = 1 << offset;
    if (us[index] >= max_value) return false;
    for (size_t i = index + 1; i < length; i++) {
        if (us[i] != 0) return false;
    }
    return true;
};

template <typename DestType, typename SourceType>
DestType binary_cast(const SourceType& source) {
    static_assert(sizeof(DestType) == sizeof(SourceType));
    DestType result;
    copy_raw_buffer(&result, &source, 1);
    return result;
}


