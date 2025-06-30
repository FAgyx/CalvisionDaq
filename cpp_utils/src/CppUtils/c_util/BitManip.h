#pragma once

#include "CUtil.h"

constexpr static size_t n_bits_per_byte = 8;

template <typename T>
constexpr size_t bit_sizeof() {
    return sizeof(T) * n_bits_per_byte;
}

template <size_t left, size_t middle, size_t right, typename T>
constexpr T interval_mask() {
    static_assert(left + middle + right == bit_sizeof<T>());
    if constexpr (middle == bit_sizeof<T>()) {
        return ~T{0};
    } else {
        return ((T{0b1} << middle) - 1) << right;
    }
}

/*
 * Left justifies the first N-bits of T.
 */
template <size_t N_bits, typename T>
constexpr T left_justify(T value) {
    static_assert(N_bits <= bit_sizeof<T>());
    return value << (bit_sizeof<T>() - N_bits);
}

/*
 * Right justifies the last N-bits of T.
 */
template <size_t N_bits, typename T>
constexpr T right_justify(T value) {
    static_assert(N_bits <= bit_sizeof<T>());
    return value >> (bit_sizeof<T>() - N_bits);
}
