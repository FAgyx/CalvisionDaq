#pragma once

#include "ByteArray.h"

constexpr size_t containing_size_bytes(size_t N_bits) {
    return (N_bits / n_bits_per_byte) + static_cast<size_t>((N_bits % n_bits_per_byte) > 0);
}

template <typename T>
void print_bits(T value);

/*
 * Handles an N-bit data type. Provides bitwise access, as well as
 * simple operations on the type as an N-bit unsigned / signed integer.
 *
 * The underlying data is stored in a buffer which is endian independent.
 * Bits in the BitArray are indexed in the canonical bit-string format
 * as though they were a fundamental integer type, e.g.
 *
 * 12-bit array:
 * x x x x 0 1 0 1     1 1 0 1 1 0 1 0 0 1
 *        11  <- 8     7              <- 0
 *
 * Padding bits are reserved and shouldn't be used. Data is always
 * assumed to be right-justified (padding bits are to the left).
 */
template <Endianness endian, size_t N_bits>
class BitArray {
public:
    using Self = BitArray<endian, N_bits>;

    constexpr static size_t N_bytes = containing_size_bytes(N_bits);
    constexpr static size_t N_padding = n_bits_per_byte * N_bytes - N_bits;
    constexpr static uint8_t padding_mask = interval_mask<0, N_padding, n_bits_per_byte - N_padding, uint8_t>();

    BitArray(uint8_t* data)
        : data_(data)
    {}

    BitArray(const ByteArray<endian, N_bytes>& data)
        : data_(data)
    {}
    
    constexpr bool operator[](size_t i) const {
        return get_bit(i);
    }

    constexpr bool get_bit(size_t i) const {
        return (data_[byte_index(i)] & bit_flag(i)) != 0;
    }

    void set_bit_on(size_t i) {
        data_[byte_index(i)] |= bit_flag(i);
    }

    void set_bit_off(size_t i) {
        data_[byte_index(i)] &= ~bit_flag(i);
    }

    void set_bit(size_t i, bool value) {
        if (value)  set_bit_on (i);
        else        set_bit_off(i);
    }

    constexpr static size_t size() { return N_bits; }

    /*
     * Convert the bit array to a fundamental integral type represented by the
     * same number of bits.
     */
    template <typename T>
    T convert() const {
        static_assert(sizeof(T) >= data_.size());
        T result = 0;
        if (!data_.empty()) {
            if (std::is_signed_v<T> && get_bit(N_bits - 1)) {
                constexpr size_t N_msb_bits = n_bits_per_byte - N_padding;
                result = interval_mask<0, bit_sizeof<T>() - N_msb_bits, N_msb_bits, T>();
                result |= static_cast<T>(padding_mask | data_.msb());
            } else {
                result |= static_cast<T>(~padding_mask & data_.msb());
            }
        }
        for (size_t i = 1; i < data_.size(); i++) {
            result = result << n_bits_per_byte;
            result |= static_cast<T>(data_[data_.size() - 1 - i]);
        }
        
        return result;
    }

    void twos_compliment() {
        if (data_.size() == 0)
            return;

        data_.lsb() = (~data_.lsb()) + 1;
        bool carry_bit = data_.lsb() == 0;
        for (size_t i = 1; i < data_.size(); i++) {
            data_[i] = ~data_[i];
            if (carry_bit) {
                carry_bit = (++data_[i]) == 0;
            }
        }
    }

    ByteArray<endian, N_bytes>& bytes() { return data_; }
    const ByteArray<endian, N_bytes>& bytes() const { return data_; }

    friend std::ostream& operator<<(std::ostream& out, const Self& self) {
        std::ios_base::fmtflags flags(out.flags());

        out << std::noboolalpha;
        for (size_t i = 0; i < N_bits; i++) {
            out << self.get_bit(N_bits-1-i);
        }
        
        out.flags(flags);
        return out;
    }

    constexpr static bool empty() { return N_bits == 0; }

    /*  length = 7
     *  offset = 6
     *
     *  10100010 01011110
     *     XXXXX XX
     * 15         6  <- 0
     */
    template <typename T, size_t length, size_t offset>
    constexpr T interval() const {
        static_assert(length <= bit_sizeof<T>());
        static_assert(length + offset <= N_bits);

        constexpr size_t start_byte = containing_size_bytes(length + offset) - 1;
        constexpr size_t start_bits = length + offset - start_byte * n_bits_per_byte;

        constexpr size_t end_byte = offset / n_bits_per_byte;
        constexpr size_t end_bits = n_bits_per_byte * (end_byte + 1) - offset;

        if constexpr (start_byte == end_byte) {
            return static_cast<T>(right_justify<length>(left_justify<start_bits>(data_[start_byte])));
        }

        T result = 0;

        if constexpr (start_byte < N_bytes) {
            constexpr uint8_t mask = interval_mask<n_bits_per_byte - start_bits, start_bits, 0, uint8_t>();
            result |= mask & data_[start_byte];
        } 

        for (size_t i = 1; i+1 < start_byte - end_byte; i++) {
            size_t r_index = start_byte - i;
            result = (result << n_bits_per_byte) | data_[r_index];
        }

        result = (result << end_bits) | right_justify<end_bits>(data_[end_byte]);

        return result;
    }

private:
    constexpr size_t byte_index(size_t i) const {
        return i/n_bits_per_byte;
    }

    constexpr size_t bit_flag(size_t i) const {
        return 0b1 << (i % n_bits_per_byte);
    }

    ByteArray<endian, N_bytes> data_;
};

template <typename T>
void print_bits(T value) {
    std::cout << BitArray<Endianness::Little, bit_sizeof<T>()>((uint8_t*) &value);
}

template <Endianness endian, size_t N, typename T, typename U>
BitArray<endian, N> make_bit_array(ArrayView<T, sizeof(U)/sizeof(T)> buffer, U value) {
    static_assert(buffer.size() * sizeof(T) == sizeof(U));
    return BitArray<endian, N>(make_byte_array<endian>(buffer, value));
}

/*
 * Left-justifies a big-endian bit array. This invalidates the bit array.
 */
template <size_t N_bits>
void left_justify(BitArray<Endianness::Big, N_bits>& data) {
    if (data.empty()) return;

    for (size_t i = 0; i+1 < data.N_bytes; i++) {
        size_t index = data.N_bytes - i - 1;
        data.bytes()[index] = left_justify<n_bits_per_byte - data.N_padding>(data.bytes()[index]) 
                              | right_justify<data.N_padding>(data.bytes()[index-1]);
    }
    data.bytes().lsb() = left_justify<n_bits_per_byte - data.N_padding>(data.bytes().lsb());
}

/*
 * Right-justifies a data array and returns it as a big-endian bit array.
 */
template <size_t N_bits>
BitArray<Endianness::Big, N_bits> right_justify(ArrayView<uint8_t, containing_size_bytes(N_bits)> buffer) {
    if constexpr (N_bits == 0) return BitArray<Endianness::Big,N_bits>(nullptr);

    BitArray<Endianness::Big, N_bits> data(buffer);

    for (size_t i = 0; i+1 < data.N_bytes; i++) {
        data.bytes()[i] = right_justify<n_bits_per_byte - data.N_padding>(data.bytes()[i])
                          | left_justify<data.N_padding>(data.bytes()[i+1]);
    }
    data.bytes().msb() = right_justify<n_bits_per_byte - data.N_padding>(data.bytes().msb());
    return data;
}

template <typename... BitArrays>
constexpr size_t total_size() {
    return (BitArrays::size() + ...);
}

namespace detail {
template <typename T, size_t N, size_t i_buffer, size_t i_bits, size_t j_bits, typename... BitArrays,
         std::enable_if_t<sizeof...(BitArrays) == 0, bool> = true>
void pack(ArrayView<T,N>, const BitArrays&...) {
    static_assert(i_buffer == N);
    static_assert(i_bits == 0);
    static_assert(j_bits == 0);
}

template <typename T, size_t N, size_t i_buffer, size_t i_bits, size_t j_bits, typename BitArrayType, typename... BitArrays>
void pack(ArrayView<T,N> buffer, const BitArrayType& bits, const BitArrays&... bit_arrays) {
    static_assert(i_buffer < N);
    static_assert(i_bits < bit_sizeof<T>());
    static_assert(j_bits < BitArrayType::size());

    constexpr size_t pack_bits = std::min(bit_sizeof<T>() - i_bits, BitArrayType::size() - j_bits);

    // Don't try to left shfit by the size of T
    if constexpr (pack_bits < bit_sizeof<T>()) {
        buffer[i_buffer] = buffer[i_buffer] << pack_bits;
    } 

    buffer[i_buffer] |= bits.template interval<T, pack_bits, bits.size() - (pack_bits + j_bits)>();

    if constexpr (j_bits + pack_bits == BitArrayType::size()) {
        if constexpr (i_bits + pack_bits == bit_sizeof<T>()) {
            pack<T,N, i_buffer+1, 0, 0, BitArrays...>(buffer, bit_arrays...);
        } else {
            pack<T,N, i_buffer, i_bits+pack_bits, 0, BitArrays...>(buffer, bit_arrays...);
        }
    } else {
        if constexpr (i_bits + pack_bits == bit_sizeof<T>()) {
            pack<T,N, i_buffer+1, 0, j_bits + pack_bits, BitArrayType, BitArrays...>(buffer, bits, bit_arrays...);
        } else {
            pack<T,N, i_buffer, i_bits+pack_bits, j_bits + pack_bits, BitArrayType, BitArrays...>(buffer, bits, bit_arrays...);
        }
    }
}
}


template <typename T, size_t N, typename... BitArrays>
void pack(ArrayView<T, N> buffer_view, const BitArrays&... bit_arrays) {
    detail::pack<T, N, 0, 0, 0, BitArrays...>(buffer_view, bit_arrays...);
}
