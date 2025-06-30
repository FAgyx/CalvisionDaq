#define CATCH_CONFIG_MAIN

#include <catch2/catch.hpp>

#include "CppUtils/c_util/BitManip.h"
#include "CppUtils/c_util/ByteArray.h"
#include "CppUtils/c_util/BitArray.h"

#include <iostream>
#include <sstream>

using BitArray12 = BitArray<Endianness::Big, 12>;

template <size_t N, typename T, typename U>
bool equals(const T& a, const U& b) {
    for (size_t i = 0; i < N; i++) {
        if (a[i] != b[i])
            return false;
    }
    return true;
}

TEST_CASE("Byte Array") {
    std::array<uint8_t, 5> data = {0x5a, 0xb3, 0x01, 0x37, 0x4f};
    ByteArray<Endianness::Little, 5> little(data.data());
    ByteArray<Endianness::Big, 5> big(data.data());

    for (size_t i = 0; i < 5; i++) {
        REQUIRE(little[i] == data[i]);
        REQUIRE(big[i] == data[4-i]);
    }
}

TEST_CASE("Containg size bytes") {
    REQUIRE(containing_size_bytes(0) == 0);
    REQUIRE(containing_size_bytes(6) == 1);
    REQUIRE(containing_size_bytes(8) == 1);
    REQUIRE(containing_size_bytes(12) == 2);
    REQUIRE(containing_size_bytes(16) == 2);
    REQUIRE(containing_size_bytes(32) == 4);
    REQUIRE(containing_size_bytes(64) == 8);
}

TEST_CASE("Interval Mask") {
    REQUIRE(interval_mask<3,0,5,uint8_t>() == 0b0000'0000);
    REQUIRE(interval_mask<0,8,0,uint8_t>() == 0b1111'1111);
    REQUIRE(interval_mask<1,3,4,uint8_t>() == 0b0111'0000);
    REQUIRE(interval_mask<0,1,7,uint8_t>() == 0b1000'0000);
}

TEST_CASE("Bit Array") {
    std::array<uint8_t, 2> data = {0b0100'0110, 0b0001'1110};
    BitArray12 bits(data.data());

    REQUIRE(bits.get_bit(0) == 0);
    REQUIRE(bits.get_bit(1) == 1);
    REQUIRE(bits.get_bit(2) == 1);
    REQUIRE(bits.get_bit(6) == 0);
    REQUIRE(bits.get_bit(8) == 0);
    REQUIRE(bits.get_bit(9) == 1);
    REQUIRE(bits.get_bit(11) == 0);

    
    bits.set_bit_on(7);
    REQUIRE(bits.get_bit(7) == 1);

    bits.set_bit_off(3);
    REQUIRE(bits.get_bit(3) == 0);

    bits.set_bit(4, true);
    REQUIRE(bits.get_bit(4) == 1);
    bits.set_bit(4, false);
    REQUIRE(bits.get_bit(4) == 0);
}

TEST_CASE("Bit Array Convert Small") {
    {
        uint8_t data = 0b0000'0101;
        BitArray<Endianness::Little, 4> bits(&data);

        REQUIRE(bits.convert<uint8_t>() == 5u);
        REQUIRE(bits.convert<int8_t>() == 5);
    }

    {
        uint8_t data = 0b0000'1101;
        BitArray<Endianness::Little, 4> bits(&data);

        REQUIRE(bits.convert<uint8_t>() == 13u);
        REQUIRE(bits.convert<int8_t>() == -3);
    }
}

TEST_CASE("Bit Array Convert Large") {
    {
        std::array<uint8_t, 2> data = {0b0000'0100, 0b1010'1110};
        BitArray12 bits(data.data());

        REQUIRE(bits.convert<uint16_t>() == 1198u);
        REQUIRE(bits.convert<int16_t>() == 1198);
        REQUIRE(bits.convert<uint32_t>() == bits.convert<uint16_t>());
        REQUIRE(bits.convert<uint64_t>() == bits.convert<uint16_t>());
        REQUIRE(bits.convert<int32_t>() == bits.convert<int16_t>());
        REQUIRE(bits.convert<int64_t>() == bits.convert<int16_t>());
    }

    {
        std::array<uint8_t, 2> data = {0b0000'1100, 0b1010'1110};
        BitArray12 bits(data.data());

        REQUIRE(bits.convert<uint16_t>() == 3246u);
        REQUIRE(bits.convert<int16_t>() == -850);
        REQUIRE(bits.convert<uint32_t>() == bits.convert<uint16_t>());
        REQUIRE(bits.convert<uint64_t>() == bits.convert<uint16_t>());
        REQUIRE(bits.convert<int32_t>() == bits.convert<int16_t>());
        REQUIRE(bits.convert<int64_t>() == bits.convert<int16_t>());
    }
}

TEST_CASE("Bit Array Convert Consistency") {
    uint32_t value = 0x3F'2A'88'17;
    uint32_t buffer_data;
    ArrayView<uint32_t, 1> buffer(&buffer_data);
    REQUIRE(make_bit_array<Endianness::Little>(buffer, value).convert<uint32_t>() == value);

    int32_t signed_value = -384999384;
    REQUIRE(make_bit_array<Endianness::Little>(buffer, signed_value).convert<int32_t>() == signed_value);
}

TEST_CASE("Twos compliment") {
    auto check = [](int32_t value) {
            uint32_t buffer;
            auto bits = make_bit_array<Endianness::Little>(ArrayView<uint32_t,1>(&buffer), value);
            bits.twos_compliment();
            REQUIRE(bits.convert<int32_t>() == -value);
        };

    check(0);
    check(123);
    check(-384999384);

    {
        std::array<uint8_t, 2> data = {0b0000'1100, 0b1010'1110};
        BitArray12 bits(data.data());

        bits.twos_compliment();
        REQUIRE(bits.convert<int16_t>() == 850);
    }
}

TEST_CASE("Justify") {
    std::array<uint8_t, 2> data = {0b0000'1010, 0b1101'0111};
    BitArray12 bits(data.data());

    left_justify(bits);
    REQUIRE(equals<2>(data ,std::array<uint8_t, 2>{0b1010'1101, 0b0111'0000}));

    BitArray12 new_bits = right_justify<12>(ArrayView<uint8_t,2>(data.data()));
    REQUIRE(equals<2>(data, std::array<uint8_t,2>{0b0000'1010, 0b1101'0111}));
}

TEST_CASE("Print") {
    std::array<uint8_t, 2> data = {0b0000'1010, 0b1101'0111};

    std::stringstream byte_result;
    ByteArray<Endianness::Big, 2> bytes(data.data());
    byte_result << bytes;
    REQUIRE(byte_result.str() == "0xad7");

    std::stringstream bit_result;
    BitArray12 bits(data.data());
    bit_result << bits;
    REQUIRE(bit_result.str() == "101011010111");
}


// TEST_CASE("Unpack Array") {
//     std::array<uint32_t, 3> data = {0xf1c373'8d5732'aa,
//                                     0x8392'bc0314'73f1,
//                                     0x73'9ff213'7a3258};
// 
//     std::array<uint32_t, 8> target = {0xf1c373,
//                                       0x8d5732,
//                                       0xaa8392,
//                                       0xbc0314,
//                                       0x73f173,
//                                       0x9ff213,
//                                       0x7a3258};
// 
//     std::array<uint16_t, 8> buffer = {0, 0, 0, 0, 0, 0, 0, 0};
//     std::array<BitArray12 > bits_array = BitArray12::read_arrays(data, buffer);
//     for (size_t i = 0; i < 8; i++) {
//         REQUIRE(static_cast<uint32_t>(bits_array[i].convert<uint16_t>()) == target[i]);
//     }
// 
//     std::array<uint32_t, 8> result = {0, 0, 0, 0, 0, 0, 0, 0};
//     BitArray12::unpack_into(data, result);
//     REQUIRE(equals<8>(target, result));
// }
// 
// TEST_CASE("Pack Array") {
//     std::array<uint16_t, 8> data = {0xf1c373,
//                                     0x8d5732,
//                                     0xaa8392,
//                                     0xbc0314,
//                                     0x73f173,
//                                     0x9ff213,
//                                     0x7a3258};
// 
//     std::array<uint32_t, 3> target = {0xf1c373'8d5732'aa,
//                                       0x8392'bc0314'73f1,
//                                       0x73'9ff213'7a3258};
// 
//     std::array<uint16_t, 8> buffer = {0, 0, 0, 0, 0, 0, 0, 0};
//     std::array<BitArray12, 8> bit_arrays = BitArray12::make_arrays(data);
//     std::array<uint32_t, 3> result = {0, 0, 0};
//     BitArray12::pack(bit_arrays, result);
// 
//     REQUIRE(equals<3>(result, target));
// 
//     result = {0, 0, 0};
//     BitArray12::pack_into(data, result);
//     REQUIRE(equals<3>(result, target));
// }
// 
// TEST_CASE("Unpack bits") {
// }
// 
// TEST_CASE("Pack bits") {
//     std::array<uint8_t, 2> result = 0;
//     pack_bits<Endianness::Big, 1,4,2,1,2,5,1>(result, 0b1, 0b1011, 0b10, 0b0, 0b11, 0b01101, 0b1);
//     REQUIRE(equals<2>(result == {0b1'1011'10'0, 0b11'01101'1});
// }


TEST_CASE("Interval") {
    std::array<uint8_t, 2> data = {0b1011'0110, 0b0111'0001};

    const BitArray<Endianness::Big, 12> bits(data.data());

    REQUIRE(bits.interval<uint16_t, 4, 8>() == 0b0110);
    REQUIRE(bits.interval<uint16_t, 7, 3>() == 0b1001110);
    
    const BitArray<Endianness::Big, 16> bits16(data.data());

    REQUIRE(bits16.interval<uint16_t, 1, 0>() == 0b1);
    REQUIRE(bits16.interval<uint16_t, 8, 0>() == 0b01110001);
    REQUIRE(bits16.interval<uint16_t, 8, 8>() == 0b10110110);
    REQUIRE(bits16.interval<uint16_t, 16, 0>() == 0b1011011001110001);
}


TEST_CASE("Pack") {
    std::array<uint8_t, 2> result = {0, 0};

    std::array<uint8_t, 4> data = {0b0000'0010,
                                   0b1011'1110,
                                   0b0000'0101,
                                   0b0000'0110};

    pack(ArrayView<uint8_t, 2>(result),
            BitArray<Endianness::Big, 2>(&data[0]),
            BitArray<Endianness::Big, 8>(&data[1]),
            BitArray<Endianness::Big, 3>(&data[2]),
            BitArray<Endianness::Big, 3>(&data[3]));

    REQUIRE(equals<2>(result, std::array<uint8_t, 2>{0b1010'1111, 0b1010'1110}));
}
