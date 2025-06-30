#define CATCH_CONFIG_MAIN

#include <catch2/catch.hpp>

#include "CppUtils/c_util/CUtil.h"

#include <iostream>

TEST_CASE("Narrowing check") {

    uint16_t value = 0b0000'0000'0000'0000;
    REQUIRE(narrowed_type_fits<12>(value));

    value = 0b0000'1100'0100'0010;
    REQUIRE(narrowed_type_fits<12>(value));

    value = 0b0001'0000'0000'0000;
    REQUIRE(!narrowed_type_fits<12>(value));

    value = 0b1101'0101'1011'0100;
    REQUIRE(!narrowed_type_fits<12>(value));

    value = 0b1111'1111'1111'1111;
    REQUIRE(!narrowed_type_fits<12>(value));

    
    double x = 3.1415;
    REQUIRE(narrowed_type_fits<sizeof_bits<double>()>(x));

    float y = 2.87;
    REQUIRE(narrowed_type_fits<sizeof_bits<float>()>(y));

    int16_t z = 0b0000'0000'0000'0000;
    REQUIRE(narrowed_type_fits<12>(z));

    z = 0b1111'0000'0000'0000;
    REQUIRE(!narrowed_type_fits<12>(z));
}
