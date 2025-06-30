#define CATCH_CONFIG_MAIN

#include <catch2/catch.hpp>

#include "CppUtils/io/BinaryIO.h"
#include "CppUtils/io/FileHandle.h"
#include "CppUtils/io/DeviceHandle.h"

#include <iostream>

template <typename T>
void run_test() {
    using Writer = BinaryWriterTemplate<T>;
    Writer writer("temp.txt", OpenMode::Truncate);
    REQUIRE(writer.good());
    writer.template write<char>('a');
    writer.template write<int16_t>(3745);
    writer.template write<double>(-3.14159263);
    writer.close();

    using Reader = BinaryReaderTemplate<T>;
    Reader reader("temp.txt", OpenMode::Read);
    REQUIRE(reader.good());
    char a;
    reader.template read<char>(a);
    REQUIRE(a == 'a');

    int16_t x;
    reader.template read<int16_t>(x);
    REQUIRE(x == 3745);

    double y;
    reader.template read<double>(y);
    REQUIRE(y == -3.14159263);

    reader.close();
}

TEST_CASE("Device Handle") {
    run_test<DeviceHandle>();
}

TEST_CASE("File Handle") {
    run_test<FileHandle>();
}
