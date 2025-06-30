FetchContent_Declare(
    catch
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG v2.13.6)
FetchContent_MakeAvailable(catch)


function(make_CppUtils_test TEST_NAME TEST_SOURCE TEST_LIBS)
    add_executable(${TEST_NAME} ${TEST_SOURCE})

    target_compile_features(${TEST_NAME} PRIVATE cxx_std_17)

    target_include_directories(${TEST_NAME} PRIVATE "${CppUtils_BUILD_INCLUDE_DIR}")
    target_link_libraries(${TEST_NAME} PRIVATE Catch2::Catch2 ${TEST_LIBS})

    add_test(NAME ${TEST_NAME}_test COMMAND ${TEST_NAME})
endfunction()

make_CppUtils_test(test_io "TestIO.cpp" "CppUtilsIO")
make_CppUtils_test(test_c_util "TestCUtil.cpp" "CppUtilsCUtils")
make_CppUtils_test(test_enum "TestEnum.cpp" "CppUtilsCUtils")
make_CppUtils_test(test_bitmanip "TestBitManip.cpp" "CppUtilsCUtils")
