include("${CMAKE_SOURCE_DIR}/cmake/FindCaenDigitizer.cmake")

set(LIBRARY_NAME "${PROJECT_NAME}Digitizer")

make_CalvisionDaq_library(${LIBRARY_NAME})

# Ensure cblas.h is found
target_include_directories(${LIBRARY_NAME} PRIVATE ${CBLAS_INCLUDE_DIR})

target_link_libraries(${LIBRARY_NAME} PUBLIC
    CalvisionDaqCommon
    CAENComm
    CAENDigitizer
    CAENVME
    usb-1.0
    CppUtils::CppUtilsCUtils
    BLAS::BLAS
)

install_CalvisionDaq_library(${LIBRARY_NAME} digitizer)
