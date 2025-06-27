set(LIBRARY_NAME "${PROJECT_NAME}Root")

make_CalvisionDaq_library(${LIBRARY_NAME})
# Ensure cblas.h is found
target_include_directories(${LIBRARY_NAME} PRIVATE ${CBLAS_INCLUDE_DIR})

target_link_libraries(${LIBRARY_NAME} PUBLIC
    blas
    CalvisionDaqCommon
    CalvisionDaqDigitizer
    CppUtils::CppUtilsIO
    ROOT::Core
    ROOT::Tree
    ROOT::RIO
    ROOT::Matrix
    ROOT::Hist
    ROOT::Graf
    ROOT::Gpad
    ROOT::Imt
)

install_CalvisionDaq_library(${LIBRARY_NAME} root)
