set(FOLDER_NAME "networking")
set_CppUtils_library_name("Networking")

make_CppUtils_library(${LIBRARY_NAME} ${FOLDER_NAME})

target_link_libraries(${LIBRARY_NAME} CppUtilsIO CppUtilsCUtils)

install_CppUtils_library(${LIBRARY_NAME} ${FOLDER_NAME})
