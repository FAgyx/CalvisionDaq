set(FOLDER_NAME "c_util")
set_CppUtils_library_name("CUtils")

make_CppUtils_library(${LIBRARY_NAME} ${FOLDER_NAME})

target_link_libraries(${LIBRARY_NAME} CppUtilsPreProc)

install_CppUtils_library(${LIBRARY_NAME} ${FOLDER_NAME})
