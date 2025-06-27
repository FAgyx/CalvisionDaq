function(make_CalvisionDaq_exec EXEC_NAME LINK_LIBS)
    add_executable(${EXEC_NAME} "${EXEC_NAME}.cpp")

    foreach(LINK_LIB ${LINK_LIBS})
        target_link_libraries(${EXEC_NAME} PRIVATE ${LINK_LIB})
    endforeach()

    target_include_directories(${EXEC_NAME} PRIVATE ${CBLAS_INCLUDE_DIR})


    install(TARGETS ${EXEC_NAME}
        LIBRARY DESTINATION ${CalvisionDaq_INSTALL_LIB_DIR}
        ARCHIVE DESTINATION ${CalvisionDaq_INSTALL_LIB_DIR}
        RUNTIME DESTINATION ${CalvisionDaq_INSTALL_BIN_DIR}
    )
endfunction()


# make_CalvisionDaq_exec(decoder "CalvisionDaqCommon;CalvisionDaqDigitizer;CalvisionDaqRoot")
# make_CalvisionDaq_exec(readout "CalvisionDaqCommon;CalvisionDaqDigitizer")
make_CalvisionDaq_exec(calibrate "CalvisionDaqCommon;CalvisionDaqDigitizer")
# make_CalvisionDaq_exec(read_and_decode "CalvisionDaqCommon;CalvisionDaqDigitizer;CalvisionDaqRoot")
make_CalvisionDaq_exec(dual_readout "CalvisionDaqCommon;CalvisionDaqDigitizer;CalvisionDaqRoot")
# make_CalvisionDaq_exec(serial_dual_readout "CalvisionDaqCommon;CalvisionDaqDigitizer;CalvisionDaqRoot")
make_CalvisionDaq_exec(reset_digitizer "CalvisionDaqCommon;CalvisionDaqDigitizer")
# make_CalvisionDaq_exec(check_timing "CalvisionDaqCommon;CalvisionDaqDigitizer")
# make_CalvisionDaq_exec(test "CalvisionDaqCommon")
make_CalvisionDaq_exec(optical_test "CalvisionDaqCommon;CalvisionDaqDigitizer;CalvisionDaqRoot")
