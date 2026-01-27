# attitude manager files
set(AM_SRC
    "src/attitude_manager/attitude_manager.cpp"
    "src/attitude_manager/direct_mapping.cpp"
    "src/attitude_manager/MahonyAHRS.cpp"
)
set(AM_INC
    "include/attitude_manager/"
)

# system manager files
set(SM_SRC
    "src/system_manager/system_manager.cpp"
)
set(SM_INC
    "include/system_manager/"
)

# telemetry manager files
set(TM_SRC
    "src/telemetry_manager/telemetry_manager.cpp"
)
set(TM_INC
    "include/telemetry_manager/"
    "include/thread_msgs/"
)

# external library files (does not apply compiler warnings)
set(EXTERNAL_INC
    "../external/c_library_v2/all/"
)

# combined files
set(ZP_SRC
    ${AM_SRC}
    ${SM_SRC}
    ${TM_SRC}
)
set(ZP_INC
    "include/"
    "include/driver_ifaces/"
    "include/thread_msgs/"
    ${AM_INC}
    ${SM_INC}
    ${TM_INC}
)