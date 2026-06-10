# attitude manager files
set(AM_SRC
    "src/attitude_manager/attitude_manager.cpp"
    "src/attitude_manager/direct_mapping.cpp"
    "src/attitude_manager/fbwa_mapping.cpp"
    "src/attitude_manager/pid.cpp"
    "src/attitude_manager/MahonyAHRS.cpp"
    "src/attitude_manager/am_param_setup.cpp"
)
set(AM_INC
    "include/attitude_manager/"
)

# system manager files
set(SM_SRC
    "src/system_manager/system_manager.cpp"
    "src/system_manager/sm_param_setup.cpp"
)
set(SM_INC
    "include/system_manager/"
)

# telemetry manager files
set(TM_SRC
    "src/telemetry_manager/telemetry_manager.cpp"
    "src/telemetry_manager/tm_param_setup.cpp"
)
set(TM_INC
    "include/telemetry_manager/"
    "include/thread_msgs/"
)

# zp param files
set(ZP_PARAM_SRC
    "src/zp_param/zp_params.cpp"
)
set(ZP_PARAM_INC
    "include/zp_param/"
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
    ${ZP_PARAM_SRC}
)
set(ZP_INC
    "include/driver_ifaces/"
    "include/thread_msgs/"
    ${AM_INC}
    ${SM_INC}
    ${TM_INC}
    ${ZP_PARAM_INC}
)
