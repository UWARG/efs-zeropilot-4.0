cmake_minimum_required(VERSION 3.18)
project(gtestzeropilot4.0)

# ========== test files begin ==========
# attitude manager test files
set(AM_TSRC

)

# system manager test files
set(SM_TSRC

)

# all test files
set(ALL_TSRC
    ${AM_TSRC}
    ${SM_TSRC}
)
# ========== test files end ==========

set(CMAKE_C_COMPILER "gcc")
set(CMAKE_CXX_COMPILER "g++")

# GoogleTest requires at least C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-g")

find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
enable_testing()

include(${CMAKE_SOURCE_DIR}/../common.cmake)
include(GoogleTest)

set(RELATIVE_ZP_SRC)
foreach(SRC_FILE IN LISTS ZP_SRC)
    string(PREPEND SRC_FILE "${CMAKE_SOURCE_DIR}/../")
    list(APPEND RELATIVE_ZP_SRC ${SRC_FILE})
endforeach()

set(RELATIVE_ZP_INC)
foreach(INC_FILE IN LISTS ZP_INC)
    string(PREPEND INC_FILE "${CMAKE_SOURCE_DIR}/../")
    list(APPEND RELATIVE_ZP_INC ${INC_FILE})
endforeach()

set(RELATIVE_EXTERNAL_INC)
foreach(INC_FILE IN LISTS EXTERNAL_INC)
    string(PREPEND INC_FILE "${CMAKE_SOURCE_DIR}/../")
    list(APPEND RELATIVE_EXTERNAL_INC ${INC_FILE})
endforeach()

add_executable(${PROJECT_NAME} 
    ${RELATIVE_ZP_SRC} 
    ${ALL_TSRC}
)
target_include_directories(${PROJECT_NAME} 
    PRIVATE ${RELATIVE_ZP_INC} 
    PRIVATE "${CMAKE_SOURCE_DIR}/driver_mocks"
)
target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${RELATIVE_EXTERNAL_INC})
target_link_libraries(${PROJECT_NAME} GTest::gmock_main)

gtest_discover_tests(${PROJECT_NAME})
