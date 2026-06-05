cmake_minimum_required(VERSION 3.12)
project(zeropilot4.0)

include(${CMAKE_SOURCE_DIR}/common.cmake)

set(RELATIVE_ZP_SRC)
foreach(SRC_FILE IN LISTS ZP_SRC)
    string(PREPEND SRC_FILE "${CMAKE_SOURCE_DIR}/")
    list(APPEND RELATIVE_ZP_SRC ${SRC_FILE})
endforeach()

set(RELATIVE_ZP_INC)
foreach(INC_FILE IN LISTS ZP_INC)
    string(PREPEND INC_FILE "${CMAKE_SOURCE_DIR}/")
    list(APPEND RELATIVE_ZP_INC ${INC_FILE})
endforeach()

set(RELATIVE_EXTERNAL_INC)
foreach(INC_FILE IN LISTS EXTERNAL_INC)
    string(PREPEND INC_FILE "${CMAKE_SOURCE_DIR}/")
    list(APPEND RELATIVE_EXTERNAL_INC ${INC_FILE})
endforeach()

add_library(${PROJECT_NAME} STATIC ${RELATIVE_ZP_SRC})
target_include_directories(${PROJECT_NAME} PRIVATE ${RELATIVE_ZP_INC})
target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${RELATIVE_EXTERNAL_INC})
