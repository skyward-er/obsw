######################################################################
# CMakeLists.txt generated for component run_ADA
# Product type: executable
######################################################################
cmake_minimum_required(VERSION 3.12)
project(run_ADA)
if(DEFINED ENV{CMAKE_EXPORT_COMPILE_COMMANDS} AND NOT DEFINED CMAKE_EXPORT_COMPILE_COMMANDS)
    set(CMAKE_EXPORT_COMPILE_COMMANDS $ENV{CMAKE_EXPORT_COMPILE_COMMANDS})
endif()


######################################################################
## Path variables
######################################################################
get_filename_component(START_DIR .. ABSOLUTE)
set(MATLAB_ROOT C:/Program\ Files/MATLAB/R2023a CACHE PATH "")
if(${CMAKE_CURRENT_BINARY_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})
    set(BUILDING_IN_SOURCE_DIR TRUE)
    set(BINARY_START_DIR "${START_DIR}")
else()
    set(BUILDING_IN_SOURCE_DIR FALSE)
    set(BINARY_START_DIR "${CMAKE_BINARY_DIR}")
endif()
if(WIN32 AND NOT MINGW)
    set(SYSLIB_PREFIX "lib")
else()
    set(SYSLIB_PREFIX "")
endif()

######################################################################
## System libraries
######################################################################
find_library(FOUND_LIBM m NO_SYSTEM_ENVIRONMENT_PATH PATHS ${CMAKE_C_IMPLICIT_LINK_DIRECTORIES}
    ${CMAKE_CXX_IMPLICIT_LINK_DIRECTORIES})

######################################################################
## Target definition and commands
######################################################################
add_executable(run_ADA  ${START_DIR}/run_ADA_grt_rtw/run_ADA.cpp
    ${START_DIR}/run_ADA_grt_rtw/run_ADA_data.cpp
    ${MATLAB_ROOT}/rtw/c/src/common/rt_cppclass_main.cpp)
set_target_properties(run_ADA PROPERTIES
    PREFIX ""
    POSITION_INDEPENDENT_CODE ON
    RUNTIME_OUTPUT_DIRECTORY "${BINARY_START_DIR}/./$<0:>"
    LIBRARY_OUTPUT_DIRECTORY "${BINARY_START_DIR}/./$<0:>"
    ARCHIVE_OUTPUT_DIRECTORY "${BINARY_START_DIR}/./$<0:>")
target_compile_features(run_ADA PUBLIC cxx_std_11)
target_compile_definitions(run_ADA PRIVATE -DMODEL=run_ADA
    -DNUMST=1
    -DNCSTATES=0
    -DHAVESTDIO
    -DRT
    -DUSE_RTMODEL
    -DCLASSIC_INTERFACE=0
    -DALLOCATIONFCN=0
    -DTID01EQ=0
    -DMAT_FILE=0
    -DONESTEPFCN=1
    -DTERMFCN=1
    -DMULTI_INSTANCE_CODE=1
    -DINTEGER_CODE=0
    -DMT=0)
target_include_directories(run_ADA PRIVATE ${START_DIR}
    ${START_DIR}/run_ADA_grt_rtw
    ${MATLAB_ROOT}/extern/include
    ${MATLAB_ROOT}/simulink/include
    ${MATLAB_ROOT}/rtw/c/src
    ${MATLAB_ROOT}/rtw/c/src/ext_mode/common)
target_link_libraries(run_ADA PRIVATE $<$<BOOL:${FOUND_LIBM}>:m>)
if(WIN32)
   target_link_libraries(run_ADA PRIVATE kernel32
       ws2_32
       mswsock
       advapi32)
endif()

######################################################################
## Extract debug symbols
######################################################################
if(APPLE AND (CMAKE_C_COMPILER_ID STREQUAL "AppleClang"))
    string(APPEND _dsymutilcmd "$<IF:$<CONFIG:DEBUG,RELWITHDEBINFO>,"
                "xcrun;dsymutil;$<TARGET_FILE:run_ADA>;--flat,"
                "true"
        >)
    add_custom_command(TARGET run_ADA
            POST_BUILD
            COMMAND "${_dsymutilcmd}"
            COMMAND_EXPAND_LISTS
            BYPRODUCTS run_ADA.dwarf)
endif()


######################################################################
## Build success message
######################################################################
add_custom_command(TARGET run_ADA POST_BUILD
     COMMAND ${CMAKE_COMMAND} -E cmake_echo_color --cyan "\\#\\#\\# Created executable: $<TARGET_FILE:run_ADA>")

######################################################################
## Call toolchain hook function if defined
######################################################################
if(COMMAND toolchain_target_hook)
    toolchain_target_hook(run_ADA)
endif()
