cmake_minimum_required(VERSION 3.0...3.13)

message("CMake the running...\n")

include_directories(inc)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 ")
set(SOURCES src/main.c 
            src/debug.c 
            src/fc_init.c 
            src/flight_data.c 
            src/iface.c 
            src/motors.c 
            src/scheduler.c 
            src/tasks 
            src/drone_time 
            src/pid.c 
            src/maths.c
            src/filter.c
            src/smith_predictor.c
            src/control_rate_profile.c
            src/rc_curves.c
            src/mixer.c
            src/runtime_config.c
            src/rc_controls.c
            )

add_executable(drone_firmware ${SOURCES})

project(drone_firmware)

target_link_libraries(drone_firmware PRIVATE m)

message("CMake the stop\n")