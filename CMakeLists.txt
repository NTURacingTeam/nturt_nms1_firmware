cmake_minimum_required(VERSION 3.20.0)
set(CMAKE_C_STANDARD 11)

set(OVERLAY_CONFIG thread_analyzer.conf)
list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_LIST_DIR}/libmicroros)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

# must be after finding zephyr
project(nms1)

target_include_directories(app PRIVATE include)
target_sources(app PRIVATE
        src/encoder_pcnt.c
        src/main.c
        src/micro_ros.c
        src/nms1.c
)
