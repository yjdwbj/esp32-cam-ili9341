# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)
set(CMAKE_VERBOSE_MAKEFILE ON)
set(COMPONENTS main)

add_compile_definitions(CMAKE_VERBOSE_MAKEFILE=ON)
add_definitions(-DBOARD_ESP32CAM_AITHINKER=1 -DUSE_ILI9341_DRIVER=1)

# set(BOARD_NAME "BOARD_ESP32CAM_AITHINKER" CACHE STRING "BOARD_NAME chose by the use at CMake configure time")
# set_property(CACHE BOARD_NAME PROPERTY STRING "BOARD_ESP32CAM_AITHINKER" "BOARD_ESP32_S2_OV2640")

if(NOT DEFINED WIFI_SSID)
	message("You're not set WIFI_SSID variable for the build.
    You can do this via -DWIFI_SSID=xxx on the cmake command line")
else ()
	add_definitions(-D_WIFI_SSID=${WIFI_SSID})
endif()

if(NOT DEFINED WIFI_PWD)
	message("You're not set WIFI_PWD variable for the build.
    You can do this via -DWIFI_PWD=xxx on the cmake command line")
else ()
	add_definitions(-D_WIFI_PWD=${WIFI_PWD})
endif()
# The above custom define must be at before the following line.

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(esp32-cam-ili9341)

