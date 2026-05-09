cmake_minimum_required(VERSION 3.5)

set(IDF_TARGET "esp32s3")
set(COMPONENTS main)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(drone_comms)
