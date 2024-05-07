# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

set (EXTRA_COMPONENTR_DIRS "")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ESP32-Robot-V4)
# directory with project components
#set(COMPONENT_DIRS "components")
#idf_build_set_property(COMPILE_OPTIONS "-Wno-error=class-memaccess" APPEND)

#idf.py menuconfig -> compiler options -> enable c++ exceptions


