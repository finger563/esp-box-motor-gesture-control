# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

set(EXTRA_COMPONENT_DIRS
  "components/"
  "components/espp/components/"
)

set(
  COMPONENTS
  "main esptool_py esp-box esp-now timer filters bldc_motor gui"
  CACHE STRING
  "List of components to include"
  )

project(esp-box-motor-gesture-control)

set(CMAKE_CXX_STANDARD 20)
