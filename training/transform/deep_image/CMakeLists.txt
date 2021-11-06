cmake_minimum_required(VERSION 3.15)
project(transform_2)

set(CMAKE_CXX_STANDARD 11)
add_compile_definitions(PROJECT_DIR="${PROJECT_SOURCE_DIR}" )
find_package( OpenCV REQUIRED )

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories("/usr/include/eigen3")

add_executable(transform_2 transform_2.cpp)
target_link_libraries(  transform_2 ${OpenCV_LIBS}  )
