cmake_minimum_required(VERSION 3.15)
project(find_energy)

set(CMAKE_CXX_STANDARD 11)
add_compile_definitions(PROJECT_DIR="${PROJECT_SOURCE_DIR}" )
find_package( OpenCV REQUIRED )

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(find_energy find_energy.cpp)
target_link_libraries(  find_energy ${OpenCV_LIBS}  )
