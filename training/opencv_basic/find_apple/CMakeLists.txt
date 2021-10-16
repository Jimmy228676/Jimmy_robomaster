cmake_minimum_required(VERSION 3.15)
project(find_apple)

set(CMAKE_CXX_STANDARD 11)
add_compile_definitions(PROJECT_DIR="${PROJECT_SOURCE_DIR}" )
find_package( OpenCV REQUIRED )

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(find_apple find_apple.cpp)
target_link_libraries(  find_apple ${OpenCV_LIBS}  )
