cmake_minimum_required(VERSION 3.15)
project(camera)

set(CMAKE_CXX_STANDARD 11)
add_compile_definitions(PROJECT_DIR="${PROJECT_SOURCE_DIR}" )
find_package( OpenCV REQUIRED )

include_directories(${OpenCV_INCLUDE_DIRS})

# add_executable(camera camera.cpp)
add_executable(camera_calibration camera_calibration.cpp)
# target_link_libraries(  camera ${OpenCV_LIBS}  )
target_link_libraries(  camera_calibration ${OpenCV_LIBS}  )
