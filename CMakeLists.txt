cmake_minimum_required(VERSION 2.8.3)
project(alvar_logger)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    cv_bridge
    image_transport
    ar_track_alvar
)
find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

### EXECUTABLES ###############################################
add_executable(alvar_logger
  ros/src/alvar_logger_node.cpp
)
target_link_libraries(alvar_logger
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
