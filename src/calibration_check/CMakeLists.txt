cmake_minimum_required(VERSION 3.0.2)
project(calibration_check)

find_package(catkin REQUIRED COMPONENTS cv_bridge pcl_ros image_transport message_generation sensor_msgs)
generate_messages(DEPENDENCIES sensor_msgs)
catkin_package(CATKIN_DEPENDS cv_bridge image_transport message_runtime sensor_msgs)
find_package(OpenCV)
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(calibration_check_node src/main.cpp)
add_dependencies(calibration_check_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(calibration_check_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})