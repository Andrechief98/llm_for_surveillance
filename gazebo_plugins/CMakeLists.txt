cmake_minimum_required(VERSION 3.0.2)
project(gazebo_plugins)  # Renamed package

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  geometry_msgs
  message_generation
  cv_bridge
  sensor_msgs
)

find_package(gazebo REQUIRED)

# Message generation
add_message_files(FILES ModelBoundingBoxes.msg ActorVelocities.msg)

# Service generation
add_service_files(FILES retrieveBoundingBoxes.srv doorStringCommand.srv)


generate_messages(DEPENDENCIES std_msgs geometry_msgs)




find_package(OpenCV REQUIRED)
# find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs cv_bridge)





# Include directories
include_directories(${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

# Build the plugin library
add_library(bounding_box_plugin SHARED src/bounding_box_plugin.cpp)
target_link_libraries(bounding_box_plugin 
  ${GAZEBO_LIBRARIES} 
  ${catkin_LIBRARIES}
)

add_library(door_control_plugin SHARED src/door_control_plugin.cpp)
target_link_libraries(door_control_plugin 
  ${GAZEBO_LIBRARIES} 
  ${catkin_LIBRARIES}
)

add_library(camera_plugin SHARED src/camera_plugin.cpp)
target_link_libraries(camera_plugin 
  ${GAZEBO_LIBRARIES} 
  ${catkin_LIBRARIES}
)


# Catkin package configuration
catkin_package(
  LIBRARIES bounding_box_plugin door_control_plugin camera_plugin # Match the library name
  CATKIN_DEPENDS gazebo_ros roscpp geometry_msgs
)

# Install the library
install(TARGETS bounding_box_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS door_control_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS camera_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)



