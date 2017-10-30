cmake_minimum_required(VERSION 2.8.3)
project(image_saver)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
add_definitions(-std=c++0x)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  image_transport
  cv_bridge 
  image_geometry 
  tf
  camera_info_manager 
	message_filters
)

find_package(OpenCV REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES publish_images
#  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)




 add_executable(another src/another.cpp)
 target_link_libraries(another  ${catkin_LIBRARIES} ${OpenCV_LIBS} )





add_executable(data_writer
  src/data_writer.cpp)
add_dependencies(data_writer ${catkin_EXPORTED_TARGETS})


target_link_libraries(data_writer  ${OpenCV_LIBS} ${catkin_LIBRARIES})
find_package(Boost REQUIRED COMPONENTS signals thread)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(data_writer  ${Boost_LIBRARIES} ${catkin_LIBRARIES})

	
add_executable(stereo_saver
  src/stereo_saver.cpp)
add_dependencies(stereo_saver ${catkin_EXPORTED_TARGETS} )

target_link_libraries(stereo_saver  ${OpenCV_LIBS} ${catkin_LIBRARIES})
find_package(Boost REQUIRED COMPONENTS signals thread)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(stereo_saver  ${Boost_LIBRARIES} ${catkin_LIBRARIES})
