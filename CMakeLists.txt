cmake_minimum_required(VERSION 2.8)
project(voris_bringup)
set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11;-Wall;-Wextra")
# Add module path for <name>.cmake conf files
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules)
## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_STANDARD 14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        image_transport
        roscpp
        sensor_msgs
        std_msgs
        message_generation
        message_filters
        )

find_package(OpenCV REQUIRED)




generate_messages(
        DEPENDENCIES
        std_msgs
        sensor_msgs
)


catkin_package(
        CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
)

add_executable(image_rescale src/image_rescale.cpp)
target_link_libraries(image_rescale ${catkin_LIBRARIES} ${OpenCV_LIBS})

## Specify additional locations of header files
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        )
