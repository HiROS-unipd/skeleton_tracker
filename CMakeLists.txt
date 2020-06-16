cmake_minimum_required(VERSION 3.0)
project(hiros_skeleton_tracker)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# Find catkin macros and libraries if COMPONENTS list like find_package(catkin
# REQUIRED COMPONENTS xyz) is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs skeleton_msgs)

# System dependencies are found with CMake's conventions
find_package(OpenCV)
find_package(RTB_Filter)

# Declare the package to be exported and made available to ros
catkin_package(CATKIN_DEPENDS std_msgs skeleton_msgs)

# ##############################################################################
# Build ##
# ##############################################################################
# Specify additional locations of header files Your package locations should be
# listed before other locations
include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${RTB_Filter_INCLUDE_DIRS})

# Display additional files in qtcreator
file(GLOB_RECURSE FILES_LIST "*.launch" "*.msg" "*.srv" "*.xml")
add_custom_target(additional_files_${PROJECT_NAME} SOURCES ${FILES_LIST})

# Declare a C++ executable With catkin_make all packages are built within a
# single CMake context The recommended prefix ensures that target names across
# packages don't collide add_executable(${PROJECT_NAME}_node
add_executable(${PROJECT_NAME}_node src/main.cpp src/Filter.cpp src/Munkres.cpp
                                    src/Tracker.cpp src/utils.cpp)

# Rename C++ executable without prefix The above recommended prefix causes long
# target names, the following renames the target back to the shorter version for
# ease of user use e.g. "rosrun someones_pkg node" instead of "rosrun
# someones_pkg someones_pkg_node"
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME
                                                      ${PROJECT_NAME} PREFIX "")

# Add cmake target dependencies of the executable same as for the library above
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})

# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

set_target_properties(${PROJECT_NAME}_node PROPERTIES PUBLIC_HEADER
                                                      include/opt/Tracker.h)

target_include_directories(
    ${PROJECT_NAME}_node PRIVATE include ${catkin_INCLUDE_DIRS}
                                 ${OpenCV_INCLUDE_DIRS})

# ##############################################################################
# Install ##
# ##############################################################################
install(TARGETS ${PROJECT_NAME}_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES include/opt/Tracker.h
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
