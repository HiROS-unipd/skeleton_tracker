cmake_minimum_required(VERSION 3.0)
project(hiros_skeleton_tracker)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

# Find catkin macros and libraries if COMPONENTS list like find_package(catkin
# REQUIRED COMPONENTS xyz) is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs hiros_skeleton_msgs)

# System dependencies are found with CMake's conventions
find_package(OpenCV)
find_package(RTB_Filter)

# Declare the package to be exported and made available to ros
catkin_package(CATKIN_DEPENDS std_msgs hiros_skeleton_msgs)

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
add_executable(hiros_marker_skeleton_tracker_node src/mainMST.cpp src/Filter.cpp src/Munkres.cpp
               src/MarkerSkeletonTracker.cpp src/utils.cpp)

# Rename C++ executable without prefix The above recommended prefix causes long
# target names, the following renames the target back to the shorter version for
# ease of user use e.g. "rosrun someones_pkg node" instead of "rosrun
# someones_pkg someones_pkg_node"
set_target_properties(hiros_marker_skeleton_tracker_node PROPERTIES OUTPUT_NAME
                      hiros_marker_skeleton_tracker PREFIX "")

# Add cmake target dependencies of the executable same as for the library above
add_dependencies(hiros_marker_skeleton_tracker_node ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})

# Specify libraries to link a library or executable target against
target_link_libraries(hiros_marker_skeleton_tracker_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

set_target_properties(hiros_marker_skeleton_tracker_node PROPERTIES PUBLIC_HEADER
                      include/skeleton_tracker/MarkerSkeletonTracker.h)

target_include_directories(hiros_marker_skeleton_tracker_node PRIVATE include ${catkin_INCLUDE_DIRS}
                                                                              ${OpenCV_INCLUDE_DIRS})


                                                                          # Declare a C++ executable With catkin_make all packages are built within a
# single CMake context The recommended prefix ensures that target names across
# packages don't collide add_executable(${PROJECT_NAME}_node
add_executable(hiros_orientation_skeleton_tracker_node src/mainOST.cpp src/Filter.cpp src/Munkres.cpp
               src/OrientationSkeletonTracker.cpp src/utils.cpp)

# Rename C++ executable without prefix The above recommended prefix causes long
# target names, the following renames the target back to the shorter version for
# ease of user use e.g. "rosrun someones_pkg node" instead of "rosrun
# someones_pkg someones_pkg_node"
set_target_properties(hiros_orientation_skeleton_tracker_node PROPERTIES OUTPUT_NAME
                      hiros_orientation_skeleton_tracker PREFIX "")

# Add cmake target dependencies of the executable same as for the library above
add_dependencies(hiros_orientation_skeleton_tracker_node ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})

# Specify libraries to link a library or executable target against
target_link_libraries(hiros_orientation_skeleton_tracker_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

set_target_properties(hiros_orientation_skeleton_tracker_node PROPERTIES PUBLIC_HEADER
                      include/skeleton_tracker/OrientationSkeletonTracker.h)

target_include_directories(hiros_orientation_skeleton_tracker_node PRIVATE include ${catkin_INCLUDE_DIRS}
                                                                                   ${OpenCV_INCLUDE_DIRS})

# ##############################################################################
# Install ##
# ##############################################################################
install(TARGETS hiros_marker_skeleton_tracker_node
                hiros_orientation_skeleton_tracker_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES include/skeleton_tracker/MarkerSkeletonTracker.h
              include/skeleton_tracker/OrientationSkeletonTracker.h
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
