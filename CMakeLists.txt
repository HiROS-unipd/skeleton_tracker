cmake_minimum_required(VERSION 3.5)
project(hiros_skeleton_tracker)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hiros_skeleton_msgs REQUIRED)
find_package(hiros_skeleton_filter REQUIRED)

find_package(OpenCV REQUIRED)

set(HEADER_FILES
    include/skeleton_tracker/Munkres.h
    include/skeleton_tracker/SkeletonGroupBuffer.h
    include/skeleton_tracker/utils.h
)

set(SOURCE_FILES
    src/Munkres.cpp
    src/SkeletonGroupBuffer.cpp
    src/utils.cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_library(skeleton_tracker ${SOURCE_FILES} ${HEADER_FILES})
target_include_directories(skeleton_tracker PRIVATE include)
ament_target_dependencies(skeleton_tracker hiros_skeleton_msgs OpenCV)
ament_export_targets(skeleton_tracker HAS_LIBRARY_TARGET)
ament_export_libraries(skeleton_tracker)
ament_export_include_directories(include)
ament_export_dependencies(ament_cmake hiros_skeleton_msgs OpenCV)

add_executable(${PROJECT_NAME}_node
    src/main.cpp
    src/Tracker.cpp
)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp hiros_skeleton_filter)
target_link_libraries(${PROJECT_NAME}_node skeleton_tracker)

set_target_properties(${PROJECT_NAME}_node
  PROPERTIES OUTPUT_NAME
  ${PROJECT_NAME}
  PREFIX
  ""
)

target_include_directories(${PROJECT_NAME}_node
  PRIVATE include
)

install(
  DIRECTORY include/skeleton_tracker
  DESTINATION include
)

install(
  TARGETS skeleton_tracker
  EXPORT skeleton_tracker
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
