cmake_minimum_required(VERSION 3.5)
project(my_package)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ros2_controllers REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/InferenceResult.msg"
    "msg/Yolov8Inference.msg"
    DEPENDENCIES std_msgs
 )

find_package(PCL REQUIRED)
# ✅ Ensure PCL include directories are set before adding executables
include_directories(
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    include  # If you have an 'include/' directory in your package
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# ✅ Define executables first, then link them properly
add_executable(voxelizing src/voxelizing.cpp)
add_executable(planner_seg src/planner_seg.cpp)
add_executable(clustering src/clustering.cpp)

# ✅ Ensure correct linking order
target_link_libraries(voxelizing ${PCL_LIBRARIES})
target_link_libraries(planner_seg ${PCL_LIBRARIES})
target_link_libraries(clustering ${PCL_LIBRARIES})


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
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

install(
  DIRECTORY config description launch worlds scripts
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
  launch/imu_filter.py
  scripts/lidar_estimation_node.py
  scripts/yolov8_ros2_pt.py
  scripts/yolov8_ros2_subscriber.py
  launch/launch_yolov8.launch.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
voxelizing planner_seg clustering
  DESTINATION lib/${PROJECT_NAME})

ament_package()