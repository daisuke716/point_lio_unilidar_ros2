cmake_minimum_required(VERSION 3.8)
project(point_lio_unilidar)

# 编译优化
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")

add_compile_options(-O3 -fexceptions -pthread)

# 兼容多线程（MP_EN）
message("Current CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")
include(ProcessorCount)
ProcessorCount(N)
message("Processor number: ${N}")

if(N GREATER 5)
    add_definitions(-DMP_EN -DMP_PROC_NUM=4)
elseif(N GREATER 3)
    math(EXPR PROC_NUM "${N}-2")
    add_definitions(-DMP_EN -DMP_PROC_NUM=${PROC_NUM})
else()
    add_definitions(-DMP_PROC_NUM=1)
endif()

# ============================
# ROS2 依赖
# ============================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(Eigen3 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
    include
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

# ============================
# 主节点 pointlio_mapping
# ============================
add_executable(pointlio_mapping
    src/laserMapping.cpp
    src/preprocess.cpp
    src/parameters.cpp
    src/Estimator.cpp
    include/ikd-Tree/ikd_Tree.cpp
)

ament_target_dependencies(pointlio_mapping
    rclcpp
    sensor_msgs
    nav_msgs
    geometry_msgs
    tf2
    tf2_ros
    PCL
    pcl_conversions
    yaml-cpp
    visualization_msgs
)

target_link_libraries(pointlio_mapping
    ${PCL_LIBRARIES}
    yaml-cpp
)

# Define BOOST_BIND_GLOBAL_PLACEHOLDERS for this target to suppress deprecation
# warnings about boost bind placeholders being injected into the global namespace.
target_compile_definitions(pointlio_mapping PRIVATE BOOST_BIND_GLOBAL_PLACEHOLDERS)

install(
    TARGETS pointlio_mapping
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY launch config rviz_cfg
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
