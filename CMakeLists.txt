PROJECT(cs393r_starter)
cmake_minimum_required(VERSION 3.5)


MESSAGE(STATUS "Using compiler: ${CMAKE_CXX_COMPILER}")
MESSAGE(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "Arch: ${CMAKE_SYSTEM_PROCESSOR}")

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

SET(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Werror")

IF(${CMAKE_BUILD_TYPE} MATCHES "Release")
  MESSAGE(STATUS "Additional Flags for Release mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fopenmp -O2 -DNDEBUG")
ELSEIF(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  MESSAGE(STATUS "Additional Flags for Debug mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g")
ENDIF()

find_package(ament_cmake REQUIRED)

MESSAGE(STATUS "ROS-Overrride Build Type: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "CXX Flags: ${CMAKE_CXX_FLAGS}")

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/submodules")

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(amrl_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(irobot_create_msgs REQUIRED)

SET(libs glog gflags amrl-shared-lib boost_system lua5.1
    pthread)

ADD_LIBRARY(shared_library
            src/visualization/visualization.cc
            src/vector_map/vector_map.cc)
ament_target_dependencies(shared_library rclcpp amrl_msgs sensor_msgs geometry_msgs)

ADD_SUBDIRECTORY(src/shared)
INCLUDE_DIRECTORIES(src/shared)
INCLUDE_DIRECTORIES(src)

# This package installs libraries without exporting them.
# Export the library path to ensure that the installed libraries are available.
if(NOT WIN32)
  ament_environment_hooks(
          "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

add_executable(slam
                        src/slam/slam_main.cc
                        src/slam/slam.cc)
TARGET_LINK_LIBRARIES(slam shared_library ${libs})
ament_target_dependencies(slam rclcpp amrl_msgs sensor_msgs geometry_msgs)


add_executable(particle_filter
                        src/particle_filter/particle_filter_main.cc
                        src/particle_filter/particle_filter.cc)
TARGET_LINK_LIBRARIES(particle_filter shared_library ${libs})
ament_target_dependencies(particle_filter rclcpp amrl_msgs sensor_msgs geometry_msgs)

add_executable(navigation
                        src/navigation/navigation_main.cc
                        src/navigation/navigation.cc)
TARGET_LINK_LIBRARIES(navigation shared_library ${libs})
ament_target_dependencies(navigation rclcpp rclcpp_action amrl_msgs sensor_msgs geometry_msgs irobot_create_msgs)

add_executable(eigen_tutorial
               src/eigen_tutorial.cc)

add_executable(simple_queue_test
               src/navigation/simple_queue_test.cc)

add_executable(cimg_example
               src/cimg_example.cc)
TARGET_LINK_LIBRARIES(cimg_example shared_library ${libs} X11)


install(TARGETS
        slam
        particle_filter
        navigation
        eigen_tutorial
        simple_queue_test
        cimg_example
        DESTINATION lib/${PROJECT_NAME})


ament_package()