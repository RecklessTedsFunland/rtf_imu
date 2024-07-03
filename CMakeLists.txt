cmake_minimum_required(VERSION 3.8)
project(rtf_sensors)

# Default to C17
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 17)
endif()

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


include(FetchContent)

# squaterion ------------
FetchContent_Declare(squaternion
    GIT_REPOSITORY "https://github.com/MomsFriendlyRobotCompany/squaternion.git"
    GIT_TAG "origin/master"
    SOURCE_SUBDIR "cpp"
)

FetchContent_Declare(gcisensors
    GIT_REPOSITORY "https://github.com/the-guild-of-calamitous-intent/gciSensors.git"
    GIT_TAG "origin/main"
    SOURCE_SUBDIR "linux"
)

FetchContent_MakeAvailable( squaternion gcisensors )

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


list(APPEND node_list
    imu_node
    gps_node
)

foreach(app ${node_list})
    add_executable(${app} src/${app}.cpp)
    target_include_directories(${app} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
    target_link_libraries(${app} squaternion gcisensors)
    ament_target_dependencies(
        ${app}
        rclcpp std_msgs sensor_msgs geometry_msgs
    )

    # you NEED this so setup.bash can find the binary
    install(
        TARGETS "${app}"
        DESTINATION "lib/${PROJECT_NAME}")
endforeach()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

# rosidl_generate_interfaces(${PROJECT_NAME}
#    "msgs/NemaSentence.msg"
#    "msgs/Gpgga.msg"
#    "msgs/Gpgsa.msg"
#    "msgs/Gpgst.msg"
# #    "msgs/Gpgsv.msg"
# #    "msgs/GpgsvSatellite.msg"
#    "msgs/Gprmc.msg"
# #    "msgs/Gpvtg.msg"
# #    "msgs/Gpzda.msg"
#    DEPENDENCIES std_msgs
# )

# ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
