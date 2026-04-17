cmake_minimum_required(VERSION 3.10)
project(virtuose_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# 与 catkin 包一致：若存在已验证的 ROS1 工作区 SDK，则作为默认 VIRTUOSE_SDK_ROOT（可用 -DVIRTUOSE_SDK_ROOT= 覆盖）
get_filename_component(_haption_test "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
set(_ros1_virtuose_api "${_haption_test}/haption_ros1_ws/src/ROS-Virtuose/src/virtuose/external/VirtuoseAPI")
set(_pkg_virtuose_api "${CMAKE_CURRENT_SOURCE_DIR}/external/VirtuoseAPI")
if(EXISTS "${_ros1_virtuose_api}/libvirtuose.a" AND EXISTS "${_ros1_virtuose_api}/virtuoseAPI.so")
  set(_virtuose_sdk_default "${_ros1_virtuose_api}")
else()
  set(_virtuose_sdk_default "${_pkg_virtuose_api}")
endif()
set(VIRTUOSE_SDK_ROOT "${_virtuose_sdk_default}" CACHE PATH
  "Virtuose SDK：libvirtuose.a + virtuoseAPI.so；默认同 ~/haption_ros1_ws 中 catkin 包 external 目录")
if(EXISTS "${_ros1_virtuose_api}/libvirtuose.a" AND VIRTUOSE_SDK_ROOT STREQUAL "${_ros1_virtuose_api}")
  message(STATUS "virtuose_ros2: VIRTUOSE_SDK_ROOT -> ROS1 catkin external (与 rosrun virtuose virtuose_node 相同 SDK)")
endif()
if(NOT EXISTS "${VIRTUOSE_SDK_ROOT}/libvirtuose.a")
  message(FATAL_ERROR "libvirtuose.a not found under VIRTUOSE_SDK_ROOT='${VIRTUOSE_SDK_ROOT}'. Copy the Virtuose SDK there.")
endif()
if(NOT EXISTS "${VIRTUOSE_SDK_ROOT}/virtuoseAPI.so")
  message(FATAL_ERROR
    "virtuoseAPI.so missing in '${VIRTUOSE_SDK_ROOT}'. "
    "libvirtuose.a loads it via dlopen at runtime; copy the full Virtuose SDK (including .so) next to libvirtuose.a.")
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/InVirtuoseForce.msg"
  "msg/InVirtuosePose.msg"
  "msg/InVirtuoseSpeed.msg"
  "msg/OutVirtuoseForce.msg"
  "msg/OutVirtuosePhysicalPose.msg"
  "msg/OutVirtuosePose.msg"
  "msg/OutVirtuoseSpeed.msg"
  "msg/OutVirtuoseStatus.msg"
  "srv/VirtuoseAdmittance.srv"
  "srv/VirtuoseCalibrate.srv"
  "srv/VirtuoseImpedance.srv"
  "srv/VirtuoseReset.srv"
  DEPENDENCIES geometry_msgs builtin_interfaces
)

function(virtuose_link_sdk target_name)
  target_include_directories(${target_name} PRIVATE "${VIRTUOSE_SDK_ROOT}")
  target_include_directories(${target_name} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include")
  target_link_libraries(${target_name} "${VIRTUOSE_SDK_ROOT}/libvirtuose.a")
  target_link_libraries(${target_name} pthread dl)
endfunction()

add_executable(virtuose_node src/virtuose_node.cpp)
virtuose_link_sdk(virtuose_node)
# libvirtuose 在运行时 dlopen virtuoseAPI.so；可执行文件在 lib/<pkg>/ 下，$ORIGIN/.. 即包的 lib/
set_target_properties(virtuose_node PROPERTIES
  INSTALL_RPATH "\$ORIGIN/.."
  BUILD_RPATH "${VIRTUOSE_SDK_ROOT}"
)
ament_target_dependencies(virtuose_node rclcpp geometry_msgs)
rosidl_target_interfaces(virtuose_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

macro(virtuose_test_executable name)
  add_executable(${name} src/${name}.cpp)
  ament_target_dependencies(${name} rclcpp geometry_msgs)
  rosidl_target_interfaces(${name} ${PROJECT_NAME} "rosidl_typesupport_cpp")
endmacro()

virtuose_test_executable(test_admittance_node)
virtuose_test_executable(test_calibration_node)
virtuose_test_executable(test_impedance_node)

install(TARGETS virtuose_node test_admittance_node test_calibration_node test_impedance_node
  DESTINATION lib/${PROJECT_NAME})

install(FILES "${VIRTUOSE_SDK_ROOT}/virtuoseAPI.so"
  DESTINATION lib)
if(EXISTS "${VIRTUOSE_SDK_ROOT}/virtuoseAPI_6d.so")
  install(FILES "${VIRTUOSE_SDK_ROOT}/virtuoseAPI_6d.so"
    DESTINATION lib)
endif()

install(DIRECTORY docs/
  DESTINATION share/${PROJECT_NAME}/doc
  FILES_MATCHING PATTERN "*.md")

ament_package()
