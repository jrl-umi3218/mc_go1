# Here we find a typical ROS description package, this set the
# JVRC_DESCRIPTION_PATH CMake variable
# find_description_package(mc_go1_description) set(GO1_DESCRIPTION_PATH
# "/home/ahmedaymen/WorkSpace/mc_rtc/src/catkin_data_ws/src/mc_rtc_data/go1_description")
# set(GO1_DESCRIPTION_PATH
# "/home/ahmedaymen/WorkSpace/mc_rtc/src/catkin_ws/src/mc_go1_description")

if("${mc_go1_description_INSTALL_PREFIX}" STREQUAL "")
  if("${mc_go1_description_SOURCE_PREFIX}" STREQUAL "")
    message(
      FATAL_ERROR
        "Your mc_go1_description package does not define where to find the data"
    )
  endif()
  set(GO1_DESCRIPTION_PATH "${mc_go1_description_SOURCE_PREFIX}")
else()
  set(GO1_DESCRIPTION_PATH "${mc_go1_description_INSTALL_PREFIX}")
  if(EXISTS "${GO1_DESCRIPTION_PATH}/share/mc_go1_description")
    set(GO1_DESCRIPTION_PATH "${GO1_DESCRIPTION_PATH}/share/mc_go1_description")
  endif()
endif()
if(DEFINED GO1_DESCRIPTION_PATH)
  message(STATUS "Found mc_go1_description: ${GO1_DESCRIPTION_PATH}")
endif()

# Generate the robot module C++ code using this value
configure_file(config.in.h "${CMAKE_CURRENT_BINARY_DIR}/config.h")

set(module_SRC Go1.cpp)

set(module_HDR Go1.h)

add_robot(Go1 "${module_SRC}" "${module_HDR}")
target_include_directories(Go1 PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_include_directories(Go1 PRIVATE "${CMAKE_CURRENT_BINARY_DIR}")
