# Here we find a typical ROS description package, this set the JVRC_DESCRIPTION_PATH CMake variable
find_description_package(jvrc_description)
# Generate the robot module C++ code using this value
configure_file(Go1.in.cpp "${CMAKE_CURRENT_BINARY_DIR}/Go1.cpp")

set(module_SRC
  "${CMAKE_CURRENT_BINARY_DIR}/Go1.cpp"
)

set(module_HDR
  Go1.h
)

add_robot(Go1 "${module_SRC}" "${module_HDR}")
target_include_directories(Go1 PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
