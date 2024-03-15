message(STATUS "Found MC_GO1_DESCRIPTION_PATH: ${MC_GO1_DESCRIPTION_PATH}")

# Generate the robot module C++ code using this value
configure_file(config.in.h "${CMAKE_CURRENT_BINARY_DIR}/config.h")

set(module_SRC Go1.cpp)

set(module_HDR Go1.h)

add_robot(Go1 "${module_SRC}" "${module_HDR}")
target_include_directories(Go1 PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_include_directories(Go1 PRIVATE "${CMAKE_CURRENT_BINARY_DIR}")
