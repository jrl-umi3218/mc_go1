if(TARGET Go1)
  set(ROBOT_MODULE_PARAMS "\"Go1\"")
  configure_file("${CMAKE_CURRENT_SOURCE_DIR}/loader.in.cpp"
                 "${CMAKE_CURRENT_BINARY_DIR}/Go1/loader.cpp")
  add_executable(loader_Go1 "${CMAKE_CURRENT_BINARY_DIR}/Go1/loader.cpp")
  target_link_libraries(loader_Go1 PUBLIC mc_rtc::mc_rbdyn)
  add_test(NAME loader_Go1 COMMAND loader_Go1 "$<TARGET_FILE_DIR:Go1>")
endif()

if(EXISTS "${PROJECT_BINARY_DIR}/yaml/Go1YAML.yaml")
  set(ROBOT_MODULE_PARAMS
      "\"json\", \"${PROJECT_BINARY_DIR}/yaml/Go1YAML.yaml\"")
  configure_file("${CMAKE_CURRENT_SOURCE_DIR}/loader.in.cpp"
                 "${CMAKE_CURRENT_BINARY_DIR}/Go1YAML/loader.cpp")
  add_executable(loader_Go1YAML
                 "${CMAKE_CURRENT_BINARY_DIR}/Go1YAML/loader.cpp")
  target_link_libraries(loader_Go1YAML PUBLIC mc_rtc::mc_rbdyn)
  add_test(NAME loader_Go1YAML COMMAND loader_Go1YAML
                                       "${MC_ROBOTS_RUNTIME_INSTALL_PREFIX}")
endif()
