cmake_minimum_required(VERSION 3.22)
project(inspire_hand_driver)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED)
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/rh56dfx_hardware.cpp
  src/rh56dfx_hand.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
pluginlib_export_plugin_description_file(hardware_interface plugins.xml)
ament_auto_package(INSTALL_TO_SHARE config launch urdf)
