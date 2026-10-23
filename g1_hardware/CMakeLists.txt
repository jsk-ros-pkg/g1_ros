cmake_minimum_required(VERSION 3.22)
project(g1_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # For clang-tidy

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED audio_common_msgs sound_play_msgs)

# unitree_sdk2 uses a namespaced target, not auto-detectable via ament_auto
find_package(unitree_sdk2 REQUIRED)
find_package(cppzmq REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/g1_hardware_interface.cpp
  src/inspire_rh56dfx_hardware_interface.cpp
)

target_link_libraries(${PROJECT_NAME}
  unitree_sdk2
)

ament_auto_add_library(gear_sonic_interface SHARED
  src/gear_sonic_interface.cpp
)

target_link_libraries(gear_sonic_interface
  cppzmq
)

rclcpp_components_register_node(gear_sonic_interface
  PLUGIN "g1_hardware::GearSonicInterface"
  EXECUTABLE gear_sonic_interface_node
)

target_compile_definitions(${PROJECT_NAME} PRIVATE "PLUGINLIB_BUILDING_LIBRARY")

pluginlib_export_plugin_description_file(hardware_interface g1_hardware_plugin.xml)

ament_auto_add_library(gear_sonic_controller SHARED
  src/gear_sonic_controller.cpp
)

rclcpp_components_register_node(gear_sonic_controller
  PLUGIN "g1_hardware::GearSonicController"
  EXECUTABLE gear_sonic_controller_node
)

ament_auto_add_library(loco_cmd_adapter SHARED
  src/loco_cmd_adapter.cpp
)

rclcpp_components_register_node(loco_cmd_adapter
  PLUGIN "loco_cmd_adapter::LocoCmdAdapterNode"
  EXECUTABLE loco_cmd_adapter_node
)

ament_auto_add_library(g1_livox_interface SHARED
  src/g1_livox_interface.cpp
)

target_link_libraries(g1_livox_interface
  unitree_sdk2
)

rclcpp_components_register_node(g1_livox_interface
  PLUGIN "g1_hardware::G1LivoxInterfaceNode"
  EXECUTABLE g1_livox_interface_node
)

ament_auto_add_library(g1_audio_interface SHARED
  src/g1_audio_interface.cpp
)
target_link_libraries(g1_audio_interface unitree_sdk2)
rclcpp_components_register_node(g1_audio_interface
  PLUGIN "g1_hardware::G1AudioInterfaceNode"
  EXECUTABLE g1_audio_interface_node
)

install(FILES g1_hardware_plugin.xml
  DESTINATION share/${PROJECT_NAME})

install(
  PROGRAMS
    scripts/play_audio.py
    scripts/example_sonic_pose.py
    scripts/replay_smpl_motion.py
    scripts/example_smpl_jump
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_g1_audio_interface test/test_g1_audio_interface.cpp)
  target_link_libraries(test_g1_audio_interface g1_audio_interface)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_play_audio test/test_play_audio.py)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cppcheck_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_clang_format_CONFIG_FILE .clang-format)
  set(ament_cmake_clang_tidy_CONFIG_FILE .clang-tidy)
  set(ament_cmake_flake8_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE config data launch
)
