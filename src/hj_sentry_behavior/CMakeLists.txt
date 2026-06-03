cmake_minimum_required(VERSION 3.16)
project(pb2025_sentry_behavior)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

ament_auto_add_executable(pb2025_sentry_behavior_server src/pb2025_sentry_behavior_server.cpp)

ament_auto_add_library(action_client SHARED
  src/pb2025_sentry_behavior_client.cpp
)

rclcpp_components_register_node(action_client
  PLUGIN pb2025_sentry_behavior::SentryBehaviorClient
  EXECUTABLE pb2025_sentry_behavior_client
)

####### Conditions #######
ament_auto_add_library(is_game_status SHARED plugins/condition/is_game_status.cpp)
list(APPEND plugin_libs is_game_status)

ament_auto_add_library(is_status_ok SHARED plugins/condition/is_status_ok.cpp)
list(APPEND plugin_libs is_status_ok)

ament_auto_add_library(is_detect_enemy SHARED plugins/condition/is_detect_enemy.cpp)
list(APPEND plugin_libs is_detect_enemy)

ament_auto_add_library(enemy_stop_gate SHARED plugins/condition/enemy_stop_gate.cpp)
list(APPEND plugin_libs enemy_stop_gate)

ament_auto_add_library(is_blackboard_string SHARED plugins/condition/is_blackboard_string.cpp)
list(APPEND plugin_libs is_blackboard_string)

ament_auto_add_library(is_blackboard_bool SHARED plugins/condition/is_blackboard_bool.cpp)
list(APPEND plugin_libs is_blackboard_bool)

####### Actions #######
ament_auto_add_library(nav2_pose SHARED plugins/action/nav2_pose.cpp)
list(APPEND plugin_libs nav2_pose)

ament_auto_add_library(nav2_pose_rough_road SHARED plugins/action/nav2_pose_rough_road.cpp)
list(APPEND plugin_libs nav2_pose_rough_road)

ament_auto_add_library(nav2_through_poses SHARED plugins/action/nav2_through_poses.cpp)
list(APPEND plugin_libs nav2_through_poses)

ament_auto_add_library(send_nav2_goal SHARED plugins/action/send_nav2_goal.cpp)
list(APPEND plugin_libs send_nav2_goal)

ament_auto_add_library(pub_nav2_goal SHARED plugins/action/pub_nav2_goal.cpp)
list(APPEND plugin_libs pub_nav2_goal)

ament_auto_add_library(pub_mode_control SHARED plugins/action/pub_mode_control.cpp)
list(APPEND plugin_libs pub_mode_control)

ament_auto_add_library(rough_road_speed_control SHARED plugins/action/rough_road_speed_control.cpp)
list(APPEND plugin_libs rough_road_speed_control)

ament_auto_add_library(sentry_posture_manager SHARED plugins/action/sentry_posture_manager.cpp)
list(APPEND plugin_libs sentry_posture_manager)

ament_auto_add_library(pub_twist SHARED plugins/action/pub_twist.cpp)
list(APPEND plugin_libs pub_twist)

ament_auto_add_library(calculate_attack_pose SHARED plugins/action/calculate_attack_pose.cpp)
list(APPEND plugin_libs calculate_attack_pose)

ament_auto_add_library(load_waypoints SHARED plugins/action/load_waypoints.cpp)
list(APPEND plugin_libs load_waypoints)

ament_auto_add_library(get_current_waypoint SHARED plugins/action/get_current_waypoint.cpp)
list(APPEND plugin_libs get_current_waypoint)

ament_auto_add_library(update_manual_nav_goal SHARED plugins/action/update_manual_nav_goal.cpp)
list(APPEND plugin_libs update_manual_nav_goal)

ament_auto_add_library(wait_duration SHARED plugins/action/wait_duration.cpp)
list(APPEND plugin_libs wait_duration)

ament_auto_add_library(next_waypoint SHARED plugins/action/next_waypoint.cpp)
list(APPEND plugin_libs next_waypoint)

ament_auto_add_library(select_supply_route_by_area SHARED plugins/action/select_supply_route_by_area.cpp)
list(APPEND plugin_libs select_supply_route_by_area)

ament_auto_add_library(reset_supply_route SHARED plugins/action/reset_supply_route.cpp)
list(APPEND plugin_libs reset_supply_route)

####### Decorators #######
ament_auto_add_library(tick_after_timeout_node SHARED plugins/decorator/tick_after_timeout_node.cpp)
list(APPEND plugin_libs tick_after_timeout_node)

foreach(bt_plugin ${plugin_libs})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

#############
## Testing ##
#############

if(BUILD_TESTING)
  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-format")
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
    ament_cmake_flake8
  )
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_omni_perception_utils test/test_omni_perception_utils.cpp)
  target_include_directories(test_omni_perception_utils PRIVATE include)
  ament_target_dependencies(test_omni_perception_utils robot_msgs)

  ament_add_gtest(test_update_manual_nav_goal test/test_update_manual_nav_goal.cpp)
  target_include_directories(test_update_manual_nav_goal PRIVATE include)
  ament_target_dependencies(
    test_update_manual_nav_goal
    robot_msgs
    geometry_msgs
    tf2_geometry_msgs
  )
  target_link_libraries(test_update_manual_nav_goal update_manual_nav_goal)
endif()

#############
## Install ##
#############

# 安装 Python 脚本
install(PROGRAMS
  scripts/fake_referee.py
  scripts/fake_referee_bad_status.py
  scripts/fake_referee_scenarios.py
  scripts/mapping_bag_recorder.py
  scripts/polygon_editor.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS ${plugin_libs}
  LIBRARY DESTINATION share/${PROJECT_NAME}/bt_plugins
  ARCHIVE DESTINATION share/${PROJECT_NAME}/bt_plugins
  RUNTIME DESTINATION share/${PROJECT_NAME}/bt_plugins
)

ament_auto_package(
  INSTALL_TO_SHARE
  behavior_trees
  params
  launch
)
