cmake_minimum_required(VERSION 3.5)
project(plansys2_cooking_example)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(plansys2_bt_actions REQUIRED)
find_package(visualization_msgs REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(dependencies
    rclcpp
    rclcpp_action
    geometry_msgs
    tf2_geometry_msgs
    nav2_msgs
    plansys2_msgs
    plansys2_executor
    ament_index_cpp
    plansys2_bt_actions
    visualization_msgs
)

include_directories(include)

add_library(plansys2_recharge_bt_node SHARED src/behavior_tree_nodes/Recharge.cpp)
list(APPEND plugin_libs plansys2_recharge_bt_node)

add_library(plansys2_close_gripper_bt_node SHARED src/behavior_tree_nodes/CloseGripper.cpp)
list(APPEND plugin_libs plansys2_close_gripper_bt_node)

add_library(plansys2_open_gripper_bt_node SHARED src/behavior_tree_nodes/OpenGripper.cpp)
list(APPEND plugin_libs plansys2_open_gripper_bt_node)

add_library(plansys2_move_bt_node SHARED src/behavior_tree_nodes/Move.cpp)
list(APPEND plugin_libs plansys2_move_bt_node)

add_library(plansys2_movef_bt_node SHARED src/behavior_tree_nodes/MoveF.cpp)
list(APPEND plugin_libs plansys2_movef_bt_node)

add_library(plansys2_approach_object_bt_node SHARED src/behavior_tree_nodes/ApproachObject.cpp)
list(APPEND plugin_libs plansys2_approach_object_bt_node)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(cook_cake_action_node src/cook_cake_action_node.cpp)
ament_target_dependencies(cook_cake_action_node ${dependencies})

add_executable(cook_omelette_action_node src/cook_omelette_action_node.cpp)
ament_target_dependencies(cook_omelette_action_node ${dependencies})

add_executable(cook_spaghetti_action_node src/cook_spaghetti_action_node.cpp)
ament_target_dependencies(cook_spaghetti_action_node ${dependencies})


add_executable(cooking_controller_node src/cooking_controller_node.cpp)
ament_target_dependencies(cooking_controller_node ${dependencies})

add_executable(cooking_controller_node_three_robots_sim
  src/cooking_controller_node_three_robots_sim.cpp)
ament_target_dependencies(cooking_controller_node_three_robots_sim ${dependencies})

add_executable(marker_pub src/marker_pub.cpp)
ament_target_dependencies(marker_pub ${dependencies})

add_executable(nav2_sim_node src/nav2_sim_node.cpp)
ament_target_dependencies(nav2_sim_node ${dependencies})

install(
  DIRECTORY maps params launch pddl behavior_trees_xml config 
  DESTINATION share/${PROJECT_NAME})

install(TARGETS
  cook_cake_action_node
  cook_omelette_action_node
  cook_spaghetti_action_node
  cooking_controller_node
  cooking_controller_node_three_robots_sim
  nav2_sim_node
  marker_pub
  ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_dependencies(${dependencies})

ament_package()