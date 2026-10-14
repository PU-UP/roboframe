add_library(demo_robot_contract INTERFACE)
add_library(DemoRobot::contract ALIAS demo_robot_contract)
target_include_directories(demo_robot_contract INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(demo_robot_contract INTERFACE Roboframe::sdk)
