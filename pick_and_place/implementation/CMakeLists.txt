project(marm_code)
cmake_minimum_required(VERSION 3.5)

SET(CMAKE_CXX_STANDARD 17)

include_directories(/usr/local/include/ddscxx /usr/local/include/iceoryx/v2.0.2)
link_libraries(unitree_sdk2 ddsc ddscxx rt pthread)

# Pick and Place Applications
add_executable(pick_and_place pick_and_place.cpp msg/ArmString_.cpp)
add_executable(pap_with_trajectory_planning pap_with_trajectory_planning.cpp msg/ArmString_.cpp)
add_executable(test_single_move test_single_move.cpp msg/ArmString_.cpp msg/PubServoInfo_.cpp)
add_executable(move_to_safe move_to_safe.cpp msg/ArmString_.cpp)
