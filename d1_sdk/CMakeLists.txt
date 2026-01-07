project(marm_code)
cmake_minimum_required(VERSION 3.5)

SET(CMAKE_CXX_STANDARD 17)

include_directories(/usr/local/include/ddscxx /usr/local/include/iceoryx/v2.0.2)
link_libraries(unitree_sdk2 ddsc ddscxx rt pthread)

# Original SDK Examples
add_executable(joint_angle_control src/examples/joint_angle_control.cpp src/msg/ArmString_.cpp)
add_executable(multiple_joint_angle_control src/examples/multiple_joint_angle_control.cpp src/msg/ArmString_.cpp)
add_executable(joint_enable_control src/examples/joint_enable_control.cpp src/msg/ArmString_.cpp)
add_executable(arm_zero_control src/examples/arm_zero_control.cpp src/msg/ArmString_.cpp)
add_executable(get_arm_joint_angle src/examples/get_arm_joint_angle.cpp src/msg/ArmString_.cpp src/msg/PubServoInfo_.cpp)

# Custom Applications
add_executable(pick_and_place src/pick_and_place.cpp src/msg/ArmString_.cpp)
add_executable(pap_with_trajectory_planning src/pap_with_trajectory_planning.cpp src/msg/ArmString_.cpp)
add_executable(test_single_move src/test_single_move.cpp src/msg/ArmString_.cpp src/msg/PubServoInfo_.cpp)
add_executable(move_to_safe src/move_to_safe.cpp src/msg/ArmString_.cpp)
