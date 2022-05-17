cmake_minimum_required(VERSION 3.4)

project(clover-franka CXX)

list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_LIST_DIR}/../cmake)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Poco REQUIRED COMPONENTS Foundation)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_library(clover_common STATIC
  clover_common.cpp
)

target_link_libraries(clover_common PUBLIC Franka::Franka Eigen3::Eigen3)

set(CLOVER
        generate_cartesian_circular_motion_left
        generate_cartesian_circular_motion_right
        cartesian_circular_motion_with_joint_impedance_torque_left
        cartesian_circular_motion_with_joint_impedance_torque_right
        )

foreach(example ${CLOVER})
  add_executable(${example} ${example}.cpp)
  target_link_libraries(${example} Franka::Franka clover_common Eigen3::Eigen3)
endforeach()

target_link_libraries(generate_cartesian_circular_motion_left Threads::Threads)
target_link_libraries(generate_cartesian_circular_motion_right Threads::Threads)
target_link_libraries(cartesian_circular_motion_with_joint_impedance_torque_left Threads::Threads)
target_link_libraries(cartesian_circular_motion_with_joint_impedance_torque_right Threads::Threads)
# target_link_libraries(motion_with_control Poco::Foundation)

include(GNUInstallDirs)
install(TARGETS ${CLOVER}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
