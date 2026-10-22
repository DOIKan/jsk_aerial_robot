cmake_minimum_required(VERSION 3.0.2)
project(uuv_d)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  aerial_robot_msgs
  aerial_robot_control
  aerial_robot_model
  aerial_robot_simulation
  gazebo_ros_control
  geometry_msgs
  mujoco_ros_control
  mujoco_ros_utils
  pluginlib
  spinal
  tf
  eigen_conversions
  tf_conversions
  std_srvs
  )

find_package(Eigen3 REQUIRED)
find_package(GAZEBO REQUIRED)

catkin_package(
  INCLUDE_DIRS include test
  LIBRARIES uuv_d_robot_model uuv_d_controller_pluginlib uuv_d_buoyancy_hw_sim uuv_d_numerical_jacobians
  CATKIN_DEPENDS roscpp aerial_robot_msgs aerial_robot_control aerial_robot_model aerial_robot_simulation gazebo_ros_control geometry_msgs mujoco_ros_utils pluginlib spinal tf eigen_conversions tf_conversions std_srvs
  DEPENDS GAZEBO
)

# Eigen requires optimization to get good performance
# http://eigen.tuxfamily.org/index.php?title=FAQ#Optimization
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g -DNDEBUG")


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  test
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)


### flight control plugin
add_library(uuv_d_robot_model
  src/model/uuv_d_multilink_robot_model.cpp)
target_link_libraries(uuv_d_robot_model ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
add_dependencies(uuv_d_robot_model aerial_robot_msgs_generate_messages_cpp spinal_generate_messages_cpp)

add_library(uuv_d_controller_pluginlib
  src/uuv_d_controller.cpp
  src/uuv_d_multilink_controller.cpp)
target_link_libraries(uuv_d_controller_pluginlib uuv_d_robot_model ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
add_dependencies(uuv_d_controller_pluginlib aerial_robot_msgs_generate_messages_cpp spinal_generate_messages_cpp)

add_library(uuv_d_buoyancy_hw_sim src/model/uuv_d_buoyancy_hw_sim.cpp)
target_compile_definitions(uuv_d_buoyancy_hw_sim PRIVATE SIMULATION)
target_link_libraries(uuv_d_buoyancy_hw_sim ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

catkin_install_python(PROGRAMS
  scripts/robstride_joint_bridge.py
  scripts/robstride_startup.py
  scripts/gimbal_pwm_bridge.py
  scripts/joint_state_merge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/uuv_d/ test/uuv_d/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY config plugins launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(DIRECTORY bin
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(DIRECTORY arduino
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS uuv_d_buoyancy_hw_sim
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

mujoco_model_convert(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/config/mujoco_model.yaml)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_subdirectory(test)
endif()
