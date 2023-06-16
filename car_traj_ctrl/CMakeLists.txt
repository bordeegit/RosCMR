cmake_minimum_required(VERSION 3.0.2)
project(car_traj_ctrl)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  dynamic_reconfigure
)


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

generate_dynamic_reconfigure_options(
  config/gainPI.cfg
)


###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES car_traj_ctrl
  CATKIN_DEPENDS roscpp rospy std_msgs dynamic_reconfigure
)


###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(car_kin_PI src/car_kin_PI.cpp src/car_kin_PI_node.cpp src/car_kin_fblin.cpp)
add_dependencies(car_kin_PI ${PROJECT_NAME}_gencfg)
target_link_libraries(car_kin_PI ${catkin_LIBRARIES})

add_executable(car_kin_sim src/car_kin_sim.cpp src/car_kin_sim_node.cpp src/car_kin_ode.cpp)
target_link_libraries(car_kin_sim ${catkin_LIBRARIES})
